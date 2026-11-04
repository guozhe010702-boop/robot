"""EC/EA 8055 JSON-RPC transport and verified convenience methods.

Source: EC_SDK_Socket手册_Ver3.22.2, sections 2.1, 2.2.1, 2.2.2,
2.2.4 and 2.2.8. The manual targets Ver3.22.2; a controller may support
only a subset. This module does not implement firmware-side functions.
"""

from __future__ import annotations

import json
import math
import re
import socket
import threading
import time
from collections.abc import Mapping, Sequence
from typing import Any


class ECSDKError(RuntimeError):
    """Base SDK error."""


class DisconnectedError(ECSDKError):
    """No usable 8055 connection is available."""


class ProtocolError(ECSDKError):
    """A reply could not be safely matched or decoded."""


class UnsafeCallError(ECSDKError):
    """A method outside the verified read-only set was not explicitly enabled."""


class RPCError(ECSDKError):
    """An error returned by the controller, with its original code/details."""

    def __init__(self, method: str, error: dict[str, Any]):
        self.method = method
        self.code = error["code"]
        self.message = error["message"]
        self.data = error.get("data")
        self.detail = error
        super().__init__(f"{method}: JSON-RPC {self.code}: {self.message}")


# Only methods checked against the indicated manual sections are freely callable.
# All other exact method names remain available through call(..., control=True)
# after the client is explicitly constructed with allow_control=True.
READ_ONLY_METHODS = frozenset({
    "getSoftVersion", "getJointVersion", "getRobotMode", "getRobotState",
    "getServoStatus", "getMotorStatus", "get_estop_status", "getAlarmNum",
    "get_joint_pos", "get_tcp_pose", "get_base_flange_pose", "poseMul",
    "poseInv", "pose_to_matrix", "matrix_to_pose", "positiveKinematic",
    "inverseKinematic",
})

_METHOD_NAME = re.compile(r"[A-Za-z_][A-Za-z_0-9]*\Z")


def _unit_type(value: int) -> int:
    if type(value) is not int or value not in (0, 1):
        raise ValueError("unit_type 必须是 0（角度）或 1（弧度）")
    return value


def _finite_vector(value: Sequence[float], count: int, label: str) -> list[float]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence) or len(value) != count:
        raise ValueError(f"{label} 必须是 {count} 个数")
    result = []
    for item in value:
        if isinstance(item, bool) or not isinstance(item, (int, float)) or not math.isfinite(item):
            raise ValueError(f"{label} 包含非有限数值")
        result.append(float(item))
    return result


class ECClient:
    """Serialized 8055/TCP JSON-RPC client. No implicit robot actions or retries.

    The safe default exposes only READ_ONLY_METHODS. For any other method,
    two explicit choices are required: allow_control=True in the constructor
    and control=True in that particular call. This is an API guard, not a
    substitute for site safety interlocks or method-specific validation.
    """

    def __init__(self, host: str, *, port: int = 8055, timeout: float = 3.0,
                 max_response_bytes: int = 1_048_576, allow_control: bool = False):
        if not isinstance(host, str) or not host.strip():
            raise ValueError("host 不能为空")
        if type(port) is not int or not 1 <= port <= 65535:
            raise ValueError("port 必须是 1–65535 的整数")
        if not isinstance(timeout, (int, float)) or not 0 < timeout < 60:
            raise ValueError("timeout 必须在 0–60 秒之间")
        if type(max_response_bytes) is not int or max_response_bytes < 1024:
            raise ValueError("max_response_bytes 必须不小于 1024")
        self.host, self.port = host.strip(), port
        self.timeout = float(timeout)
        self.max_response_bytes = max_response_bytes
        self.allow_control = bool(allow_control)
        self._socket: socket.socket | None = None
        self._buffer = bytearray()
        self._request_id = 0
        self._lock = threading.RLock()

    @property
    def connected(self) -> bool:
        return self._socket is not None

    def connect(self) -> ECClient:
        """Open a connection only; no enable, alarm reset, or motion calls."""
        with self._lock:
            self.close()
            self._socket = socket.create_connection((self.host, self.port), self.timeout)
            self._socket.settimeout(self.timeout)
            return self

    def close(self) -> None:
        with self._lock:
            sock, self._socket = self._socket, None
            self._buffer.clear()
            if sock is not None:
                sock.close()

    def __enter__(self) -> ECClient:
        return self.connect()

    def __exit__(self, _exc_type, _exc, _traceback) -> None:
        self.close()

    def _read_line(self, deadline: float) -> bytes:
        assert self._socket is not None
        while b"\n" not in self._buffer:
            if len(self._buffer) > self.max_response_bytes:
                raise ProtocolError("8055 响应超过大小上限或缺少换行")
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("8055 响应超时")
            self._socket.settimeout(remaining)
            chunk = self._socket.recv(min(65536, self.max_response_bytes + 1))
            if not chunk:
                raise DisconnectedError("8055 对端已关闭连接")
            self._buffer.extend(chunk)
        raw, _, remainder = self._buffer.partition(b"\n")
        self._buffer = bytearray(remainder)
        if len(raw) > self.max_response_bytes:
            raise ProtocolError("8055 响应超过大小上限")
        return bytes(raw)

    def call(self, method: str, params: Mapping[str, Any] | Sequence[Any] | None = None,
             *, control: bool = False) -> Any:
        """Call an exact manual method name using newline-delimited JSON-RPC.

        Any unlisted method is treated as potentially state-changing. A
        successful RPC return does NOT establish motion completion.
        Errors never trigger reconnection or automatic replay.
        """
        if not isinstance(method, str) or not _METHOD_NAME.fullmatch(method):
            raise ValueError("method 必须是手册中的精确 ASCII 方法名")
        if method not in READ_ONLY_METHODS and not (self.allow_control and control):
            raise UnsafeCallError(f"{method} 未列为已核对只读接口；需要双重显式控制许可")
        if params is None:
            params = []
        elif isinstance(params, Mapping):
            params = dict(params)
        elif isinstance(params, (list, tuple)):
            params = list(params)
        else:
            raise ValueError("params 必须是 JSON 对象或数组")
        with self._lock:
            if self._socket is None:
                raise DisconnectedError("尚未连接 EC 8055")
            self._request_id += 1
            request_id = self._request_id
            payload = {"jsonrpc": "2.0", "method": method, "params": params, "id": request_id}
            try:
                wire = (json.dumps(payload, ensure_ascii=False, allow_nan=False,
                                   separators=(",", ":")) + "\n").encode("utf-8")
                deadline = time.monotonic() + self.timeout
                self._socket.settimeout(self.timeout)
                self._socket.sendall(wire)
                raw = self._read_line(deadline)
                try:
                    reply = json.loads(raw)
                except (ValueError, UnicodeError) as exc:
                    raise ProtocolError("8055 响应不是完整有效的 JSON") from exc
                if (not isinstance(reply, dict) or reply.get("jsonrpc") != "2.0"
                        or type(reply.get("id")) is not int or reply["id"] != request_id
                        or (("result" in reply) == ("error" in reply))):
                    raise ProtocolError(f"8055 响应结构或 id 不匹配: {reply!r}")
                if "error" in reply:
                    error = reply["error"]
                    if (not isinstance(error, dict) or type(error.get("code")) is not int
                            or not isinstance(error.get("message"), str)):
                        raise ProtocolError(f"8055 error 结构无效: {error!r}")
                    raise RPCError(method, error)
                value = reply["result"]
                if isinstance(value, str):
                    try:
                        value = json.loads(value)
                    except ValueError:
                        pass
                return value
            except Exception:
                # Once a request may have been sent, a late reply cannot be
                # safely attributed to a later call. Reconnect explicitly.
                self.close()
                raise

    def get_soft_version(self) -> str:
        value = self.call("getSoftVersion")
        if not isinstance(value, str):
            raise ProtocolError(f"getSoftVersion 响应不是字符串: {value!r}")
        return value

    def get_robot_mode(self) -> int:
        return self._integer_reply("getRobotMode")

    def get_robot_state(self) -> int:
        return self._integer_reply("getRobotState")

    def get_estop_status(self) -> int:
        return self._integer_reply("get_estop_status")

    def get_servo_status(self) -> bool:
        return self._boolean_reply("getServoStatus")

    def get_motor_status(self) -> bool:
        return self._boolean_reply("getMotorStatus")

    def get_alarm_numbers(self) -> Any:
        """Return the controller's last-five alarm serials or false, unchanged."""
        return self.call("getAlarmNum")

    def get_joint_pos(self) -> list[float]:
        """Raw six joint values; the manual entry does not establish units."""
        return _finite_vector(self.call("get_joint_pos"), 6, "get_joint_pos 响应")

    def get_joint_version(self, axis: int) -> Any:
        if type(axis) is not int or not 0 <= axis <= 7:
            raise ValueError("axis 必须是 0–7；EC66 本体仍为六轴")
        return self.call("getJointVersion", {"axis": axis})

    def get_tcp_pose(self, *, coordinate_num: int | None = None,
                     tool_num: int | None = None, unit_type: int = 1) -> list[float]:
        """Controller-selected TCP pose, not the bare flange pose (PDF p21)."""
        params: dict[str, int] = {"unit_type": _unit_type(unit_type)}
        if coordinate_num is not None:
            if type(coordinate_num) is not int or not -1 <= coordinate_num <= 15:
                raise ValueError("coordinate_num 必须是 -1 或 0–15")
            if tool_num is None:
                raise ValueError("coordinate_num 不能单独使用，需同时给 tool_num")
            params["coordinate_num"] = coordinate_num
        if tool_num is not None:
            if type(tool_num) is not int or not -1 <= tool_num <= 7:
                raise ValueError("tool_num 必须是 -1 或 0–7")
            params["tool_num"] = tool_num
        return _finite_vector(self.call("get_tcp_pose", params), 6, "get_tcp_pose 响应")

    def get_base_flange_pose(self, *, unit_type: int = 1) -> list[float]:
        """Actual flange pose in base coordinates; first three mm (PDF p42)."""
        return _finite_vector(self.call("get_base_flange_pose", {"unit_type": _unit_type(unit_type)}),
                              6, "get_base_flange_pose 响应")

    def pose_mul(self, pose1: Sequence[float], pose2: Sequence[float],
                 *, unit_type: int = 1) -> list[float]:
        params = {"pose1": _finite_vector(pose1, 6, "pose1"),
                  "pose2": _finite_vector(pose2, 6, "pose2"),
                  "unit_type": _unit_type(unit_type)}
        return _finite_vector(self.call("poseMul", params), 6, "poseMul 响应")

    def pose_inv(self, pose: Sequence[float], *, unit_type: int = 1) -> list[float]:
        params = {"pose": _finite_vector(pose, 6, "pose"), "unit_type": _unit_type(unit_type)}
        return _finite_vector(self.call("poseInv", params), 6, "poseInv 响应")

    def pose_to_matrix(self, pose_mm_rad: Sequence[float]) -> list[list[float]]:
        """Ver3.22.2 method; current V3.14.2 may return -32601."""
        value = self.call("pose_to_matrix", {"pose": _finite_vector(pose_mm_rad, 6, "pose")})
        if not isinstance(value, list) or len(value) != 4:
            raise ProtocolError("pose_to_matrix 响应不是 4×4 矩阵")
        return [_finite_vector(row, 4, "matrix row") for row in value]

    def matrix_to_pose(self, matrix: Sequence[Sequence[float]]) -> list[float]:
        """Convert a 4x4 homogeneous matrix to [mm, mm, mm, rad, rad, rad]."""
        if isinstance(matrix, (str, bytes)) or not isinstance(matrix, Sequence) or len(matrix) != 4:
            raise ValueError("matrix 必须是 4×4 矩阵")
        rows = [_finite_vector(row, 4, "matrix row") for row in matrix]
        if any(abs(rows[3][i] - expected) > 1e-9 for i, expected in enumerate((0, 0, 0, 1))):
            raise ValueError("matrix 最后一行必须是 [0, 0, 0, 1]")
        return _finite_vector(self.call("matrix_to_pose", {"matrix": rows}), 6,
                              "matrix_to_pose 响应")

    def positive_kinematic(self, joints: Sequence[float], *, unit_type: int = 1) -> list[float]:
        """Forward kinematics. Check joint input units against controller version."""
        params = {"targetPos": _finite_vector(joints, 6, "joints"),
                  "unit_type": _unit_type(unit_type)}
        return _finite_vector(self.call("positiveKinematic", params), 6,
                              "positiveKinematic 响应")

    def _integer_reply(self, method: str) -> int:
        value = self.call(method)
        if type(value) is not int:
            raise ProtocolError(f"{method} 响应不是整数: {value!r}")
        return value

    def _boolean_reply(self, method: str) -> bool:
        value = self.call(method)
        if type(value) is not bool:
            raise ProtocolError(f"{method} 响应不是布尔值: {value!r}")
        return value
