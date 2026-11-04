"""Guarded EC66 6D-pose motion trial using the adjacent elite-ec-sdk package.

Importing this module, running --help, or using --offline never contacts a robot.
Execution requires --execute and an interactive confirmation. No servo enable,
alarm clear, coordinate switch, or automatic motion retry is performed.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import re
import sys
import time
from typing import Any, Sequence


SDK_DIR = Path(__file__).resolve().parents[1] / "elite-ec-sdk"
if SDK_DIR.is_dir():
    sys.path.insert(0, str(SDK_DIR))

from elite_ec_sdk import ECClient, ECSDKError, RPCError  # noqa: E402


def pose6(values: Sequence[float]) -> list[float]:
    """Validate a TCP pose: xyz mm, Rx/Ry/Rz rad (not joint angles)."""
    if len(values) != 6:
        raise ValueError("目标位姿必须恰好包含 x y z rx ry rz 六个数")
    pose = [float(v) for v in values]
    if not all(math.isfinite(v) for v in pose):
        raise ValueError("目标位姿不能包含 NaN 或无穷大")
    if any(abs(v) > 1e9 for v in pose[:3]):
        raise ValueError("位置超出手册 target_pose 范围")
    if any(abs(v) > math.pi for v in pose[3:]):
        raise ValueError("姿态角必须为弧度且位于 [-π, π]")
    return pose


def translation_delta_mm(a: Sequence[float], b: Sequence[float]) -> float:
    return math.dist(a[:3], b[:3])


def angle_component_delta_rad(a: Sequence[float], b: Sequence[float]) -> float:
    # Conservative component-wise bound, not a physical SO(3) angle.
    return max(abs(x - y) for x, y in zip(a[3:], b[3:]))


def controller_version(text: str) -> str:
    match = re.search(r"\d+\.\d+\.\d+", text)
    if not match:
        raise RuntimeError(f"无法识别控制器软件版本: {text!r}")
    return match.group()


def require_int(value: Any, label: str, low: int, high: int) -> int:
    if type(value) is not int or not low <= value <= high:
        raise RuntimeError(f"{label} 响应异常: {value!r}")
    return value


def preflight(robot: ECClient, expected_version: str | None,
              expected_tool: int | None, *, require_motion_ready: bool = True) -> list[float]:
    """Read robot state; never changes controller configuration."""
    actual_version = robot.get_soft_version()
    version = controller_version(actual_version)
    print(f"软件版本: {actual_version}")
    if expected_version is not None and version != controller_version(expected_version):
        raise RuntimeError(f"版本不符：预期 {expected_version}，实机 {actual_version}")

    mode = robot.get_robot_mode()
    state = robot.get_robot_state()
    estop = robot.get_estop_status()
    motor = robot.get_motor_status()
    servo = robot.get_servo_status()
    # These two are documented read-only calls, but not on the shared SDK's
    # small vetted allow-list. The SDK therefore requires control=True.
    coord = require_int(robot.call("getCurrentCoord", control=True), "当前坐标系", 0, 4)
    tool = require_int(robot.call("getAutoRunToolNumber", control=True), "当前工具号", 0, 7)
    current = pose6(robot.get_tcp_pose(coordinate_num=-1, tool_num=tool, unit_type=1))
    print(f"模式={mode} 状态={state} 急停={estop} 同步={motor} 伺服={servo}")
    coord_names = {0: "关节", 1: "基座", 2: "工具", 3: "用户", 4: "圆柱"}
    print(f"当前坐标模式={coord} ({coord_names[coord]}；1=基座), 当前工具号={tool}")
    print(f"当前基座下 TCP 位姿 [mm, rad]: {current}")

    if expected_tool is not None and tool != expected_tool:
        raise RuntimeError(f"工具号不符：预期 {expected_tool}，实机 {tool}")
    if not require_motion_ready:
        return current
    if mode != 2:
        raise RuntimeError("机械臂不是 REMOTE 模式；程序不会切换模式")
    if state != 0 or estop != 0:
        raise RuntimeError("机械臂未处于停止且非急停状态")
    if motor is not True:
        raise RuntimeError("伺服尚未同步；请在示教器检查同步状态，程序不会自动同步")
    if servo is not True:
        raise RuntimeError("伺服已同步但尚未使能；请核对现场条件后在示教器使能，程序不会自动使能")
    if coord != 1:
        raise RuntimeError("当前坐标模式不是基座；程序不会切换坐标系")
    return current


def build_move_params(target: Sequence[float], current: Sequence[float],
                      speed: float, acc: int, dec: int,
                      max_step_mm: float, max_angle_rad: float) -> dict[str, Any]:
    target, current = pose6(target), pose6(current)
    if not 0.01 <= speed <= 100:
        raise ValueError("moveByJoint speed 必须在 [0.01, 100] 内")
    if type(acc) is not int or type(dec) is not int or not (1 <= acc <= 100 and 1 <= dec <= 100):
        raise ValueError("acc 和 dec 必须是 [1, 100] 的整数百分比")
    if not math.isfinite(max_step_mm) or max_step_mm <= 0:
        raise ValueError("max_step_mm 必须大于零")
    if not math.isfinite(max_angle_rad) or not 0 < max_angle_rad <= math.pi:
        raise ValueError("max_angle_rad 必须在 (0, π] 内")
    travel = translation_delta_mm(current, target)
    rotation = angle_component_delta_rad(current, target)
    if travel > max_step_mm:
        raise RuntimeError(f"目标平移 {travel:.2f} mm 超过本次限值 {max_step_mm:.2f} mm")
    if rotation > max_angle_rad:
        raise RuntimeError(f"姿态分量变化 {rotation:.3f} rad 超过本次限值 {max_angle_rad:.3f} rad")
    # Local planning data, never sent directly to moveByJoint.
    return {"target_tcp_pose": target, "speed": speed, "acc": acc, "dec": dec}


def joint6(values: Sequence[float]) -> list[float]:
    if not isinstance(values, (list, tuple)) or len(values) != 6:
        raise RuntimeError("控制器未返回有效的六关节解，取消运动")
    if any(isinstance(v, bool) or not isinstance(v, (int, float))
           or not math.isfinite(v) or abs(v) > 360 for v in values):
        raise RuntimeError("关节解包含非法数值或超出接口范围 [-360,360] 度，取消运动")
    return [float(v) for v in values]


def resolve_joint_move(robot: ECClient, plan: dict[str, Any],
                       current_tcp: Sequence[float]) -> dict[str, Any]:
    """Read-only IK/FK validation, then build legacy-compatible joint payload.

    V3.14.2 rejects target_pose in moveByJoint. Cartesian targetPose belongs to
    inverseKinematic; moveByJoint.targetPos contains JOINT angles in degrees.
    Check FK against actual TCP first to catch joint units/tool-frame mismatch.
    """
    reference = joint6(robot.get_joint_pos())
    actual_fk = pose6(robot.positive_kinematic(reference, unit_type=1))
    if not at_target(actual_fk, current_tcp, 1.0, 0.01):
        raise RuntimeError("当前关节正解与基座 TCP 反馈不一致；请核实关节单位和正解工具/坐标定义，未发送运动")
    target = pose6(plan["target_tcp_pose"])
    joints = joint6(robot.call("inverseKinematic", {
        "targetPose": target, "referencePos": reference, "unit_type": 1,
    }))
    solved_fk = pose6(robot.positive_kinematic(joints, unit_type=1))
    if not at_target(solved_fk, target, 1.0, 0.01):
        raise RuntimeError("目标逆解的正解复核失败；请核实逆解单位和工具/坐标定义，未发送运动")
    return {"targetPos": joints, "speed": plan["speed"],
            "acc": plan["acc"], "dec": plan["dec"]}


def at_target(current: Sequence[float], target: Sequence[float],
              position_tolerance_mm: float, angle_tolerance_rad: float) -> bool:
    return (translation_delta_mm(current, target) <= position_tolerance_mm
            and angle_component_delta_rad(current, target) <= angle_tolerance_rad)


def execute_and_wait(robot: ECClient, params: dict[str, Any], tool: int,
                     timeout_s: float, position_tolerance_mm: float,
                     angle_tolerance_rad: float, *, target_tcp: Sequence[float]) -> list[float]:
    """Issue once, then verify stopped state and pose; never replay on failure."""
    if "target_pose" in params or "target_tcp_pose" in params or "targetPos" not in params:
        raise ValueError("moveByJoint 只接受已解算的 targetPos 关节目标，禁止直接发送 TCP 位姿")
    joint6(params["targetPos"])
    target = pose6(target_tcp)
    accepted = robot.call("moveByJoint", params, control=True)
    if accepted is not True:
        raise RuntimeError(f"moveByJoint 未返回 true: {accepted!r}")
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        time.sleep(0.3)
        state = robot.get_robot_state()
        if state in (2, 4, 5):
            raise RuntimeError(f"运动中出现急停/报警/碰撞，状态={state}；请在示教器检查")
        actual = pose6(robot.get_tcp_pose(coordinate_num=-1, tool_num=tool, unit_type=1))
        if state == 0 and at_target(actual, target, position_tolerance_mm, angle_tolerance_rad):
            return actual
    # Best effort on the existing connection only; this is not a safety stop.
    if robot.connected:
        try:
            robot.call("stop", control=True)
        except (ECSDKError, OSError, TimeoutError) as exc:
            print(f"警告：stop 命令未确认成功：{exc}", file=sys.stderr)
    raise TimeoutError("等待到位超时；已尝试普通 stop，不能据此确认停机，请立即检查示教器")


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="EC66 指定位姿运动试验；默认只读预检，不发运动命令")
    parser.add_argument("--host", required=True, help="已核实的机械臂 IP，例如 192.168.1.200")
    parser.add_argument("--port", type=int, default=8055, help="EC JSON-RPC 端口，默认 8055")
    parser.add_argument("--pose", nargs=6, type=float,
                        metavar=("X", "Y", "Z", "RX", "RY", "RZ"),
                        help="基座下当前工具 TCP 目标位姿，前三项 mm，后三项 rad")
    parser.add_argument("--expected-version", default="V3.14.2", help="执行前必须填写查询到的软件版本，如 V3.14.2")
    parser.add_argument("--expected-tool", type=int, choices=range(8), metavar="0..7",
                        help="执行前必须填写经核对的远程模式工具号")
    parser.add_argument("--speed", type=float, default=5.0, help="moveByJoint speed，默认 5")
    parser.add_argument("--acc", type=int, default=10, help="加速度百分比，默认 10")
    parser.add_argument("--dec", type=int, default=10, help="减速度百分比，默认 10")
    parser.add_argument("--max-step-mm", type=float, default=500.0, help="单次平移上限，默认 %(default)s mm")
    parser.add_argument("--max-angle-rad", type=float, default=2,
                        help="姿态各分量变化上限，默认 %(default)s rad")
    parser.add_argument("--timeout", type=float, default=60.0, help="到位等待上限，默认 60 s")
    parser.add_argument("--show-current", action="store_true", help="只读取当前位姿和状态，不需要 --pose")
    parser.add_argument("--offline", action="store_true", help="仅验证参数，不连接机械臂")
    parser.add_argument("--execute", action="store_true", help="允许发送一次运动指令")
    parser.add_argument("--confirm-base-tcp-rad", action="store_true",
                        help="确认目标是基座下当前工具 TCP 位姿，单位 mm/rad，不是法兰或枪尖")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = make_parser().parse_args(argv)
    if args.pose is None and not args.show_current:
        raise ValueError("请提供 --pose x y z rx ry rz，或使用 --show-current")
    if args.show_current and (args.offline or args.execute):
        raise ValueError("--show-current 不能与 --offline 或 --execute 同用")
    target = pose6(args.pose) if args.pose is not None else None
    if not 0 < args.timeout <= 600:
        raise ValueError("timeout 必须在 (0, 600] 秒内")
    if args.offline and args.execute:
        raise ValueError("--offline 和 --execute 不能同时使用")
    if args.offline:
        assert target is not None
        build_move_params(target, target, args.speed, args.acc, args.dec,
                          args.max_step_mm, args.max_angle_rad)
        print(f"离线参数检查通过；目标 [mm, rad]: {target}；未连接机械臂")
        return 0
    if args.execute:
        if not args.expected_version or args.expected_tool is None:
            raise ValueError("执行需提供 --expected-version 和 --expected-tool")
        if not args.confirm_base_tcp_rad:
            raise ValueError("执行需确认 --confirm-base-tcp-rad；法兰/枪尖目标不能直接输入")
    with ECClient(args.host, port=args.port, timeout=3, allow_control=True) as robot:
        current = preflight(robot, args.expected_version, args.expected_tool,
                            require_motion_ready=not args.show_current)
        if args.show_current:
            print("只读状态查询完成；未发送运动指令")
            return 0
        assert target is not None
        plan = build_move_params(target, current, args.speed, args.acc, args.dec,
                                 args.max_step_mm, args.max_angle_rad)
        if at_target(current, target, 2.0, 0.05):
            print("已在目标容差内，不发送运动指令")
            return 0
        params = resolve_joint_move(robot, plan, current)
        print(f"目标 TCP [mm, rad]: {target}")
        print(f"逆解及正解复核通过，拟发送关节目标 [deg]: moveByJoint {params}")
        if not args.execute:
            print("只读预检完成；未发送运动指令。实机执行需额外添加 --execute 等确认参数")
            return 0
        print("确认人员已离开工作空间、路径无障碍、急停可用。输入 MOVE 才执行一次运动。")
        try:
            confirmation = input("确认输入: ").strip()
        except EOFError:
            confirmation = ""
        if confirmation != "MOVE":
            print("已取消；未发送运动指令")
            return 0
        latest = preflight(robot, args.expected_version, args.expected_tool)
        if translation_delta_mm(current, latest) > 1.0 or angle_component_delta_rad(current, latest) > 0.01:
            raise RuntimeError("确认期间当前位姿发生变化，已取消；请重新预检")
        latest_plan = build_move_params(target, latest, args.speed, args.acc, args.dec,
                                        args.max_step_mm, args.max_angle_rad)
        latest_params = resolve_joint_move(robot, latest_plan, latest)
        if max(abs(a - b) for a, b in zip(params["targetPos"], latest_params["targetPos"])) > 0.1:
            raise RuntimeError("确认期间目标关节解发生变化，取消运动；请重新预检")
        actual = execute_and_wait(robot, latest_params, args.expected_tool, args.timeout,
                                  2.0, 0.05, target_tcp=target)
        print(f"机械臂已停止且目标位姿在容差内: {actual}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("程序中断不等于机械臂停机！请立即通过示教器或现场安全装置确认状态。", file=sys.stderr)
        raise SystemExit(130)
    except (ECSDKError, OSError, TimeoutError, ValueError, RuntimeError) as exc:
        print(f"未确认到位：{exc}", file=sys.stderr)
        raise SystemExit(1)
