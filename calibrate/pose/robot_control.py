"""Read actual EC flange pose, apply calibrated TCP offset, append mm/rad pose.

The tool origin is the calibrated tip; its axes are explicitly parallel to the
flange axes because three-pose pivot calibration determines translation only.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
PROJECT = HERE.parents[1]
SDK_DIR = PROJECT / "elite-ec-sdk"
if SDK_DIR.is_dir():
    sys.path.insert(0, str(SDK_DIR))

from elite_ec_sdk import ECClient, ECSDKError

CALIBRATION = PROJECT / "threePoint" / "output.json"
OUTPUT = HERE / "output.txt"


def finite_vector(value, count, label):
    if not isinstance(value, (list, tuple)) or len(value) != count:
        raise ValueError(f"{label} 必须包含 {count} 个数")
    if any(isinstance(v, bool) or not isinstance(v, (int, float))
           or not math.isfinite(v) for v in value):
        raise ValueError(f"{label} 包含非法数值")
    return [float(v) for v in value]


def load_tcp_offset(path=CALIBRATION):
    with Path(path).open(encoding="utf-8-sig") as stream:
        data = json.load(stream)
    if not isinstance(data, dict) or data.get("status") != "calibrated":
        raise ValueError(f"{path} 尚未完成 TCP 标定（要求 status=calibrated）")
    if (data.get("calibration_type") != "three_pose_fixed_point_tcp"
            or data.get("length_unit") != "mm"
            or data.get("input_frame") != "robot_base_flange"):
        raise ValueError("标定文件类型/单位/坐标系不匹配；要求三姿态 TCP、mm、robot_base_flange")
    if data.get("tool_orientation") is not None:
        raise ValueError("该采集程序采用工具轴平行法兰的约定，不支持文件中另行指定的工具姿态")
    return finite_vector(data.get("tcp_offset_flange_mm"), 3, "TCP 法兰偏移")


def require_stopped(robot):
    state = robot.get_robot_state()
    if type(state) is not int or state != 0:
        raise RuntimeError(f"请停稳后采集；当前机器人状态为 {state!r}，要求 0（停止）")


def read_tool_pose(robot, tcp_offset):
    """Return tool-in-base [mm,mm,mm,rad,rad,rad], using controller poseMul.

    T_base_tool = T_base_flange * Trans(tcp_offset_flange_mm).
    poseMul handles controller angle conventions without local Euler guessing.
    Reading the actual flange avoids applying the tool offset twice.
    """
    offset = finite_vector(tcp_offset, 3, "TCP 法兰偏移")
    require_stopped(robot)
    flange = finite_vector(robot.get_base_flange_pose(unit_type=1), 6, "当前法兰位姿")
    tool = finite_vector(robot.pose_mul(flange, offset + [0.0, 0.0, 0.0], unit_type=1),
                         6, "换算后的工具位姿")
    require_stopped(robot)
    return flange, tool


def append_pose(path, pose):
    pose = finite_vector(pose, 6, "待保存工具位姿")
    # Preserve the existing samples. Always save next to this script by default.
    with Path(path).open("a+", encoding="utf-8", newline="\n") as stream:
        stream.seek(0)
        previous = stream.read()
        separator = "\n" if previous and not previous.endswith("\n") else ""
        stream.write(separator + " ".join(format(v, ".15g") for v in pose) + "\n")


def parser():
    result = argparse.ArgumentParser(description="使用 EC SDK 和三点 TCP 结果读取工具基座位姿，追加到 output.txt")
    result.add_argument("--host", required=True, help="实际机械臂 IP，如 192.168.1.200")
    result.add_argument("--port", type=int, default=8055)
    result.add_argument("--calibration", type=Path, default=CALIBRATION)
    result.add_argument("--output", type=Path, default=OUTPUT)
    result.add_argument("--expected-version", help="可选，严格核对控制器版本，如 V3.14.2")
    return result


def main(argv=None):
    args = parser().parse_args(argv)
    # Invalid/pending calibration must fail before connecting or opening output.
    offset = load_tcp_offset(args.calibration)
    with ECClient(args.host, port=args.port, timeout=3) as robot:
        version = robot.get_soft_version()
        if args.expected_version and version.casefold() != args.expected_version.casefold():
            raise RuntimeError(f"控制器版本不符：预期 {args.expected_version}，实机 {version}")
        print(f"控制器版本：{version}")
        print(f"TCP 法兰偏移 [mm]：{offset}")
        flange, tool = read_tool_pose(robot, offset)
    append_pose(args.output, tool)
    print(f"当前法兰位姿 [mm/rad]：{flange}")
    print(f"工具尖点基座位姿 [mm/rad]（坐标轴平行法兰）：{tool}")
    print(f"已追加保存：{args.output.resolve()}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("采集已中断。", file=sys.stderr)
        raise SystemExit(130)
    except (ECSDKError, OSError, ValueError, RuntimeError) as exc:
        print(f"采集失败：{exc}", file=sys.stderr)
        raise SystemExit(1)
