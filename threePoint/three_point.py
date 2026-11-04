"""Four-pose fixed-point TCP workflow: fit three poses, validate on the fourth.

For each pose: R_base_flange @ tcp_in_flange + p_base_flange = fixed_point_base.
Three poses determine TCP translation only, not the orientation of tool axes.
The held-out fourth pose never changes the fitted TCP or fixed reference point.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import time
import uuid

import numpy as np

HERE = Path(__file__).resolve().parent
OUTPUT = HERE / "output.json"


class RunLog:
    """Append one UTF-8 JSON object per line; keep runs and raw data identifiable."""

    def __init__(self, path):
        self.path = Path(path)
        self.run_id = uuid.uuid4().hex

    def emit(self, event, data):
        entry = {"time": datetime.now(timezone.utc).isoformat(),
                 "run_id": self.run_id, "event": event, "data": data}
        line = json.dumps(entry, ensure_ascii=False, allow_nan=False, default=str) + "\n"
        with self.path.open("a", encoding="utf-8", newline="\n") as stream:
            stream.write(line)
            stream.flush()
            os.fsync(stream.fileno())


def result_only(result):
    """Public calibration payload; observations, diagnostics and source stay in log."""
    return {
        "schema_version": 3,
        "status": result["status"],
        "calibration_type": result["calibration_type"],
        "length_unit": result["length_unit"],
        "input_frame": result["input_frame"],
        "tcp_offset_flange_mm": result["tcp_offset_flange_mm"],
        "tool_orientation": result.get("tool_orientation"),
    }


def finite_array(value, shape, label):
    raw = np.asarray(value)
    if raw.shape != shape or raw.dtype.kind not in "iuf":
        raise ValueError(f"{label} 必须是形状 {shape} 的数值数组")
    array = raw.astype(float)
    if not np.isfinite(array).all() or np.max(np.abs(array)) > 1e9:
        raise ValueError(f"{label} 包含非有限数值或超出 ±1e9")
    return array


def validate_transform(value):
    matrix = finite_array(value, (4, 4), "T_base_flange")
    rotation = matrix[:3, :3]
    if not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8, rtol=0):
        raise ValueError("齐次矩阵最后一行须为 [0,0,0,1]")
    if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6, rtol=0)
            or abs(np.linalg.det(rotation) - 1) > 1e-6):
        raise ValueError("旋转矩阵须正交且行列式为 +1")
    return matrix


def pose_to_matrix(pose, angle_unit):
    """Explicit mathematical convention R=Rz(rz) Ry(ry) Rx(rx).

    This does not claim that an unverified controller uses that convention.
    """
    pose = finite_array(pose, (6,), "法兰位姿")
    if angle_unit not in ("deg", "rad"):
        raise ValueError("angle_unit 必须是 deg 或 rad")
    angles = np.deg2rad(pose[3:]) if angle_unit == "deg" else pose[3:]
    cx, cy, cz = np.cos(angles)
    sx, sy, sz = np.sin(angles)
    rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    matrix = np.eye(4)
    matrix[:3, :3] = rz @ ry @ rx
    matrix[:3, 3] = pose[:3]
    return matrix


def positive(value, label):
    if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
        raise ValueError(f"{label} 必须是正有限数")
    return value


def calibrate(transforms, *, max_residual_mm=1.0, max_condition=100.0,
              min_singular_value=0.05):
    """Centered least squares; SVD checks observability before solving.

    Eliminating the unknown fixed point gives (Ri-Rmean)t = pmean-pi.
    All three poses are used symmetrically, avoiding a privileged reference pose.
    """
    for name, value in (("max_residual_mm", max_residual_mm),
                        ("max_condition", max_condition),
                        ("min_singular_value", min_singular_value)):
        positive(value, name)
    if len(transforms) != 3:
        raise ValueError("TCP 三姿态定点标定必须提供恰好 3 个法兰位姿")
    matrices = np.stack([validate_transform(m) for m in transforms])
    rotations, positions = matrices[:, :3, :3], matrices[:, :3, 3]
    a = (rotations - rotations.mean(axis=0)).reshape(9, 3)
    b = (positions.mean(axis=0) - positions).reshape(9)
    singular = np.linalg.svd(a, compute_uv=False)
    if singular[-1] < min_singular_value:
        raise ValueError(f"姿态退化/变化不足，最小奇异值 {singular[-1]:.6g} < "
                         f"{min_singular_value}；请绕不同轴改变姿态并保持尖点接触同一点")
    condition = float(singular[0] / singular[-1])
    if condition > max_condition:
        raise ValueError(f"姿态条件数过大 {condition:.3f} > {max_condition}，请重新选择姿态")
    tcp, _, rank, _ = np.linalg.lstsq(a, b, rcond=None)
    if rank != 3:
        raise ValueError("姿态不能唯一确定 TCP 偏移")
    tips = np.einsum("nij,j->ni", rotations, tcp) + positions
    fixed = tips.mean(axis=0)
    errors = tips - fixed
    residuals = np.linalg.norm(errors, axis=1)
    max_error = float(residuals.max())
    if max_error > max_residual_mm:
        raise ValueError(f"定点接触残差过大 {max_error:.4f} mm > {max_residual_mm} mm；"
                         "请检查是否接触同一点、法兰坐标及姿态约定")
    return {
        "schema_version": 1,
        "status": "calibrated",
        "calibration_type": "three_pose_fixed_point_tcp",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "length_unit": "mm",
        "input_frame": "robot_base_flange",
        "tcp_offset_flange_mm": tcp.tolist(),
        "fixed_point_base_mm": fixed.tolist(),
        "tool_orientation": None,
        "orientation_note": "三姿态定点法仅求 TCP 原点 XYZ，不能确定工具坐标轴姿态。",
        "T_base_flange": matrices.tolist(),
        "predicted_tip_points_base_mm": tips.tolist(),
        "residual_vectors_mm": errors.tolist(),
        "residual_norms_mm": residuals.tolist(),
        "quality": {
            "rms_residual_mm": float(np.sqrt(np.mean(residuals**2))),
            "max_residual_mm": max_error,
            "singular_values": singular.tolist(),
            "condition_number": condition,
            "rank": int(rank),
            "thresholds": {"max_residual_mm": max_residual_mm,
                           "max_condition": max_condition,
                           "min_singular_value": min_singular_value},
            "note": "残差和条件数仅检验这三次采样的一致性；精度须用独立姿态验证。",
        },
        "equation": "R_base_flange[i] @ tcp_offset_flange_mm + p_base_flange[i] = fixed_point_base_mm",
    }


def save_json(path, data):
    """Atomically replace a JSON file, preserving old data on failed writes."""
    path = Path(path)
    payload = json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    temp_path = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                         prefix=path.name + ".", suffix=".tmp",
                                         delete=False) as stream:
            temp_path = Path(stream.name)
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temp_path, path)
    finally:
        if temp_path is not None and temp_path.exists():
            temp_path.unlink()


def calibrate_with_validation(transforms, *, max_validation_error_mm=1.0,
                              max_residual_mm=1.0, max_condition=100.0,
                              min_singular_value=0.05):
    """Fit only poses 1-3, compare pose 4's predicted tip against fitted contact.

    A geometrically valid but failed validation is returned as validation_failed
    so its error can be saved without exposing it as a usable calibration.
    """
    positive(max_validation_error_mm, "max_validation_error_mm")
    if len(transforms) != 4:
        raise ValueError("必须提供 4 个法兰位姿：前三个标定，第四个验证")
    matrices = [validate_transform(m) for m in transforms]
    result = calibrate(matrices[:3], max_residual_mm=max_residual_mm,
                       max_condition=max_condition, min_singular_value=min_singular_value)
    tcp = np.array(result["tcp_offset_flange_mm"])
    fixed = np.array(result["fixed_point_base_mm"])
    fourth = matrices[3]
    predicted = fourth[:3, :3] @ tcp + fourth[:3, 3]
    error = predicted - fixed
    error_mm = float(np.linalg.norm(error))
    passed = error_mm <= max_validation_error_mm
    result.update({
        "schema_version": 2,
        "status": "calibrated" if passed else "validation_failed",
        "sample_roles": ["calibration", "calibration", "calibration", "validation"],
        "T_base_flange": [m.tolist() for m in matrices],
        "validation": {
            "pose_number": 4,
            "used_for_calibration": False,
            "predicted_tip_base_mm": predicted.tolist(),
            "reference_fixed_point_base_mm": fixed.tolist(),
            "error_vector_mm": error.tolist(),
            "error_norm_mm": error_mm,
            "max_error_mm": max_validation_error_mm,
            "passed": passed,
            "min_rotation_difference_from_calibration_deg": min(
                rotation_distance_deg(fourth[:3, :3], m[:3, :3]) for m in matrices[:3]),
            "note": "第四姿态尖点须接触同一固定点；验证不参与重新求解。单点验证不等于完整精度评估。",
        },
    })
    return result


def load_transforms(path, *, exclude_run_id=None):
    path = Path(path)
    with path.open(encoding="utf-8-sig") as stream:
        if path.suffix.lower() in (".log", ".jsonl"):
            entries = [json.loads(line) for line in stream if line.strip()]
            if not entries or any(not isinstance(e, dict) for e in entries):
                raise ValueError("日志为空或格式错误")
            entries = [e for e in entries if e.get("run_id") != exclude_run_id]
            if not entries:
                raise ValueError("日志中没有已有的采样记录")
            latest_run = entries[-1].get("run_id")
            snapshots = [e["data"] for e in entries if e.get("run_id") == latest_run
                         and e.get("event") == "captured_poses"]
            if not snapshots or snapshots[-1].get("status") != "complete":
                raise ValueError("日志最后一次运行没有完整的四姿态记录，请选择完整采样日志")
            data = snapshots[-1]
        else:
            data = json.load(stream)
    if (not isinstance(data, dict) or data.get("length_unit") != "mm"
            or data.get("input_frame") != "robot_base_flange"):
        raise ValueError("点文件须声明 length_unit=mm、input_frame=robot_base_flange")
    matrices = data.get("T_base_flange")
    if not isinstance(matrices, list) or len(matrices) != 4:
        raise ValueError("T_base_flange 须包含 4 个 4×4 法兰到基座变换：前三个标定，第四个验证")
    return [validate_transform(m) for m in matrices]


def require_stopped(robot):
    state = robot.get_robot_state()
    if type(state) is not int or state != 0:
        raise RuntimeError(f"采样要求停止状态 0，实际状态 {state!r}；本次不采样")


def rotation_distance_deg(a, b):
    cosine = (np.trace(a.T @ b) - 1) / 2
    return float(np.degrees(np.arccos(np.clip(cosine, -1, 1))))


def matrix_from_pose_mul(robot, pose):
    """Build R using the controller's documented poseMul (no Euler assumptions).

    Translating along local X/Y/Z yields the columns of R. These are arbitrary
    mathematical poses passed to a coordinate operation, never motion targets.
    Use zero origin and a 100 mm basis to avoid subtracting large base positions.
    """
    pose = finite_array(pose, (6,), "基座法兰位姿")
    orientation = [0.0, 0.0, 0.0, *pose[3:].tolist()]
    matrix = np.eye(4)
    step_mm = 100.0
    for axis in range(3):
        local_translation = [0.0] * 6
        local_translation[axis] = step_mm
        transformed = finite_array(robot.pose_mul(orientation, local_translation, unit_type=1),
                                   (6,), "poseMul 基向量变换")
        matrix[:3, axis] = transformed[:3] / step_mm
    matrix[:3, 3] = pose[:3]
    return validate_transform(matrix)


def read_stationary_pose(robot, *, samples=5, stability_mm=0.2,
                         stability_deg=0.1, rotation_source="controller", interval=0.1):
    if type(samples) is not int or not 2 <= samples <= 100:
        raise ValueError("samples 须在 2–100 内")
    positive(stability_mm, "stability_mm")
    positive(stability_deg, "stability_deg")
    if rotation_source not in ("controller", "pose-mul", "matrix", "rpy-xyz"):
        raise ValueError("未知姿态转换方式")
    matrices, poses = [], []
    for index in range(samples):
        require_stopped(robot)
        pose = finite_array(robot.get_base_flange_pose(unit_type=1), (6,), "基座法兰位姿")
        if rotation_source in ("controller", "pose-mul"):
            matrix = matrix_from_pose_mul(robot, pose)
        elif rotation_source == "matrix":
            matrix = validate_transform(robot.pose_to_matrix(pose.tolist()))
        else:
            matrix = pose_to_matrix(pose, "rad")
        if not np.allclose(matrix[:3, 3], pose[:3], atol=1e-5, rtol=0):
            raise RuntimeError("位姿转矩阵返回的位置与原始法兰位置不一致")
        require_stopped(robot)
        matrices.append(matrix)
        poses.append(pose.tolist())
        if index + 1 < samples:
            time.sleep(interval)
    spread_mm = max(float(np.linalg.norm(a[:3, 3] - b[:3, 3]))
                    for a in matrices for b in matrices)
    spread_deg = max(rotation_distance_deg(a[:3, :3], b[:3, :3])
                     for a in matrices for b in matrices)
    if spread_mm > stability_mm or spread_deg > stability_deg:
        raise RuntimeError(f"法兰采样不稳定：{spread_mm:.4f} mm / {spread_deg:.4f} deg")
    # Select an actual stationary observation, rather than averaging rotations.
    return matrices[len(matrices)//2], {
        "rotation_method": {"controller": "poseMul_basis", "pose-mul": "poseMul_basis",
                            "matrix": "pose_to_matrix", "rpy-xyz": "Rz_Ry_Rx"}[rotation_source],
        "flange_poses_mm_rad": poses,
        "T_base_flange_samples": [m.tolist() for m in matrices],
        "selected_sample_index": len(matrices)//2,
        "spread_mm": spread_mm, "spread_deg": spread_deg,
        "captured_at": datetime.now(timezone.utc).isoformat(),
    }


def capture_transforms(args, log):
    sdk_dir = HERE.parent / "elite-ec-sdk"
    if sdk_dir.is_dir():
        sys.path.insert(0, str(sdk_dir))
    from elite_ec_sdk import ECClient

    matrices, records = [], []
    first_version = None
    source = {"mode": "robot_read_only", "host": args.host, "port": args.port,
              "rotation_source": args.rotation_source,
              "samples_per_pose": args.samples,
              "stability_mm": args.stability_mm, "stability_deg": args.stability_deg}
    print("依次使用四个姿态触碰同一个固定点：前三个标定，第四个独立验证。")
    print("只读基座下法兰位姿；移动/姿态调整由示教器完成。")
    for index in range(4):
        role = "标定" if index < 3 else "独立验证（请选不同于前三次的姿态）"
        input(f"姿态 {index+1}/4 [{role}]：尖点接触同一固定点并停稳后按回车读取（Ctrl+C 取消）：")
        with ECClient(args.host, port=args.port, timeout=3) as robot:
            version = robot.get_soft_version()
            log.emit("controller_version", {"pose_number": index+1, "version": version})
            print(f"控制器软件版本：{version}")
            if first_version is None:
                first_version = version
            if version != first_version:
                raise RuntimeError("四次采样期间控制器版本发生变化，请重新采集")
            if args.expected_version and version.casefold() != args.expected_version.casefold():
                raise RuntimeError(f"版本不符：预期 {args.expected_version}，实机 {version}")
            matrix, record = read_stationary_pose(
                robot, samples=args.samples, stability_mm=args.stability_mm,
                stability_deg=args.stability_deg, rotation_source=args.rotation_source)
        matrices.append(matrix)
        records.append(record)
        source["controller_version"] = first_version
        log.emit("captured_poses", {
            "status": "complete" if len(matrices) == 4 else "incomplete",
            "length_unit": "mm", "input_frame": "robot_base_flange",
            "sample_roles": ["calibration" if i < 3 else "validation" for i in range(len(matrices))],
            "T_base_flange": [m.tolist() for m in matrices],
            "source": source, "capture_records": records,
        })
        print(f"已记录姿态 {index+1}，法兰 XYZ: {matrix[:3, 3].tolist()}")
    source["capture_records"] = records
    return matrices, source


def parser():
    result = argparse.ArgumentParser(description="工具 TCP 四姿态采集：结果 → output.json；位姿与过程 → calibration.log")
    modes = result.add_subparsers(dest="mode", required=True)
    capture = modes.add_parser("capture", help="四次只读采集：前三个标定，第四个验证")
    capture.add_argument("--host", required=True, help="已核实的机械臂 IP")
    capture.add_argument("--port", type=int, default=8055)
    capture.add_argument("--expected-version", help="可选，严格核对软件版本，如 V3.14.2")
    capture.add_argument("--samples", type=int, default=5)
    capture.add_argument("--stability-mm", type=float, default=0.2)
    capture.add_argument("--stability-deg", type=float, default=0.1)
    capture.add_argument("--rotation-source", choices=("controller", "pose-mul", "matrix", "rpy-xyz"),
                         default="controller",
                         help="默认 controller 使用 poseMul 构造矩阵，兼容旧固件；matrix 要求支持 pose_to_matrix；"
                              "rpy-xyz 仅适用于已核实 R=Rz Ry Rx 的实机")
    manual = modes.add_parser("manual", help="交互输入四组基座法兰六维位姿，第四组验证，离线")
    manual.add_argument("--angle-unit", choices=("deg", "rad"), required=True)
    manual.add_argument("--rotation-convention", choices=("rpy-xyz",), required=True,
                        help="必须确认输入采用 R=Rz(rz) Ry(ry) Rx(rx)")
    file_mode = modes.add_parser("file", help="读取 JSON 或日志中的四组法兰位姿，第四组验证，离线")
    file_mode.add_argument("poses_file", type=Path)
    for mode in (capture, manual, file_mode):
        mode.add_argument("--log-file", type=Path, help="日志路径，默认脚本目录下 calibration.log（追加）")
        mode.add_argument("--max-residual-mm", type=float, default=10)
        mode.add_argument("--max-condition", type=float, default=100.0)
        mode.add_argument("--min-singular-value", type=float, default=0.05)
        mode.add_argument("--max-validation-error-mm", type=float, default=10,
                          help="第四姿态允许的最大位置误差，默认 %(default)s mm")
    return result


def run_calibration(args, log):
    for label in ("max_residual_mm", "max_condition", "min_singular_value", "max_validation_error_mm"):
        positive(getattr(args, label), label)
    if args.mode == "capture":
        if not 1 <= args.port <= 65535 or not 2 <= args.samples <= 100:
            raise ValueError("port 须在 1–65535 内，samples 须在 2–100 内")
        positive(args.stability_mm, "stability-mm")
        positive(args.stability_deg, "stability-deg")
        matrices, source = capture_transforms(args, log)
    elif args.mode == "manual":
        matrices, poses = [], []
        print(f"输入基座下法兰 x y z rx ry rz，长度 mm，角度 {args.angle_unit}。")
        for index in range(4):
            role = "标定" if index < 3 else "独立验证"
            raw = input(f"姿态 {index+1}/4 [{role}]: ").replace(",", " ").replace("，", " ")
            pose = [float(v) for v in raw.split()]
            matrices.append(pose_to_matrix(pose, args.angle_unit))
            poses.append(pose)
            log.emit("manual_pose", {"pose_number": index+1, "role": role,
                                      "flange_pose": pose, "angle_unit": args.angle_unit,
                                      "T_base_flange": matrices[-1].tolist()})
        source = {"mode": "manual", "angle_unit": args.angle_unit,
                  "rotation_convention": "R=Rz(rz) Ry(ry) Rx(rx)", "flange_poses": poses}
    else:
        matrices = load_transforms(args.poses_file, exclude_run_id=log.run_id)
        source = {"mode": "file", "path": str(args.poses_file.resolve())}
    log.emit("captured_poses", {
        "status": "complete", "length_unit": "mm", "input_frame": "robot_base_flange",
        "sample_roles": ["calibration", "calibration", "calibration", "validation"],
        "T_base_flange": [m.tolist() for m in matrices], "source": source,
    })
    result = calibrate_with_validation(
        matrices, max_residual_mm=args.max_residual_mm, max_condition=args.max_condition,
        min_singular_value=args.min_singular_value,
        max_validation_error_mm=args.max_validation_error_mm)
    result["source"] = source
    log.emit("calibration_details", result)
    save_json(OUTPUT, result_only(result))
    log.emit("result_saved", {"path": str(OUTPUT), "status": result["status"]})
    print(f"TCP 偏移（法兰坐标系，mm）：{result['tcp_offset_flange_mm']}")
    print(f"最大定点残差：{result['quality']['max_residual_mm']:.6f} mm")
    validation = result["validation"]
    print(f"第四姿态验证误差：{validation['error_norm_mm']:.6f} mm，"
          f"阈值：{validation['max_error_mm']:.6f} mm")
    print(f"已保存：{OUTPUT}")
    print(f"位姿及过程日志：{log.path}")
    if not validation["passed"]:
        print("验证未通过，结果标记为 validation_failed，不可用于后续位姿采集。", file=sys.stderr)
        return 1
    print("第四姿态验证通过。")
    return 0


def main(argv=None):
    args = parser().parse_args(argv)
    log = RunLog(args.log_file or HERE / "calibration.log")
    log.emit("run_started", {"arguments": vars(args)})
    try:
        code = run_calibration(args, log)
    except (Exception, KeyboardInterrupt) as exc:
        log.emit("run_failed", {"error_type": type(exc).__name__, "error": str(exc)})
        raise
    log.emit("run_finished", {"exit_code": code})
    return code


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (KeyboardInterrupt, EOFError):
        print("已取消；未写入新的标定结果。", file=sys.stderr)
        raise SystemExit(130)
    except (ValueError, RuntimeError, OSError, ImportError) as exc:
        print(f"标定未完成：{exc}；未写入新的标定结果。", file=sys.stderr)
        raise SystemExit(1)
