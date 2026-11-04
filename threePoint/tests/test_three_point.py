import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "threePoint"))
sys.path.insert(0, str(ROOT / "elite-ec-sdk"))
import three_point as tp


TCP = np.array([35.0, -12.0, 160.0])
FIXED = np.array([500.0, 100.0, 220.0])
ROTATIONS = [
    np.eye(3),
    np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]]),
    np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]]),
]


def observations(rotations=None):
    result = []
    for rotation in ROTATIONS if rotations is None else rotations:
        matrix = np.eye(4)
        matrix[:3, :3] = rotation
        matrix[:3, 3] = FIXED - rotation @ TCP
        result.append(matrix)
    return result


def four_observations():
    rz90 = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
    return observations(ROTATIONS + [rz90])


class FakeRobot:
    def __init__(self, matrix, *, state=0, version="V3.14.2"):
        self.matrix = matrix
        self.state = state
        self.version = version
        self.calls = []

    def __enter__(self):
        return self

    def __exit__(self, *args):
        pass

    def get_soft_version(self):
        return self.version

    def get_robot_state(self):
        self.calls.append("state")
        return self.state

    def get_base_flange_pose(self, **kwargs):
        self.calls.append(("flange", kwargs))
        # Orientation is deliberately not reconstructed locally in controller mode.
        return self.matrix[:3, 3].tolist() + [0.31, -0.42, 0.17]

    def pose_to_matrix(self, pose):
        self.calls.append(("matrix", pose))
        return self.matrix.tolist()

    def pose_mul(self, pose1, pose2, *, unit_type):
        self.calls.append(("poseMul", pose1, pose2, unit_type))
        if unit_type != 1 or pose1[:3] != [0, 0, 0] or pose2[3:] != [0, 0, 0]:
            raise AssertionError("Unexpected basis conversion")
        xyz = self.matrix[:3, :3] @ np.array(pose2[:3])
        return xyz.tolist() + pose1[3:]


class CalibrationTests(unittest.TestCase):
    def test_fourth_pose_validation_passes_for_known_tcp(self):
        result = tp.calibrate_with_validation(four_observations())
        self.assertEqual(result["status"], "calibrated")
        self.assertTrue(result["validation"]["passed"])
        self.assertFalse(result["validation"]["used_for_calibration"])
        self.assertLess(result["validation"]["error_norm_mm"], 1e-10)
        self.assertEqual(len(result["T_base_flange"]), 4)
        self.assertEqual(len(result["residual_norms_mm"]), 3)

    def test_fourth_pose_error_does_not_change_tcp_or_fitted_reference(self):
        matrices = four_observations()
        fit = tp.calibrate(matrices[:3])
        matrices[3][:3, 3] += [3, 4, 0]
        result = tp.calibrate_with_validation(matrices)
        self.assertEqual(result["tcp_offset_flange_mm"], fit["tcp_offset_flange_mm"])
        self.assertEqual(result["fixed_point_base_mm"], fit["fixed_point_base_mm"])
        self.assertAlmostEqual(result["validation"]["error_norm_mm"], 5)
        np.testing.assert_allclose(result["validation"]["error_vector_mm"], [3, 4, 0], atol=1e-10)
        self.assertEqual(result["status"], "validation_failed")

    def test_fourth_pose_validation_threshold_is_configurable(self):
        matrices = four_observations()
        matrices[3][0, 3] += 0.5
        self.assertFalse(tp.calibrate_with_validation(matrices, max_validation_error_mm=.2)["validation"]["passed"])
        self.assertTrue(tp.calibrate_with_validation(matrices, max_validation_error_mm=.8)["validation"]["passed"])

    def test_workflow_requires_four_and_first_three_must_be_observable(self):
        with self.assertRaisesRegex(ValueError, "4 个"):
            tp.calibrate_with_validation(observations())
        matrices = four_observations()
        matrices[:3] = [matrices[0]] * 3
        with self.assertRaisesRegex(ValueError, "退化"):
            tp.calibrate_with_validation(matrices)

    def test_known_tcp_and_contact_recovered(self):
        result = tp.calibrate(observations())
        np.testing.assert_allclose(result["tcp_offset_flange_mm"], TCP, atol=1e-10)
        np.testing.assert_allclose(result["fixed_point_base_mm"], FIXED, atol=1e-10)
        self.assertLess(result["quality"]["max_residual_mm"], 1e-10)
        self.assertIsNone(result["tool_orientation"])

    def test_pose_order_does_not_change_result(self):
        result = tp.calibrate(observations()[::-1])
        np.testing.assert_allclose(result["tcp_offset_flange_mm"], TCP, atol=1e-10)

    def test_base_change_leaves_tcp_unchanged(self):
        change = np.eye(4)
        change[:3, :3] = ROTATIONS[1]
        change[:3, 3] = [91, -28, 7]
        result = tp.calibrate([change @ m for m in observations()])
        np.testing.assert_allclose(result["tcp_offset_flange_mm"], TCP, atol=1e-10)
        np.testing.assert_allclose(result["fixed_point_base_mm"],
                                   change[:3, :3] @ FIXED + change[:3, 3], atol=1e-10)

    def test_small_noise_has_nonzero_bounded_residual(self):
        matrices = observations()
        matrices[2][:3, 3] += [0.02, -0.01, 0.03]
        result = tp.calibrate(matrices)
        self.assertGreater(result["quality"]["rms_residual_mm"], 0)
        self.assertLess(result["quality"]["max_residual_mm"], 0.05)
        np.testing.assert_allclose(result["tcp_offset_flange_mm"], TCP, atol=0.05)

    def test_identical_poses_and_single_rotation_axis_rejected(self):
        for rotations in ([np.eye(3)] * 3,
                          [np.eye(3), ROTATIONS[1], ROTATIONS[1].T]):
            with self.subTest(rotations=rotations), self.assertRaisesRegex(ValueError, "退化"):
                tp.calibrate(observations(rotations))

    def test_tiny_rotations_rejected_despite_good_relative_condition(self):
        rotations = [tp.pose_to_matrix([0, 0, 0, *angles], "deg")[:3, :3]
                     for angles in ([0, 0, 0], [0.01, 0, 0], [0, 0.01, 0])]
        with self.assertRaisesRegex(ValueError, "变化不足"):
            tp.calibrate(observations(rotations))

    def test_inconsistent_contact_rejected(self):
        matrices = observations()
        matrices[2][:3, 3] += [10, -10, 20]
        with self.assertRaisesRegex(ValueError, "残差过大"):
            tp.calibrate(matrices)

    def test_invalid_matrices_and_pose_count(self):
        bad = []
        matrix = np.eye(4)
        matrix[0, 0] = -1
        bad.append(matrix)
        matrix = np.eye(4)
        matrix[0, 0] = 2
        bad.append(matrix)
        matrix = np.eye(4)
        matrix[3, 0] = 1
        bad.append(matrix)
        matrix = np.eye(4)
        matrix[0, 3] = np.nan
        bad.append(matrix)
        bad.append(np.zeros((3, 3)))
        for matrix in bad:
            with self.subTest(matrix=matrix), self.assertRaises(ValueError):
                tp.validate_transform(matrix)
        with self.assertRaises(ValueError):
            tp.calibrate(observations()[:2])

    def test_explicit_local_euler_conversion(self):
        matrix = tp.pose_to_matrix([4, 5, 6, 90, 0, 0], "deg")
        np.testing.assert_allclose(matrix[:3, :3], ROTATIONS[1], atol=1e-12)
        rad = tp.pose_to_matrix([4, 5, 6, np.pi/2, 0, 0], "rad")
        np.testing.assert_allclose(matrix, rad, atol=1e-12)


class CaptureAndFileTests(unittest.TestCase):
    def test_stationary_flange_read_uses_controller_conversion(self):
        robot = FakeRobot(observations()[1])
        matrix, record = tp.read_stationary_pose(robot, samples=2, interval=0)
        np.testing.assert_allclose(matrix, observations()[1])
        self.assertEqual(record["selected_sample_index"], 1)
        self.assertEqual(sum(c == "state" for c in robot.calls), 4)
        self.assertIn(("flange", {"unit_type": 1}), robot.calls)
        self.assertEqual(record["rotation_method"], "poseMul_basis")
        self.assertEqual(sum(isinstance(c, tuple) and c[0] == "poseMul" for c in robot.calls), 6)
        self.assertFalse(any(isinstance(c, tuple) and c[0] == "matrix" for c in robot.calls))

    def test_pose_mul_matrix_matches_nontrivial_rotation_and_translation(self):
        rotation = ROTATIONS[1] @ ROTATIONS[2]
        matrix = observations([rotation])[0]
        robot = FakeRobot(matrix)
        pose = matrix[:3, 3].tolist() + [.31, -.42, .17]
        np.testing.assert_allclose(tp.matrix_from_pose_mul(robot, pose), matrix, atol=1e-12)
        # A transposed rotation is different for this fixture.
        self.assertFalse(np.allclose(rotation, rotation.T))

    def test_legacy_capture_never_calls_missing_pose_to_matrix(self):
        robot = FakeRobot(observations()[2])
        with patch.object(robot, "pose_to_matrix", side_effect=AssertionError("旧固件不支持")):
            matrix, _ = tp.read_stationary_pose(robot, samples=2, interval=0)
        np.testing.assert_allclose(matrix, observations()[2], atol=1e-12)

    def test_invalid_pose_mul_result_is_rejected(self):
        robot = FakeRobot(observations()[0])
        with patch.object(robot, "pose_mul", return_value=[0, 0, 0, 0, 0, 0]):
            with self.assertRaisesRegex(ValueError, "旋转矩阵"):
                tp.matrix_from_pose_mul(robot, [0, 0, 0, 0, 0, 0])

    def test_modern_matrix_method_remains_explicitly_available(self):
        robot = FakeRobot(observations()[1])
        matrix, record = tp.read_stationary_pose(robot, samples=2, interval=0, rotation_source="matrix")
        np.testing.assert_allclose(matrix, observations()[1])
        self.assertEqual(record["rotation_method"], "pose_to_matrix")

    def test_motion_is_rejected_before_reading_pose(self):
        robot = FakeRobot(observations()[0], state=3)
        with self.assertRaisesRegex(RuntimeError, "停止状态"):
            tp.read_stationary_pose(robot, samples=2, interval=0)
        self.assertEqual(robot.calls, ["state"])

    def test_position_and_rotation_instability_rejected(self):
        for kind in ("position", "rotation"):
            first, second = observations()[0], observations()[0].copy()
            if kind == "position":
                second[0, 3] += 0.5
            else:
                second[:3, :3] = ROTATIONS[1]
            robot = FakeRobot(first)
            with patch.object(robot, "pose_to_matrix", side_effect=[first.tolist(), second.tolist()]), \
                    patch.object(robot, "get_base_flange_pose", side_effect=[
                        first[:3, 3].tolist() + [0, 0, 0], second[:3, 3].tolist() + [0, 0, 0]]):
                with self.subTest(kind=kind), self.assertRaisesRegex(RuntimeError, "不稳定"):
                    tp.read_stationary_pose(robot, samples=2, interval=0, rotation_source="matrix")

    def test_atomic_save_and_invalid_write_preserves_old_data(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "output.json"
            tp.save_json(path, {"test": "中文"})
            with self.assertRaises(ValueError):
                tp.save_json(path, {"bad": float("nan")})
            self.assertEqual(json.loads(path.read_text(encoding="utf-8")), {"test": "中文"})
            self.assertEqual(len(list(Path(folder).iterdir())), 1)

    def test_file_cli_saves_actual_result_and_preserves_on_failure(self):
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            source, output = folder / "poses.json", folder / "output.json"
            data = {"length_unit": "mm", "input_frame": "robot_base_flange",
                    "T_base_flange": [m.tolist() for m in four_observations()]}
            tp.save_json(source, data)
            with patch.object(tp, "OUTPUT", output), contextlib.redirect_stdout(io.StringIO()):
                argv = ["file", str(source), "--log-file", str(folder / "calibration.log")]
                self.assertEqual(tp.main(argv), 0)
                result = json.loads(output.read_text(encoding="utf-8"))
                np.testing.assert_allclose(result["tcp_offset_flange_mm"], TCP, atol=1e-10)
                before = output.read_bytes()
                data["T_base_flange"] = [observations()[0].tolist()] * 4
                tp.save_json(source, data)
                with self.assertRaises(ValueError):
                    tp.main(argv)
                self.assertEqual(output.read_bytes(), before)

    def test_capture_cli_end_to_end_with_fake_controller(self):
        robots = [FakeRobot(m) for m in four_observations()]
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            with patch.object(tp, "HERE", folder), patch.object(tp, "OUTPUT", folder / "output.json"), \
                    patch("elite_ec_sdk.ECClient", side_effect=robots) as client, \
                    patch("builtins.input", return_value=""), patch.object(tp.time, "sleep"), \
                    contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(tp.main(["capture", "--host", "192.0.2.1", "--samples", "2"]), 0)
                result = json.loads((folder / "output.json").read_text(encoding="utf-8"))
                np.testing.assert_allclose(result["tcp_offset_flange_mm"], TCP, atol=1e-10)
                self.assertEqual(result["status"], "calibrated")
                self.assertEqual(client.call_count, 4)
                entries = [json.loads(line) for line in (folder / "calibration.log").read_text(encoding="utf-8").splitlines()]
                captured = next(e["data"] for e in entries if e["event"] == "captured_poses"
                                and e["data"]["status"] == "complete")
                self.assertEqual(captured["status"], "complete")
                self.assertEqual(len(captured["capture_records"]), 4)
                self.assertEqual(captured["sample_roles"], ["calibration"] * 3 + ["validation"])
                self.assertFalse((folder / "captured_poses.json").exists())
                self.assertEqual(set(result), {"schema_version", "status", "calibration_type", "length_unit",
                                               "input_frame", "tcp_offset_flange_mm", "tool_orientation"})
                details = next(e["data"] for e in entries if e["event"] == "calibration_details")
                self.assertTrue(details["validation"]["passed"])
                self.assertEqual(len(details["T_base_flange"]), 4)

    def test_failed_validation_is_saved_and_cli_returns_failure(self):
        matrices = four_observations()
        matrices[3][0, 3] += 4
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            source, output = folder / "poses.json", folder / "output.json"
            tp.save_json(source, {"length_unit": "mm", "input_frame": "robot_base_flange",
                                  "T_base_flange": [m.tolist() for m in matrices]})
            with patch.object(tp, "OUTPUT", output), contextlib.redirect_stdout(io.StringIO()), \
                    contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(tp.main(["file", str(source), "--max-validation-error-mm", "1",
                                          "--log-file", str(folder / "calibration.log")]), 1)
            result = json.loads(output.read_text(encoding="utf-8"))
            self.assertEqual(result["status"], "validation_failed")
            entries = [json.loads(line) for line in (folder / "calibration.log").read_text(encoding="utf-8").splitlines()]
            details = next(e["data"] for e in entries if e["event"] == "calibration_details")
            self.assertFalse(details["validation"]["passed"])
            self.assertAlmostEqual(details["validation"]["error_norm_mm"], 4)

    def test_incomplete_four_pose_file_is_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            source = Path(folder) / "poses.json"
            tp.save_json(source, {"length_unit": "mm", "input_frame": "robot_base_flange",
                                  "T_base_flange": [m.tolist() for m in observations()]})
            with self.assertRaisesRegex(ValueError, "4 个"):
                tp.load_transforms(source)

    def test_capture_version_mismatch_does_not_save_result(self):
        robot = FakeRobot(observations()[0], version="V9.0.0")
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            with patch.object(tp, "HERE", folder), patch.object(tp, "OUTPUT", folder / "output.json"), \
                    patch("elite_ec_sdk.ECClient", return_value=robot), \
                    patch("builtins.input", return_value=""), contextlib.redirect_stdout(io.StringIO()):
                with self.assertRaisesRegex(RuntimeError, "版本不符"):
                    tp.main(["capture", "--host", "192.0.2.1", "--expected-version", "V3.14.2"])
            self.assertFalse((folder / "output.json").exists())
            entries = [json.loads(line) for line in (folder / "calibration.log").read_text(encoding="utf-8").splitlines()]
            self.assertEqual(entries[-1]["event"], "run_failed")

    def test_log_replay_works_when_input_and_output_log_are_the_same(self):
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            log_path = folder / "calibration.log"
            log = tp.RunLog(log_path)
            log.emit("captured_poses", {"status": "complete", "length_unit": "mm",
                                        "input_frame": "robot_base_flange",
                                        "T_base_flange": [m.tolist() for m in four_observations()]})
            with patch.object(tp, "HERE", folder), patch.object(tp, "OUTPUT", folder / "output.json"), \
                    contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(tp.main(["file", str(log_path)]), 0)
            np.testing.assert_allclose(tp.load_transforms(log_path), four_observations())
            entries = [json.loads(line) for line in log_path.read_text(encoding="utf-8").splitlines()]
            self.assertEqual(len({e["run_id"] for e in entries}), 2)


if __name__ == "__main__":
    unittest.main()
