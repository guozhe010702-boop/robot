import contextlib
import importlib.util
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

import numpy as np

POSE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(POSE_DIR))
import robot_control as rc

spec = importlib.util.spec_from_file_location("pose_save", POSE_DIR / "save.py")
pose_save = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pose_save)


def calibration():
    return {"status": "calibrated", "calibration_type": "three_pose_fixed_point_tcp",
            "length_unit": "mm", "input_frame": "robot_base_flange",
            "tcp_offset_flange_mm": [10, 20, 30], "tool_orientation": None}


def fake_robot():
    robot = Mock(spec=["get_robot_state", "get_base_flange_pose", "pose_mul",
                      "get_soft_version", "__enter__", "__exit__"])
    robot.get_robot_state.return_value = 0
    robot.get_soft_version.return_value = "V3.14.2"
    robot.get_base_flange_pose.return_value = [100, 200, 300, 0, 0, math.pi/2]
    # Independent fixture: Rz(90deg) maps [10,20,30] to [-20,10,30].
    robot.pose_mul.return_value = [80, 210, 330, 0, 0, math.pi/2]
    return robot


class PoseCaptureTests(unittest.TestCase):
    def temp(self):
        return tempfile.TemporaryDirectory(dir=Path(__file__).resolve().parent)

    def write_calibration(self, directory, data=None):
        path = Path(directory) / "calibration.json"
        path.write_text(json.dumps(calibration() if data is None else data), encoding="utf-8")
        return path

    def test_offset_rotates_with_flange_and_order_is_correct(self):
        robot = fake_robot()
        flange, tool = rc.read_tool_pose(robot, [10, 20, 30])
        self.assertEqual(tool[:3], [80, 210, 330])
        self.assertEqual(tool[3:], flange[3:])
        robot.get_base_flange_pose.assert_called_once_with(unit_type=1)
        robot.pose_mul.assert_called_once_with(flange, [10, 20, 30, 0, 0, 0], unit_type=1)
        self.assertEqual(robot.get_robot_state.call_count, 2)

    def test_pending_calibration_fails_before_connecting_and_preserves_output(self):
        with self.temp() as folder:
            path = self.write_calibration(folder, {"status": "pending"})
            output = Path(folder) / "output.txt"
            output.write_text("old data\n", encoding="utf-8")
            with patch.object(rc, "ECClient") as client, self.assertRaisesRegex(ValueError, "尚未完成"):
                rc.main(["--host", "192.0.2.1", "--calibration", str(path), "--output", str(output)])
            client.assert_not_called()
            self.assertEqual(output.read_text(), "old data\n")

    def test_wrong_units_type_and_invalid_offset_rejected(self):
        for field, value in (("length_unit", "m"), ("input_frame", "robot_base_tcp"),
                             ("calibration_type", "three_point_user_frame"),
                             ("tcp_offset_flange_mm", [1, 2, float("nan")]),
                             ("tcp_offset_flange_mm", [1, 2, True]),
                             ("tool_orientation", [0, 0, 0])):
            with self.subTest(field=field), self.temp() as folder:
                data = calibration()
                data[field] = value
                path = self.write_calibration(folder, data)
                with self.assertRaises(ValueError):
                    rc.load_tcp_offset(path)

    def test_motion_before_and_after_read_is_rejected(self):
        for states in ([3], [0, 3]):
            robot = fake_robot()
            robot.get_robot_state.side_effect = states
            with self.subTest(states=states), self.assertRaisesRegex(RuntimeError, "停稳"):
                rc.read_tool_pose(robot, [10, 20, 30])

    def test_full_capture_appends_six_mm_rad_values(self):
        with self.temp() as folder:
            path = self.write_calibration(folder)
            output = Path(folder) / "output.txt"
            output.write_text("1 2 3 0 0 0", encoding="utf-8")
            robot = fake_robot()
            connection = Mock()
            connection.__enter__ = Mock(return_value=robot)
            connection.__exit__ = Mock(return_value=False)
            with patch.object(rc, "ECClient", return_value=connection) as client, \
                    contextlib.redirect_stdout(io.StringIO()):
                rc.main(["--host", "192.0.2.1", "--calibration", str(path), "--output", str(output)])
            client.assert_called_once_with("192.0.2.1", port=8055, timeout=3)
            rows = [[float(v) for v in line.split()] for line in output.read_text().splitlines()]
            self.assertEqual(rows[0], [1, 2, 3, 0, 0, 0])
            np.testing.assert_allclose(rows[1], [80, 210, 330, 0, 0, math.pi/2])

    def test_default_paths_are_anchored_to_project(self):
        self.assertEqual(rc.OUTPUT, POSE_DIR / "output.txt")
        self.assertEqual(rc.CALIBRATION, POSE_DIR.parents[1] / "threePoint" / "output.json")

    def test_invalid_saved_pose_does_not_create_file(self):
        with self.temp() as folder:
            output = Path(folder) / "output.txt"
            with self.assertRaises(ValueError):
                rc.append_pose(output, [1, 2, 3])
            self.assertFalse(output.exists())

    def test_save_npy_preserves_six_components_and_checks_all_rows_first(self):
        with self.temp() as folder:
            folder = Path(folder)
            source = folder / "output.txt"
            source.write_text("1 2 3 0.1 0.2 0.3\n\n4 5 6 0.4 0.5 0.6\n", encoding="utf-8")
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(pose_save.save_poses(source, folder / "result"), 2)
            np.testing.assert_allclose(np.load(folder / "result" / "result2.npy"), [4, 5, 6, .4, .5, .6])
            source.write_text("1 2 3 0 0 0\n1 2 nan 0 0 0\n", encoding="utf-8")
            with self.assertRaises(ValueError):
                pose_save.save_poses(source, folder / "bad")
            self.assertFalse((folder / "bad").exists())


if __name__ == "__main__":
    unittest.main()
