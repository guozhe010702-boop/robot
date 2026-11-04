import unittest
from unittest.mock import patch
import sys
import os

# 获取当前文件的绝对路径
current_dir = os.path.dirname(os.path.abspath(__file__))
# 获取上一级目录的绝对路径
parent_dir = os.path.dirname(current_dir)

# 将上一级目录添加到系统路径中
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

import move_to_pose as motion
import run_move_test as runner


CURRENT = [100.0, 200.0, 300.0, 0.0, 0.0, 0.0]
TARGET = [110.0, 200.0, 300.0, 0.0, 0.0, 0.0]
CURRENT_JOINTS = [20.0, -80.0, 95.0, -90.0, 80.0, 10.0]
TARGET_JOINTS = [21.0, -79.0, 96.0, -90.0, 80.0, 10.0]


class FakeRobot:
    def __init__(self, mode=2, state=0, servo=True, motor=True, coord=1):
        self.mode = mode
        self.state = state
        self.servo = servo
        self.motor = motor
        self.coord = coord
        self.calls = []
        self.moved = False
        self.connected = True

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        self.connected = False

    def get_soft_version(self):
        return "V3.14.2"

    def get_robot_mode(self):
        return self.mode

    def get_robot_state(self):
        return self.state

    def get_estop_status(self):
        return 0

    def get_motor_status(self):
        return self.motor

    def get_servo_status(self):
        return self.servo

    def get_tcp_pose(self, **_kwargs):
        return TARGET if self.moved else CURRENT

    def get_joint_pos(self):
        return CURRENT_JOINTS

    def positive_kinematic(self, joints, *, unit_type):
        if unit_type != 1:
            raise AssertionError("FK 必须返回 mm/rad")
        if joints == CURRENT_JOINTS:
            return CURRENT
        if joints == TARGET_JOINTS:
            return TARGET
        raise AssertionError(f"Unexpected joints: {joints}")

    def call(self, method, params=None, *, control=False):
        self.calls.append((method, params, control))
        if method == "getCurrentCoord":
            return self.coord
        if method == "getAutoRunToolNumber":
            return 0
        if method == "inverseKinematic":
            if params != {"targetPose": TARGET, "referencePos": CURRENT_JOINTS, "unit_type": 1}:
                raise AssertionError(f"Unexpected IK params: {params}")
            return TARGET_JOINTS
        if method == "moveByJoint":
            if "target_pose" in params or "target_tcp_pose" in params or params.get("targetPos") != TARGET_JOINTS:
                raise AssertionError("V3.14.2 moveByJoint 仅接受关节 targetPos，不能直接传 TCP")
            self.moved = True
            return True
        raise AssertionError(f"Unexpected method: {method}")


class MotionTests(unittest.TestCase):
    def argv(self, *extra):
        return ["--host", "192.168.1.200", "--pose", *map(str, TARGET), *extra]

    def test_offline_never_creates_client(self):
        with patch.object(motion, "ECClient") as client:
            self.assertEqual(motion.main(self.argv("--offline")), 0)
            client.assert_not_called()

    def test_read_only_preview_never_sends_motion(self):
        robot = FakeRobot()
        with patch.object(motion, "ECClient", return_value=robot):
            self.assertEqual(motion.main(self.argv()), 0)
        self.assertNotIn("moveByJoint", [method for method, _, _ in robot.calls])

    def test_show_current_with_disabled_servo_and_cylindrical_coord(self):
        robot = FakeRobot(servo=False, coord=4)
        with patch.object(motion, "ECClient", return_value=robot):
            self.assertEqual(motion.main(["--host", "192.0.2.1", "--show-current"]), 0)
        self.assertEqual([method for method, _, _ in robot.calls],
                         ["getCurrentCoord", "getAutoRunToolNumber"])

    def test_show_current_in_teach_mode_without_sync_or_enable(self):
        robot = FakeRobot(mode=0, motor=False, servo=False, coord=2)
        with patch.object(motion, "ECClient", return_value=robot):
            self.assertEqual(motion.main(["--host", "192.0.2.1", "--show-current"]), 0)
        self.assertNotIn("moveByJoint", [method for method, _, _ in robot.calls])

    def test_execute_still_blocks_disabled_servo_without_sending_commands(self):
        robot = FakeRobot(servo=False, coord=4)
        argv = self.argv("--expected-tool", "0", "--confirm-base-tcp-rad", "--execute")
        with patch.object(motion, "ECClient", return_value=robot), \
             patch.object(motion, "input") as confirm:
            with self.assertRaisesRegex(RuntimeError, "已同步但尚未使能"):
                motion.main(argv)
            confirm.assert_not_called()
        self.assertEqual([method for method, _, _ in robot.calls],
                         ["getCurrentCoord", "getAutoRunToolNumber"])

    def test_execute_still_requires_base_coord(self):
        robot = FakeRobot(coord=4)
        with patch.object(motion, "ECClient", return_value=robot):
            with self.assertRaisesRegex(RuntimeError, "不是基座"):
                motion.main(self.argv("--expected-tool", "0", "--confirm-base-tcp-rad", "--execute"))
        self.assertNotIn("moveByJoint", [method for method, _, _ in robot.calls])

    def test_non_remote_is_blocked(self):
        robot = FakeRobot(mode=1)
        with patch.object(motion, "ECClient", return_value=robot):
            with self.assertRaisesRegex(RuntimeError, "REMOTE"):
                motion.main(self.argv())
        self.assertNotIn("moveByJoint", [method for method, _, _ in robot.calls])

    def test_move_limit_blocks_before_send(self):
        with self.assertRaisesRegex(RuntimeError, "超过本次限值"):
            motion.build_move_params([200, 200, 300, 0, 0, 0], CURRENT, 5, 10, 10, 50, 0.2)

    def test_execute_once_and_verify(self):
        robot = FakeRobot()
        argv = self.argv("--expected-version", "V3.14.2", "--expected-tool", "0",
                         "--confirm-base-tcp-rad", "--execute")
        with patch.object(motion, "ECClient", return_value=robot), \
             patch.object(motion, "input", return_value="MOVE"), \
             patch.object(motion.time, "sleep"):
            self.assertEqual(motion.main(argv), 0)
        moves = [(method, params) for method, params, _ in robot.calls if method == "moveByJoint"]
        self.assertEqual(len(moves), 1)
        self.assertEqual(moves[0][1]["targetPos"], TARGET_JOINTS)
        self.assertNotIn("target_pose", moves[0][1])

    def test_execution_requires_explicit_confirmation(self):
        robot = FakeRobot()
        argv = self.argv("--expected-version", "V3.14.2", "--expected-tool", "0",
                         "--confirm-base-tcp-rad", "--execute")
        with patch.object(motion, "ECClient", return_value=robot), \
             patch.object(motion, "input", return_value="no"):
            self.assertEqual(motion.main(argv), 0)
        self.assertNotIn("moveByJoint", [method for method, _, _ in robot.calls])

    def test_pose_change_during_confirmation_blocks_motion(self):
        class MovingRobot(FakeRobot):
            reads = 0

            def get_tcp_pose(self, **_kwargs):
                self.reads += 1
                return [102, 200, 300, 0, 0, 0] if self.reads >= 2 else CURRENT

        robot = MovingRobot()
        argv = self.argv("--expected-version", "V3.14.2", "--expected-tool", "0",
                         "--confirm-base-tcp-rad", "--execute")
        with patch.object(motion, "ECClient", return_value=robot), \
             patch.object(motion, "input", return_value="MOVE"):
            with self.assertRaisesRegex(RuntimeError, "位姿发生变化"):
                motion.main(argv)
        self.assertNotIn("moveByJoint", [method for method, _, _ in robot.calls])


class LegacyJointCompatibilityTests(unittest.TestCase):
    def plan(self):
        return motion.build_move_params(TARGET, CURRENT, 5, 10, 10, 50, 0.2)

    def test_cartesian_plan_resolves_to_joint_degrees(self):
        robot = FakeRobot()
        params = motion.resolve_joint_move(robot, self.plan(), CURRENT)
        self.assertEqual(params, {"targetPos": TARGET_JOINTS, "speed": 5, "acc": 10, "dec": 10})
        self.assertNotIn("moveByJoint", [method for method, _, _ in robot.calls])
        self.assertIn(("inverseKinematic", {"targetPose": TARGET,
                       "referencePos": CURRENT_JOINTS, "unit_type": 1}, False), robot.calls)

    def test_invalid_inverse_solution_cannot_be_sent(self):
        for value in (False, [], [1, 2, 3], [0, 0, 0, 0, 0, float("nan")], [400, 0, 0, 0, 0, 0]):
            robot = FakeRobot()
            original_call = robot.call

            def call(method, params=None, *, control=False):
                if method == "inverseKinematic":
                    return value
                return original_call(method, params, control=control)

            with self.subTest(value=value), patch.object(robot, "call", side_effect=call):
                with self.assertRaises(RuntimeError):
                    motion.resolve_joint_move(robot, self.plan(), CURRENT)
            self.assertFalse(robot.moved)

    def test_current_joint_unit_or_tool_mismatch_blocks_before_ik(self):
        robot = FakeRobot()
        with patch.object(robot, "positive_kinematic", return_value=TARGET):
            with self.assertRaisesRegex(RuntimeError, "当前关节正解"):
                motion.resolve_joint_move(robot, self.plan(), CURRENT)
        self.assertEqual(robot.calls, [])

    def test_ik_round_trip_mismatch_cannot_be_sent(self):
        robot = FakeRobot()
        with patch.object(robot, "positive_kinematic", return_value=CURRENT):
            with self.assertRaisesRegex(RuntimeError, "正解复核失败"):
                motion.resolve_joint_move(robot, self.plan(), CURRENT)
        self.assertFalse(robot.moved)

    def test_motion_sender_rejects_raw_tcp_payload(self):
        robot = FakeRobot()
        for params in ({"target_pose": TARGET}, {"target_tcp_pose": TARGET}, {}):
            with self.subTest(params=params), self.assertRaises(ValueError):
                motion.execute_and_wait(robot, params, 0, 1, 2, .05, target_tcp=TARGET)
        self.assertEqual(robot.calls, [])

    def test_rpc_rejection_does_not_retry_motion_with_another_parameter(self):
        robot = FakeRobot()
        params = {"targetPos": TARGET_JOINTS, "speed": 5, "acc": 10, "dec": 10}
        rejection = motion.RPCError("moveByJoint", {"code": -32603, "message": "parameter invalid"})
        with patch.object(robot, "call", side_effect=rejection) as call:
            with self.assertRaises(motion.RPCError):
                motion.execute_and_wait(robot, params, 0, 1, 2, .05, target_tcp=TARGET)
            call.assert_called_once_with("moveByJoint", params, control=True)


class TestEntryTests(unittest.TestCase):
    def test_default_entry_reads_current_using_actual_endpoint(self):
        robot = FakeRobot(servo=False, coord=4)
        with patch.object(motion, "ECClient", return_value=robot) as client:
            self.assertEqual(runner.main([]), 0)
        client.assert_called_once_with("192.168.1.200", port=8055, timeout=3, allow_control=True)
        self.assertEqual([method for method, _, _ in robot.calls],
                         ["getCurrentCoord", "getAutoRunToolNumber"])

    def test_target_entry_only_previews_without_execute(self):
        robot = FakeRobot()
        with patch.object(motion, "ECClient", return_value=robot), \
             patch.object(motion, "input") as confirm:
            self.assertEqual(runner.main(["--pose", *map(str, TARGET)]), 0)
            confirm.assert_not_called()
        self.assertNotIn("moveByJoint", [method for method, _, _ in robot.calls])

    def test_entry_offline_never_connects(self):
        with patch.object(motion, "ECClient") as client:
            self.assertEqual(runner.main(["--pose", *map(str, TARGET), "--offline"]), 0)
            client.assert_not_called()

    def test_entry_execute_keeps_confirmation_and_single_send(self):
        robot = FakeRobot()
        argv = ["--pose", *map(str, TARGET), "--expected-version", "V3.14.2",
                "--expected-tool", "0", "--confirm-base-tcp-rad", "--execute"]
        with patch.object(motion, "ECClient", return_value=robot), \
             patch.object(motion, "input", return_value="MOVE") as confirm, \
             patch.object(motion.time, "sleep"):
            self.assertEqual(runner.main(argv), 0)
            confirm.assert_called_once()
        self.assertEqual(sum(method == "moveByJoint" for method, _, _ in robot.calls), 1)


if __name__ == "__main__":
    unittest.main()
