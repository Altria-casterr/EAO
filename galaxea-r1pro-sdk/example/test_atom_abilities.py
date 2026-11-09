"""Galaxea_R1Pro 能力模块测试：类形式（end_effector.py 风格）提交命令。"""

import unittest
from uuid import uuid4

from galaxea_r1pro.galaxea_r1pro_0.eef_control import EEFControl
from galaxea_r1pro.galaxea_r1pro_0.sensors import SensorsClient
from galaxea_r1pro.galaxea_r1pro_0.state import StateClient
from galaxea_r1pro.galaxea_r1pro_0.gripper_control import GripperControl
from galaxea_r1pro.galaxea_r1pro_0.joint_control import JointControl
from galaxea_r1pro.galaxea_r1pro_0.torso_control import TorsoControl
from galaxea_r1pro.galaxea_r1pro_0.woosh_move import WooshMove
from galaxea_r1pro.galaxea_r1pro_0.woosh_navigation import WooshNavigation
from galaxea_r1pro.galaxea_r1pro_0.utils.types import CommandHandle, CommandResult, CommandStatus


class _FakeBackend:
    def __init__(self) -> None:
        self.calls: list[tuple[str, str, dict]] = []
        self.get_calls: list[tuple[str, str, dict]] = []

    def start_command(self, resource: str, operation: str, target: dict) -> CommandHandle:
        self.calls.append((resource, operation, target))
        return CommandHandle(str(uuid4()), "r1pro-x", resource, operation)

    def get_command(
        self,
        command_id: str | None = None,
        *,
        resource: str | None = None,
        operation: str | None = None,
        target: dict | None = None,
    ) -> CommandResult:
        self.get_calls.append((resource, operation, target or {}))
        return CommandResult(
            command_id=command_id or "",
            status=CommandStatus.SUCCEEDED,
            feedback=CommandStatus.SUCCEEDED.value,
        )


class AbilityClassTest(unittest.TestCase):
    def setUp(self) -> None:
        self.backend = _FakeBackend()

    def test_joint_control_submits_command(self) -> None:
        client = JointControl(self.backend)
        handle = client.control_single_arm_joint(
            arm_side="left", positions=[0.1, 0.2], joint_names=["j1", "j2"]
        )
        resource, operation, target = self.backend.calls[-1]
        self.assertEqual((resource, operation), ("joint_control", "control_single_arm_joint"))
        self.assertEqual(target["arm_side"], "left")
        self.assertEqual(handle.resource, "joint_control")

    def test_gripper_control_submits_command(self) -> None:
        client = GripperControl(self.backend)
        client.control_single_gripper(arm_side="right", position=30.0)
        resource, operation, target = self.backend.calls[-1]
        self.assertEqual((resource, operation), ("gripper_control", "control_single_gripper"))
        self.assertEqual(target["side"], "right")
        self.assertEqual(target["position"], 30.0)

    def test_torso_control_submits_command(self) -> None:
        client = TorsoControl(self.backend)
        client.control_torso_joints(
            positions=[0.1, 0.2, 0.3, 0.4],
            effort=[0.1, 0.2, 0.3, 0.4],
            names=["torso_joint_1", "torso_joint_2", "torso_joint_3", "torso_joint_4"],
        )
        resource, operation, target = self.backend.calls[-1]
        self.assertEqual((resource, operation), ("torso_control", "control_torso_joints"))
        self.assertEqual(target["efforts"], [0.1, 0.2, 0.3, 0.4])
        self.assertEqual(
            target["names"],
            ["torso_joint_1", "torso_joint_2", "torso_joint_3", "torso_joint_4"],
        )

    def test_eef_control_submits_command(self) -> None:
        client = EEFControl(self.backend)
        client.control_single_arm(x=0.4, y=0.3, z=0.1, axis="left")
        resource, operation, target = self.backend.calls[-1]
        self.assertEqual((resource, operation), ("eef_control", "control_single_arm"))
        self.assertEqual(target["axis"], "left")
        self.assertEqual(target["target"].position, (0.4, 0.3, 0.1))
        self.assertEqual(target["target"].orientation_xyzw, (0.0, 0.0, 0.0, 1.0))

    def test_euler_to_quaternion_from_utils_common(self) -> None:
        from galaxea_r1pro.galaxea_r1pro_0.utils.utils_common import euler_to_quaternionx

        qx, qy, qz, qw = (float(value) for value in euler_to_quaternionx([0.0, 0.0, 90.0], seq="yxz"))
        self.assertAlmostEqual(qx, 0.0)
        self.assertAlmostEqual(qy, 0.0)
        self.assertAlmostEqual(qz, 0.70710678, places=6)
        self.assertAlmostEqual(qw, 0.70710678, places=6)

    def test_control_both_arms_joint_runs_in_parallel(self) -> None:
        client = JointControl(self.backend)
        handles = client.control_both_arms_joint(
            left_positions=[0.1], right_positions=[0.2],
            left_joint_names=["l1"], right_joint_names=["r1"],
        )
        self.assertEqual(len(handles), 2)
        arm_sides = {call[2]["arm_side"] for call in self.backend.calls}
        self.assertEqual(arm_sides, {"left", "right"})

    def test_control_both_gripper_runs_in_parallel(self) -> None:
        client = GripperControl(self.backend)
        handles = client.control_both_gripper(left_position=10, right_position=20)
        self.assertEqual(len(handles), 2)
        sides = {call[2]["side"] for call in self.backend.calls}
        self.assertEqual(sides, {"left", "right"})

    def test_control_both_arm_runs_in_parallel(self) -> None:
        client = EEFControl(self.backend)
        handles = client.control_both_arm(
            left_x=0.4, left_y=0.3, left_z=0.1, left_roll=0, left_pitch=0, left_yaw=0,
            right_x=0.4, right_y=-0.3, right_z=0.1, right_roll=0, right_pitch=0, right_yaw=0,
        )
        self.assertEqual(len(handles), 2)
        axes = {call[2]["axis"] for call in self.backend.calls}
        self.assertEqual(axes, {"left", "right"})

    def test_sensors_force_feedback_uses_get_command(self) -> None:
        client = SensorsClient(self.backend)
        client.get_force_data()
        resource, operation, _ = self.backend.get_calls[-1]
        self.assertEqual((resource, operation), ("force_feedback", "get_force_data"))


    def test_state_torso_status_uses_get_command(self) -> None:
        client = StateClient(self.backend)
        client.get_torso_status()
        resource, operation, _ = self.backend.get_calls[-1]
        self.assertEqual((resource, operation), ("torso_control", "get_torso_status"))

    def test_state_floating_base_uses_get_command(self) -> None:
        client = StateClient(self.backend)
        client.get_floating_base()
        resource, operation, _ = self.backend.get_calls[-1]
        self.assertEqual((resource, operation), ("eef_control", "get_floating_base"))

    def test_woosh_move_submits_command(self) -> None:
        client = WooshMove(self.backend)
        client.move_robot(axis="x", target_value=0.5, speed=0.1)
        resource, operation, target = self.backend.calls[-1]
        self.assertEqual((resource, operation), ("woosh_move", "move_robot"))
        self.assertEqual(target["dx"], 0.1)
        self.assertEqual(target["dy"], 0.0)
        self.assertEqual(target["dyaw"], 0.0)

    def test_woosh_move_uses_preset_speed_when_none(self) -> None:
        client = WooshMove(self.backend)
        client.move_robot(axis="x", target_value=0.5)
        resource, operation, target = self.backend.calls[-1]
        self.assertEqual(target["dx"], WooshMove.DEFAULT_LINEAR_SPEED_X)

    def test_woosh_move_clamps_speed_to_max(self) -> None:
        client = WooshMove(self.backend)
        client.move_robot(axis="x", target_value=0.5, speed=1.0)
        target = self.backend.calls[-1][2]
        self.assertEqual(target["dx"], WooshMove.MAX_LINEAR_SPEED)
        client.move_robot(axis="z", target_value=90.0, speed=45.0)
        target = self.backend.calls[-1][2]
        self.assertEqual(target["dyaw"], WooshMove.MAX_ANGULAR_SPEED)

    def test_woosh_move_rejects_invalid_axis(self) -> None:
        client = WooshMove(self.backend)
        with self.assertRaises(ValueError):
            client.move_robot(axis="a", target_value=1.0)

    def test_woosh_move_rejects_when_emergency_stop(self) -> None:
        client = WooshMove(self.backend)
        client._emergency_stop = True
        with self.assertRaises(RuntimeError):
            client.move_robot(axis="x", target_value=1.0)

    def test_woosh_move_unlocks_brake_before_moving(self) -> None:
        client = WooshMove(self.backend)
        client.move_robot(axis="x", target_value=0.5, speed=0.1)
        resource, operation, target = self.backend.calls[0]
        self.assertEqual((resource, operation), ("woosh_move", "unlock_brake"))
        resource, operation, target = self.backend.calls[-1]
        self.assertEqual((resource, operation), ("woosh_move", "move_robot"))

    def test_woosh_navigation_submits_command(self) -> None:
        client = WooshNavigation(self.backend)
        client.navigate_to_map_pose(x=3.2, y=-1.3, yaw=180.0)
        resource, operation, target = self.backend.calls[-1]
        self.assertEqual((resource, operation), ("woosh_navigation", "navigate_to_map_pose"))
        self.assertEqual(target["x"], 3.2)

    def test_woosh_navigation_loops_on_current_status(self) -> None:
        client = WooshNavigation(self.backend)
        result = client.navigate_to_map_pose(x=3.2, y=-1.3, yaw=180.0, timeout=30.0)
        self.assertEqual(result.status, CommandStatus.SUCCEEDED)
        self.assertIn(("woosh_navigation", "get_current_status", {}), self.backend.get_calls)

    def test_woosh_navigation_stop_sets_emergency(self) -> None:
        client = WooshNavigation(self.backend)
        client.stop()
        resource, operation, target = self.backend.calls[-1]
        self.assertEqual((resource, operation), ("woosh_move", "move_robot"))
        self.assertEqual(target, {"dx": 0.0, "dy": 0.0, "dyaw": 0.0})
        self.assertTrue(client._emergency_stop)

    def test_state_transformer_martix_submits_params(self) -> None:
        client = StateClient(self.backend)
        client.get_transformer_martix(source_frame="map", target_frame="body")
        resource, operation, target = self.backend.get_calls[-1]
        self.assertEqual((resource, operation), ("coordinate_transformation", "get_transformer_martix"))
        self.assertEqual(target["source_frame"], "map")
        self.assertEqual(target["target_frame"], "body")

    def test_sensors_torso_positions_uses_get_command(self) -> None:
        client = SensorsClient(self.backend)
        client.get_torso_positions()
        resource, operation, _ = self.backend.get_calls[-1]
        self.assertEqual((resource, operation), ("torso_control", "get_torso_positions"))

    def test_sensors_chassis_feedback_uses_get_command(self) -> None:
        client = SensorsClient(self.backend)
        client.get_chassis_feedback()
        resource, operation, _ = self.backend.get_calls[-1]
        self.assertEqual((resource, operation), ("woosh_move", "get_chassis_feedback"))

    def test_sensors_localization_uses_get_command(self) -> None:
        client = SensorsClient(self.backend)
        client.get_localization()
        resource, operation, _ = self.backend.get_calls[-1]
        self.assertEqual((resource, operation), ("woosh_move", "get_localization"))



if __name__ == "__main__":
    unittest.main()
