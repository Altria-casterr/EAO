"""关节控制 Task 全流程测试（test_mode SDK，mock 关节状态反馈）。"""

from __future__ import annotations

import unittest
from unittest import mock

from galaxea_r1pro import (
    BackendKind,
    CommandFeedback,
    CommandResult,
    CommandStatus,
    RobotProfile,
    R1ProSDK,
)

from galaxea_r1pro_0.service.ability_service import R1ProAbilityService
from galaxea_r1pro_0.tasks import role_task_types


def _state_result(data) -> CommandResult:
    return CommandResult(
        command_id="",
        status=CommandStatus.SUCCEEDED,
        feedback=CommandFeedback("", CommandStatus.SUCCEEDED, 1.0, "completed"),
        output={"data": data},
    )


class JointControlFlowTest(unittest.TestCase):
    def _sdk(self) -> R1ProSDK:
        return R1ProSDK.open(
            RobotProfile(
                robot_id="r1pro-ability-test",
                backend=BackendKind.REAL,
                firmware_profile="test-v1",
                options={"test_mode": True, "defer_commands": True},
            )
        )

    def test_role_task_types(self) -> None:
        self.assertEqual(
            role_task_types("joint_control"),
            (
                (0, "control_single_arm_joint"),
                (1, "control_both_arms_joint"),
            ),
        )

    def test_single_arm_joint_full_flow(self) -> None:
        sdk = self._sdk()
        try:
            service = R1ProAbilityService("joint_control", sdk)
            with mock.patch.object(
                sdk.sensors,
                "get_joint_state",
                return_value=_state_result([0.1, 0.2, 0.3]),
            ):
                result = service.invoke(
                    "control_single_arm_joint",
                    {
                        "invocation_id": "j1",
                        "arm_side": "left",
                        "positions": [0.1, 0.2, 0.3],
                        "joint_names": ["l1", "l2", "l3"],
                        "tol": 0.01,
                        "timeout": 2.0,
                    },
                )
            self.assertEqual(result["status"], "running")
            self.assertEqual(result["task_name"], "control_single_arm_joint")
            self.assertTrue(result["command_id"])

            with self.assertRaises(ValueError):
                service.invoke("control_single_arm_joint", {"arm_side": "middle"})

            with mock.patch.object(
                sdk.sensors,
                "get_joint_state",
                return_value=_state_result(None),
            ):
                with self.assertRaises(RuntimeError):
                    service.invoke(
                        "control_single_arm_joint",
                        {
                            "arm_side": "left",
                            "positions": [0.1],
                            "joint_names": ["l1"],
                            "timeout": 0.05,
                        },
                    )
        finally:
            sdk.close()

    def test_both_arms_joint_full_flow(self) -> None:
        sdk = self._sdk()
        try:
            service = R1ProAbilityService("joint_control", sdk)
            with mock.patch.object(
                sdk.sensors,
                "get_joint_state",
                side_effect=lambda axis: _state_result([0.1, 0.2] if axis == "left" else [0.3, 0.4]),
            ):
                result = service.invoke(
                    "control_both_arms_joint",
                    {
                        "invocation_id": "jb",
                        "left_positions": [0.1, 0.2],
                        "right_positions": [0.3, 0.4],
                        "left_joint_names": ["l1", "l2"],
                        "right_joint_names": ["r1", "r2"],
                        "tol": 0.01,
                        "timeout": 2.0,
                    },
                )
            self.assertEqual(result["status"], "running")
            self.assertEqual(result["task_name"], "control_both_arms_joint")
        finally:
            sdk.close()

    def test_semantic_move_to_posture_single_arm(self) -> None:
        """semantic motion.move_to_posture_single 走 taskType 0 分支。"""

        sdk = self._sdk()
        try:
            service = R1ProAbilityService("joint_control", sdk)
            with mock.patch.object(
                sdk.sensors,
                "get_joint_state",
                return_value=_state_result([0.0] * 7),
            ):
                result = service.invoke(
                    "control_single_arm_joint",
                    {
                        "invocation_id": "jp1",
                        "semantic_flag": True,
                        "arm_side": "left",
                        "position_m": [0.0] * 7,
                        "tol": 0.01,
                        "timeout": 2.0,
                    },
                )
            self.assertEqual(result["status"], "running")
            self.assertEqual(result["task_name"], "control_single_arm_joint")
            self.assertTrue(result["command_id"])

            # position_m 长度不是 7 报错
            with self.assertRaises(ValueError):
                service.invoke(
                    "control_single_arm_joint",
                    {"semantic_flag": True, "arm_side": "left", "position_m": [0.0, 0.0]},
                )
            # arm_side 非法报错
            with self.assertRaises(ValueError):
                service.invoke(
                    "control_single_arm_joint",
                    {"semantic_flag": True, "arm_side": "middle", "position_m": [0.0] * 7},
                )
        finally:
            sdk.close()

    def test_semantic_move_to_posture_both_arms_requires_both_sides(self) -> None:
        """semantic 双臂分支：left_positions/right_positions 都要 7 元素。"""

        sdk = self._sdk()
        try:
            service = R1ProAbilityService("joint_control", sdk)
            # 双臂 7+7 正常调用
            with mock.patch.object(
                sdk.sensors,
                "get_joint_state",
                side_effect=lambda axis: _state_result([0.0] * 7),
            ):
                result = service.invoke(
                    "control_both_arms_joint",
                    {
                        "semantic_flag": True,
                        "left_positions": [0.0] * 7,
                        "right_positions": [0.0] * 7,
                        "timeout": 2.0,
                    },
                )
                self.assertEqual(result["status"], "running")

            # 左侧只给 2 个 → ValueError
            with self.assertRaises(ValueError):
                service.invoke(
                    "control_both_arms_joint",
                    {
                        "semantic_flag": True,
                        "left_positions": [0.0, 0.0],
                        "right_positions": [0.0] * 7,
                        "timeout": 1.0,
                    },
                )
        finally:
            sdk.close()


if __name__ == "__main__":
    unittest.main()
