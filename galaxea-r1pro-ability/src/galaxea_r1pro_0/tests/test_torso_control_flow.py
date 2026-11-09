"""躯干控制 Task 全流程测试（test_mode SDK，mock 躯干状态反馈）。"""

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


class TorsoControlFlowTest(unittest.TestCase):
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
            role_task_types("torso_control"),
            (
                (0, "control_torso_joints"),
                (1, "control_single_torso_joint"),
            ),
        )

    def test_control_torso_joints_full_flow(self) -> None:
        sdk = self._sdk()
        try:
            service = R1ProAbilityService("torso_control", sdk)
            with mock.patch.object(
                sdk.sensors,
                "get_torso_positions",
                return_value=_state_result([0.1, 0.2, 0.3, 0.4]),
            ):
                result = service.invoke(
                    "control_torso_joints",
                    {
                        "invocation_id": "t1",
                        "positions": [0.1, 0.2, 0.3, 0.4],
                        "speeds": [0.5, 0.5, 0.5, 0.5],
                        "tol": 0.01,
                        "timeout": 2.0,
                    },
                )
            self.assertEqual(result["status"], "running")
            self.assertEqual(result["task_name"], "control_torso_joints")
            self.assertTrue(result["command_id"])

            # positions 长度非 4 → ValueError
            with self.assertRaises(ValueError):
                service.invoke("control_torso_joints", {"positions": [0.1, 0.2, 0.3]})

            # 无反馈超时 → RuntimeError
            with mock.patch.object(
                sdk.sensors,
                "get_torso_positions",
                return_value=_state_result(None),
            ):
                with self.assertRaises(RuntimeError):
                    service.invoke(
                        "control_torso_joints",
                        {"positions": [0.1, 0.2, 0.3, 0.4], "timeout": 0.05},
                    )
        finally:
            sdk.close()

    def test_control_single_torso_joint_full_flow(self) -> None:
        sdk = self._sdk()
        try:
            service = R1ProAbilityService("torso_control", sdk)
            # 真实登记一个命令作为 mock 返回的 handle，保证 executions.refresh 可查询
            handle = sdk.torso.control_torso_joints(
                positions=[0.0, 0.0, 0.0, 0.0],
                speeds=[0.5, 0.5, 0.5, 0.5],
            )
            with mock.patch.object(
                sdk.torso,
                "control_single_torso_joint",
                return_value=handle,
            ) as single_mock, mock.patch.object(
                sdk.sensors,
                "get_torso_positions",
                return_value=_state_result([0.0, 0.0, 0.0, 0.0]),
            ):
                result = service.invoke(
                    "control_single_torso_joint",
                    {
                        "invocation_id": "t2",
                        "joint_id": 0,
                        "target_pos": 0.0,
                        "tol": 0.01,
                        "timeout": 2.0,
                    },
                )
                # joint_id 从 0 开始，直接透传给底层 SDK
                single_mock.assert_called_once_with(
                    joint_id=0,
                    target_pos=0.0,
                    speed=1.5,
                    effort=None,
                    names=[],
                )
            self.assertEqual(result["status"], "running")
            self.assertEqual(result["task_name"], "control_single_torso_joint")
            self.assertTrue(result["command_id"])

            # 关节 ID 越界 → ValueError
            for bad_id in (-1, 4):
                with self.assertRaises(ValueError):
                    service.invoke(
                        "control_single_torso_joint",
                        {"joint_id": bad_id, "target_pos": 0.0},
                    )

            # 缺少 target_pos → ValueError
            with self.assertRaises(ValueError):
                service.invoke("control_single_torso_joint", {"joint_id": 0})
        finally:
            sdk.close()


if __name__ == "__main__":
    unittest.main()
