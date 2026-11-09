"""WooshNavigation Task 全流程测试（test_mode SDK）。"""

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


class WooshNavigationFlowTest(unittest.TestCase):
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
            role_task_types("woosh_navigation"),
            (
                (0, "navigate_to_map_pose"),
                (1, "stop"),
                (2, "verify_arrival"),
            ),
        )

    def test_navigate_to_map_pose_registers_handle(self) -> None:
        sdk = self._sdk()
        try:
            service = R1ProAbilityService("woosh_navigation", sdk)
            # test_mode 下 SDK 内部 start_command 登记导航命令并返回 RUNNING，
            # Task 据此构造 CommandHandle，invoke 登记执行记录并可 GetExecution
            result = service.invoke(
                "navigate_to_map_pose",
                {
                    "invocation_id": "n1",
                    "target": {
                        "target_ref": "manual-map-pose",
                        "pose": {
                            "frame_id": "map",
                            "position_m": [0.2, 0.0, 0.0],
                            "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
                        },
                    },
                    "maximum_speed_mps": 0.1,
                },
            )
            self.assertEqual(result["status"], "running")
            self.assertEqual(result["task_name"], "navigate_to_map_pose")
            self.assertTrue(result["command_id"])

            execution = service.invoke("GetExecution", {"invocation_id": "n1"})
            self.assertEqual(execution["command_id"], result["command_id"])

            # 缺少必填 target.pose.position_m → ValueError
            with self.assertRaises(ValueError):
                service.invoke("navigate_to_map_pose", {"target": {"pose": {}}})
        finally:
            sdk.close()

    def test_navigate_to_map_pose_failure_raises(self) -> None:
        sdk = self._sdk()
        try:
            service = R1ProAbilityService("woosh_navigation", sdk)
            with mock.patch.object(
                sdk.woosh_navigation,
                "navigate_to_map_pose",
                return_value=CommandResult(
                    command_id="cmd-nav-fail",
                    status=CommandStatus.FAILED,
                    feedback=CommandFeedback("cmd-nav-fail", CommandStatus.FAILED, 1.0, "failed"),
                    output={},
                    error_code="NAV_FAILED",
                    error_message="navigation failed",
                ),
            ):
                with self.assertRaises(RuntimeError):
                    service.invoke(
                        "navigate_to_map_pose",
                        {
                            "target": {
                                "target_ref": "manual-map-pose",
                                "pose": {
                                    "frame_id": "map",
                                    "position_m": [0.2, 0.0, 0.0],
                                    "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
                                },
                            },
                        },
                    )
        finally:
            sdk.close()

    def test_stop_registers_command(self) -> None:
        sdk = self._sdk()
        try:
            service = R1ProAbilityService("woosh_navigation", sdk)
            result = service.invoke("stop", {"invocation_id": "n3"})
            self.assertEqual(result["status"], "running")
            self.assertEqual(result["task_name"], "stop")
            self.assertTrue(result["command_id"])

            execution = service.invoke("GetExecution", {"invocation_id": "n3"})
            self.assertEqual(execution["command_id"], result["command_id"])
        finally:
            sdk.close()


if __name__ == "__main__":
    unittest.main()
