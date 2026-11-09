"""夹爪 Task 全流程测试（test_mode SDK，无需真机/框架）。"""

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
from galaxea_r1pro_0.service.roles import AbilityRole


def _state_result(data: float | None) -> CommandResult:
    """构造读取型 CommandResult：output.data = 夹爪实际开度。"""

    return CommandResult(
        command_id="",
        status=CommandStatus.SUCCEEDED,
        feedback=CommandFeedback("", CommandStatus.SUCCEEDED, 1.0, "completed"),
        output={"data": data},
    )


class _FakeTaskManager:
    def __init__(self) -> None:
        self.tasks: dict[int, object] = {}

    def register_tasks(self, tasks) -> None:
        self.tasks = dict(tasks)


class EndEffectorFlowTest(unittest.TestCase):
    def _sdk(self) -> R1ProSDK:
        return R1ProSDK.open(
            RobotProfile(
                robot_id="r1pro-ability-test",
                backend=BackendKind.REAL,
                firmware_profile="test-v1",
                options={"test_mode": True, "defer_commands": True},
            )
        )

    def test_single_gripper_task_full_flow(self) -> None:
        sdk = self._sdk()
        try:
            service = R1ProAbilityService("gripper_controller", sdk)
            with mock.patch.object(
                sdk.sensors,
                "get_gripper_state",
                return_value=_state_result(30.0),
            ):
                result = service.invoke(
                    "HandleSingleGripper",
                    {"invocation_id": "run-1", "hand": "left", "position": 30.0, "speed": 0.4, "tol": 1.0, "timeout": 2.0},
                )
            self.assertEqual(result["status"], "running")
            self.assertEqual(result["invocation_id"], "run-1")
            self.assertEqual(result["task_name"], "HandleSingleGripper")
            self.assertTrue(result["command_id"])

            # 幂等：同 invocation_id 再次调用返回同一记录，不启动第二个命令
            again = service.invoke(
                "HandleSingleGripper",
                {"invocation_id": "run-1", "hand": "left", "position": 30.0},
            )
            self.assertEqual(again["command_id"], result["command_id"])

            # 无效 hand → ValueError
            with self.assertRaises(ValueError):
                service.invoke("HandleSingleGripper", {"hand": "middle"})

            # 无反馈超时 → RuntimeError
            with mock.patch.object(
                sdk.sensors,
                "get_gripper_state",
                return_value=_state_result(None),
            ):
                with self.assertRaises(RuntimeError):
                    service.invoke(
                        "HandleSingleGripper",
                        {"hand": "left", "position": 30.0, "timeout": 0.05},
                    )
        finally:
            sdk.close()

    def test_both_gripper_and_execution_registry(self) -> None:
        sdk = self._sdk()
        try:
            service = R1ProAbilityService("gripper_controller", sdk)
            with mock.patch.object(
                sdk.sensors,
                "get_gripper_state",
                side_effect=lambda axis: _state_result(10.0 if axis == "left" else 90.0),
            ):
                both = service.invoke(
                    "HandleBothGripper",
                    {
                        "invocation_id": "run-both",
                        "left_position": 10.0,
                        "right_position": 90.0,
                        "tol": 1.0,
                        "timeout": 2.0,
                    },
                )
            self.assertEqual(both["status"], "running")
            self.assertEqual(both["task_name"], "HandleBothGripper")
            self.assertTrue(both["command_id"])  # 主记录：左夹爪

            execution = service.invoke("GetExecution", {"invocation_id": "run-both"})
            self.assertEqual(execution["command_id"], both["command_id"])

            stopped = service.invoke("StopExecution", {"invocation_id": "run-both", "reason": "user stop"})
            self.assertEqual(stopped["status"], "stopped")
        finally:
            sdk.close()


class EntrypointRegistrationTest(unittest.TestCase):
    def test_run_ability_registers_business_plus_execution_tasks(self) -> None:
        from galaxea_r1pro_0.service import entrypoint
        from galaxea_r1pro_0.tasks import ROLE_TASK_SPECS

        fake_service = mock.Mock()
        fake_service.run.return_value = None

        # 所有已注册角色：业务 Task（taskType 由 Task 类声明）+ 末尾追加 Get/StopExecution
        for role in sorted(ROLE_TASK_SPECS):
            task_manager = _FakeTaskManager()
            with mock.patch.object(entrypoint, "task_manager", task_manager), mock.patch.object(
                entrypoint, "AbilityService", mock.Mock(return_value=fake_service)
            ):
                entrypoint.run_ability(AbilityRole(role), f"{role}.Demo")

            business = [(cls.task_type, cls.name) for cls in ROLE_TASK_SPECS[role]]
            next_type = max((task_type for task_type, _ in business), default=-1) + 1
            expected = business + [(next_type, "GetExecution"), (next_type + 1, "StopExecution")]
            names = {task_type: task.task_name for task_type, task in task_manager.tasks.items()}
            self.assertEqual(names, dict(expected), msg=role)
        fake_service.run.assert_called()

if __name__ == "__main__":
    unittest.main()
