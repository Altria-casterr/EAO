"""WooshMove Task 全流程测试（test_mode SDK，mock 定位/底盘反馈）。

入参契约为扁平参数：target_x / target_y / target_orientation（四元数，仅取 yaw 映射到 z 轴），
三者中恰好一个非零；与 ability.manifest.yaml 中 move_robot 的 params 保持一致。
"""

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


def _result(data) -> CommandResult:
    return CommandResult(
        command_id="",
        status=CommandStatus.SUCCEEDED,
        feedback=CommandFeedback("", CommandStatus.SUCCEEDED, 1.0, "completed"),
        output={"data": data},
    )


def _loc_data(x: float = 0.2, y: float = 0.0, yaw: float = 0.0) -> dict:
    return {
        "localization_available": True,
        "position_global": {"x": x, "y": y, "available": 1},
        "position_local": {"x": x, "y": y, "available": 1},
        "orientation_global": {"yaw": yaw, "available": 1},
        "orientation_local": {"yaw": yaw, "available": 1},
        "status_available": 1,
    }


def _feedback_data(speed: float = 1.0) -> dict:
    return {
        "feedback_received": True,
        "wheel_speeds": {"left_front": speed, "right_front": speed, "rear": 0.0},
        "wheel_angles": {"left_front": 0.0, "right_front": 0.0, "rear": 0.0},
    }


class WooshMoveFlowTest(unittest.TestCase):
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
            role_task_types("woosh_move"),
            (
                (0, "move_robot"),
                (1, "stop"),
            ),
        )

    def test_move_robot_loc_mode(self) -> None:
        """定位模式（x 轴）：首次记录初始定位，之后定位位移到达目标时到位。"""

        sdk = self._sdk()
        try:
            service = R1ProAbilityService("woosh_move", sdk)
            calls = {"n": 0}

            def _loc() -> CommandResult:
                calls["n"] += 1
                return _result(_loc_data(x=0.0 if calls["n"] == 1 else 0.2))

            with mock.patch.object(
                sdk.sensors,
                "get_localization",
                side_effect=_loc,
            ), mock.patch.object(
                sdk.woosh_move,
                "stop",
                wraps=sdk.woosh_move.stop,
            ) as stop_mock:
                result = service.invoke(
                    "move_robot",
                    {
                        "invocation_id": "m1",
                        "target_x": 0.2,
                        "speed": 0.1,
                        "tolerance": 0.02,
                        "timeout": 2.0,
                    },
                )
                # 到位后底盘只允许停止一次（由 finally 统一收口）
                stop_mock.assert_called_once()

            self.assertEqual(result["status"], "running")
            self.assertEqual(result["task_name"], "move_robot")
            self.assertTrue(result["command_id"])

            execution = service.invoke("GetExecution", {"invocation_id": "m1"})
            self.assertEqual(execution["command_id"], result["command_id"])
        finally:
            sdk.close()

    def test_move_robot_y_axis_loc_mode(self) -> None:
        """定位模式（y 轴）：左右方向相对位移同样由定位闭环判定到位。"""

        sdk = self._sdk()
        try:
            service = R1ProAbilityService("woosh_move", sdk)
            calls = {"n": 0}

            def _loc() -> CommandResult:
                calls["n"] += 1
                return _result(_loc_data(x=0.0, y=0.0 if calls["n"] == 1 else 0.2))

            with mock.patch.object(
                sdk.sensors,
                "get_localization",
                side_effect=_loc,
            ):
                result = service.invoke(
                    "move_robot",
                    {"target_y": 0.2, "speed": 0.1, "tolerance": 0.02, "timeout": 2.0},
                )
            self.assertEqual(result["status"], "running")
            self.assertTrue(result["command_id"])
        finally:
            sdk.close()

    def test_move_robot_z_yaw_axis(self) -> None:
        """定位模式（z 轴）：orientation 的 yaw=30° 映射为 z 轴自转并到位。"""

        sdk = self._sdk()
        try:
            service = R1ProAbilityService("woosh_move", sdk)
            calls = {"n": 0}

            def _loc() -> CommandResult:
                calls["n"] += 1
                return _result(_loc_data(x=0.0, y=0.0, yaw=0.0 if calls["n"] == 1 else 30.0))

            with mock.patch.object(
                sdk.sensors,
                "get_localization",
                side_effect=_loc,
            ):
                # 四元数（xyzw）：绕 Z 轴 30° → [0, 0, sin15°, cos15°]
                result = service.invoke(
                    "move_robot",
                    {
                        "target_orientation": [0.0, 0.0, 0.2588, 0.9659],
                        "speed": 5.0,
                        "tolerance": 0.5,
                        "timeout": 2.0,
                    },
                )
            self.assertEqual(result["status"], "running")
            self.assertTrue(result["command_id"])
        finally:
            sdk.close()

    def test_move_robot_integration_mode(self) -> None:
        """积分模式：无定位反馈时用底盘反馈积分判断到位。"""

        sdk = self._sdk()
        try:
            service = R1ProAbilityService("woosh_move", sdk)
            with mock.patch.object(
                sdk.sensors,
                "get_localization",
                return_value=_result(None),
            ), mock.patch.object(
                sdk.sensors,
                "get_chassis_feedback",
                # 反馈轮速与指令速度一致（0.1 m/s），避免积分单步位移超过目标
                return_value=_result(_feedback_data(0.1)),
            ):
                result = service.invoke(
                    "move_robot",
                    {
                        "invocation_id": "m1b",
                        "target_x": 0.2,
                        "speed": 0.1,
                        "tolerance": 0.05,
                        "timeout": 4.0,
                    },
                )
            self.assertEqual(result["status"], "running")
            self.assertEqual(result["task_name"], "move_robot")
            self.assertTrue(result["command_id"])
        finally:
            sdk.close()

    def test_move_robot_timeout_raises_and_stops(self) -> None:
        """无反馈且底盘不动 → 超时：先 stop 再抛 RuntimeError。"""

        sdk = self._sdk()
        try:
            service = R1ProAbilityService("woosh_move", sdk)
            with mock.patch.object(
                sdk.sensors,
                "get_localization",
                return_value=_result(None),
            ), mock.patch.object(
                sdk.sensors,
                "get_chassis_feedback",
                return_value=_result(_feedback_data(0.0)),
            ), mock.patch.object(
                sdk.woosh_move,
                "stop",
                wraps=sdk.woosh_move.stop,
            ) as stop_mock:
                with self.assertRaises(RuntimeError) as ctx:
                    service.invoke(
                        "move_robot",
                        {"target_x": 0.2, "timeout": 0.2},
                    )
                self.assertIn("超时", str(ctx.exception))
                stop_mock.assert_called_once()
        finally:
            sdk.close()

    def test_move_robot_invalid_params(self) -> None:
        """零位移、多轴同时非零、四元数元素数不对，都应被拒绝。"""

        sdk = self._sdk()
        try:
            service = R1ProAbilityService("woosh_move", sdk)
            # 三个方向全零
            with self.assertRaises(ValueError):
                service.invoke("move_robot", {})
            # x 与 y 同时非零（多轴）
            with self.assertRaises(ValueError):
                service.invoke("move_robot", {"target_x": 0.2, "target_y": 0.1})
            # x 与 yaw 同时非零（多轴）
            with self.assertRaises(ValueError):
                service.invoke(
                    "move_robot",
                    {"target_x": 0.2, "target_orientation": [0.0, 0.0, 0.2588, 0.9659]},
                )
            # target_orientation 元素数不足 4
            with self.assertRaises(ValueError):
                service.invoke(
                    "move_robot",
                    {"target_x": 0.2, "target_orientation": [0.0, 0.0, 1.0]},
                )
        finally:
            sdk.close()

    def test_stop_registers_command(self) -> None:
        sdk = self._sdk()
        try:
            service = R1ProAbilityService("woosh_move", sdk)
            result = service.invoke("stop", {"invocation_id": "m2"})
            self.assertEqual(result["status"], "running")
            self.assertEqual(result["task_name"], "stop")
            self.assertTrue(result["command_id"])

            execution = service.invoke("GetExecution", {"invocation_id": "m2"})
            self.assertEqual(execution["command_id"], result["command_id"])
        finally:
            sdk.close()


if __name__ == "__main__":
    unittest.main()
