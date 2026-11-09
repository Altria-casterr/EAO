import unittest

from galaxea_r1pro import (
    BackendKind,
    BackendUnavailableError,
    CommandStatus,
    R1ProSDK,
    RobotProfile,
)
from galaxea_r1pro.galaxea_r1pro_0.backends import create_backend


class BackendConformanceTest(unittest.TestCase):
    def _profile(self, backend: BackendKind, **options) -> RobotProfile:
        return RobotProfile(
            robot_id=f"r1pro-{backend.value}",
            backend=backend,
            firmware_profile="test-firmware",
            options={"test_mode": True, **options},
        )

    def test_real_backend_exposes_expected_resource_surface(self) -> None:
        resources = []
        with R1ProSDK.open(self._profile(BackendKind.REAL)) as sdk:
            resources = list(sdk.state.capabilities().resources)
            state = sdk.state.snapshot()
            self.assertTrue(state.connected)
            self.assertEqual(state.robot_id, "r1pro-real")
            self.assertEqual(sdk.sensors.capture_rgb("head_camera").encoding, "rgb8")
        self.assertIn("base", resources)

    def test_unavailable_backend_error_carries_structured_details(self) -> None:
        profile = RobotProfile(
            robot_id="r1pro-mujoco",
            backend=BackendKind.MUJOCO,
            firmware_profile="production-firmware",
            options={},
        )
        with self.assertRaises(BackendUnavailableError) as ctx:
            R1ProSDK.open(profile)

        error = ctx.exception
        self.assertEqual(error.backend, "mujoco")
        self.assertEqual(error.robot_id, "r1pro-mujoco")
        self.assertEqual(error.reason, BackendUnavailableError.NOT_IMPLEMENTED)
        self.assertEqual(error.operation, "create_backend")
        self.assertIn("mujoco", str(error))
        self.assertIn("not_implemented", str(error))
        self.assertEqual(error.to_dict()["hint"], "当前仅支持 real 后端；mujoco/isaac 后端尚未接入")

    def test_unconnected_backend_error_carries_not_connected_reason(self) -> None:
        profile = RobotProfile(
            robot_id="r1pro-real",
            backend=BackendKind.REAL,
            firmware_profile="test-firmware",
            options={"test_mode": True},
        )
        sdk = R1ProSDK(profile, create_backend(profile))
        try:
            with self.assertRaises(BackendUnavailableError) as ctx:
                sdk.joint.control_single_arm_joint(arm_side="left", positions=[0.1], joint_names=["j1"])

            error = ctx.exception
            self.assertEqual(error.backend, "real")
            self.assertEqual(error.robot_id, "r1pro-real")
            self.assertEqual(error.reason, BackendUnavailableError.NOT_CONNECTED)
            self.assertEqual(error.operation, "start_command")
            self.assertIn("请先调用 R1ProSDK.open()", str(error))
        finally:
            sdk.close()

    def test_real_production_connect_requires_ros_transport(self) -> None:
        # 生产模式 connect 必须走 transport 层并明确失败（缺 roslibpy 或连接失败），
        # 不能伪装成功；reason 取决于部署环境，这里只校验异常类型与归属后端。
        profile = RobotProfile(
            robot_id="r1pro-real",
            backend=BackendKind.REAL,
            firmware_profile="production-firmware",
            endpoint="192.168.1.100:9091",
            options={},
        )
        with self.assertRaises(BackendUnavailableError) as ctx:
            R1ProSDK.open(profile)

        error = ctx.exception
        self.assertEqual(error.backend, "real")
        self.assertEqual(error.operation, "RealBackend.connect")

    def test_unimplemented_production_backends_never_simulate_success(self) -> None:
        # real 未接入时不允许在非 test_mode 下伪装成功；mujoco/isaac 后端
        # 已删除，选择这些 backend 同样必须明确失败而不是假装执行。
        for backend in BackendKind:
            profile = RobotProfile(
                robot_id=f"r1pro-{backend.value}",
                backend=backend,
                firmware_profile="production-firmware",
                endpoint="192.168.1.100:9091" if backend is BackendKind.REAL else None,
                options={},
            )
            with self.subTest(backend=backend.value):
                with self.assertRaises(BackendUnavailableError):
                    R1ProSDK.open(profile)

    def test_command_stop_reaches_hold(self) -> None:
        with R1ProSDK.open(self._profile(BackendKind.REAL, defer_commands=True)) as sdk:
            command = sdk.joint.control_single_arm_joint(arm_side="left", positions=[0.1], joint_names=["j1"])
            self.assertIs(sdk.command.get(command.command_id).status, CommandStatus.RUNNING)

            result = sdk.command.stop(command.command_id, "test stop")

            self.assertIs(result.status, CommandStatus.CANCELLED)
            self.assertIn("joint_control", sdk.state.snapshot().held_resources)

    def test_unconfirmed_stop_is_not_reported_as_cancelled(self) -> None:
        with R1ProSDK.open(
            self._profile(BackendKind.REAL, defer_commands=True, stop_confirmed=False)
        ) as sdk:
            command = sdk.joint.control_single_arm_joint(arm_side="left", positions=[0.1], joint_names=["j1"])
            result = sdk.command.stop(command.command_id, "transport lost")

            self.assertIs(result.status, CommandStatus.UNKNOWN)
            self.assertNotIn("joint_control", sdk.state.snapshot().held_resources)

    def test_memory_mode_read_without_feedback_is_succeeded(self) -> None:
        with R1ProSDK.open(self._profile(BackendKind.REAL)) as sdk:
            result = sdk.sensors.get_force_data()
            self.assertIs(result.status, CommandStatus.SUCCEEDED)

    def test_firmware_profile_does_not_change_public_clients(self) -> None:
        first = R1ProSDK.open(self._profile(BackendKind.REAL))
        second = R1ProSDK.open(
            RobotProfile(
                robot_id="r1pro-real",
                backend=BackendKind.REAL,
                firmware_profile="next-firmware",
                options={"test_mode": True},
            )
        )
        try:
            self.assertEqual(type(first.joint), type(second.joint))
            self.assertEqual(type(first.gripper), type(second.gripper))
            self.assertEqual(type(first.eef), type(second.eef))
        finally:
            first.close()
            second.close()


if __name__ == "__main__":
    unittest.main()
