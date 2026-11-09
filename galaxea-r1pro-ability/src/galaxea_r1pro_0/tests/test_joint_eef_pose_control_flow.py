"""JointEEFPoseControl Task 全流程测试（test_mode SDK，mock 末端位姿反馈）。"""

from __future__ import annotations

import unittest
from unittest import mock

from scipy.spatial.transform import Rotation

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


def _ee_result(frame_id: str, pose: dict | None) -> CommandResult:
    """构造读取型 CommandResult：output.data = {"header": {"frame_id"}, "pose"}。"""

    return _result({"header": {"frame_id": frame_id}, "pose": pose})


def _pose(x: float = 0.4, y: float = 0.0, z: float = 0.1) -> dict:
    """零姿态目标位姿（roll=pitch=yaw=0 → 单位四元数）。"""

    return {
        "position": {"x": x, "y": y, "z": z},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    }


def _target_pose_input(x: float = 0.4, y: float = 0.0, z: float = 0.1) -> dict:
    """构造 *_target_pose 嵌套入参（零姿态），键与 utils._require_pose_values 要求一致。"""

    return {
        "target_x": x,
        "target_y": y,
        "target_z": z,
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
        "w": 1.0,
    }


def _force_result(left=None, right=None) -> CommandResult:
    """构造力反馈 CommandResult：left/right 为 (fx, fy, fz) 元组，None 表示该臂无数据。"""

    data: dict = {}
    if left is not None:
        data["left_arm"] = {"force_x": left[0], "force_y": left[1], "force_z": left[2]}
    if right is not None:
        data["right_arm"] = {"force_x": right[0], "force_y": right[1], "force_z": right[2]}
    return _result(data)


class JointEEFPoseControlFlowTest(unittest.TestCase):
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
            role_task_types("joint_eef_pose_control"),
            (
                (0, "control_single_arm"),
                (1, "control_both_arm"),
                (2, "force_align"),
                (3, "control_single_arm_trajectory"),
                (4, "control_both_arms_trajectory"),
            ),
        )

    def test_control_single_arm_full_flow(self) -> None:
        sdk = self._sdk()
        try:
            service = R1ProAbilityService("joint_eef_pose_control", sdk)
            with mock.patch.object(
                sdk.sensors,
                "get_ee_pose",
                return_value=_ee_result("torso_link4", _pose(0.4, 0.0, 0.1)),
            ):
                result = service.invoke(
                    "control_single_arm",
                    {
                        "invocation_id": "e1",
                        "axis": "left",
                        "_target_pose": _target_pose_input(0.4, 0.0, 0.1),
                        "pos_tol": 0.01,
                        "orient_tol": 0.01,
                        "timeout": 2.0,
                    },
                )
            self.assertEqual(result["status"], "running")
            self.assertEqual(result["task_name"], "control_single_arm")
            self.assertTrue(result["command_id"])

            # frame_id 不一致：用浮动基座单位逆变换后仍到位
            with mock.patch.object(
                sdk.sensors,
                "get_ee_pose",
                return_value=_ee_result("map", _pose(0.4, 0.0, 0.1)),
            ), mock.patch.object(
                sdk.state,
                "get_floating_base",
                return_value=_result({"position": (0.0, 0.0, 0.0), "orientation": (0.0, 0.0, 0.0, 1.0)}),
            ):
                transformed = service.invoke(
                    "control_single_arm",
                    {
                        "invocation_id": "e1b",
                        "axis": "left",
                        "_target_pose": _target_pose_input(0.4, 0.0, 0.1),
                        "pos_tol": 0.01,
                        "orient_tol": 0.01,
                        "timeout": 2.0,
                    },
                )
            self.assertEqual(transformed["status"], "running")

            # 无效 axis → ValueError
            with self.assertRaises(ValueError):
                service.invoke(
                    "control_single_arm",
                    {"axis": "middle", "_target_pose": _target_pose_input(0.4, 0.0, 0.1)},
                )

            # 缺少必填 _target_pose → ValueError
            with self.assertRaises(ValueError):
                service.invoke("control_single_arm", {"axis": "left"})

            # 无反馈超时 → RuntimeError
            with mock.patch.object(
                sdk.sensors,
                "get_ee_pose",
                return_value=_result(None),
            ):
                with self.assertRaises(RuntimeError):
                    service.invoke(
                        "control_single_arm",
                        {
                            "axis": "left",
                            "_target_pose": _target_pose_input(0.4, 0.0, 0.1),
                            "timeout": 0.05,
                        },
                    )
        finally:
            sdk.close()

    def test_control_both_arm_full_flow(self) -> None:
        sdk = self._sdk()
        try:
            service = R1ProAbilityService("joint_eef_pose_control", sdk)
            with mock.patch.object(
                sdk.sensors,
                "get_ee_pose",
                side_effect=lambda axis: _ee_result(
                    "1",
                    _pose(0.4, 0.3, 0.1) if axis == "left" else _pose(0.4, -0.3, 0.1),
                ),
            ):
                result = service.invoke(
                    "control_both_arm",
                    {
                        "invocation_id": "e2",
                        "left_target_pose": _target_pose_input(0.4, 0.3, 0.1),
                        "right_target_pose": _target_pose_input(0.4, -0.3, 0.1),
                        "left_frame_id": "1",
                        "right_frame_id": "1",
                        "left_pos_tol": 0.01,
                        "right_pos_tol": 0.01,
                        "timeout": 2.0,
                    },
                )
            self.assertEqual(result["status"], "running")
            self.assertEqual(result["task_name"], "control_both_arm")
            self.assertTrue(result["command_id"])
        finally:
            sdk.close()

    def _patch_motion(self, sdk, ee_state):
        """patch 末端位姿反馈与控制发布：发布目标即写回 ee_state（模拟真机到位）。

        ee_state: {"left": [x, y, z], "right": [x, y, z]}，返回 published 记录列表。
        """

        published: list[tuple[str, float, float, float]] = []
        orig_control = sdk.eef.control_single_arm

        def fake_control(*, x, y, z, roll=0.0, pitch=0.0, yaw=0.0,
                         frame_id="torso_link4", axis="left"):
            ee_state[axis] = [x, y, z]
            published.append((axis, x, y, z))
            return orig_control(
                x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw,
                frame_id=frame_id, axis=axis,
            )

        ee_patch = mock.patch.object(
            sdk.sensors, "get_ee_pose",
            side_effect=lambda axis: _ee_result(
                "torso_link4", _pose(*ee_state[axis])
            ),
        )
        control_patch = mock.patch.object(
            sdk.eef, "control_single_arm", side_effect=fake_control
        )
        return ee_patch, control_patch, published

    def test_force_align_contact_stop(self) -> None:
        sdk = self._sdk()
        try:
            service = R1ProAbilityService("joint_eef_pose_control", sdk)
            ee_state = {"left": [0.4, 0.3, 0.1], "right": [0.4, -0.3, 0.1]}
            ee_patch, control_patch, published = self._patch_motion(sdk, ee_state)
            # 基线 0N → 1N（未触阈，步进一次）→ 20N（触阈即停）
            forces = [
                _force_result(left=(0.0, 0.0, 0.0)),
                _force_result(left=(0.0, 1.0, 0.0)),
                _force_result(left=(0.0, 20.0, 0.0)),
            ]
            with ee_patch, control_patch, mock.patch.object(
                sdk.sensors, "get_force_data", side_effect=forces
            ):
                result = service.invoke(
                    "force_align",
                    {
                        "invocation_id": "fa1",
                        "axis": "left",
                        "move_coordinate": "y",
                        "force_coordinate": ["y"],
                        "force_threshold_n": 10.0,
                        "step_size_m": 0.01,
                        "loop_freq_s": 0.01,
                        "timeout_s": 2.0,
                        "direction": "negative",
                    },
                )
            self.assertEqual(result["task_name"], "force_align")
            self.assertTrue(result["command_id"])
            # 首次发布当前位姿 y=0.3；未触阈沿 negative 步进一次 y=0.29；触阈后停
            ys = [y for axis, _x, y, _z in published if axis == "left"]
            self.assertEqual(ys, [0.3, 0.29])
        finally:
            sdk.close()

    def test_force_align_base_link_translation_both_arms(self) -> None:
        for axis, direction, sign in (("left", "negative", -1), ("right", "positive", 1)):
            with self.subTest(axis=axis):
                sdk = self._sdk()
                try:
                    service = R1ProAbilityService("joint_eef_pose_control", sdk)
                    forces = [_force_result(**{axis: (0.0, y, 0.0)}) for y in (0.0, 1.0, 2.5)]
                    with mock.patch.object(sdk.sensors, "get_ee_pose", return_value=_ee_result(
                        "base_link", _pose(0.4116, 0.25 * -sign, 1.2679)
                    )), mock.patch.object(sdk.state, "get_floating_base", return_value=_result({
                        "position": (0.08, 0.0, -1.148), "orientation": (0.0, 0.0, 0.0, 1.0),
                    })), mock.patch.object(sdk.sensors, "get_force_data", side_effect=forces), \
                            mock.patch.object(sdk.eef, "control_single_arm", wraps=sdk.eef.control_single_arm) as control:
                        result = service.invoke("force_align", {
                            "axis": axis, "move_coordinate": "y", "force_coordinate": ["y"],
                            "force_threshold_n": 2.0, "step_size_m": 0.02, "direction": direction,
                            "loop_freq_s": 0.001, "timeout_s": 2.0,
                        })
                    self.assertTrue(result["command_id"])
                    self.assertEqual(control.call_count, 2)
                    for index, call in enumerate(control.call_args_list):
                        command = call.kwargs
                        self.assertEqual(command["frame_id"], "torso_link4")
                        self.assertEqual(command["axis"], axis)
                        self.assertAlmostEqual(command["x"], 0.4916)
                        self.assertAlmostEqual(command["z"], 0.1199)
                        self.assertAlmostEqual(command["y"], 0.25 * -sign + index * sign * 0.02)
                finally:
                    sdk.close()

    def test_force_align_rotates_position_and_orientation(self) -> None:
        sdk = self._sdk()
        try:
            service = R1ProAbilityService("joint_eef_pose_control", sdk)
            rotation = Rotation.from_euler("z", 90.0, degrees=True)
            pose = _pose(0.4, 0.2, 1.2)
            initial_rotation = Rotation.from_euler("x", 30.0, degrees=True)
            pose["orientation"] = dict(zip(("x", "y", "z", "w"), initial_rotation.as_quat()))
            with mock.patch.object(sdk.sensors, "get_ee_pose", return_value=_ee_result("/base_link", pose)), \
                    mock.patch.object(sdk.state, "get_floating_base", return_value=_result({
                        "position": (0.1, 0.0, -1.1), "orientation": tuple(rotation.as_quat()),
                    })), mock.patch.object(sdk.sensors, "get_force_data", side_effect=[
                        _force_result(left=(0.0, 0.0, 0.0)), _force_result(left=(0.0, 2.5, 0.0)),
                    ]), mock.patch.object(sdk.eef, "control_single_arm", wraps=sdk.eef.control_single_arm) as control:
                service.invoke("force_align", {"axis": "left", "force_threshold_n": 2.0})
            control.assert_called_once()
            command = control.call_args.kwargs
            self.assertEqual(command["frame_id"], "torso_link4")
            self.assertAlmostEqual(command["x"], -0.1)
            self.assertAlmostEqual(command["y"], 0.4)
            self.assertAlmostEqual(command["z"], 0.1)
            published_rotation = Rotation.from_euler(
                "YXZ", [command["roll"], command["pitch"], command["yaw"]], degrees=True,
            )
            self.assertAlmostEqual((published_rotation.inv() * rotation * initial_rotation).magnitude(), 0.0)
        finally:
            sdk.close()

    def test_force_align_torso_pose_needs_no_transform(self) -> None:
        sdk = self._sdk()
        try:
            service = R1ProAbilityService("joint_eef_pose_control", sdk)
            with mock.patch.object(sdk.sensors, "get_ee_pose", return_value=_ee_result("torso_link4", _pose())), \
                    mock.patch.object(sdk.state, "get_floating_base") as transform, \
                    mock.patch.object(sdk.sensors, "get_force_data", side_effect=[
                        _force_result(left=(0.0, 0.0, 0.0)), _force_result(left=(0.0, 2.5, 0.0)),
                    ]), mock.patch.object(sdk.eef, "control_single_arm", wraps=sdk.eef.control_single_arm) as control:
                service.invoke("force_align", {"axis": "left", "force_threshold_n": 2.0})
            transform.assert_not_called()
            self.assertEqual(control.call_args.kwargs["frame_id"], "torso_link4")
            self.assertAlmostEqual(control.call_args.kwargs["z"], 0.1)
        finally:
            sdk.close()

    def test_force_align_invalid_frames_and_transforms_never_publish(self) -> None:
        valid_transform = {"position": (0.0, 0.0, -1.1), "orientation": (0.0, 0.0, 0.0, 1.0)}
        cases = [
            ("missing frame", "", _pose(), valid_transform),
            ("unknown frame", "map", _pose(), valid_transform),
            ("missing transform", "base_link", _pose(), None),
            ("invalid translation", "base_link", _pose(), {**valid_transform, "position": (float("nan"), 0, 0)}),
            ("invalid rotation", "base_link", _pose(), {**valid_transform, "orientation": (0, 0, 0, 0)}),
            ("invalid dimensions", "base_link", _pose(), {**valid_transform, "position": (0, 0)}),
            ("invalid pose", "torso_link4", _pose(z=float("inf")), None),
            ("missing orientation", "torso_link4", {"position": _pose()["position"]}, None),
            ("missing pose", "torso_link4", None, None),
            ("zero quaternion", "torso_link4", {**_pose(), "orientation": dict.fromkeys(("x", "y", "z", "w"), 0)}, None),
        ]
        for axis in ("left", "right"):
            for label, frame, pose, transform in cases:
                with self.subTest(axis=axis, case=label):
                    sdk = self._sdk()
                    try:
                        service = R1ProAbilityService("joint_eef_pose_control", sdk)
                        with mock.patch.object(sdk.sensors, "get_ee_pose", return_value=_ee_result(frame, pose)), \
                                mock.patch.object(sdk.state, "get_floating_base", return_value=_result(transform)), \
                                mock.patch.object(sdk.eef, "control_single_arm") as control, \
                                mock.patch.object(sdk.sensors, "get_force_data") as force:
                            with self.assertRaises(RuntimeError):
                                service.invoke("force_align", {"axis": axis, "force_threshold_n": 2.0})
                        control.assert_not_called()
                        force.assert_not_called()
                    finally:
                        sdk.close()

    def test_force_align_both_rejected(self) -> None:
        sdk = self._sdk()
        try:
            service = R1ProAbilityService("joint_eef_pose_control", sdk)
            with self.assertRaises(ValueError):
                service.invoke(
                    "force_align",
                    {
                        "axis": "both",
                        "force_threshold_n": 10.0,
                        "step_size_m": 0.01,
                        "loop_freq_s": 0.01,
                        "timeout_s": 2.0,
                    },
                )
        finally:
            sdk.close()

    def test_force_align_timeout_and_validation(self) -> None:
        sdk = self._sdk()
        try:
            service = R1ProAbilityService("joint_eef_pose_control", sdk)
            ee_state = {"left": [0.4, 0.3, 0.1], "right": [0.4, -0.3, 0.1]}
            ee_patch, control_patch, _published = self._patch_motion(sdk, ee_state)
            base_params = {
                "axis": "left",
                "force_threshold_n": 10.0,
                "step_size_m": 0.01,
                "loop_freq_s": 0.01,
                "timeout_s": 0.05,
            }
            # 力反馈始终无变化 → 超时
            with ee_patch, control_patch, mock.patch.object(
                sdk.sensors, "get_force_data",
                return_value=_force_result(left=(0.0, 0.0, 0.0)),
            ):
                with self.assertRaises(RuntimeError):
                    service.invoke("force_align", base_params)

            # 参数校验
            with self.assertRaises(ValueError):
                service.invoke("force_align", {**base_params, "axis": "middle"})
            with self.assertRaises(ValueError):
                service.invoke("force_align", {**base_params, "direction": "sideways"})
            with self.assertRaises(ValueError):
                service.invoke("force_align", {**base_params, "move_coordinate": "w"})
            with self.assertRaises(ValueError):
                service.invoke(
                    "force_align",
                    {k: v for k, v in base_params.items() if k != "force_threshold_n"},
                )
        finally:
            sdk.close()

if __name__ == "__main__":
    unittest.main()
