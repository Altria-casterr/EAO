"""RealBackend 命令转发与 Galaxea_R1Pro 能力函数映射测试。"""

import unittest
from unittest import mock

from galaxea_r1pro import BackendKind, BackendUnavailableError, CommandNotFoundError, MessageNotFoundError, R1ProSDK, RobotProfile
from galaxea_r1pro.galaxea_r1pro_0.backends.real import RealBackend
from galaxea_r1pro.galaxea_r1pro_0.transport.process_pub import (
    GALAXEA_R1PRO_PUBLISH_COMMANDS,
    build_gripper_message,
    build_joint_state_message,
    build_pose_stamped_message,
    build_torso_message,
    build_twist_stamped_message,
)
from galaxea_r1pro.galaxea_r1pro_0.transport import process_sub
from galaxea_r1pro.galaxea_r1pro_0.transport.process_sub import GALAXEA_R1PRO_GET_COMMANDS
from galaxea_r1pro.galaxea_r1pro_0.utils.types import CommandStatus, Pose
from galaxea_r1pro.galaxea_r1pro_0.woosh_move import WooshMove
from galaxea_r1pro.galaxea_r1pro_0.woosh_navigation import WooshNavigation


class _FakePublisher:
    def __init__(self) -> None:
        self.published: list[dict] = []

    def publish(self, message: dict) -> None:
        self.published.append(message)


class _FakeAction:
    def __init__(self) -> None:
        self.sent: list[tuple[dict, callable, callable, callable]] = []
        self.cancelled: list[str] = []

    def send_goal(self, goal, on_result, on_feedback, on_error) -> str:
        self.sent.append((goal, on_result, on_feedback, on_error))
        return "goal-1"

    def cancel_goal(self, goal_id: str) -> None:
        self.cancelled.append(goal_id)


class _FakeRosClient:
    def __init__(self, connected: bool = True) -> None:
        self.is_connected = connected


class _FakeRosConnection:
    def __init__(self, publishers: dict, actions: dict | None = None, connected: bool = True) -> None:
        self.publishers = publishers
        self.actions = actions or {}
        self.latest_messages: dict[str, dict] = {}
        self.navigation_state: dict = {}
        self.connect_flag = connected
        self.client = _FakeRosClient(connected)

    def register_topics(self, registry=None) -> "_FakeRosConnection":
        return self

    def close(self) -> None:
        self.client.is_connected = False


def _real_backend(
    connected: bool = True,
    publishers: dict | None = None,
    actions: dict | None = None,
    options: dict | None = None,
) -> RealBackend:
    profile = RobotProfile(
        robot_id="r1pro-x",
        backend=BackendKind.REAL,
        firmware_profile="test-v1",
        endpoint="192.168.1.100:9091",
        options=options or {},
    )
    backend = RealBackend(profile)
    if connected:
        backend._connected = True
        backend.ros_connection = _FakeRosConnection(publishers or {}, actions)
    return backend


class GalaxeaCommandMapTest(unittest.TestCase):
    def test_ability_functions_are_mapped(self) -> None:
        for key in [
            ("joint_control", "control_single_arm_joint"),
            ("gripper_control", "control_single_gripper"),
            ("torso_control", "control_torso_joints"),
            ("eef_control", "control_single_arm"),
            ("woosh_move", "move_robot"),
            ("woosh_move", "unlock_brake"),
            ("woosh_navigation", "navigate_to_map_pose"),
        ]:
            self.assertIn(key, GALAXEA_R1PRO_PUBLISH_COMMANDS)

    def test_get_commands_are_separate_from_publish_commands(self) -> None:
        for key in [
            ("joint_control", "get_left_joint_state"),
            ("gripper_control", "get_right_gripper_state"),
            ("force_feedback", "get_force_data"),
            ("eef_control", "get_left_ee_pose"),
            ("eef_control", "get_floating_base"),
            ("woosh_move", "get_chassis_feedback"),
            ("woosh_move", "get_localization"),
            ("woosh_navigation", "get_current_status"),
            ("coordinate_transformation", "get_transformer_martix"),
        ]:
            self.assertIn(key, GALAXEA_R1PRO_GET_COMMANDS)
            self.assertNotIn(key, GALAXEA_R1PRO_PUBLISH_COMMANDS)

    def test_navigation_is_action_kind(self) -> None:
        self.assertEqual(
            GALAXEA_R1PRO_PUBLISH_COMMANDS[("woosh_navigation", "navigate_to_map_pose")]["kind"],
            "action",
        )

    def test_build_joint_state_message(self) -> None:
        message = build_joint_state_message(
            {"names": ["a", "b"], "positions": [0.1, 0.2], "velocities": [0.0, 0.0], "efforts": [0.0, 0.0]}
        )
        self.assertEqual(message["name"], ["a", "b"])
        self.assertEqual(message["position"], [0.1, 0.2])

    def test_build_pose_stamped_message(self) -> None:
        pose = Pose(frame_id="map", position=(1.0, 2.0, 3.0), orientation_xyzw=(0.0, 0.0, 0.0, 1.0))
        message = build_pose_stamped_message({"target": pose})
        self.assertEqual(message["pose"]["position"]["x"], 1.0)
        self.assertEqual(message["pose"]["orientation"]["w"], 1.0)

    def test_build_gripper_message(self) -> None:
        message = build_gripper_message({"side": "right", "position": 0.08})
        self.assertEqual(message["name"], ["gripper_right_joint"])
        self.assertEqual(message["position"], [0.08])

    def test_build_torso_message(self) -> None:
        message = build_torso_message(
            {
                "names": ["torso_joint_1", "torso_joint_2", "torso_joint_3", "torso_joint_4"],
                "positions": [0.1, 0.2, 0.3, 0.4],
                "velocities": [0.5, 0.5, 0.5, 0.5],
                "efforts": [0.1, 0.2, 0.3, 0.4],
            }
        )
        self.assertEqual(
            message["name"],
            ["torso_joint_1", "torso_joint_2", "torso_joint_3", "torso_joint_4"],
        )
        self.assertEqual(message["position"], [0.1, 0.2, 0.3, 0.4])
        self.assertEqual(message["velocity"], [0.5, 0.5, 0.5, 0.5])
        self.assertEqual(message["effort"], [0.1, 0.2, 0.3, 0.4])

    def test_build_twist_stamped_message(self) -> None:
        message = build_twist_stamped_message({"dx": 0.1, "dy": 0.0, "dyaw": 90.0})
        self.assertEqual(message["twist"]["linear"]["x"], 0.1)
        self.assertAlmostEqual(message["twist"]["angular"]["z"], 1.5708, places=4)


class RealBackendCommandTest(unittest.TestCase):
    def test_start_command_publishes_to_mapped_topic(self) -> None:
        topic = "/motion_target/target_pose_arm_left"
        publisher = _FakePublisher()
        backend = _real_backend(publishers={topic: publisher})

        handle = backend.start_command(
            "eef_control",
            "control_single_arm",
            {
                "axis": "left",
                "target": Pose(frame_id="map", position=(0.4, 0.3, 0.1), orientation_xyzw=(0, 0, 0, 1)),
            },
        )

        self.assertEqual(handle.resource, "eef_control")
        self.assertEqual(len(publisher.published), 1)
        self.assertIn("pose", publisher.published[0])

    def test_action_command_sends_goal(self) -> None:
        action = _FakeAction()
        backend = _real_backend(
            publishers={},
            actions={"/system_manager/navi/action": action},
        )

        handle = backend.start_command(
            "woosh_navigation", "navigate_to_map_pose",
            {"x": 1.0, "y": 2.0, "yaw": 0.0, "frame_id": "map", "target_point_type": 0},
        )

        self.assertEqual(handle.resource, "woosh_navigation")
        self.assertEqual(len(action.sent), 1)
        self.assertEqual(action.sent[0][0]["pose"]["position"]["x"], 1.0)

        # command_id 查询只反映登记状态（deferred=False → SUCCEEDED 快照）
        result = backend.get_command(handle.command_id)
        self.assertEqual(result.status, CommandStatus.SUCCEEDED)

    def test_action_command_resolves_after_result_callback(self) -> None:
        action = _FakeAction()
        backend = _real_backend(
            publishers={},
            actions={"/system_manager/navi/action": action},
            options={"defer_commands": True},
        )

        handle = backend.start_command(
            "woosh_navigation", "navigate_to_map_pose",
            {"x": 1.0, "y": 2.0, "yaw": 0.0, "frame_id": "map", "target_point_type": 0},
        )
        self.assertEqual(backend.get_command(handle.command_id).status, CommandStatus.RUNNING)

        action.sent[0][1]({"success": True})
        result = backend.get_command(handle.command_id)
        self.assertEqual(result.status, CommandStatus.SUCCEEDED)
        self.assertEqual(result.output["navigation"], {"success": True})

    def test_action_command_fails_on_error_callback(self) -> None:
        action = _FakeAction()
        backend = _real_backend(
            publishers={},
            actions={"/system_manager/navi/action": action},
            options={"defer_commands": True},
        )

        handle = backend.start_command(
            "woosh_navigation", "navigate_to_map_pose",
            {"x": 1.0, "y": 2.0, "yaw": 0.0, "frame_id": "map", "target_point_type": 0},
        )
        action.sent[0][3]({"error": "nav timeout"})
        result = backend.get_command(handle.command_id)
        self.assertEqual(result.status, CommandStatus.FAILED)
        self.assertIn("nav timeout", result.output["error"])

    def test_action_command_fails_when_result_success_false(self) -> None:
        action = _FakeAction()
        backend = _real_backend(
            publishers={},
            actions={"/system_manager/navi/action": action},
            options={"defer_commands": True},
        )

        handle = backend.start_command(
            "woosh_navigation", "navigate_to_map_pose",
            {"x": 1.0, "y": 2.0, "yaw": 0.0, "frame_id": "map", "target_point_type": 0},
        )
        action.sent[0][1]({"success": False, "message": "goal blocked"})
        result = backend.get_command(handle.command_id)
        self.assertEqual(result.status, CommandStatus.FAILED)
        self.assertEqual(result.output["navigation"]["success"], False)

    def test_stop_command_cancels_action_goal(self) -> None:
        action = _FakeAction()
        backend = _real_backend(
            publishers={},
            actions={"/system_manager/navi/action": action},
            options={"defer_commands": True},
        )

        handle = backend.start_command(
            "woosh_navigation", "navigate_to_map_pose",
            {"x": 1.0, "y": 2.0, "yaw": 0.0, "frame_id": "map", "target_point_type": 0},
        )
        backend.stop_command(handle.command_id, "user stop")
        self.assertEqual(action.cancelled, ["goal-1"])
        self.assertFalse(backend.ros_connection.navigation_state["is_navigating"])

    def test_action_command_exposes_navigation_state(self) -> None:
        action = _FakeAction()
        backend = _real_backend(
            publishers={},
            actions={"/system_manager/navi/action": action},
        )

        backend.start_command(
            "woosh_navigation", "navigate_to_map_pose",
            {"x": 1.0, "y": 2.0, "yaw": 0.0, "frame_id": "map", "target_point_type": 0},
        )
        status = backend.get_command(resource="woosh_navigation", operation="get_current_status", target={})
        self.assertTrue(status.output["data"]["is_navigating"])
        self.assertIsNone(status.output["data"]["error"])

    def test_woosh_move_unlock_brake_publishes_false(self) -> None:
        brake = _FakePublisher()
        backend = _real_backend(publishers={"/motion_target/brake_mode": brake})

        handle = WooshMove(backend)._unlock_brake()
        self.assertEqual(handle.resource, "woosh_move")
        self.assertEqual(len(brake.published), 10)
        self.assertTrue(all(message == {"data": False} for message in brake.published))

    def test_woosh_navigation_unlock_brake_publishes_false(self) -> None:
        brake = _FakePublisher()
        backend = _real_backend(publishers={"/motion_target/brake_mode": brake})

        handle = WooshNavigation(backend)._unlock_brake()
        self.assertEqual(handle.resource, "woosh_move")
        self.assertEqual(len(brake.published), 10)
        self.assertTrue(all(message == {"data": False} for message in brake.published))

    def test_woosh_navigation_stop_publishes_zero_twist(self) -> None:
        speed = _FakePublisher()
        backend = _real_backend(publishers={"/motion_target/target_speed_chassis": speed})
        client = WooshNavigation(backend)

        client.stop()
        self.assertEqual(len(speed.published), 10)
        self.assertEqual(speed.published[0]["twist"]["linear"], {"x": 0.0, "y": 0.0, "z": 0.0})
        self.assertEqual(speed.published[0]["twist"]["angular"]["z"], 0.0)

    def test_woosh_navigation_stop_sets_emergency_stop(self) -> None:
        speed = _FakePublisher()
        backend = _real_backend(publishers={"/motion_target/target_speed_chassis": speed})
        client = WooshNavigation(backend)

        client.stop()
        self.assertTrue(client._emergency_stop)
        self.assertEqual(len(speed.published), 10)

    def test_woosh_navigation_emergency_stop_aborts(self) -> None:
        brake = _FakePublisher()
        action = _FakeAction()
        backend = _real_backend(
            publishers={"/motion_target/brake_mode": brake},
            actions={"/system_manager/navi/action": action},
        )
        client = WooshNavigation(backend)
        client._emergency_stop = True

        result = client.navigate_to_map_pose(x=1.0, y=2.0, timeout=5.0)
        self.assertEqual(result.status, CommandStatus.CANCELLED)
        self.assertEqual(action.cancelled, ["goal-1"])

    def test_woosh_navigation_cancel_task(self) -> None:
        action = _FakeAction()
        backend = _real_backend(publishers={}, actions={"/system_manager/navi/action": action})
        client = WooshNavigation(backend)

        handle = backend.start_command(
            "woosh_navigation", "navigate_to_map_pose",
            {"x": 1.0, "y": 2.0, "yaw": 0.0, "frame_id": "map", "target_point_type": 0},
        )
        result = client._cancel_navigation_task(handle, "test cancel")
        self.assertEqual(result.status, CommandStatus.CANCELLED)
        self.assertEqual(action.cancelled, ["goal-1"])

    def test_action_command_rejects_concurrent_navigation(self) -> None:
        action = _FakeAction()
        backend = _real_backend(publishers={}, actions={"/system_manager/navi/action": action})

        backend.start_command(
            "woosh_navigation", "navigate_to_map_pose",
            {"x": 1.0, "y": 2.0, "yaw": 0.0, "frame_id": "map", "target_point_type": 0},
        )
        with self.assertRaises(BackendUnavailableError) as ctx:
            backend.start_command(
                "woosh_navigation", "navigate_to_map_pose",
                {"x": 3.0, "y": 4.0, "yaw": 0.0, "frame_id": "map", "target_point_type": 0},
            )
        self.assertEqual(ctx.exception.reason, BackendUnavailableError.NAVIGATION_BUSY)
        self.assertEqual(len(action.sent), 1)

    def test_action_command_handles_send_goal_none(self) -> None:
        class _NoneAction:
            def send_goal(self, goal, on_result, on_feedback, on_error):
                return None

        backend = _real_backend(publishers={}, actions={"/system_manager/navi/action": _NoneAction()})
        handle = backend.start_command(
            "woosh_navigation", "navigate_to_map_pose",
            {"x": 1.0, "y": 2.0, "yaw": 0.0, "frame_id": "map", "target_point_type": 0},
        )
        result = backend.get_command(handle.command_id)
        self.assertEqual(result.status, CommandStatus.FAILED)
        self.assertIn("send_goal 返回 None", result.output["error"])

    def test_sdk_open_singleton_reuses_same_endpoint(self) -> None:
        profile = RobotProfile(
            robot_id="r1pro-x", backend=BackendKind.REAL, firmware_profile="test-v1",
            endpoint="192.168.1.100:9091", options={},
        )
        fake = _FakeRosConnection({})
        with mock.patch("galaxea_r1pro.galaxea_r1pro_0.backends.real.ConnectRos", return_value=fake):
            sdk1 = R1ProSDK.open(profile)
            sdk2 = R1ProSDK.open(profile)

        self.assertIs(sdk1, sdk2)
        self.assertTrue(sdk2.is_connected())

        # 引用计数 2→1：仍未断开
        sdk1.close()
        self.assertTrue(sdk2.is_connected())
        # 归零：才真正断开并移除缓存
        sdk2.close()
        self.assertFalse(sdk2.is_connected())
        self.assertNotIn("192.168.1.100:9091", R1ProSDK._instances)

    def test_sdk_open_creates_separate_instance_for_other_endpoint(self) -> None:
        profile_a = RobotProfile(
            robot_id="r1pro-a", backend=BackendKind.REAL, firmware_profile="test-v1",
            endpoint="192.168.1.100:9091", options={},
        )
        profile_b = RobotProfile(
            robot_id="r1pro-b", backend=BackendKind.REAL, firmware_profile="test-v1",
            endpoint="192.168.1.101:9091", options={},
        )
        fake_a = _FakeRosConnection({})
        fake_b = _FakeRosConnection({})
        with mock.patch(
            "galaxea_r1pro.galaxea_r1pro_0.backends.real.ConnectRos",
            side_effect=[fake_a, fake_b],
        ):
            sdk_a = R1ProSDK.open(profile_a)
            sdk_b = R1ProSDK.open(profile_b)

        self.assertIsNot(sdk_a, sdk_b)
        sdk_a.close()
        sdk_b.close()

    def test_connect_reuses_alive_connection(self) -> None:
        backend = _real_backend(publishers={})
        connection = backend.ros_connection

        backend.connect()
        self.assertIs(backend.ros_connection, connection)
        self.assertTrue(backend.is_connected())

    def test_connect_recovers_after_ros_drop(self) -> None:
        backend = _real_backend(publishers={})
        old_connection = backend.ros_connection
        old_connection.client.is_connected = False  # 模拟 rosbridge 断连

        new_connection = _FakeRosConnection({})
        with mock.patch("galaxea_r1pro.galaxea_r1pro_0.backends.real.ConnectRos", return_value=new_connection):
            backend.connect()

        self.assertIsNot(backend.ros_connection, old_connection)
        self.assertFalse(old_connection.client.is_connected)  # 旧连接已清理
        self.assertTrue(backend.is_connected())

    def test_is_connected_reflects_ros_state(self) -> None:
        backend = _real_backend(publishers={})
        self.assertTrue(backend.is_connected())
        backend.ros_connection.client.is_connected = False
        self.assertFalse(backend.is_connected())

    def test_unmapped_operation_fails_clearly(self) -> None:
        backend = _real_backend()
        with self.assertRaises(BackendUnavailableError) as ctx:
            backend.start_command("base", "navigate_to_pose", {"x": 1.0, "y": 2.0, "yaw": 0.0})
        self.assertEqual(ctx.exception.reason, BackendUnavailableError.NOT_IMPLEMENTED)

    def test_unconnected_start_command_fails(self) -> None:
        backend = _real_backend(connected=False)
        with self.assertRaises(BackendUnavailableError) as ctx:
            backend.start_command(
                "joint_control", "control_single_arm_joint",
                {"arm_side": "left", "names": ["j1"], "positions": [0.1]},
            )
        self.assertEqual(ctx.exception.reason, BackendUnavailableError.NOT_CONNECTED)

    def test_get_command_returns_registered_status_snapshot(self) -> None:
        topic = "/motion_target/target_pose_arm_left"
        backend = _real_backend(publishers={topic: _FakePublisher()})

        handle = backend.start_command(
            "eef_control", "control_single_arm",
            {"axis": "left", "target": Pose(frame_id="map", position=(0, 0, 0), orientation_xyzw=(0, 0, 0, 1))},
        )
        result = backend.get_command(handle.command_id)

        # 只返回登记时的状态快照（deferred=False → SUCCEEDED），不依赖反馈话题
        self.assertEqual(result.status, CommandStatus.SUCCEEDED)
        self.assertIn("target", result.output)

    def test_get_command_reads_feedback(self) -> None:
        backend = _real_backend(publishers={})
        backend.ros_connection.latest_messages["/hdas/feedback_left_arm_wrench"] = {
            "wrench": {"force": {"x": 1.0}, "torque": {"x": 0.0}}
        }
        backend.ros_connection.latest_messages["/hdas/feedback_right_arm_wrench"] = {
            "wrench": {"force": {"x": -1.0}, "torque": {"x": 0.0}}
        }

        result = backend.get_command(resource="force_feedback", operation="get_force_data")

        self.assertEqual(result.status, CommandStatus.SUCCEEDED)
        self.assertIsNotNone(result.output["data"]["left_arm"])

    def test_command_not_found_carries_structured_details(self) -> None:
        backend = _real_backend()
        with self.assertRaises(CommandNotFoundError) as ctx:
            backend.get_command(command_id="no-such-command")
        error = ctx.exception
        self.assertEqual(error.command_id, "no-such-command")
        self.assertEqual(error.reason, CommandNotFoundError.COMMAND_NOT_FOUND)
        self.assertIn("no-such-command", str(error))
        self.assertEqual(error.to_dict()["command_id"], "no-such-command")

    def test_get_command_reads_unmapped_operation_fails(self) -> None:
        backend = _real_backend()
        with self.assertRaises(BackendUnavailableError) as ctx:
            backend.get_command(resource="joint_control", operation="control_single_arm_joint")
        self.assertEqual(ctx.exception.reason, BackendUnavailableError.NOT_IMPLEMENTED)

    def test_stop_command_marks_cancelled(self) -> None:
        topic = "/motion_target/target_pose_arm_left"
        backend = _real_backend(publishers={topic: _FakePublisher()})

        handle = backend.start_command(
            "eef_control", "control_single_arm",
            {"axis": "left", "target": Pose(frame_id="map", position=(0, 0, 0), orientation_xyzw=(0, 0, 0, 1))},
        )
        result = backend.stop_command(handle.command_id, "user stop")

        self.assertEqual(result.status, CommandStatus.CANCELLED)

    def test_stop_command_unconfirmed_returns_unknown(self) -> None:
        topic = "/motion_target/target_pose_arm_left"
        backend = _real_backend(
            publishers={topic: _FakePublisher(), "/motion_target/brake_mode": _FakePublisher()},
            options={"stop_confirmed": False},
        )

        handle = backend.start_command(
            "eef_control", "control_single_arm",
            {"axis": "left", "target": Pose(frame_id="map", position=(0, 0, 0), orientation_xyzw=(0, 0, 0, 1))},
        )
        result = backend.stop_command(handle.command_id, "transport lost")

        self.assertEqual(result.status, CommandStatus.UNKNOWN)
        self.assertNotIn("eef_control", backend._held_resources)


class ProcessSubWaitTest(unittest.TestCase):
    """循环检测机制：无消息时超时返回空，有消息时立即返回。"""

    def test_wait_times_out_without_message(self) -> None:
        backend = _real_backend(publishers={})
        with mock.patch("galaxea_r1pro.galaxea_r1pro_0.transport.process_sub._WAIT_JOINT", 0.05):
            with self.assertRaises(MessageNotFoundError) as ctx:
                process_sub.get_joint_state(backend.ros_connection, axis="left")
        error = ctx.exception
        self.assertEqual(error.topic, "/hdas/feedback_arm_left")
        self.assertEqual(error.reason, MessageNotFoundError.MESSAGE_TIMEOUT)
        self.assertIn("feedback_arm_left", str(error))

    def test_wait_returns_message_when_present(self) -> None:
        backend = _real_backend(publishers={})
        backend.ros_connection.latest_messages["/hdas/feedback_arm_left"] = {"position": [0.1, 0.2]}
        result = process_sub.get_joint_state(backend.ros_connection, axis="left")
        self.assertEqual(result, [0.1, 0.2])

    def test_transform_martix_returns_direct_transform(self) -> None:
        backend = _real_backend(publishers={})
        backend.ros_connection.latest_messages["/tf"] = {"transforms": [
            {"header": {"frame_id": "map"}, "child_frame_id": "body",
             "transform": {"translation": {"x": 1.0, "y": 2.0, "z": 3.0},
                           "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}}
        ]}
        transform = process_sub._get_transform_martix(backend.ros_connection, "body", "map")
        self.assertEqual(transform["parent_frame"], "map")
        self.assertEqual(transform["translation"]["x"], 1.0)

    def test_transform_martix_raises_when_path_missing(self) -> None:
        backend = _real_backend(publishers={})
        backend.ros_connection.latest_messages["/tf"] = {"transforms": [
            {"header": {"frame_id": "map"}, "child_frame_id": "odom",
             "transform": {"translation": {"x": 0.0, "y": 0.0, "z": 0.0},
                           "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}}
        ]}
        with self.assertRaises(MessageNotFoundError) as ctx:
            process_sub._get_transform_martix(backend.ros_connection, "body", "map")
        self.assertEqual(ctx.exception.reason, MessageNotFoundError.MESSAGE_NOT_FOUND)

    def test_wait_for_multiple_topics(self) -> None:
        backend = _real_backend(publishers={})
        backend.ros_connection.latest_messages["/hdas/feedback_left_arm_wrench"] = {
            "wrench": {"force": {"x": 1.0, "y": 0.0, "z": 0.0}, "torque": {"x": 0.0, "y": 0.0, "z": 0.0}}
        }
        backend.ros_connection.latest_messages["/hdas/feedback_right_arm_wrench"] = {
            "wrench": {"force": {"x": -1.0, "y": 0.0, "z": 0.0}, "torque": {"x": 0.0, "y": 0.0, "z": 0.0}}
        }
        result = process_sub.get_force_data(backend.ros_connection)
        self.assertEqual(result["left_arm"]["force_x"], 1.0)
        self.assertEqual(result["right_arm"]["force_x"], -1.0)

    def test_chassis_feedback_parses_wheels(self) -> None:
        backend = _real_backend(publishers={})
        backend.ros_connection.latest_messages["/hdas/feedback_chassis"] = {
            "velocity": [0.1, 0.2, 0.3],
            "position": [0.4, 0.5, 0.6],
        }
        result = process_sub.get_chassis_feedback(backend.ros_connection)
        self.assertEqual(result["wheel_speeds"], {"left_front": 0.1, "right_front": 0.2, "rear": 0.3})
        self.assertEqual(result["wheel_angles"], {"left_front": 0.4, "right_front": 0.5, "rear": 0.0})
        self.assertTrue(result["feedback_received"])

    def test_localization_parses_availability(self) -> None:
        backend = _real_backend(publishers={})
        backend.ros_connection.latest_messages["/localization/localization_results"] = {
            "position": {"position_global": {"x": 1.0, "y": 2.0, "available": 1}},
            "orientation": {"euler_global": {"yaw": 0.5}, "available": 1},
            "status": {"available": 1},
        }
        result = process_sub.get_localization(backend.ros_connection)
        self.assertTrue(result["localization_available"])
        self.assertEqual(result["position_global"]["x"], 1.0)
        self.assertEqual(result["orientation_global"]["yaw"], 0.5)

    def test_get_force_data_missing_raises(self) -> None:
        backend = _real_backend(publishers={})
        backend.ros_connection.latest_messages["/hdas/feedback_left_arm_wrench"] = {
            "wrench": {"force": {"x": 1.0, "y": 0.0, "z": 0.0}, "torque": {"x": 0.0, "y": 0.0, "z": 0.0}}
        }
        with mock.patch("galaxea_r1pro.galaxea_r1pro_0.transport.process_sub._WAIT_FORCE", 0.05):
            with self.assertRaises(MessageNotFoundError) as ctx:
                process_sub.get_force_data(backend.ros_connection)
        error = ctx.exception
        self.assertEqual(error.reason, MessageNotFoundError.MESSAGE_TIMEOUT)
        self.assertIn("feedback_right_arm_wrench", error.topic)
        self.assertEqual(error.to_dict()["topic"], error.topic)


if __name__ == "__main__":
    unittest.main()
