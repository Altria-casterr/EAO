"""RobotObservation Task 全流程测试（test_mode SDK，内存后端返回确定性数据）。"""

from __future__ import annotations

import unittest

from galaxea_r1pro import BackendKind, RobotProfile, R1ProSDK

from galaxea_r1pro_0.service.ability_service import R1ProAbilityService
from galaxea_r1pro_0.tasks import role_task_types


class RobotObservationFlowTest(unittest.TestCase):
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
            role_task_types("robot_observation"),
            (
                (0, "get_robot_state"),
                (1, "get_sensors_state"),
                (2, "capture_rgbd"),
            ),
        )

    def test_get_robot_state_single_item(self) -> None:
        sdk = self._sdk()
        try:
            service = R1ProAbilityService("robot_observation", sdk)
            result = service.invoke(
                "get_robot_state",
                {"invocation_id": "o1", "observation": "snapshot"},
            )
            self.assertEqual(result["status"], "completed")
            self.assertEqual(result["task_name"], "get_robot_state")
            observations = result["observations"]
            self.assertEqual(len(observations), 1)
            item = observations[0]
            self.assertEqual(item["source"], "state")
            self.assertEqual(item["name"], "snapshot")
            self.assertEqual(item["data"]["robot_id"], "r1pro-ability-test")

            # 其它观测项
            floating = service.invoke(
                "get_robot_state",
                {"invocation_id": "o1b", "observation": "floating_base"},
            )
            self.assertEqual(floating["observations"][0]["name"], "floating_base")

            # 缺少 observation → ValueError
            with self.assertRaises(ValueError):
                service.invoke("get_robot_state", {"invocation_id": "o1c"})

            # 未知观测项 → ValueError
            with self.assertRaises(ValueError):
                service.invoke("get_robot_state", {"observation": "unknown_item"})
        finally:
            sdk.close()

    def test_get_robot_state_semantic(self) -> None:
        """语义 robot.get_state：无必填入参，返回 kind=robot.state 的 semantic Observation。"""

        sdk = self._sdk()
        try:
            service = R1ProAbilityService("robot_observation", sdk)
            result = service.invoke(
                "get_robot_state",
                {"invocation_id": "s1", "semantic_flag": True},
            )
            self.assertEqual(result["status"], "completed")
            self.assertEqual(result["task_name"], "get_robot_state")
            observations = result["observations"]
            self.assertEqual(len(observations), 1)
            obs = observations[0]
            self.assertEqual(obs["kind"], "robot.state")
            self.assertEqual(obs["schema_version"], 2)
            self.assertEqual(obs["subject_ref"], "r1pro-ability-test")
            value = obs["value"]
            self.assertEqual(value["robot_id"], "r1pro-ability-test")
            self.assertIsInstance(value["connected"], bool)
            self.assertTrue(obs["evidence_refs"])
        finally:
            sdk.close()

    def test_get_sensors_state_single_item(self) -> None:
        sdk = self._sdk()
        try:
            service = R1ProAbilityService("robot_observation", sdk)
            result = service.invoke(
                "get_sensors_state",
                {"invocation_id": "o2", "observation": "force_data"},
            )
            self.assertEqual(result["status"], "completed")
            self.assertEqual(result["task_name"], "get_sensors_state")
            observations = result["observations"]
            self.assertEqual(len(observations), 1)
            item = observations[0]
            self.assertEqual(item["source"], "sensor")
            self.assertEqual(item["name"], "force_data")

            ee = service.invoke(
                "get_sensors_state",
                {"invocation_id": "o2b", "observation": "ee_pose_left"},
            )
            self.assertEqual(ee["observations"][0]["name"], "ee_pose_left")

            torso = service.invoke(
                "get_sensors_state",
                {"invocation_id": "o2c", "observation": "torso_positions"},
            )
            self.assertEqual(torso["observations"][0]["name"], "torso_positions")

            # 未知观测项 → ValueError
            with self.assertRaises(ValueError):
                service.invoke("get_sensors_state", {"observation": "unknown_item"})
        finally:
            sdk.close()

    def test_capture_rgbd(self) -> None:
        sdk = self._sdk()
        try:
            service = R1ProAbilityService("robot_observation", sdk)
            result = service.invoke(
                "capture_rgbd",
                {"invocation_id": "o3", "sensor_id": "camera_head"},
            )
            self.assertEqual(result["status"], "completed")
            self.assertEqual(result["task_name"], "capture_rgbd")
            observations = result["observations"]
            self.assertEqual([item["name"] for item in observations], ["rgb", "depth"])
            self.assertEqual(observations[0]["sensor_id"], "camera_head")
            self.assertEqual(observations[0]["encoding"], "rgb8")
            self.assertEqual(observations[1]["encoding"], "32FC1")

            # 缺少 sensor_id → ValueError
            with self.assertRaises(ValueError):
                service.invoke("capture_rgbd", {})
        finally:
            sdk.close()


if __name__ == "__main__":
    unittest.main()
