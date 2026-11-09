"""RobotObservation Task 全流程测试（test_mode SDK，内存后端返回确定性数据）。"""

from __future__ import annotations

import unittest
import json
from datetime import datetime, timezone
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import patch

import cv2
import numpy as np

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

    def _frame(self):
        pixels = np.zeros((24, 32, 3), dtype=np.uint8)
        pixels[:, :] = (0, 0, 240)  # SDK BGR: red, not blue.
        return SimpleNamespace(sensor_id="camera_head", frame_id="head_optical",
                               encoding="rgb8", timestamp=datetime.now(timezone.utc),
                               data=pixels)

    def test_capture_rgbd(self):
        sdk = self._sdk()
        try:
            with TemporaryDirectory() as directory, patch.dict(
                "os.environ", {"SEMANTIC_ABILITY_ARTIFACT_ROOT": directory}
            ), patch.object(sdk.sensors, "capture_rgb", return_value=self._frame()) as capture:
                service = R1ProAbilityService("robot_observation", sdk)
                request = {"invocation_id": "o3", "sensor_id": "camera_head"}
                result = service.invoke("capture_rgbd", request)
                self.assertEqual(result["status"], "completed")
                observation = result["observations"][0]
                self.assertEqual(observation["kind"], "sensor.frame")
                self.assertEqual(observation["evidence_refs"], [])
                candidate = result["result"]["artifact_candidates"][0]
                self.assertEqual(observation["value"], candidate)
                self.assertEqual(candidate["media_type"], "image/jpeg")
                self.assertFalse(Path(candidate["exchange_path"]).is_absolute())
                path = Path(directory) / candidate["exchange_path"]
                decoded = cv2.imread(str(path))
                self.assertEqual(decoded.shape, (24, 32, 3))
                self.assertGreater(int(decoded[0, 0, 2]), 220)
                self.assertLess(int(decoded[0, 0, 0]), 10)
                self.assertEqual(path.stat().st_size, candidate["payload_size"])
                self.assertEqual(list(Path(directory).rglob("*.partial")), [])
                json.dumps(result)  # No ndarray/bytes leak into the wire result.
                repeated = service.invoke("capture_rgbd", request)
                self.assertEqual(repeated["result"], result["result"])
                self.assertEqual(capture.call_count, 1)
                both = service.invoke("capture_rgbd", {**request, "invocation_id": "o4", "axis": "both"})
                self.assertEqual([o["name"] for o in both["observations"]], ["rgb", "depth"])
                self.assertEqual(len(both["result"]["artifact_candidates"]), 1)
        finally:
            sdk.close()

    def test_capture_configuration_and_depth_compatibility(self):
        sdk = self._sdk()
        try:
            service = R1ProAbilityService("robot_observation", sdk)
            with patch.dict("os.environ", {"SEMANTIC_ABILITY_ARTIFACT_ROOT": ""}), patch.object(
                sdk.sensors, "capture_rgb"
            ) as capture:
                with self.assertRaisesRegex(RuntimeError, "SEMANTIC_ABILITY_ARTIFACT_ROOT"):
                    service.invoke("capture_rgbd", {"sensor_id": "camera_head"})
                capture.assert_not_called()
                depth = service.invoke("capture_rgbd", {"sensor_id": "camera_head", "axis": "depth"})
                self.assertEqual(depth["observations"][0]["name"], "depth")
                self.assertEqual(depth["result"]["artifact_candidates"], [])
                with self.assertRaises(ValueError):
                    service.invoke("capture_rgbd", {"sensor_id": "camera_head", "axis": "invalid"})
                with self.assertRaises(ValueError):
                    service.invoke("capture_rgbd", {})
        finally:
            sdk.close()

    def test_capture_publication_failures(self):
        from galaxea_r1pro_0.utils.capture_artifact import publish_rgb
        with TemporaryDirectory() as directory:
            root = Path(directory)
            frame = self._frame()
            with patch("galaxea_r1pro_0.utils.capture_artifact.cv2.imencode", return_value=(False, None)):
                with self.assertRaisesRegex(RuntimeError, "编码失败"):
                    publish_rgb(frame, root)
            with patch("galaxea_r1pro_0.utils.capture_artifact.os.replace", side_effect=OSError("disk failure")):
                with self.assertRaisesRegex(OSError, "disk failure"):
                    publish_rgb(frame, root)
            self.assertEqual(list(root.rglob("*.partial")), [])
            self.assertEqual(list(root.rglob("*.jpg")), [])
            frame.data = np.zeros((4, 4), dtype=np.float32)
            with self.assertRaisesRegex(ValueError, "uint8"):
                publish_rgb(frame, root)

    def test_capture_rejects_exchange_directory_escape(self):
        from galaxea_r1pro_0.utils.capture_artifact import publish_rgb
        with TemporaryDirectory() as directory, TemporaryDirectory() as outside:
            root = Path(directory)
            (root / "captures").symlink_to(outside, target_is_directory=True)
            with self.assertRaisesRegex(ValueError, "超出交换目录"):
                publish_rgb(self._frame(), root)
            self.assertEqual(list(Path(outside).iterdir()), [])


if __name__ == "__main__":
    unittest.main()
