import json
import tempfile
import unittest
from pathlib import Path

from galaxea_r1pro import BackendKind, ProfileError, RobotProfile
from galaxea_r1pro.galaxea_r1pro_0.utils.utils_common import parse_endpoint


class RobotProfileTest(unittest.TestCase):
    def test_load_profile(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "robot.json"
            path.write_text(
                json.dumps(
                    {
                        "robot_id": "r1pro-test",
                        "backend": "mujoco",
                        "firmware_profile": "sim-v1",
                        "options": {"test_mode": True},
                    }
                ),
                encoding="utf-8",
            )
            profile = RobotProfile.load(path)

        self.assertEqual(profile.robot_id, "r1pro-test")
        self.assertIs(profile.backend, BackendKind.MUJOCO)

    def test_invalid_backend_is_rejected(self) -> None:
        with self.assertRaises(ProfileError):
            RobotProfile.from_mapping(
                {"robot_id": "r1pro-test", "backend": "unknown", "firmware_profile": "v1"}
            )


    def test_invalid_backend_error_carries_structured_details(self) -> None:
        with self.assertRaises(ProfileError) as ctx:
            RobotProfile.from_mapping(
                {"robot_id": "r1pro-test", "backend": "unknown", "firmware_profile": "v1"}
            )
        error = ctx.exception
        self.assertEqual(error.field, "backend")
        self.assertEqual(error.reason, ProfileError.INVALID_BACKEND)
        self.assertIn("backend", str(error))
        self.assertEqual(error.to_dict()["reason"], ProfileError.INVALID_BACKEND)

    def test_parse_endpoint(self) -> None:
        self.assertEqual(parse_endpoint("192.168.1.100:8080"), ("192.168.1.100", 8080))
        self.assertEqual(parse_endpoint("ros2://192.168.1.100:9091"), ("192.168.1.100", 9091))
        self.assertEqual(parse_endpoint("192.168.1.100"), ("192.168.1.100", 9091))
        with self.assertRaises(ProfileError):
            parse_endpoint("")
        with self.assertRaises(ProfileError):
            parse_endpoint("192.168.1.100:not-a-port")


if __name__ == "__main__":
    unittest.main()

