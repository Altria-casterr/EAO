"""transport 层测试：Galaxea_R1Pro 话题注册表与 ROS 连接依赖。"""

import unittest

from galaxea_r1pro import BackendUnavailableError
from galaxea_r1pro.galaxea_r1pro_0.utils.Connect_Ros import (
    GALAXEA_R1PRO_TOPICS,
    ConnectRos,
)


class GalaxeaTopicsRegistryTest(unittest.TestCase):
    def test_registry_contains_four_kinds(self) -> None:
        self.assertEqual(
            set(GALAXEA_R1PRO_TOPICS.keys()),
            {"publishers", "subscribers", "services", "actions"},
        )

    def test_registry_topic_names_are_unique(self) -> None:
        names = [
            name
            for group in GALAXEA_R1PRO_TOPICS.values()
            for name in group
        ]
        self.assertEqual(len(names), len(set(names)), "话题名存在重复")

    def test_galaxea_r1pro_expected_topic_counts(self) -> None:
        self.assertEqual(len(GALAXEA_R1PRO_TOPICS["publishers"]), 9)
        self.assertEqual(len(GALAXEA_R1PRO_TOPICS["subscribers"]), 17)
        self.assertEqual(len(GALAXEA_R1PRO_TOPICS["services"]), 0)
        self.assertEqual(len(GALAXEA_R1PRO_TOPICS["actions"]), 1)
        self.assertIn(
            "/system_manager/navi/action",
            GALAXEA_R1PRO_TOPICS["actions"],
        )

    def test_key_topics_present(self) -> None:
        self.assertIn(
            "/motion_target/target_speed_chassis",
            GALAXEA_R1PRO_TOPICS["publishers"],
        )
        self.assertIn(
            "/hdas/feedback_chassis",
            GALAXEA_R1PRO_TOPICS["subscribers"],
        )
        self.assertIn(
            "/localization/localization_results",
            GALAXEA_R1PRO_TOPICS["subscribers"],
        )


class RosConnectionFailureTest(unittest.TestCase):
    def test_connect_fails_when_ros_unreachable(self) -> None:
        # roslibpy 为直接依赖；连接不可达时 connect_flag 必须为 False，不伪装成功
        connection = ConnectRos(ros_host="127.0.0.1", ros_port=1, timeout=0.2)
        self.assertFalse(connection.connect_flag)

    def test_register_topics_requires_successful_connection(self) -> None:
        # 连接未成功时不允许创建话题，必须抛结构化 BackendUnavailableError
        connection = ConnectRos(ros_host="127.0.0.1", ros_port=1, timeout=0.2)
        with self.assertRaises(BackendUnavailableError) as ctx:
            connection.register_topics()
        error = ctx.exception
        self.assertEqual(error.reason, BackendUnavailableError.NOT_CONNECTED)
        self.assertEqual(error.operation, "ConnectRos.register_topics")


if __name__ == "__main__":
    unittest.main()
