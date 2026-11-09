"""ROS 连接层：连接 rosbridge 并注册机器人全部话题。

- ``ConnectRos``：建立 ROS 连接并管理四类话题字典
  （publisher / subscriber / service / action）；
- ``GALAXEA_R1PRO_TOPICS``：Galaxea_R1Pro 使用到的全部话题注册表；
- ``GALAXEA_R1PRO_ABILITY_TOPICS``：ability 名称 → 该 ability 需要拉起的话题集合，
  控制类 ability 登记控制话题及其闭环必需的最小反馈话题，全域反馈订阅由 RobotObservation 负责，
  供后端按当前 ability 裁剪注册表后再注册；
- ``ConnectRos.register_topics``：连接成功后一次性创建传入注册表中的全部话题；
- 命令映射与消息构造位于 ``transport/process_pub.py``，读取处理位于 ``transport/process_sub.py``。

roslibpy 为直接引入的运行时依赖，部署环境需安装（``pip install roslibpy``）。
"""

from __future__ import annotations

import logging
import time
from typing import Any, Callable, Dict, Optional

import roslibpy

from .errors import BackendUnavailableError
from .tf_buffer import TransformBuffer

logger = logging.getLogger(__name__)

GALAXEA_R1PRO_TOPICS: dict[str, dict[str, str]] = {
    "publishers": {
        "/motion_target/target_joint_state_arm_left": "sensor_msgs/msg/JointState",
        "/motion_target/target_joint_state_arm_right": "sensor_msgs/msg/JointState",
        "/motion_target/target_position_gripper_left": "sensor_msgs/msg/JointState",
        "/motion_target/target_position_gripper_right": "sensor_msgs/msg/JointState",
        "/motion_target/target_joint_state_torso": "sensor_msgs/msg/JointState",
        "/motion_target/target_pose_arm_left": "geometry_msgs/msg/PoseStamped",
        "/motion_target/target_pose_arm_right": "geometry_msgs/msg/PoseStamped",
        "/motion_target/target_speed_chassis": "geometry_msgs/msg/TwistStamped",
        "/motion_target/brake_mode": "std_msgs/msg/Bool",
    },
    "subscribers": {
        "/hdas/feedback_arm_left": "sensor_msgs/msg/JointState",
        "/hdas/feedback_arm_right": "sensor_msgs/msg/JointState",
        "/hdas/feedback_gripper_left": "sensor_msgs/msg/JointState",
        "/hdas/feedback_gripper_right": "sensor_msgs/msg/JointState",
        "/hdas/feedback_torso": "sensor_msgs/msg/JointState",
        "/hdas/feedback_status_torso": "hdas_msg/msg/FeedbackStatus",  # 这个应该放到State或者sensor中
        "/hdas/feedback_left_arm_wrench": "geometry_msgs/msg/WrenchStamped",  # 这个应该放到sensor中
        "/hdas/feedback_right_arm_wrench": "geometry_msgs/msg/WrenchStamped", # 这个应该放到sensor中
        "/hdas/feedback_chassis": "sensor_msgs/msg/JointState",
        "/localization/localization_results": "localization_msg/msg/LocLocalization",  # 这个应该放到State或者sensor中
        "/motion_control/pose_ee_arm_left": "geometry_msgs/msg/PoseStamped",
        "/motion_control/pose_ee_arm_right": "geometry_msgs/msg/PoseStamped",
        "/motion_control/pose_floating_base": "geometry_msgs/msg/PoseStamped",  # 这个放到state或者sensor中
        "/tf_static": "tf2_msgs/TFMessage",
        "/tf": "tf2_msgs/TFMessage",  # 这个放到state或者sensor中
        "/camera/camera/color/image_raw/compressed": "sensor_msgs/msg/CompressedImage",
        "/camera/camera/aligned_depth_to_color/image_raw/compressedDepth": "sensor_msgs/msg/CompressedImage",
    },
    "services": {},
    "actions": {
        "/system_manager/navi/action": "system_manager_msg/action/NavigationTask",
    },
}

# ability（ability.manifest.yaml 的 abilityName）→ 该 ability 实际需要拉起的话题集合。
# 分工：控制类 ability 拉起「控制话题 + 自身闭环必需的最小反馈话题」，不引入其他无关订阅；
# 全量反馈订阅由 RobotObservation.Demo 负责；YoloDetect.Demo 作为独立识别 ability，
# 自带相机与 TF 订阅。话题名必须能在 GALAXEA_R1PRO_TOPICS 中命中。
GALAXEA_R1PRO_ABILITY_TOPICS: dict[str, set[str]] = {
    # 夹爪控制：发布左右夹爪目标；闭环读取左右夹爪反馈
    "Gripper.Demo": {
        "/motion_target/target_position_gripper_left",
        "/motion_target/target_position_gripper_right",
        "/hdas/feedback_gripper_left",
        "/hdas/feedback_gripper_right",
    },
    # 手臂关节控制：发布左右臂关节目标；闭环读取左右臂关节反馈
    "JointControl.Demo": {
        "/motion_target/target_joint_state_arm_left",
        "/motion_target/target_joint_state_arm_right",
        "/hdas/feedback_arm_left",
        "/hdas/feedback_arm_right",
    },
    # 躯干控制：发布躯干关节目标；闭环读取躯干关节反馈
    "TorsoControl.Demo": {
        "/motion_target/target_joint_state_torso",
        "/hdas/feedback_torso",
    },
    # 末端位姿控制：发布左右臂位姿目标；闭环/转换读取末端位姿、浮动基座、腕力（力反馈精调）
    "JointEEFPoseControl.Demo": {
        "/motion_target/target_pose_arm_left",
        "/motion_target/target_pose_arm_right",
        "/motion_control/pose_ee_arm_left",
        "/motion_control/pose_ee_arm_right",
        "/motion_control/pose_floating_base",
        "/hdas/feedback_left_arm_wrench",
        "/hdas/feedback_right_arm_wrench",
    },
    # 底盘相对运动：发布底盘速度/刹车；闭环读取定位（优先）与底盘反馈（积分兜底）
    "WooshMove.Demo": {
        "/motion_target/target_speed_chassis",
        "/motion_target/brake_mode",
        "/localization/localization_results",
        "/hdas/feedback_chassis",
    },
    # 导航：Action 目标 + 解锁刹车/停车速度；到达持物检查读取腕力
    "WooshNavigation.Demo": {
        "/system_manager/navi/action",
        "/motion_target/target_speed_chassis",
        "/motion_target/brake_mode",
        "/hdas/feedback_left_arm_wrench",
        "/hdas/feedback_right_arm_wrench",
    },
    # 观测：负责全域反馈订阅（关节/夹爪/躯干/腕力/底盘/定位/末端/浮动基座/TF/相机）
    "RobotObservation.Demo": {
        "/hdas/feedback_arm_left",
        "/hdas/feedback_arm_right",
        "/hdas/feedback_gripper_left",
        "/hdas/feedback_gripper_right",
        "/hdas/feedback_torso",
        "/hdas/feedback_status_torso",
        "/hdas/feedback_left_arm_wrench",
        "/hdas/feedback_right_arm_wrench",
        "/hdas/feedback_chassis",
        "/localization/localization_results",
        "/motion_control/pose_ee_arm_left",
        "/motion_control/pose_ee_arm_right",
        "/motion_control/pose_floating_base",
        "/tf",
        "/tf_static",
        "/camera/camera/color/image_raw/compressed",
        "/camera/camera/aligned_depth_to_color/image_raw/compressedDepth",
    },
    # YOLO 识别：自带相机 RGB-D + 动态/静态 TF 订阅
    "YoloDetect.Demo": {
        "/camera/camera/color/image_raw/compressed",
        "/camera/camera/aligned_depth_to_color/image_raw/compressedDepth",
        "/tf",
        "/tf_static",
    },
}


class ConnectRos:
    """基于 roslibpy 的 ROS 连接封装：管理连接与四类话题字典。"""

    def __init__(
        self,
        ros_host: str = "172.130.0.100",
        ros_port: int = 9091,
        ros_connect: bool = True,
        ros: Optional[Any] = None,
        timeout: float = 5.0,
    ):
        assert not ros_connect or ros is None, "参数冲突：不能同时开启 ros_connect 并传入 ros 实例"
        assert ros_connect or ros is not None, "关闭 ros_connect 时必须传入有效 ros 实例"

        self.ros_host = ros_host
        self.ros_port = ros_port
        self.timeout = timeout

        self.client = roslibpy.Ros(host=ros_host, port=ros_port) if ros_connect else ros

        self.publishers: Dict[str, Any] = {}
        self.subscribers: Dict[str, Any] = {}
        self.services: Dict[str, Any] = {}
        self.actions: Dict[str, Any] = {}
        self.latest_messages: Dict[str, Any] = {}
        self.tf_buffer = TransformBuffer()
        self.message_received_at: Dict[str, float] = {}
        # 导航 Action 状态（is_navigating 等），由 RealBackend 写入、读取型 get_current_status 返回
        self.navigation_state: Dict[str, Any] = {"is_navigating": False, "result": None, "feedback": None, "error": None}

        self.connect_flag = self.connect()

    """ 建立ROS连接 """
    def connect(self) -> bool:
        try:
            self.client.run()

            max_attempts = int(self.timeout / 0.1)
            for _ in range(max_attempts):
                if self.client.is_connected:
                    logging.info(f"ROS连接成功：{self.ros_host}:{self.ros_port}")
                    return True
                time.sleep(0.1)
            else:
                logging.error(f"ROS连接超时 {self.timeout} 秒")
                return False

        except Exception as e:
            logging.error(f"ROS连接失败：{e}")
            return False

    """ 创建发布者-topic """
    def create_publisher(self, topic: str, message_type: str) -> Any:
        if topic in self.publishers:
            logging.warning(f"已存在话题 {topic} 的发布者")
            return self.publishers[topic]

        try:
            publisher = roslibpy.Topic(self.client, topic, message_type)
            publisher.advertise()
            self.publishers[topic] = publisher
            logging.info(f"发布者创建成功: {topic}")
            return publisher
        except Exception as e:
            logging.error(f"发布者创建失败: {e}")
            return None

    """ 创建服务客户端 """
    def create_service(self, topic: str, message_type: str) -> Any:
        if topic in self.services:
            logging.warning(f"已存在服务 {topic}")
            return self.services[topic]

        try:
            service = roslibpy.Service(self.client, topic, message_type)
            self.services[topic] = service
            logging.info(f"服务创建成功: {topic}")
            return service
        except Exception as e:
            logging.error(f"服务创建失败: {e}")
            return None

    """ 创建导航Action客户端 """
    def create_action(self, topic: str, message_type: str) -> Any:
        if topic in self.actions:
            logging.warning(f"已存在动作 {topic}")
            return self.actions[topic]

        try:
            action = roslibpy.ActionClient(self.client, topic, message_type)
            self.actions[topic] = action
            logging.info(f"导航Action客户端创建成功: {topic}")
            return action
        except Exception as e:
            logging.error(f"导航Action客户端创建失败: {e}")
            return None

    """ 创建订阅者 """
    def create_subscriber(
        self,
        topic: str,
        message_type: str,
        callback: Callable[[dict], None],
    ) -> Any:
        # TODO 警惕时间不同步的问题出现

        if topic in self.subscribers:
            logging.warning(f"已存在名为 {topic} 的订阅者")
            return self.subscribers[topic]

        subscriber = roslibpy.Topic(self.client, topic, message_type)
        subscriber.subscribe(callback)
        self.subscribers[topic] = subscriber

        return subscriber

    """ 注册全部话题（连接成功后才能调用）。

    发布者/订阅者/服务/动作一次性创建；订阅默认回调直接把最新消息
    写入 ``latest_messages``，供上层按话题名读取。
    """
    def register_topics(
        self,
        registry: Optional[Dict[str, Dict[str, str]]] = None,
    ) -> "ConnectRos":

        # TODO 缺少校验，是否应该重新连接
        if not self.connect_flag:
            raise BackendUnavailableError(
                backend="real",
                reason=BackendUnavailableError.NOT_CONNECTED,
                operation="ConnectRos.register_topics",
                hint=f"ROS 连接未成功（{self.ros_host}:{self.ros_port}），不能创建话题",
            )

        registry = registry if registry is not None else GALAXEA_R1PRO_TOPICS

        for topic, message_type in (registry.get("publishers") or {}).items():
            self.create_publisher(topic, message_type)

        for topic, message_type in (registry.get("subscribers") or {}).items():

            def _capture(message: dict, topic: str = topic) -> None:
                self.latest_messages[topic] = message
                self.message_received_at[topic] = time.monotonic()
                if topic in ("/tf", "/tf_static"):
                    self.tf_buffer.update(message, static=topic == "/tf_static")

            self.create_subscriber(topic, message_type, _capture)

        for topic, message_type in (registry.get("services") or {}).items():
            self.create_service(topic, message_type)
            
        for topic, message_type in (registry.get("actions") or {}).items():
            self.create_action(topic, message_type)

        logger.info(
            "ROS 话题注册完成：publisher=%d subscriber=%d service=%d action=%d",
            len(registry.get("publishers") or {}),
            len(registry.get("subscribers") or {}),
            len(registry.get("services") or {}),
            len(registry.get("actions") or {}),
        )
        return self

    """ 关闭ROS连接：取消所有订阅、清空话题字典并终止连接。 """
    def close(self) -> bool:
        for subscriber in self.subscribers.values():
            try:
                subscriber.unsubscribe()
                logging.info(f"取消订阅 {subscriber.name}")
            except Exception as e:
                logging.warning(f"取消订阅时出错：{e}")

        self.publishers.clear()
        self.services.clear()
        self.actions.clear()
        self.subscribers.clear()
        self.latest_messages.clear()
        self.message_received_at.clear()
        self.tf_buffer.clear()
        self.navigation_state = {"is_navigating": False, "result": None, "feedback": None, "error": None}
        logging.info("所有话题已关闭")

        try:
            if self.client and self.client.is_connected:
                self.client.terminate()
                logging.info("ROS连接已关闭")
                return True
            else:
                logging.warning("ROS连接未建立或已关闭")
                return False

        except Exception as e:
            logging.error(f"关闭ROS连接时发生错误：{e}")
            return False
