"""R1 Pro SDK代码主函数。"""

from __future__ import annotations

from .backends import create_backend
from .backends.base import RobotBackend
from .eef_control import EEFControl
from .gripper_control import GripperControl
from .joint_control import JointControl
from .utils.profile import RobotProfile
from .utils.safety import CommandClient, SafetyClient
from .utils.types import BackendKind
from .sensors import SensorsClient
from .state import StateClient
from .torso_control import TorsoControl
from .woosh_move import WooshMove
from .woosh_navigation import WooshNavigation
from .yolo_detect import YoloDetect
from .yolo_vision import YoloVision


class R1ProSDK:
    """Ability 只依赖此门面，不感知具体后端连接方式。"""

    def __init__(self, profile: RobotProfile, backend: RobotBackend) -> None:
        self.profile = profile
        self._backend = backend
        self.state = StateClient(backend)
        self.sensors = SensorsClient(backend)
        self.command = CommandClient(backend)
        self.safety = SafetyClient(backend)

        # Galaxea_R1Pro 能力客户端（end_effector.py 形式）
        self.joint = JointControl(backend)
        self.gripper = GripperControl(backend)
        self.torso = TorsoControl(backend, self.sensors)
        self.eef = EEFControl(backend)
        self.woosh_move = WooshMove(backend)
        self.woosh_navigation = WooshNavigation(backend)
        self.vision = YoloVision(backend, profile.options)
        # v3 运行版同款 yolo_detect 客户端；与 vision 并存，保持两侧接口兼容
        self.yolo_detect = YoloDetect(backend, self.sensors)

    @classmethod
    def open(cls, profile: RobotProfile, ability_name: str | None = None) -> "R1ProSDK":
        """打开 SDK 并建立连接，每次返回独立实例。

        ability_name 用于按能力裁剪待注册的话题
        （见 ``utils.Connect_Ros.GALAXEA_R1PRO_ABILITY_TOPICS``）：
        传入已登记的能力名时只注册该能力需要的话题，为 None 或未登记时注册全部话题。
        test_mode 由后端内部处理（走内存模拟，不建立真实 ROS 连接）。
        """

        sdk = cls(profile, create_backend(profile))
        sdk.connect(ability_name=ability_name)
        return sdk

    def connect(self, ability_name: str | None = None) -> None:
        self._backend.connect(ability_name)

    def is_connected(self) -> bool:
        """当前是否处于真实连接状态。"""

        return self._backend.is_connected()

    def close(self) -> None:
        """关闭底层连接（open 每次返回独立实例，不涉及引用计数）。"""

        self._backend.close()

    def __enter__(self) -> "R1ProSDK":
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.close()


if __name__ == "__main__":

    import time

    # 冒烟示例：test_mode 下不依赖真机与配置文件；
    # 生产模式请提供真实 endpoint 并去掉 test_mode。
    profile = RobotProfile(
        robot_id="r1pro-001",
        backend=BackendKind.REAL,
        firmware_profile="vendor-2026.08",
        options={"test_mode": True},
    )
    # 第二个参数为 ability 名：传已登记的能力名只拉起该能力话题，省略则注册全部话题
    test_sdk = R1ProSDK.open(profile, "Gripper.Demo")

    # Step1、测试是否与机器人成功连接
    test_sdk.is_connected()
    time.sleep(3)
    print(f"机器人连接成功,当前机器人ip和端口是{test_sdk._backend.profile.endpoint}")
    
    # Step2、运行夹爪开合控制
    test_sdk.gripper.control_both_gripper(left_position=50, right_position=50)
    time.sleep(1)

    print("夹爪开合控制完成")
