"""传感器读取接口（含力反馈读取）。"""

from .backends.base import RobotBackend
from .utils.types import CommandResult, SensorFrame


class SensorsClient:
    def __init__(self, backend: RobotBackend) -> None:
        self._backend = backend

    def capture_rgb(self, sensor_id: str) -> SensorFrame:
        return self._backend.capture(sensor_id, "rgb8")

    def capture_depth(self, sensor_id: str) -> SensorFrame:
        return self._backend.capture(sensor_id, "32FC1")

    def get_force_data(self) -> CommandResult:
        """读取左右臂力/力矩数据（get_current_status 与 get_force_data 合并）。

        参考 skill_library ForceFeedback：等待左右臂 wrench 话题后解析
        force/torque 分量；任一侧缺失视为数据不完整。
        """
        return self._backend.get_command(resource="force_feedback", operation="get_force_data", target={})

    def get_ee_pose(self, axis: str = "left") -> CommandResult:
        """读取单臂末端位姿（axis: left/right，合并原 get_left/right_ee_pose）。"""

        if axis not in ("left", "right"):
            raise ValueError(f"无效的轴: {axis!r}")
        return self._backend.get_command(resource="eef_control", operation=f"get_{axis}_ee_pose", target={})

    def get_gripper_state(self, axis: str = "left") -> CommandResult:
        """读取单侧夹爪状态（axis: left/right，合并原 get_left/right_gripper_state）。"""

        if axis not in ("left", "right"):
            raise ValueError(f"无效的轴: {axis!r}")
        return self._backend.get_command(resource="gripper_control", operation=f"get_{axis}_gripper_state", target={})

    def get_joint_state(self, axis: str = "left") -> CommandResult:
        """读取单臂关节状态（axis: left/right，合并原 get_left/right_joint_state）。"""

        if axis not in ("left", "right"):
            raise ValueError(f"无效的轴: {axis!r}")
        return self._backend.get_command(resource="joint_control", operation=f"get_{axis}_joint_state", target={})

    def get_torso_positions(self) -> CommandResult:
        """读取躯干四关节当前位置（自 StateClient 迁入）。"""

        return self._backend.get_command(resource="torso_control", operation="get_torso_positions", target={})

    def get_chassis_feedback(self) -> CommandResult:
        """读取底盘轮速/轮角反馈（自 StateClient 迁入，借鉴 Atom _feedback_callback）。"""

        return self._backend.get_command(resource="woosh_move", operation="get_chassis_feedback", target={})

    def get_localization(self) -> CommandResult:
        """读取定位结果（自 StateClient 迁入，借鉴 Atom _localization_callback）。"""

        return self._backend.get_command(resource="woosh_move", operation="get_localization", target={})
