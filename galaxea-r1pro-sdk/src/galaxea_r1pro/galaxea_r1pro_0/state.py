"""机器人状态读取。"""

from .backends.base import RobotBackend
from .utils.types import CapabilityInfo, CommandResult, RobotState


class StateClient:
    def __init__(self, backend: RobotBackend) -> None:
        self._backend = backend

    def capabilities(self) -> CapabilityInfo:
        return self._backend.capabilities()

    def snapshot(self) -> RobotState:
        return self._backend.snapshot()

    def get_torso_status(self) -> CommandResult:
        """读取躯干状态反馈（原 TorsoControl.get_torso_status）。"""

        return self._backend.get_command(resource="torso_control", operation="get_torso_status", target={})

    def get_floating_base(self) -> CommandResult:
        """读取浮动基座标变换（借鉴 Atom Joint_EEFPose_Control.get_trans_martix，返回逆变换 position+orientation）。"""

        return self._backend.get_command(resource="eef_control", operation="get_floating_base", target={})

    def get_transformer_martix(
        self,
        source_frame: str = "map",
        target_frame: str = "body",
        timeout: float = 15.0,
        max_age_s: float | None = None,
    ) -> CommandResult:
        """获取 source_frame → target_frame 变换矩阵（借鉴 Atom，返回 {"position": (x,y,z), "orientation": (qx,qy,qz,qw)}）。"""

        return self._backend.get_command(
            resource="coordinate_transformation",
            operation="get_transformer_martix",
            target={"source_frame": source_frame, "target_frame": target_frame, "timeout": timeout, "max_age_s": max_age_s},
        )
