"""Torso_Control 能力：躯干关节控制。

输入参数仅保留与 publisher 消息构造相关的字段。
"""

from __future__ import annotations

from typing import List, Optional

from .backends.base import RobotBackend
from .sensors import SensorsClient
from .utils.types import CommandHandle


class TorsoControl:
    """R1 Pro 躯干关节控制。"""

    def __init__(self, backend: RobotBackend, sensors: Optional[SensorsClient] = None) -> None:
        self._backend = backend
        self._sensors = sensors or SensorsClient(backend)

    def control_torso_joints(
        self,
        *,
        positions: List[float],
        speeds: Optional[List[float]] = None,
        effort: Optional[List[float]] = None,
        names: Optional[List[str]] = None,
    ) -> CommandHandle:
        return self._backend.start_command(
            "torso_control", "control_torso_joints",
            {
                "positions": positions,
                "velocities": speeds or [],
                "efforts": effort or [],
                "names": names or [],
            },
        )

    def control_single_torso_joint(
        self,
        *,
        joint_id: int,
        target_pos: float,
        speed: float = 1.5,
        effort: Optional[List[float]] = None,
        names: Optional[List[str]] = None,
    ) -> CommandHandle:
        """单独控制躯干某个关节：读取当前位置后构造目标（借鉴 Atom）。"""

        current_result = self._sensors.get_torso_positions()
        current = (current_result.output or {}).get("data")
        positions = list(current)
        positions[joint_id] = target_pos
        speeds = [speed if i == joint_id else 0.05 for i in range(4)]
        return self.control_torso_joints(positions=positions, speeds=speeds, effort=effort, names=names)
