"""Joint_Control 能力：手臂关节控制。

函数与 skill_library/Atom/Joint_Control/Galaxea_R1Pro 对齐；
输入参数仅保留与 publisher 消息构造相关的字段。
"""

from __future__ import annotations
import threading

from typing import Optional
from .backends.base import RobotBackend
from .utils.types import CommandHandle


class JointControl:
    """R1 Pro 手臂关节控制。"""

    def __init__(self, backend: RobotBackend) -> None:
        self._backend = backend

    def control_single_arm_joint(
        self,
        *,
        arm_side: str,
        positions: list,
        joint_names: list,
        velocities: Optional[list] = None,
        effort: Optional[list] = None,
    ) -> CommandHandle:
        return self._backend.start_command(
            "joint_control", "control_single_arm_joint",
            {
                "arm_side": arm_side,
                "names": joint_names,
                "positions": positions,
                "velocities": velocities or [],
                "efforts": effort or [],
            },
        )

    def control_both_arms_joint(
        self,
        *,
        left_positions: list,
        right_positions: list,
        left_joint_names: list,
        right_joint_names: list,
        left_velocities: Optional[list] = None,
        right_velocities: Optional[list] = None,
        left_effort: Optional[list] = None,
        right_effort: Optional[list] = None,
    ) -> list:
        """双臂关节控制：左右臂异步多线程并行提交（借鉴 Atom）。"""

        results: dict[str, CommandHandle] = {}

        def _run(side: str, positions: list, names: list, velocities: Optional[list], effort: Optional[list]) -> None:
            results[side] = self.control_single_arm_joint(
                arm_side=side,
                positions=positions,
                joint_names=names,
                velocities=velocities,
                effort=effort,
            )

        left_thread = threading.Thread(
            target=_run, args=("left", left_positions, left_joint_names, left_velocities, left_effort), daemon=True
        )
        right_thread = threading.Thread(
            target=_run, args=("right", right_positions, right_joint_names, right_velocities, right_effort), daemon=True
        )
        left_thread.start()
        right_thread.start()
        left_thread.join()
        right_thread.join()

        return [results["left"], results["right"]]

