"""Gripper_Control 能力：夹爪控制。

输入参数仅保留与 publisher 消息构造相关的字段
（effort 默认 0.3）。
"""

from __future__ import annotations

import threading

from .backends.base import RobotBackend
from .utils.types import CommandHandle

class GripperControl:
    """R1 Pro 夹爪控制。"""

    def __init__(self, backend: RobotBackend) -> None:
        self._backend = backend

    def control_single_gripper(
        self,
        *,
        arm_side: str,
        position: float,
        speed: float = 0.5,
        effort: float = 0.3
    ) -> CommandHandle:
        return self._backend.start_command(
            "gripper_control", "control_single_gripper",
            {"side": arm_side, "position": position, "speed": speed, "effort": effort},
        )

    def control_both_gripper(
        self,
        *,
        left_position: float = 50,
        right_position: float = 50,
        left_speed: float = 0.5,
        right_speed: float = 0.5,
        left_effort: float = 0.3,
        right_effort: float = 0.3,
    ) -> list:
        """双夹爪控制：左右夹爪异步多线程并行提交（借鉴 Atom）。"""

        results: dict[str, CommandHandle] = {}

        def _run(side: str, position: float, speed: float, effort: float) -> None:
            results[side] = self.control_single_gripper(
                arm_side=side, position=position, speed=speed, effort=effort
            )

        left_thread = threading.Thread(target=_run, args=("left", left_position, left_speed, left_effort), daemon=True)
        right_thread = threading.Thread(target=_run, args=("right", right_position, right_speed, right_effort), daemon=True)
        left_thread.start()
        right_thread.start()
        left_thread.join()
        right_thread.join()

        return [results["left"], results["right"]]

