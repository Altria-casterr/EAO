"""WooshMove 能力：底盘运动控制（借鉴 Atom WooshMove）。

axis/target_value/speed 与 Atom 对齐（仅保留运动控制相关参数）；
速度带最大/最小限制，speed=None 时使用预设速度；运动前自动解锁刹车。
"""

from __future__ import annotations

import logging
import time
from typing import Optional

from .backends.base import RobotBackend
from .utils.types import CommandHandle

logger = logging.getLogger(__name__)


class WooshMove:
    """R1 Pro 底盘运动控制（对齐 Atom WooshMove：预设速度 + 最大/最小速度限制）。"""

    # 预设速度与默认参数（对齐 Atom WooshMove）
    DEFAULT_LINEAR_SPEED_X = 0.1    # X 轴线速度 (m/s)
    DEFAULT_LINEAR_SPEED_Y = 0.08   # Y 轴线速度 (m/s)
    DEFAULT_ANGULAR_SPEED_Z = 15.0  # Z 轴角速度 (°/s)

    # 最大/最小速度限制（对齐 Atom：x/y ±0.3 m/s，z ±30 °/s）
    MAX_LINEAR_SPEED = 0.3
    MAX_ANGULAR_SPEED = 30.0

    def __init__(self, backend: RobotBackend) -> None:
        self._backend = backend
        self._emergency_stop = False

    def move_robot(
        self,
        *,
        axis: str,
        target_value: float,
        speed: Optional[float] = None,
    ) -> CommandHandle:
        """执行机器人运动控制（对齐 Atom WooshMove.move_robot）。

        Args:
            axis: 运动轴（x-前后 / y-左右 / z-旋转）
            target_value: 目标值（x/y: 米，z: 度）
            speed: 运动速度（x/y: m/s，z: °/s）；None 时使用预设速度

        Returns:
            CommandHandle: 运动命令句柄

        Raises:
            ValueError: 轴无效
            RuntimeError: 机器人已紧急停止
        """

        # 1. 参数校验（对齐 Atom）
        if axis not in ("x", "y", "z"):
            raise ValueError(f"无效的轴，必须是 x、y 或 z，收到: {axis!r}")
        if self._emergency_stop:
            raise RuntimeError("机器人已紧急停止，请先恢复")

        # 2. 解锁刹车（借鉴 Atom __init__：运动前先解锁）
        self._unlock_brake()

        # 3. 预设速度（对齐 Atom：speed=None 时使用默认值）
        if speed is None:
            speed = {
                "x": self.DEFAULT_LINEAR_SPEED_X,
                "y": self.DEFAULT_LINEAR_SPEED_Y,
                "z": self.DEFAULT_ANGULAR_SPEED_Z,
            }[axis]

        # 4. 最大/最小速度限制（对齐 Atom）
        if axis in ("x", "y"):
            speed = max(min(speed, self.MAX_LINEAR_SPEED), -self.MAX_LINEAR_SPEED)
        else:
            speed = max(min(speed, self.MAX_ANGULAR_SPEED), -self.MAX_ANGULAR_SPEED)
        target_speed = speed if target_value > 0 else -abs(speed)

        # 5. 构造 TwistStamped 的 dx/dy/dyaw（z 为度，transport 内转弧度）
        if axis == "z":
            dx, dy, dyaw = 0.0, 0.0, target_speed
        elif axis == "y":
            dx, dy, dyaw = 0.0, target_speed, 0.0
        else:  # x
            dx, dy, dyaw = target_speed, 0.0, 0.0

        logger.info("开始%s轴运动：目标 %.3f | 速度 %.3f", axis, target_value, abs(speed))
        return self._backend.start_command(
            "woosh_move", "move_robot",
            {"dx": dx, "dy": dy, "dyaw": dyaw},
        ) 

    def _unlock_brake(self) -> CommandHandle | None:
        """解锁刹车：连续发布 10 次 brake=False（借鉴 Atom Utils.unlock_brake）。

        紧急停止置位时不发送；返回最后一次发送的 handle。
        """

        if self._emergency_stop:
            return None
        handle: CommandHandle | None = None
        for _ in range(10):
            if self._emergency_stop:
                return handle
            handle = self._backend.start_command("woosh_move", "unlock_brake", {"brake": False})
            time.sleep(0.05)
        return handle

    def stop(self) -> CommandHandle:
        """主动停止所有运动和导航（借鉴 Atom stop：零速发布 move_robot）。

        置位紧急停止，连续发布 10 次零速度 TwistStamped 停止底盘运动。
        """
        stop_target = {"dx": 0.0, "dy": 0.0, "dyaw": 0.0}
        handle: CommandHandle | None = None
        for _ in range(10):
            handle = self._backend.start_command("woosh_move", "move_robot", stop_target)
            time.sleep(0.02)
        return handle


