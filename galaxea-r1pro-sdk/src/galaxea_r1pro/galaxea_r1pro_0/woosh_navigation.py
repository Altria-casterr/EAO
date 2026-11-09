"""WooshNavigation 能力：地图导航（ROS Action）。

输入参数仅保留与 action goal 消息构造相关的字段；
发送目标后的等待/超时/结果判断循环（借鉴 Atom _send_navigation_goal）在客户端完成。
"""

from __future__ import annotations

import logging
import time
from typing import Optional

from .backends.base import RobotBackend
from .utils.types import CommandHandle, CommandResult, CommandStatus

logger = logging.getLogger(__name__)


class WooshNavigation:
    """R1 Pro 地图导航（对齐 Atom WooshNavigation：紧急停止 + 刹车解锁 + 等待循环）。"""

    def __init__(self, backend: RobotBackend) -> None:
        self._backend = backend
        self._emergency_stop = False

    def navigate_to_map_pose(
        self,
        *,
        x: float, y: float,
        yaw: float = 0.0, z: float = 0.0,
        target_point_type: int = 0, frame_id: str = "map",
        qx: Optional[float] = None, qy: Optional[float] = None,
        qz: Optional[float] = None, qw: Optional[float] = None,
        timeout: float = 30.0,
    ) -> CommandResult:
        """发送导航目标并等待完成（借鉴 Atom _send_navigation_goal）。

        导航前先解锁刹车；等待循环中检测紧急停止并中止。

        Returns:
            CommandResult: 导航最终结果（SUCCEEDED / FAILED / CANCELLED）。
        """

        self._unlock_brake()

        handle = self._backend.start_command(
            "woosh_navigation", "navigate_to_map_pose",
            {"x": x, "y": y, "yaw": yaw, "z": z,
             "target_point_type": target_point_type, "frame_id": frame_id,
             "qx": qx, "qy": qy, "qz": qz, "qw": qw},
        )
        return self._wait_for_navigation(handle, timeout)

    def _wait_for_navigation(self, handle: CommandHandle, timeout: float) -> CommandResult:
        """等待导航完成或超时（借鉴 Atom _send_navigation_goal 的循环判断）。

        轮询 get_current_status（ros.navigation_state.is_navigating）；
        紧急停止或超时先 stop_command 取消导航，再返回后端登记结果。
        """

        start = time.time()
        while time.time() - start < timeout:

            if self._emergency_stop:
                logger.error("紧急停止，取消导航任务")
                return self._cancel_navigation_task(handle, "emergency stop")

            status = self._backend.get_command(resource="woosh_navigation", operation="get_current_status", target={})

            data = (status.output or {}).get("data") or {}

            if not data.get("is_navigating", False):
                break
                
            time.sleep(0.1)

        else:
            return self._cancel_navigation_task(handle, "navigation timeout")

        return self._backend.get_command(handle.command_id)

    def get_current_status(self) -> CommandResult:
        return self._backend.get_command(resource="woosh_navigation", operation="get_current_status", target={})

    def _cancel_navigation_task(self, handle: CommandHandle, reason: str) -> CommandResult:
        """取消当前导航任务（借鉴 Atom _cancel_navigation_task）。

        委托后端 stop_command：取消 action goal 并返回登记结果。
        """

        return self._backend.stop_command(handle.command_id, reason)

    def _unlock_brake(self) -> CommandHandle | None:
        """解锁刹车：连续发布 10 次 brake=False（借鉴 Atom _unlock_brake 多次重复发送）。

        紧急停止置位时不发送；返回最后一次发送的 handle。
        """

        if self._emergency_stop:
            return None
        handle: CommandHandle | None = None

        for _ in range(10):

            handle = self._backend.start_command("woosh_move", "unlock_brake", {"brake": False})

            time.sleep(0.05)

        return handle

    def stop(self) -> CommandHandle:
        """主动停止所有运动和导航（借鉴 Atom stop：零速发布 move_robot）。

        置位紧急停止，连续发布 10 次零速度 TwistStamped 停止底盘运动。
        """

        self._emergency_stop = True
        stop_target = {"dx": 0.0, "dy": 0.0, "dyaw": 0.0}
        handle: CommandHandle | None = None
        for _ in range(10):
            handle = self._backend.start_command("woosh_move", "move_robot", stop_target)
            time.sleep(0.02)
        return handle

