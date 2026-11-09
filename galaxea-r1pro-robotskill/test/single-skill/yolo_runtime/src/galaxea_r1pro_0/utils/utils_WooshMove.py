"""WooshMove 专用功能函数层（相对运动闭环，借鉴 Atom WooshMove/Utils）。"""

from __future__ import annotations

import logging
import math
from typing import Any

logger = logging.getLogger(__name__)

WHEELBASE = 0.496  # 轴距（米，对齐 Atom WooshMove.WHEELBASE）
INVALID_LOCALIZATION = -999.0  # 定位不可用标志（对齐 Atom get_localization_value）

def _record_initial_localization(data: dict | None) -> dict | None:
    """记录运动初始定位（对齐 Atom record_initial_localization）。"""

    if not data or not data.get("localization_available"):
        return None
    return {
        "position": data.get("position_global"),
        "orientation": data.get("orientation_global"),
    }

def _get_localization_value(
    initial: dict | None,
    latest: dict | None,
    axis: str) -> float:
    """基于定位计算机器人本体坐标系运动值（x/y 为距离，z 为角度，对齐 Atom get_localization_value）。

    Returns:
        float: 运动值；定位不可用返回 INVALID_LOCALIZATION。
    """

    if not initial or not latest or not latest.get("localization_available"):
        return INVALID_LOCALIZATION

    current_pos = latest.get("position_global") or {}
    current_ori = latest.get("orientation_global") or {}
    initial_pos = initial.get("position") or {}
    initial_ori = initial.get("orientation") or {}

    if axis == "z":
        current_yaw = float(current_ori.get("yaw") or 0.0) % 360.0
        initial_yaw = float(initial_ori.get("yaw") or 0.0) % 360.0
        angle_diff = current_yaw - initial_yaw
        if angle_diff > 180.0:
            angle_diff -= 360.0
        elif angle_diff < -180.0:
            angle_diff += 360.0
        return angle_diff

    if axis in ("x", "y"):
        d_x_map = float(current_pos.get("x") or 0.0) - float(initial_pos.get("x") or 0.0)
        d_y_map = float(current_pos.get("y") or 0.0) - float(initial_pos.get("y") or 0.0)
        yaw_rad = math.radians(float(initial_ori.get("yaw") or 0.0))
        # 地图增量 → 机器人本体坐标系：与 test_WooshMove.py 的前向投影一致
        dx_body = d_x_map * math.cos(yaw_rad) + d_y_map * math.sin(yaw_rad)
        dy_body = -d_x_map * math.sin(yaw_rad) + d_y_map * math.cos(yaw_rad)
        current = -dx_body if axis == "x" else -dy_body
        return current

    return INVALID_LOCALIZATION


def _integrate_feedback(sdk: Any, axis: str, dt: float) -> float:
    """底盘反馈积分：返回最近 dt 内的位移/角度增量（对齐 Atom _feedback_callback）。

    无反馈数据时返回 0（等待下一轮）。
    """

    result = sdk.sensors.get_chassis_feedback()
    data = (result.output or {}).get("data") or {}
    speeds = data.get("wheel_speeds") or {}
    angles = data.get("wheel_angles") or {}
    left_speed = float(speeds.get("left_front") or 0.0)
    right_speed = float(speeds.get("right_front") or 0.0)
    left_angle = float(angles.get("left_front") or 0.0)
    right_angle = float(angles.get("right_front") or 0.0)

    front_avg_speed = (left_speed + right_speed) / 2.0
    front_avg_angle = (left_angle + right_angle) / 2.0

    if axis == "z":
        angular_z = front_avg_speed * math.tan(front_avg_angle) / WHEELBASE
        return math.degrees(angular_z) * dt

    linear_x = front_avg_speed * math.cos(left_angle)
    linear_y = front_avg_speed * math.sin(left_angle)
    return (linear_x if axis == "x" else linear_y) * dt
