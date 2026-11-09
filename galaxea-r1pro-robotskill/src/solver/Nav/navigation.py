"""箱子目标点到 R1 Pro 底盘导航目标位姿的纯几何 Solver。

该模块供 Robot Skill 直接导入，不访问 ROS、Robot SDK、文件系统或网络，也不依赖
NumPy。输入目标和输出站位都位于同一个地图坐标系；输入 yaw 是该坐标系中的绝对
作业朝向。Solver 只负责站位几何与名义双臂可达性，路线避障仍由
``WooshNavigation.navigate_to_map_pose`` 负责。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from numbers import Real
from typing import Mapping, Sequence


SEARCH_REACH_RATIOS: tuple[float, ...] = (0.65, 0.60, 0.55, 0.50)


class NavigationSolverError(RuntimeError):
    """输入有效，但没有找到满足名义双臂可达性的底盘站位。"""


@dataclass(frozen=True)
class NavSolverConfig:
    """R1 Pro 站位几何参数。

    默认臂长、肩高和肩宽来自随原始 Nav Solver 分发的
    ``r1_pro_with_hand.urdf``。``longitudinal_offset_m`` 沿目标作业方向调整站位：
    正值让底盘离目标更远，负值让底盘更近；``lateral_offset_m`` 沿机器人左侧
    方向调整。两个偏置都会随目标 yaw 旋转，不绑定地图 X/Y 轴。
    """

    avg_arm_length_m: float = 0.9126
    shoulder_height_base_m: float = 1.4453
    shoulder_height_min_m: float = 0.30
    shoulder_height_max_m: float = 1.85
    optimal_reach_ratio: float = 0.65
    min_reach_ratio: float = 0.15
    max_reach_ratio: float = 0.95
    safe_min_distance_m: float = 0.20
    shoulder_offset_x_m: float = -0.0795
    shoulder_half_width_m: float = 0.1707
    longitudinal_offset_m: float = 0.0
    lateral_offset_m: float = 0.0
    base_z_m: float = 0.0
    search_reach_ratios: tuple[float, ...] = SEARCH_REACH_RATIOS

    def __post_init__(self) -> None:
        numeric = {
            "avg_arm_length_m": self.avg_arm_length_m,
            "shoulder_height_base_m": self.shoulder_height_base_m,
            "shoulder_height_min_m": self.shoulder_height_min_m,
            "shoulder_height_max_m": self.shoulder_height_max_m,
            "optimal_reach_ratio": self.optimal_reach_ratio,
            "min_reach_ratio": self.min_reach_ratio,
            "max_reach_ratio": self.max_reach_ratio,
            "safe_min_distance_m": self.safe_min_distance_m,
            "shoulder_offset_x_m": self.shoulder_offset_x_m,
            "shoulder_half_width_m": self.shoulder_half_width_m,
            "longitudinal_offset_m": self.longitudinal_offset_m,
            "lateral_offset_m": self.lateral_offset_m,
            "base_z_m": self.base_z_m,
        }
        for name, value in numeric.items():
            if not math.isfinite(float(value)):
                raise ValueError(f"{name} 必须是有限数值")
        if self.avg_arm_length_m <= 0:
            raise ValueError("avg_arm_length_m 必须大于 0")
        if self.shoulder_height_min_m > self.shoulder_height_max_m:
            raise ValueError("shoulder_height_min_m 不能大于 shoulder_height_max_m")
        if not 0 < self.min_reach_ratio < self.max_reach_ratio <= 1:
            raise ValueError("臂展比例必须满足 0 < min < max <= 1")
        if not self.min_reach_ratio <= self.optimal_reach_ratio <= self.max_reach_ratio:
            raise ValueError("optimal_reach_ratio 必须位于允许的臂展比例范围内")
        if self.safe_min_distance_m <= 0:
            raise ValueError("safe_min_distance_m 必须大于 0")
        if not self.search_reach_ratios:
            raise ValueError("search_reach_ratios 不能为空")
        for ratio in self.search_reach_ratios:
            if not math.isfinite(float(ratio)) or not 0 < ratio <= 1:
                raise ValueError("search_reach_ratios 必须全部位于 (0, 1] 区间")


@dataclass(frozen=True)
class NavigationPose:
    """可直接转换为 ``navigate_to_map_pose`` 参数的底盘目标。"""

    x: float
    y: float
    z: float
    yaw_rad: float
    frame_id: str
    reach_ratio: float
    shoulder_height_m: float
    left_reach_score: float
    right_reach_score: float

    @property
    def orientation_xyzw(self) -> tuple[float, float, float, float]:
        """返回绕 Z 轴旋转的四元数 ``[qx, qy, qz, qw]``。"""

        half = self.yaw_rad / 2.0
        return (0.0, 0.0, math.sin(half), math.cos(half))

    def to_action_parameters(
        self,
        *,
        target_point_type: int = 0,
        timeout: float = 60.0,
    ) -> dict[str, object]:
        """生成 WooshNavigation ``navigate_to_map_pose`` 的扁平参数。"""

        if isinstance(target_point_type, bool) or not isinstance(target_point_type, int):
            raise ValueError("target_point_type 必须是整数")
        if not math.isfinite(float(timeout)) or timeout <= 0:
            raise ValueError("timeout 必须是大于 0 的有限数值")
        return {
            "target_x": self.x,
            "target_y": self.y,
            "target_orientation": list(self.orientation_xyzw),
            "target_point_type": target_point_type,
            "frame_id": self.frame_id,
            "timeout": float(timeout),
        }


def _finite_float(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} 必须是数值")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} 必须是有限数值")
    return result


def _parse_target(
    target_position: Sequence[float] | Mapping[str, object],
    *,
    target_yaw_rad: float | None,
) -> tuple[float, float, float, float]:
    """兼容 Skill 常用字典及原 Solver 的 3/4/6 维序列输入。"""

    if isinstance(target_position, Mapping):
        try:
            x = _finite_float(target_position["x"], "target.x")
            y = _finite_float(target_position["y"], "target.y")
            z = _finite_float(target_position["z"], "target.z")
        except KeyError as exc:
            raise ValueError(f"目标字典缺少字段：{exc.args[0]}") from exc
        embedded_yaw = target_position.get("yaw_rad", 0.0)
    else:
        if isinstance(target_position, (str, bytes)) or not isinstance(
            target_position, Sequence
        ):
            raise ValueError("目标点必须是序列或包含 x/y/z 的字典")
        if len(target_position) not in {3, 4, 6}:
            raise ValueError(
                "目标点必须是 [x,y,z]、[x,y,z,yaw] 或 "
                "[x,y,z,roll,pitch,yaw]"
            )
        x = _finite_float(target_position[0], "target.x")
        y = _finite_float(target_position[1], "target.y")
        z = _finite_float(target_position[2], "target.z")
        embedded_yaw = (
            target_position[3]
            if len(target_position) == 4
            else target_position[5]
            if len(target_position) == 6
            else 0.0
        )
    yaw = _finite_float(
        embedded_yaw if target_yaw_rad is None else target_yaw_rad,
        "target.yaw_rad",
    )
    return x, y, z, _normalize_angle(yaw)


def _normalize_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def _shoulder_height(target_z: float, config: NavSolverConfig) -> float:
    return min(
        max(target_z, config.shoulder_height_min_m),
        config.shoulder_height_max_m,
    )


def _horizontal_distance(
    target_z: float,
    shoulder_z: float,
    reach_ratio: float,
    config: NavSolverConfig,
) -> float:
    reach = config.avg_arm_length_m * reach_ratio
    vertical = target_z - shoulder_z
    if abs(vertical) < reach * 0.99:
        horizontal = math.sqrt(max(0.0, reach * reach - vertical * vertical))
    else:
        horizontal = reach * 0.20
    return min(
        max(horizontal, config.safe_min_distance_m),
        config.avg_arm_length_m * 0.90,
    )


def _station(
    target: tuple[float, float, float, float],
    shoulder_z: float,
    reach_ratio: float,
    config: NavSolverConfig,
) -> tuple[float, float, float, float]:
    x, y, z, yaw = target
    distance = (
        _horizontal_distance(z, shoulder_z, reach_ratio, config)
        + config.longitudinal_offset_m
    )
    if distance < config.safe_min_distance_m:
        raise NavigationSolverError(
            "纵向偏置使底盘站位小于最小安全距离"
        )
    forward_x, forward_y = math.cos(yaw), math.sin(yaw)
    left_x, left_y = -forward_y, forward_x
    return (
        x - distance * forward_x + config.lateral_offset_m * left_x,
        y - distance * forward_y + config.lateral_offset_m * left_y,
        config.base_z_m,
        yaw,
    )


def _arm_reach_score(
    station: tuple[float, float, float, float],
    target: tuple[float, float, float, float],
    shoulder_z: float,
    side: str,
    config: NavSolverConfig,
) -> tuple[bool, float]:
    dx, dy = target[0] - station[0], target[1] - station[1]
    cos_yaw, sin_yaw = math.cos(station[3]), math.sin(station[3])
    local_x = cos_yaw * dx + sin_yaw * dy
    local_y = -sin_yaw * dx + cos_yaw * dy
    local_z = target[2] - station[2]
    shoulder_y = config.shoulder_half_width_m if side == "left" else -config.shoulder_half_width_m
    distance = math.sqrt(
        (local_x - config.shoulder_offset_x_m) ** 2
        + (local_y - shoulder_y) ** 2
        + (local_z - shoulder_z) ** 2
    )
    min_reach = config.avg_arm_length_m * config.min_reach_ratio
    max_reach = config.avg_arm_length_m * config.max_reach_ratio
    if not min_reach < distance < max_reach or local_x <= 0.03:
        return False, 0.0

    optimal = config.avg_arm_length_m * config.optimal_reach_ratio
    distance_score = max(
        0.0,
        1.0 - abs(distance - optimal) / (max_reach - min_reach),
    )
    lateral_score = (
        0.5 + 0.5 * math.tanh(local_y / 0.2)
        if side == "left"
        else 0.5 - 0.5 * math.tanh(local_y / 0.2)
    )
    height_score = max(0.0, 1.0 - abs(local_z - shoulder_z) / 0.4)
    score = 0.35 * distance_score + 0.40 * lateral_score + 0.25 * height_score
    return True, min(max(score, 0.0), 1.0)


def solve_box_navigation_pose(
    target_position: Sequence[float] | Mapping[str, object],
    config: NavSolverConfig | None = None,
    *,
    target_yaw_rad: float | None = None,
    frame_id: str = "map",
) -> NavigationPose:
    """由箱子目标点计算满足名义双臂可达性的底盘导航目标。

    支持以下输入：

    - ``[x, y, z]``，默认 yaw 为 0；
    - ``[x, y, z, yaw]``；
    - ``[x, y, z, roll, pitch, yaw]``，只消费 yaw；
    - ``{"x": ..., "y": ..., "z": ..., "yaw_rad": ...}``。

    yaw 是 ``frame_id`` 坐标系中的绝对作业朝向。函数要求箱子中心对左右臂都
    名义可达；它不替代后续抓取规划，也不检查地图障碍和路径可达性。
    """

    if not isinstance(frame_id, str) or not frame_id.strip():
        raise ValueError("frame_id 不能为空")
        
    cfg = config or NavSolverConfig()
    target = _parse_target(target_position, target_yaw_rad=target_yaw_rad)
    shoulder_z = _shoulder_height(target[2], cfg)
    tried: list[float] = []
    ratios = (cfg.optimal_reach_ratio, *cfg.search_reach_ratios)
    for ratio_value in ratios:
        ratio = float(ratio_value)
        if ratio in tried:
            continue
        tried.append(ratio)
        station = _station(target, shoulder_z, ratio, cfg)
        left_ok, left_score = _arm_reach_score(
            station, target, shoulder_z, "left", cfg
        )
        right_ok, right_score = _arm_reach_score(
            station, target, shoulder_z, "right", cfg
        )
        if left_ok and right_ok:
            return NavigationPose(
                x=round(station[0], 6),
                y=round(station[1], 6),
                z=round(station[2], 6),
                yaw_rad=round(station[3], 6),
                frame_id=frame_id.strip(),
                reach_ratio=ratio,
                shoulder_height_m=round(shoulder_z, 6),
                left_reach_score=round(left_score, 6),
                right_reach_score=round(right_score, 6),
            )

    raise NavigationSolverError(
        f"箱子目标 ({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}) "
        "在所有候选站位下不能同时满足左右臂名义可达性"
    )


def solve_navigation_pose(
    target_position: Sequence[float] | Mapping[str, object],
    config: NavSolverConfig | None = None,
    *,
    target_yaw_rad: float | None = None,
    frame_id: str = "map",
) -> NavigationPose:
    """兼容旧调用名；语义与 :func:`solve_box_navigation_pose` 相同。"""

    return solve_box_navigation_pose(
        target_position,
        config,
        target_yaw_rad=target_yaw_rad,
        frame_id=frame_id,
    )
