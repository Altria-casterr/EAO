"""Skill 内修正原始目标的地图 Y，再交给共享几何 solver。"""

import math

from solver.Nav.navigation import solve_box_navigation_pose

from .models import NavigateParameters, SemanticNavigationInput


YAW_TOLERANCE_RAD = 1e-6


def resolve_target_position(skill_input: SemanticNavigationInput) -> tuple[float, float, float]:
    """仅偏移输入副本；不改地图、输入模型或共享 solver。"""
    x, y, z = skill_input.target.position_m
    if skill_input.mode == "both" or skill_input.move_y_m == 0:
        return x, y, z
    if skill_input.move_y_m is None:
        raise ValueError("left/right 模式必须提供 move_y_m")
    yaw = math.atan2(math.sin(skill_input.target.yaw_rad), math.cos(skill_input.target.yaw_rad))
    if math.isclose(yaw, 0.0, rel_tol=0.0, abs_tol=YAW_TOLERANCE_RAD):
        heading_sign = 1
    elif math.isclose(abs(yaw), math.pi, rel_tol=0.0, abs_tol=YAW_TOLERANCE_RAD):
        heading_sign = -1
    else:
        raise ValueError("非零 move_y_m 仅支持 yaw=0 或 ±π（模 2π，容差 1e-6 rad）")
    side_sign = 1 if skill_input.mode == "left" else -1
    return x, y + side_sign * heading_sign * skill_input.move_y_m, z


def resolve_navigation_target(skill_input: SemanticNavigationInput) -> NavigateParameters:
    station = solve_box_navigation_pose(
        resolve_target_position(skill_input),
        target_yaw_rad=skill_input.target.yaw_rad,
        frame_id=skill_input.frame_id,
    )
    return NavigateParameters.model_validate(
        station.to_action_parameters(
            target_point_type=skill_input.target_point_type,
            timeout=skill_input.timeout,
        )
    )
