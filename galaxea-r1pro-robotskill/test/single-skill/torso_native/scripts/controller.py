"""每个原生 task 使用独立参数模型构造 Action。"""

from __future__ import annotations

from semantic_robot_skill_sdk import Action

from .models import AllTorsoParams, SingleTorsoParams


def build_single_action(params: SingleTorsoParams) -> Action:
    return Action.from_model(
        action_type="torso.move_to_target_pos_single",
        parameters=params,
        timeout_seconds=max(15.0, params.timeout + 5.0),
        label="设置单个躯干关节",
    )


def build_all_action(params: AllTorsoParams) -> Action:
    return Action.from_model(
        action_type="torso.move_to_posture_all",
        parameters=params,
        timeout_seconds=max(15.0, params.timeout + 5.0),
        label="设置四个躯干关节",
    )
