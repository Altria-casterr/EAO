"""将已经校验的输入构造为 Pilot Action。"""

from semantic_robot_skill_sdk import Action

from .models import BothArmsParams, SingleArmParams


def build_single_action(params: SingleArmParams) -> Action:
    return Action(
        type="motion.move_to_posture_single",
        parameters=params.model_dump(mode="json", exclude_none=True),
        timeout_seconds=max(15.0, params.timeout + 5.0),
        label="设置单臂七个关节",
    )


def build_both_action(params: BothArmsParams) -> Action:
    return Action(
        type="motion.move_to_posture_both",
        parameters=params.model_dump(mode="json", exclude_none=True),
        timeout_seconds=max(15.0, params.timeout + 5.0),
        label="设置双臂十四个关节",
    )
