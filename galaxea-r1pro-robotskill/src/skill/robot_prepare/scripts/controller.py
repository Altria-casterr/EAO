"""仅构造已有 Ability Action，不直接发布 ROS 指令。"""
from semantic_robot_skill_sdk import Action
from .models import PrepareInput

TYPES = {
    'torso': 'torso.move_to_posture_all',
    'arms': 'motion.move_to_posture_both',
}
LABELS = {'torso': '躯干四关节回零', 'arms': '双臂到达预备关节角'}

def build_action(stage: str, params: PrepareInput) -> Action:
    settings = getattr(params, stage)
    return Action(type=TYPES[stage], parameters=settings.model_dump(mode='json'),
                  timeout_seconds=settings.timeout + 15, label=LABELS[stage])
