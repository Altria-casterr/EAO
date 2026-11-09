from semantic_robot_skill_sdk import Action
from .models import action_timeout

ACTION_TYPES = {
    "single_pose": "motion.move_end_effector_single",
    "both_pose": "motion.move_end_effector_both",
    "single_trajectory": "motion.move_end_effector_single_trajectory",
    "both_trajectory": "motion.move_end_effector_both_trajectory",
    "force_align": "manipulation.force_align",
}


def build_action(request) -> Action:
    return Action(
        type=ACTION_TYPES[request.task],
        parameters=request.params.model_dump(mode="json", by_alias=True, exclude_none=True),
        timeout_seconds=action_timeout(request),
        label=f"EEF {request.task}",
    )
