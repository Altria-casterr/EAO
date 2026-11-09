"""gripper_native 的 Action 构造（新栈原生参数 + 自定义 actionType）。

每个 taskName 独立构造：各自的输入模型直接作为 Action 参数，互不共用、互不影响。
"""

from __future__ import annotations

from semantic_robot_skill_sdk import Action

from .models import SetOpeningBothParams, SetOpeningSingleParams


def build_set_opening_single_action(params: SetOpeningSingleParams) -> Action:
    """构造 gripper.set_opening_single（taskName: HandleSingleGripper）。"""

    return Action.from_model(
        action_type="gripper.set_opening_single",
        parameters=params,
        timeout_seconds=15,
        label="设置单侧夹爪开度",
    )


def build_set_opening_both_action(params: SetOpeningBothParams) -> Action:
    """构造 gripper.set_opening_both（taskName: HandleBothGripper）。"""

    return Action.from_model(
        action_type="gripper.set_opening_both",
        parameters=params,
        timeout_seconds=15,
        label="设置双侧夹爪开度",
    )
