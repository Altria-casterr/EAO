"""gripper_native 的两段 Stage 推进和停止入口。

1. set_opening_single —— 单侧夹爪到目标开度
2. set_opening_both   —— 双侧夹爪到目标开度

每段用自己独立的输入模型（skill_input.single / skill_input.both）。
"""

from __future__ import annotations

from semantic_robot_skill_sdk import SkillContext, StopOutcome, StopRequest

from .controller import build_set_opening_both_action, build_set_opening_single_action
from .models import GripperNativeInput, GripperNativeResult, GripperNativeState


async def run(ctx: SkillContext) -> None:
    ctx.check_cancelled()
    skill_input = ctx.input(GripperNativeInput)
    state = ctx.load_state(GripperNativeState, default=GripperNativeState())

    # 步骤 1：单侧夹爪
    if state.stage == "set_opening_single":
        result = await ctx.execute(
            key="gripper-native.set-opening-single:1",
            action=build_set_opening_single_action(skill_input.single),
        )
        if result.status != "succeeded":
            ctx.fail(
                "SET_OPENING_SINGLE_FAILED",
                result.error_message or "单侧夹爪开度设置失败",
                result.evidence_refs,
            )
            return
        state.invocation_ids["single"] = (result.output or {}).get("invocation_id", "")
        state.stage = "set_opening_both"
        ctx.checkpoint(state)

    # 步骤 2：双侧夹爪
    if state.stage == "set_opening_both":
        result = await ctx.execute(
            key="gripper-native.set-opening-both:1",
            action=build_set_opening_both_action(skill_input.both),
        )
        if result.status != "succeeded":
            ctx.fail(
                "SET_OPENING_BOTH_FAILED",
                result.error_message or "双侧夹爪开度设置失败",
                result.evidence_refs,
            )
            return
        state.invocation_ids["both"] = (result.output or {}).get("invocation_id", "")
        ctx.checkpoint(state)
        ctx.complete(
            GripperNativeResult(
                single_hand=skill_input.single.hand,
                single_target_position=skill_input.single.position,
                both_left_position=skill_input.both.left_position,
                both_right_position=skill_input.both.right_position,
                evidence_refs=list(result.evidence_refs),
            )
        )


async def on_stop(ctx: SkillContext, request: StopRequest) -> StopOutcome:
    """停止时把双爪一起送回目标开度，保证两侧都处于已知状态。"""

    skill_input = ctx.input(GripperNativeInput)
    result = await ctx.execute_stop(
        key=f"gripper-native:stop:{request.id}",
        action=build_set_opening_both_action(skill_input.both),
    )
    safe = result.status == "succeeded"
    return ctx.stop_outcome(
        safe=safe,
        summary="双爪已回到目标开度" if safe else "无法确认夹爪状态",
        physical_state="target_opening_confirmed" if safe else "unknown",
        requires_intervention=not safe,
        evidence_refs=list(result.evidence_refs),
    )
