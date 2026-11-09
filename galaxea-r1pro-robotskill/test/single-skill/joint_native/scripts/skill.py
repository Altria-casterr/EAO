"""先控制单臂，再控制双臂；停止时只报告物理状态未知。"""

from semantic_robot_skill_sdk import SkillContext, StopOutcome, StopRequest

from .controller import build_both_action, build_single_action
from .models import JointNativeInput, JointNativeResult, JointNativeState


async def run(ctx: SkillContext) -> None:
    ctx.check_cancelled()
    skill_input = ctx.input(JointNativeInput)
    state = ctx.load_state(JointNativeState, default=JointNativeState())

    if state.stage == "single":
        single_result = await ctx.execute(
            key="joint-native.single:1",
            action=build_single_action(skill_input.single),
        )
        if single_result.status != "succeeded":
            ctx.fail(
                "JOINT_SINGLE_FAILED",
                single_result.error_message or "单臂关节动作失败",
                list(single_result.evidence_refs),
            )
            return
        invocation_id = (single_result.output or {}).get("invocation_id")
        if invocation_id:
            state.invocation_ids["single"] = str(invocation_id)
        state.evidence_refs.extend(single_result.evidence_refs)
        state.stage = "both"
        ctx.checkpoint(state)

    ctx.check_cancelled()
    if state.stage == "both":
        both_result = await ctx.execute(
            key="joint-native.both:1",
            action=build_both_action(skill_input.both),
        )
        if both_result.status != "succeeded":
            ctx.fail(
                "JOINT_BOTH_FAILED",
                both_result.error_message or "双臂关节动作失败",
                state.evidence_refs + list(both_result.evidence_refs),
            )
            return
        invocation_id = (both_result.output or {}).get("invocation_id")
        if invocation_id:
            state.invocation_ids["both"] = str(invocation_id)
        state.evidence_refs.extend(both_result.evidence_refs)
        ctx.checkpoint(state)
        ctx.complete(
            JointNativeResult(
                single_arm_side=skill_input.single.arm_side,
                single_positions=skill_input.single.positions,
                left_positions=skill_input.both.left_positions,
                right_positions=skill_input.both.right_positions,
                invocation_ids=state.invocation_ids,
                evidence_refs=state.evidence_refs,
            )
        )


async def on_stop(ctx: SkillContext, request: StopRequest) -> StopOutcome:
    return ctx.stop_outcome(
        safe=False,
        summary=f"停止请求 {request.id} 已收到；手臂物理状态待确认",
        physical_state="unknown",
        requires_intervention=True,
        evidence_refs=ctx.recent_evidence(),
    )
