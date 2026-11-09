"""先验证单关节，再验证四关节；停止时不发新的运动指令。"""

from __future__ import annotations

from semantic_robot_skill_sdk import SkillContext, StopOutcome, StopRequest

from .controller import build_all_action, build_single_action
from .models import TorsoNativeInput, TorsoNativeResult, TorsoNativeState


async def run(ctx: SkillContext) -> None:
    ctx.check_cancelled()
    skill_input = ctx.input(TorsoNativeInput)
    state = ctx.load_state(TorsoNativeState, default=TorsoNativeState())

    if state.stage == "single":
        single_result = await ctx.execute(
            key="torso-native.single:1",
            action=build_single_action(skill_input.single),
        )
        if single_result.status != "succeeded":
            ctx.fail(
                "TORSO_SINGLE_FAILED",
                single_result.error_message or "躯干单关节动作失败",
                list(single_result.evidence_refs),
            )
            return
        single_id = (single_result.output or {}).get("invocation_id")
        if single_id:
            state.invocation_ids["single"] = str(single_id)
        state.evidence_refs.extend(single_result.evidence_refs)
        state.stage = "all"
        ctx.checkpoint(state)

    ctx.check_cancelled()
    if state.stage == "all":
        all_result = await ctx.execute(
            key="torso-native.all:1",
            action=build_all_action(skill_input.all),
        )
        if all_result.status != "succeeded":
            ctx.fail(
                "TORSO_ALL_FAILED",
                all_result.error_message or "躯干四关节动作失败",
                state.evidence_refs + list(all_result.evidence_refs),
            )
            return
        all_id = (all_result.output or {}).get("invocation_id")
        if all_id:
            state.invocation_ids["all"] = str(all_id)
        state.evidence_refs.extend(all_result.evidence_refs)
        ctx.checkpoint(state)
        ctx.complete(
            TorsoNativeResult(
                single_joint_id=skill_input.single.joint_id,
                single_target_pos=skill_input.single.target_pos,
                all_positions=skill_input.all.positions,
                invocation_ids=state.invocation_ids,
                evidence_refs=state.evidence_refs,
            )
        )


async def on_stop(ctx: SkillContext, request: StopRequest) -> StopOutcome:
    """Pilot 负责停止当前 Action；没有物理停止证据时保守报告未知。"""

    return ctx.stop_outcome(
        safe=False,
        summary=f"停止请求 {request.id} 已收到；躯干物理状态待确认",
        physical_state="unknown",
        requires_intervention=True,
        evidence_refs=ctx.recent_evidence(),
    )
