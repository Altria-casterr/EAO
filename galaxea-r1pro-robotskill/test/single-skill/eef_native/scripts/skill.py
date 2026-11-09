"""一次执行一种 EEF 动作；失败不触发另一种运动。"""
from semantic_robot_skill_sdk import SkillContext, StopOutcome, StopRequest
from .controller import build_action
from .models import EEFInput, EEFResult, EEFState


async def run(ctx: SkillContext) -> None:
    request = ctx.input(EEFInput).root
    state = ctx.load_state(EEFState, default=EEFState())
    ctx.check_cancelled()
    result = await ctx.execute(key=f"eef-native.{request.task}:1", action=build_action(request))
    state.evidence_refs.extend(result.evidence_refs)
    if result.status != "succeeded":
        ctx.fail("EEF_ACTION_FAILED", result.error_message or f"Ability 状态: {result.status}", state.evidence_refs)
        return
    state.invocation_id = str((result.output or {}).get("invocation_id") or "") or None
    state.stage = "completed"
    ctx.checkpoint(state)
    ctx.complete(EEFResult(
        task=request.task,
        invocation_id=state.invocation_id,
        ability_status=result.status,
        ability_output=result.output or {},
        evidence_refs=state.evidence_refs,
    ))


async def on_stop(ctx: SkillContext, request: StopRequest) -> StopOutcome:
    return ctx.stop_outcome(
        safe=False,
        summary=f"EEF 停止请求 {request.id} 已提交；需确认机械臂反馈",
        physical_state="unknown",
        requires_intervention=True,
        evidence_refs=ctx.recent_evidence(),
    )
