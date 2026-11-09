"""woosh_move_native 的 Stage 推进和停止入口。"""

from __future__ import annotations

from semantic_robot_skill_sdk import SkillContext, StopOutcome, StopRequest

from .controller import build_relative_move_action, build_stop_action
from .models import WooshMoveInput, WooshMoveResult, WooshMoveState


async def run(ctx: SkillContext) -> None:
    ctx.check_cancelled()
    skill_input = ctx.input(WooshMoveInput)
    state = ctx.load_state(WooshMoveState, default=WooshMoveState())

    if state.stage != "relative_move":
        ctx.fail("INVALID_STAGE", f"未知 Stage：{state.stage}")
        return

    result = await ctx.execute(
        key="woosh-move-native.relative-move:1",
        action=build_relative_move_action(skill_input),
    )
    if result.status != "succeeded":
        ctx.fail(
            "RELATIVE_MOVE_FAILED",
            result.error_message or "底盘相对移动失败",
            result.evidence_refs,
        )
        return

    state.invocation_id = (result.output or {}).get("invocation_id")
    ctx.checkpoint(state)
    ctx.complete(
        WooshMoveResult(
            axis=skill_input.axis,
            value=skill_input.value,
            evidence_refs=list(result.evidence_refs),
        )
    )


async def on_stop(ctx: SkillContext, request: StopRequest) -> StopOutcome:
    """停止底盘：调用 ability 的 navigation.stop（紧急停止 + 零速发布）。

    这里可以安全地发停止 action —— navigation.stop 是专门声明的停止语义
    （execution 层面 StopExecution 之外的显式兜底），不是移动指令：
    若改发 navigation.relative_move，ability 会把它当成一条**新的相对移动命令**，
    让底盘继续运动，正是停止时最危险的行为。
    """

    result = await ctx.execute_stop(
        key=f"woosh-move-native:stop:{request.id}",
        action=build_stop_action(),
    )
    safe = result.status == "succeeded"
    return ctx.stop_outcome(
        safe=safe,
        summary="底盘已停止（navigation.stop）" if safe else "无法确认底盘已停止",
        physical_state="base_motion_stopped" if safe else "unknown",
        requires_intervention=not safe,
        evidence_refs=list(result.evidence_refs),
    )
