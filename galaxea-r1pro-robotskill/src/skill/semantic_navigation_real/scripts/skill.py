"""箱子作业导航：navigation → 可选 verify。"""

from __future__ import annotations

import math

from pydantic import ValidationError
from semantic_robot_skill_sdk import ActionResult, SkillContext, StopOutcome, StopRequest
from solver.Nav.navigation import NavigationSolverError

from .controller import (
    build_navigation_action,
    build_verify_action,
)
from .models import (
    SemanticNavigationInput,
    SemanticNavigationResult,
    SemanticNavigationState,
    VerifyOutput,
    VerifyInput,
)
from .solver import resolve_navigation_target


async def run(ctx: SkillContext) -> None:
    ctx.check_cancelled()
    skill_input = ctx.input(SemanticNavigationInput)
    state = ctx.load_state(SemanticNavigationState, default=SemanticNavigationState())
    if state.stage == "navigation":
        await _navigation(ctx, skill_input, state)
    elif state.stage == "verify":
        await _verify(ctx, skill_input, state)


def _report(ctx: SkillContext, state: SemanticNavigationState, status: str, summary: str) -> None:
    ctx.report(
        f"stage.{status}", stage=state.stage, stage_status=status,
        summary=summary, evidence_refs=state.evidence_refs,
    )


def _remember(state: SemanticNavigationState, result: ActionResult) -> None:
    refs = [*result.evidence_refs]
    for observation in result.observations:
        refs.extend(observation.evidence_refs)
        refs.extend(observation.artifact_refs)
    state.evidence_refs = list(dict.fromkeys([*state.evidence_refs, *refs]))


def _fail(ctx: SkillContext, state: SemanticNavigationState, code: str, message: str) -> None:
    _report(ctx, state, "failed", message)
    ctx.fail(code, message, state.evidence_refs)


async def _navigation(
    ctx: SkillContext, skill_input: SemanticNavigationInput, state: SemanticNavigationState,
) -> None:

    _report(ctx, state, "running", "计算箱子作业站位并导航到位")

    if state.navigation_target is None:
        try:
            state.navigation_target = resolve_navigation_target(skill_input)

        except NavigationSolverError as error:
            _fail(ctx, state, "NAV_TARGET_UNREACHABLE", str(error))
            return
        except ValueError as error:
            _fail(ctx, state, "NAV_TARGET_INVALID", str(error))
            return
        # 下发动作前保存具体目标；进程恢复不重新求解或更换 Action 参数。
        ctx.checkpoint(state)
    if not state.navigation_succeeded:
        result = await ctx.execute(
            key="semantic-navigation-real:navigation:1",
            action=build_navigation_action(state.navigation_target),
        )
        ctx.check_cancelled()
        _remember(state, result)
        if result.status != "succeeded":
            ctx.checkpoint(state)
            code = "ACTION_STATE_UNKNOWN" if result.status == "interrupted" else "NAVIGATION_FAILED"
            _fail(ctx, state, code, result.error_message or f"导航终态为 {result.status}")
            return
        # navigate_to_map_pose 成功已包含到位检测，不用力检测替代导航验收。
        state.navigation_succeeded = True
        ctx.checkpoint(state)
    _report(ctx, state, "completed", "导航 Ability 已确认到位")
    if skill_input.navigation_purpose == "approach_grasp":
        _complete(ctx, skill_input, state)
        return
    state.stage = "verify"
    ctx.checkpoint(state)


def _load_error(
    output: VerifyOutput, parameters: VerifyInput, expected_side: str,
) -> tuple[str, str] | None:
    check = output.carrying_check
    if not check.checked or check.errors:
        return "TOOL_LOAD_UNVERIFIED", "腕力未完整读取：" + "; ".join(check.errors or check.reasons)
    if set(check.tools) != {expected_side}:
        return "TOOL_LOAD_UNVERIFIED", f"力检测结果必须且只能包含 {expected_side} 侧"
    if check.force_axis != parameters.carrying_force_axis or not math.isclose(
        check.min_load_n, parameters.carrying_min_load_n, rel_tol=1e-9, abs_tol=1e-9,
    ):
        return "VERIFY_CONTRACT_MISMATCH", "力检测返回的方向或阈值与请求不一致"
    if any(tool.force_n is None for tool in check.tools.values()):
        return "TOOL_LOAD_UNVERIFIED", "力检测结果包含缺失的力数值"
    if not (
        output.verdict == "achieved" and output.holding_object and check.holding
        and all(tool.loaded and abs(tool.force_n) >= parameters.carrying_min_load_n
                for tool in check.tools.values())
    ):
        return "CARRYING_LOAD_NOT_CONFIRMED", f"未确认 {expected_side} 手仍携带箱子：" + "; ".join(
            check.reasons or ["腕力检测未通过"]
        )
    return None


async def _verify(
    ctx: SkillContext, skill_input: SemanticNavigationInput, state: SemanticNavigationState,
) -> None:
    if (
        skill_input.navigation_purpose != "carry_to_place" or skill_input.verify is None
        or skill_input.mode == "both" or not state.navigation_succeeded
        or state.navigation_target is None
    ):
        _fail(ctx, state, "INVALID_VERIFY_STATE", "verify 必须位于单手携物导航成功之后")
        return
    _report(ctx, state, "running", f"读取{skill_input.mode}侧腕力并确认携物状态")
    result = await ctx.execute(
        key="semantic-navigation-real:verify:1",
        action=build_verify_action(skill_input.mode, skill_input.verify),
    )
    ctx.check_cancelled()
    _remember(state, result)
    ctx.checkpoint(state)
    if result.status != "succeeded":
        _fail(ctx, state, "VERIFY_ACTION_FAILED", result.error_message or "力检测 Action 执行失败")
        return
    try:
        output = VerifyOutput.model_validate(result.output)
    except ValidationError:
        _fail(ctx, state, "VERIFY_OUTPUT_INVALID", "力检测结果缺失或不符合 WooshNavigation 返回契约")
        return
    error = _load_error(output, skill_input.verify, skill_input.mode)
    if error is not None:
        _fail(ctx, state, *error)
        return
    _report(ctx, state, "completed", f"{skill_input.mode}侧力检测通过，确认仍携带箱子")
    _complete(ctx, skill_input, state, output)


def _complete(
    ctx: SkillContext, skill_input: SemanticNavigationInput,
    state: SemanticNavigationState, output: VerifyOutput | None = None,
) -> None:
    ctx.check_cancelled()
    ctx.complete(SemanticNavigationResult(
        navigation_purpose=skill_input.navigation_purpose,
        mode=skill_input.mode,
        target_ref=skill_input.target.target_ref,
        navigation_target=state.navigation_target,
        holding_object=True if output is not None else None,
        load_verification="passed" if output is not None else "not_required",
        carrying_check=output.carrying_check if output is not None else None,
        evidence_refs=state.evidence_refs,
    ))


async def on_stop(ctx: SkillContext, request: StopRequest) -> StopOutcome:
    """Pilot 先取消当前 Action；不通过下发新运动 Action 充当停止。"""
    state = ctx.load_state(SemanticNavigationState, default=SemanticNavigationState())
    # 只有所有应执行的物理运动都已确认完成，后续才不会再有底盘运动。
    confirmed = state.navigation_succeeded
    return ctx.stop_outcome(
        safe=confirmed,
        summary=("导航已确认完成，后续流程已取消" if confirmed
                 else f"停止请求 {request.id} 已收到；当前底盘运动停止状态待确认"),
        physical_state="base_motion_completed" if confirmed else "unknown",
        requires_intervention=not confirmed,
        evidence_refs=list(dict.fromkeys([*state.evidence_refs, *ctx.recent_evidence()])),
    )
