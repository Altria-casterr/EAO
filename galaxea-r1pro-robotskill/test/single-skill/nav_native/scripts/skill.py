"""nav_native 的两段 Stage 推进和停止入口。

1. navigate —— navigate_to_map_pose：导航到地图坐标系下的目标位姿（ability 内部同步等待完成）
2. verify   —— verify_arrival：复核结果（当前 ability 语义是腕部力传感持物检查）

每段用自己独立的参数模型（skill_input.navigate / skill_input.verify）。
"""

from __future__ import annotations

from semantic_robot_skill_sdk import SkillContext, StopOutcome, StopRequest

from .controller import build_navigate_action, build_stop_action, build_verify_arrival_action
from .models import NavNativeInput, NavNativeResult, NavNativeState


async def run(ctx: SkillContext) -> None:
    ctx.check_cancelled()
    skill_input = ctx.input(NavNativeInput)
    state = ctx.load_state(NavNativeState, default=NavNativeState())

    # 步骤 1：地图导航
    if state.stage == "navigate":
        result = await ctx.execute(
            key="nav-native.navigate:1",
            action=build_navigate_action(skill_input.navigate),
        )
        if result.status != "succeeded":
            ctx.fail(
                "NAVIGATE_FAILED",
                result.error_message or "地图导航失败",
                result.evidence_refs,
            )
            return
        state.stage = "verify"
        ctx.checkpoint(state)

    # 步骤 2：复核（腕力持物检查）
    if state.stage == "verify":
        result = await ctx.execute(
            key="nav-native.verify-arrival:1",
            action=build_verify_arrival_action(skill_input.verify),
        )
        if result.status != "succeeded":
            ctx.fail(
                "VERIFY_ARRIVAL_FAILED",
                result.error_message or "复核执行失败",
                result.evidence_refs,
            )
            return
        output = result.output or {}
        ctx.checkpoint(state)
        # verdict 只反映持物判定：空手跑纯导航测试时 holding_object=false / not_achieved
        # 属于正常观测，不代表导航失败，因此这里只如实回传，不据此报失败。
        ctx.complete(
            NavNativeResult(
                target_x=skill_input.navigate.target_x,
                target_y=skill_input.navigate.target_y,
                frame_id=skill_input.navigate.frame_id,
                verify_verdict=str(output.get("verdict", "uncertain")),
                holding_object=bool(output.get("holding_object", False)),
                evidence_refs=list(result.evidence_refs),
            )
        )


async def on_stop(ctx: SkillContext, request: StopRequest) -> StopOutcome:
    """停止底盘：调用 ability 的 navigation.stop（零速发布 + 急停置位）。

    注意（ability 侧已知缺陷）：navigation.stop 会把 SDK 里的 `_emergency_stop` 置位，
    而该标志**没有复位接口**，之后这个 WooshNavigation 实例的所有导航都会被直接取消，
    必须重启实例才能恢复。所以它只用于「必须立刻停住」的停止请求，不是常规结束路径；
    普通的 action 停止走 pilot 的 StopExecution（取消 action goal），不会触发这个闩锁。
    """

    result = await ctx.execute_stop(
        key=f"nav-native:stop:{request.id}",
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
