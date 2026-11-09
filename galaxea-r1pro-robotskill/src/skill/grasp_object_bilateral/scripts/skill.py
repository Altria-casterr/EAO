"""prepare → approach → lift → stand → verify；按检查点恢复，不自动重试物理动作。"""
from __future__ import annotations

import asyncio

from semantic_robot_skill_sdk import Action, ActionResult, SkillContext, StopOutcome, StopRequest
from . import controller, solver
from .models import BothPoses, GraspInput, GraspResult, GraspState

PHYSICAL = {"torso.move_to_posture_all", "motion.move_end_effector_both",
            "motion.move_end_effector_both_trajectory", "manipulation.force_align"}


class _StageFailed(Exception):
    """ctx.fail 只登记终态；显式中断当前 Stage，禁止继续发布后续动作。"""


def _report(ctx, state, status, summary):
    ctx.report(f"stage.{status}", stage=state.stage, stage_status=status,
               summary=summary, evidence_refs=state.evidence_refs)


def _fail(ctx, state, code, message):
    state.failure_code, state.failure_message = code, message
    ctx.checkpoint(state)
    _report(ctx, state, "failed", message)
    ctx.fail(code, message, state.evidence_refs)
    raise _StageFailed(message)


def _layers(output):
    """接受 Pilot 解包后的结果以及 Ability Execution 的 result/output 包装。"""
    if not isinstance(output, dict):
        return
    pending = [(output, 0)]
    while pending:
        value, depth = pending.pop(0)
        if value.get("status") in {"failed", "interrupted", "stopped"} or value.get("error"):
            raise ValueError(str(value.get("error") or value.get("status")))
        yield value
        if depth < 6:
            pending.extend((value[key], depth + 1) for key in ("result", "output")
                           if isinstance(value.get(key), dict))


def _items(result):
    for layer in _layers(result.output or {}):
        for item in layer.get("observations", []):
            if isinstance(item, dict):
                yield item
    for obs in result.observations:
        yield obs.model_dump(mode="json")


def _reading(result, name):

    for item in _items(result):

        value = item.get("value")
        if isinstance(value, dict) and value.get("name") == name:
            item = value

        if item.get("name") == name:
            data = item.get("data")

            if not isinstance(data, dict) or data.get("error"):
                raise ValueError(f"{name} 数据缺失或读取失败：{data}")

            return data

    raise ValueError(f"Ability 未返回 {name} 数据")


def _matched_center(result, relative_target_m, params):

    for layer in _layers(result.output or {}):

        if "best_match" not in layer:
            continue

        if layer.get("success") is not True or not isinstance(layer["best_match"], dict):
            raise ValueError("match_target 没有找到目标箱子")

        if layer.get("robot_frame") != params.relative_frame:
            raise ValueError("match_target 输出坐标系与目标相对坐标系不一致")

        best = layer["best_match"]
        # match_target 返回 best_match.center_robot 为 [x, y, z] 列表（世界/map 坐标约定下
        # 的机器人相对系），confidence 是 best_match 的同级字段。
        center = list(solver.point(best["center_robot"]))
        center[2] = relative_target_m[2]
        confidence = best.get("confidence")

        if confidence is None or not params.perception.minimum_confidence <= float(confidence) <= 1:
            raise ValueError("匹配目标缺少有效置信度")

        return solver.point(center)

    raise ValueError("Ability 未返回 match_target 结果")


def _refs(result):
    refs = list(result.evidence_refs)
    for obs in result.observations:
        refs.extend(obs.evidence_refs)
        refs.extend(obs.artifact_refs)
    # 当前 RobotObservation 常把原始条目放在 output.observations。
    if result.status == "succeeded":
        for item in _items(result):
            for value in (item, item.get("value", {})):
                if isinstance(value, dict):
                    refs.extend(value.get("evidence_refs") or [])
                    refs.extend(value.get("artifact_refs") or [])
    return list(dict.fromkeys(ref for ref in refs if isinstance(ref, str)))


async def _execute(ctx, state, key: str, action: Action, *, optional=False):
    ctx.check_cancelled()
    if key in state.results:
        result = state.results[key]
        if result.status != "succeeded" and not optional:
            _fail(ctx, state, "ACTION_STATE_UNKNOWN" if result.status == "interrupted" else "GRASP_ACTION_FAILED",
                  f"{action.type}：{result.error_message or result.status}")
        return result
    # 参数也保存到检查点，Action 成功后、结果保存前崩溃仍使用同一个 key/参数。
    if key not in state.actions:
        state.actions[key] = action
    action = state.actions[key]
    if action.type in PHYSICAL:
        state.physical_action_started = True
    ctx.checkpoint(state)
    result = await ctx.execute(f"grasp-object-bilateral:{key}", action)
    ctx.check_cancelled()
    refs = _refs(result)
    state.evidence_refs = list(dict.fromkeys([*state.evidence_refs, *refs]))
    if action.type == "robot.capture_rgbd":
        # 图像正文不进入检查点，仅保存可用引用与采集状态。
        result = result.model_copy(update={"output": None, "observations": [], "evidence_refs": refs})
    state.results[key] = result
    ctx.checkpoint(state)
    if action.type in PHYSICAL:
        # 物理动作结束后多等一会儿再进入下一步：给机构收敛/振动衰减留时间。
        await asyncio.sleep(2.0)
        ctx.check_cancelled()
    if result.status != "succeeded" and not optional:
        _fail(ctx, state, "ACTION_STATE_UNKNOWN" if result.status == "interrupted" else "GRASP_ACTION_FAILED",
              f"{action.type}：{result.error_message or result.status}")
    return result


async def _poses(ctx, params, state, prefix):
    poses = {}
    for side in ("left", "right"):
        result = await _execute(ctx, state, f"{prefix}:pose:{side}", controller.read_pose(side))
        poses[side] = solver.parse_pose(
            _reading(result, f"ee_pose_{side}"), params.relative_frame, state.base_to_relative)
    return BothPoses(**poses)


def _advance(ctx, state, following, summary):
    _report(ctx, state, "completed", summary)
    state.stage = following
    ctx.checkpoint(state)


async def _prepare(ctx, params, state):

    if params.torso.enabled:
        await _execute(ctx, state, "prepare:torso", controller.torso(params))

    floating_data = None

    if state.base_to_relative is None or state.relative_target_m is None:

        floating = await _execute(ctx, state, "prepare:floating", controller.read_state("floating_base"))
        floating_data = _reading(floating, "floating_base")

        if state.base_to_relative is None:
            state.base_to_relative = solver.floating_base_transform(floating_data, params.relative_frame)
            ctx.checkpoint(state)

    if state.relative_target_m is None:
        transform = await _execute(ctx, state, "prepare:transform", controller.read_state("transformer_martix"))

        state.relative_target_m = solver.relative_target(
            params, _reading(transform, "transformer_martix"),
            floating_data or state.base_to_relative.transform_data())

        ctx.checkpoint(state)

    if state.matched_center_m is None:
        # TODO 匹配现在使用m作为计量单位
        result = await _execute(ctx, state, "prepare:match", controller.match_target(params, state.relative_target_m))
        state.matched_center_m = _matched_center(result, state.relative_target_m, params)
        ctx.checkpoint(state)

    if state.capture_points is None:
        current = await _poses(ctx, params, state, "prepare")
        state.capture_points = solver.capture_points(params, state.matched_center_m, current)
        # 上游随机 Y 只求一次；任何运动开始前必须先保存结果。
        ctx.checkpoint(state)

    _advance(ctx, state, "approach", "目标匹配完成，已保存双手预备点")


async def _approach(ctx, params, state):

    if state.capture_points is None:
        raise ValueError("approach 缺少已保存的 capture_point")

    # 双手经笛卡尔轨迹到预备点；插值模式与逐轴顺序来自 approach_motion
    # （默认 sequential=True + axis_order="zyx"，单点超时 60s），与 lift 解耦。
    await _execute(ctx, state, "approach:both", controller.both(
        params, state.capture_points, trajectory=True, motion=params.approach_motion,
        label=f"双手按 {params.approach_motion.axis_order} 轨迹移动到 capture_point"))

    # TODO 力控抓取需要进行精细调整
    await _execute(ctx, state, "approach:left-force", controller.force(
        "left", params.approach_force, "左手沿 Y 负方向靠近至触阈"))

    state.left_contact = True
    ctx.checkpoint(state)

    await _execute(ctx, state, "approach:right-force", controller.force(
        "right", params.approach_force, "右手沿 Y 正方向靠近至触阈"))

    state.right_contact = True
    _advance(ctx, state, "lift", "左右两次 force_align 已完成")

async def _lift(ctx, params, state):

    if not state.left_contact or not state.right_contact:
        raise ValueError("lift 必须在左右接近完成之后")

    if state.lift_points is None:
        state.lift_points = solver.lift_points(await _poses(ctx, params, state, "lift:start"), lift_m=params.lift_m)
        ctx.checkpoint(state)

    # TODO 抬起过程中的使用 force_align 调整
    await _execute(ctx, state, "lift:up", controller.both(
        params, state.lift_points, trajectory=True,
        label=f"双手从实际接合位姿向上抬升 {params.lift_m} 米"))

    state.lifted = True
    ctx.checkpoint(state)
    if state.center_points is None:
        current = await _poses(ctx, params, state, "lift:center")
        state.center_points = solver.center_points(params, current)

        ctx.checkpoint(state)

    await _execute(ctx, state, "lift:retract", controller.both(
        params, state.center_points, trajectory=True, label="双手按 solver 点位携箱收回"))

    state.retracted = True
    _advance(ctx, state, "stand", f"抬升 {params.lift_m} 米并完成双手收箱")


async def _stand(ctx, params, state):
    if not state.lifted or not state.retracted:
        raise ValueError("stand 必须在抬升和携箱收回完成之后")

    await _execute(ctx, state, "stand:torso", controller.stand(params))
    state.stand_completed = True
    _advance(ctx, state, "verify", "躯干四关节回零完成")


async def _verify(ctx, params, state):

    if not state.stand_completed:
        raise ValueError("verify 必须在 stand 躯干回零完成之后")

    if not state.retracted or state.center_points is None or state.lift_points is None or state.capture_points is None:
        raise ValueError("verify 必须在携箱收回完成之后")

    if state.image_status == "pending":
        result = await _execute(ctx, state, "verify:image", controller.capture(params), optional=True)
        state.image_refs = [ref for ref in result.evidence_refs if ref.startswith(("artifact://", "pilot-artifact://"))]
        state.image_status = ("failed" if result.status != "succeeded" else
                              "available" if state.image_refs else "metadata_only")

        ctx.checkpoint(state)
        ctx.report("stage.evidence" if state.image_status == "available" else "stage.evidence_unavailable",
                   stage=state.stage, summary=f"RGB 日志采集：{state.image_status}", evidence_refs=state.image_refs)

    await _execute(ctx, state, "verify:force", controller.force(
        params.verify_side, params.verify_force, "单次 force_align 触阈判定持物"))

    state.verified = True
    ctx.checkpoint(state)

    _report(ctx, state, "completed", "单次 force_align 已完成，按指定判据认为持物")
    
    ctx.complete(GraspResult(
        object_ref=params.target.object_ref, relative_frame=params.relative_frame,
        matched_center_m=state.matched_center_m, verification_side=params.verify_side,
        force_threshold_n=params.verify_force.force_threshold_n,
        lift_distance_m=params.lift_m,
        capture_points=state.capture_points, lift_points=state.lift_points, center_points=state.center_points,
        image_status=state.image_status, image_refs=state.image_refs, evidence_refs=state.evidence_refs,
    ))


async def run(ctx: SkillContext) -> None:
    ctx.check_cancelled()

    params = ctx.input(GraspInput)

    state = ctx.load_state(GraspState, default=GraspState())

    if state.failure_code:
        ctx.fail(state.failure_code, state.failure_message or "本次执行已失败", state.evidence_refs)
        return

    _report(ctx, state, "running", f"执行双手抓取阶段：{state.stage}")

    try:
        try:
            await {"prepare": _prepare, "approach": _approach, "lift": _lift,
                   "stand": _stand, "verify": _verify}[state.stage](ctx, params, state)
        except (ValueError, KeyError, TypeError) as error:
            _fail(ctx, state, "GRASP_DATA_INVALID", str(error))
    except _StageFailed:
        return


async def on_stop(ctx: SkillContext, request: StopRequest) -> StopOutcome:
    state = ctx.load_state(GraspState, default=GraspState())
    # Pilot 负责取消当前 Action；Context 未提供物理停止回执，不能伪造已停。
    uncertain = state.physical_action_started
    return ctx.stop_outcome(
        safe=not uncertain, physical_state="unknown" if uncertain else "no_motion_started",
        requires_intervention=uncertain,
        summary=(f"停止请求 {request.id} 已收到，双手与躯干停止状态待确认" if uncertain else "准备阶段已停止，尚未发送物理动作"),
        evidence_refs=list(dict.fromkeys([*state.evidence_refs, *ctx.recent_evidence()])),
    )
