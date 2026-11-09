"""prepare → approach → lift → stand → verify；按检查点恢复；仅外拉持箱倍率检测未通过时有界重抓。
单手抓箱：抓取侧先接近+力控+上抬+沿 y 拉箱，辅助手再 y_move+target+力控+上抬。"""
from __future__ import annotations

import asyncio
import json
import math

from semantic_robot_skill_sdk import Action, SkillContext, StopOutcome, StopRequest
from . import controller, solver
from .models import BothPoses, GraspInput, GraspResult, GraspState, PullForceCheck

# 单手两段抬升的固定距离（米）：抓取侧与辅助手各自上抬该值。
# 双手 lift 段不用它，而用输入字段 lift_m（默认 0.1）。
GRASP_LIFT_M = 0.10
# 注：辅助手最终 Z 的下降量（z_lift）由 solver.OTHER_ARM_Z_LIFT_M 定义并在
# solver.other_arm_position 内部使用，本文件不再保留同名副本（避免两处取值不一致）。

PHYSICAL = {"torso.move_to_posture_all", "motion.move_end_effector_both",
            "motion.move_end_effector_both_trajectory", "manipulation.force_align",
            "motion.move_end_effector_single", "motion.move_end_effector_single_trajectory"}


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
        # 与双手 Skill 一致：center_robot 是 [x, y, z]，confidence 在 best_match 上。
        # 复制坐标后覆盖高度，不改写 ActionResult 中的感知证据。
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


def _attempt_key(state, key):
    if key.startswith(("approach:", "prepare:pose:")) or key == "prepare:match":
        return f"attempt:{state.grasp_attempt}:{key}"
    return key


async def _execute(ctx, state, key: str, action: Action, *, optional=False):
    key = _attempt_key(state, key)
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
    result = await ctx.execute(f"grasp-object-unilateral:{key}", action)
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


async def _pose(ctx, params, state, prefix, side):
    """仅采集指定手的反馈；独立 key 确保接触前后的读数不会混用。"""
    result = await _execute(ctx, state, f"{prefix}:pose:{side}", controller.read_pose(side))
    return solver.parse_pose(
        _reading(result, f"ee_pose_{side}"), params.relative_frame, state.base_to_relative)


def _advance(ctx, state, following, summary):
    _report(ctx, state, "completed", summary)
    state.stage = following
    ctx.checkpoint(state)


async def _prepare(ctx, params, state):

    if params.torso.enabled and state.grasp_attempt == 0:
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
        grasp_point = solver.capture_point(params, state.matched_center_m, current)
        # 包成 BothPoses：抓取侧替换为 capture_point，辅助手保留当前 ee_pose。
        other_axis = "right" if params.grasp_axis == "left" else "left"
        state.capture_points = BothPoses(**{
            params.grasp_axis: grasp_point,
            other_axis: getattr(current, other_axis),
        })
        # 上游随机 Y 只求一次；任何运动开始前必须先保存结果。
        ctx.checkpoint(state)

    _advance(ctx, state, "approach", "目标匹配完成，已保存抓取侧预备点")


async def _main_force(ctx, params, state, key):
    result = await _execute(ctx, state, key, controller.read_force())
    arm = _reading(result, "force_data").get(f"{params.grasp_axis}_arm")
    if not isinstance(arm, dict):
        raise ValueError(f"缺少主抓手 {params.grasp_axis} 力反馈")
    values = [arm.get(f"force_{axis}") for axis in "xyz"]
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
        raise ValueError("主抓手 Fx/Fy/Fz 必须全部为有限数值")
    return tuple(float(v) for v in values)


def _check_pull_force(ctx, params, state):
    check = next((c for c in state.pull_force_checks if c.attempt == state.grasp_attempt), None)
    if check is None:
        before = sum(abs(v) for v in state.force_before_xyz_n)
        after = sum(abs(v) for v in state.force_after_xyz_n)
        threshold = before * params.pull_force_ratio_threshold
        if not all(math.isfinite(v) for v in (before, after, threshold)):
            raise ValueError("合力或倍率阈值计算溢出")
        check = PullForceCheck(attempt=state.grasp_attempt, side=params.grasp_axis,
            before_xyz_n=state.force_before_xyz_n, after_xyz_n=state.force_after_xyz_n,
            before_l1_n=before, after_l1_n=after,
            ratio_threshold=params.pull_force_ratio_threshold, passed=after > threshold)
        state.pull_force_checks.append(check)
        ctx.checkpoint(state)
        ctx.report("grasp.pull_force_checked", stage=state.stage,
                   summary=f"第 {state.grasp_attempt + 1} 轮外拉持箱检测：{'通过' if check.passed else '未通过'}",
                   observation_summary=json.dumps(check.model_dump(mode="json"), ensure_ascii=False))
    if check.passed:
        return True
    if state.grasp_attempt >= params.max_grasp_retries:
        _fail(ctx, state, "GRASP_RETENTION_FAILED",
              f"外拉后合力 {check.after_l1_n:.6g} N 未严格超过 "
              f"基准 {check.before_l1_n:.6g} N × {check.ratio_threshold:.6g}；重试次数已耗尽")
    # Confirmed site contract: a missed groove leaves the box at its original location.
    # No retreat motion. Preserve all previous Action records under their attempt keys.
    state.grasp_attempt += 1
    state.matched_center_m = None
    state.capture_points = None
    state.grasp_contact = state.grasp_lifted = state.grasp_pulled = False
    state.grasp_lift_point = state.grasp_pull_point = None
    state.force_before_xyz_n = state.force_after_xyz_n = None
    state.stage = "prepare"
    ctx.checkpoint(state)
    ctx.report("grasp.retry_started", stage="prepare",
               summary=f"开始第 {state.grasp_attempt + 1} 轮：保持当前位置，重新识别目标箱")
    return False


async def _approach(ctx, params, state):

    if state.capture_points is None:
        raise ValueError("approach 缺少已保存的 capture_point")

    if state.approach_revision not in (None, 4):
        _fail(ctx, state, "APPROACH_CHECKPOINT_INCOMPATIBLE",
              "旧版 approach 检查点缺少本轮力反馈基准，不能恢复新版流程；请确认物理状态后新建执行")
    if state.approach_revision is None:
        has_history = (
            any((key.startswith("approach:") or ":approach:" in key) for key in (*state.actions, *state.results))
            or any((state.grasp_contact, state.grasp_lifted, state.grasp_pulled,
                    state.other_arm_contact, state.other_arm_lifted))
            or any(point is not None for point in (state.grasp_lift_point, state.grasp_pull_point,
                                                   state.other_arm_y_move_pose, state.other_arm_x_move_pose,
                                                   state.other_arm_target_pose,
                                                   state.other_arm_lift_point))
        )
        
        if has_history:
            _fail(ctx, state, "APPROACH_CHECKPOINT_INCOMPATIBLE",
                  "旧版 approach 已开始，不能自动替换动作或重新求点；请确认物理状态后新建执行")
        state.approach_revision = 4
        ctx.checkpoint(state)

    grasp_axis = params.grasp_axis
    other_axis = "right" if grasp_axis == "left" else "left"
    capture_point = getattr(state.capture_points, grasp_axis)

    # 1. 抓取侧移到 capture_point（非轨迹）
    await _execute(ctx, state, "approach:grasp-move",
                   controller.single(grasp_axis, capture_point, trajectory=False,
                                     label="抓取侧移到 capture_point", params=params))

    if state.force_before_xyz_n is None:
        state.force_before_xyz_n = await _main_force(ctx, params, state, "approach:force-before")
        ctx.checkpoint(state)

    # 2. 抓取侧力控对齐
    await _execute(ctx, state, "approach:grasp-force",
                   controller.force(grasp_axis, params.approach_force, "抓取侧力控对齐"))
    state.grasp_contact = True
    ctx.checkpoint(state)

    # 3. 从接触后的实际位姿上抬，只改 Z，不撤销力控产生的 X/Y/姿态变化。
    # 抬升量用常量 GRASP_LIFT_M（0.10），抓取侧与辅助手一致；双手 lift 段才用输入 lift_m。
    # TODO 当前的位姿读出来以后还是base_link，codex理解了，但是不确定Sem是否能理解，双手抓取skill同样有当前的问题
    if state.grasp_lift_point is None:
        contact = await _pose(ctx, params, state, "approach:grasp-contact", grasp_axis)
        state.grasp_lift_point = solver.lift_point(contact, lift_m=GRASP_LIFT_M)
        ctx.checkpoint(state)
    await _execute(ctx, state, "approach:grasp-lift",
                   controller.single(grasp_axis, state.grasp_lift_point, trajectory=False,
                                     label=f"抓取侧上抬 {GRASP_LIFT_M} 米", params=params))

    state.grasp_lifted = True
    ctx.checkpoint(state)

    # 4. 从上抬目标位姿沿 Y 拉箱；求点只做一次并先保存。
    # Use the commanded lift target so a within-tolerance but incomplete lift
    # does not lower the pull target and let the object slip.
    if state.grasp_pull_point is None:
        if state.grasp_lift_point is None:
            raise ValueError("拉箱前缺少已保存的抓取侧上抬目标位姿")
        state.grasp_pull_point = solver.pull_point(params, state.grasp_lift_point)
        ctx.checkpoint(state)
    await _execute(ctx, state, "approach:grasp-pull",
                   controller.single(grasp_axis, state.grasp_pull_point, trajectory=True,
                                     label="抓取侧沿 y 拉箱", params=params))
    state.grasp_pulled = True
    ctx.checkpoint(state)

    if state.force_after_xyz_n is None:
        state.force_after_xyz_n = await _main_force(ctx, params, state, "approach:force-after")
        ctx.checkpoint(state)
    if not _check_pull_force(ctx, params, state):
        return

    # 5. 拉箱完成后求辅助手 Y、Z、X 三个目标；同一检查点保存，恢复时不重新求点。
    waypoints = (state.other_arm_y_move_pose, state.other_arm_x_move_pose, state.other_arm_target_pose)
    if any(point is None for point in waypoints) and any(point is not None for point in waypoints):
        raise ValueError("辅助手 Y/Z/X 目标检查点不完整")
    if state.other_arm_y_move_pose is None:
        current = await _poses(ctx, params, state, "approach:other")
        grasp_current = getattr(current, grasp_axis)
        other_current = getattr(current, other_axis)
        solved = solver.other_arm_position(params, grasp_current, other_current)
        solved_other_axis = solved["other_axis"]
        if solved_other_axis != other_axis:
            raise ValueError(f"辅助手 axis 不一致：期望 {other_axis}，实得 {solved_other_axis}")
        state.other_arm_y_move_pose = solved["y_move_pose"]
        state.other_arm_x_move_pose = solved["x_move_pose"]
        state.other_arm_target_pose = solved["target_pose"]
        ctx.checkpoint(state)

    # 6. 先只沿 Y 到安全位置。
    await _execute(ctx, state, "approach:other-y-move",
                   controller.single(other_axis, state.other_arm_y_move_pose, trajectory=False,
                                     label="辅助手沿 Y 预定位", params=params))

    # 7. 再沿 Z 到接近高度，保持预定位 X/Y。
    await _execute(ctx, state, "approach:other-z-move",
                   controller.single(other_axis, state.other_arm_x_move_pose, trajectory=False,
                                     label="辅助手沿 Z 到接近高度", params=params))

    # 8. 最后沿 X 到 target。
    await _execute(ctx, state, "approach:other-x-move",
                   controller.single(other_axis, state.other_arm_target_pose, trajectory=True,
                                     label="辅助手沿 X 到 target", params=params))

    # 9. 辅助手力控对齐
    await _execute(ctx, state, "approach:other-force",
                   controller.force(other_axis, params.approach_force, "辅助手力控对齐"))
    state.other_arm_contact = True
    ctx.checkpoint(state)

    # 10. 从辅助手实际接触位姿上抬 GRASP_LIFT_M（见文件头常量），不退回接触前的 target。
    if state.other_arm_lift_point is None:
        contact = await _pose(ctx, params, state, "approach:other-contact", other_axis)
        state.other_arm_lift_point = solver.lift_point(contact, lift_m=GRASP_LIFT_M)
        ctx.checkpoint(state)
        
    await _execute(ctx, state, "approach:other-lift",
                   controller.single(other_axis, state.other_arm_lift_point, trajectory=True,
                                     label=f"辅助手上抬 {GRASP_LIFT_M} 米", params=params))
    state.other_arm_lifted = True
    ctx.checkpoint(state)

    required = ("grasp-move", "grasp-force", "grasp-lift", "grasp-pull",
                "other-y-move", "other-z-move", "other-x-move", "other-force", "other-lift")
    if not all((state.grasp_contact, state.grasp_lifted, state.grasp_pulled,
                state.other_arm_contact, state.other_arm_lifted)) or any(
            _attempt_key(state, f"approach:{key}") not in state.results
            or state.results[_attempt_key(state, f"approach:{key}")].status != "succeeded" for key in required):
        raise ValueError("approach 尚未完成接触、上抬、拉箱和辅助手接合，禁止进入 lift")
    _advance(ctx, state, "lift", "抓取侧拉箱与辅助手对齐上抬已完成")


async def _lift(ctx, params, state):

    if not state.grasp_contact or not state.other_arm_contact:
        raise ValueError("lift 必须在抓取侧与辅助手接近完成之后")

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
    _advance(ctx, state, "stand", f"双手抬升 {params.lift_m} 米并完成收箱")


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
        grasp_attempts=state.grasp_attempt + 1, pull_force_checks=state.pull_force_checks,
        object_ref=params.target.object_ref, relative_frame=params.relative_frame,
        matched_center_m=state.matched_center_m, verification_side=params.verify_side,
        force_threshold_n=params.verify_force.force_threshold_n,
        lift_distance_m=GRASP_LIFT_M,
        # 辅助手的实际上抬量用 GRASP_LIFT_M（与同段 label 一致）；solver.OTHER_ARM_Z_LIFT_M
        # 是"辅助手最终 Z 相对主抓手高度的下降量"，是另一个量，不能拿来当上抬量上报。
        other_arm_lift_distance_m=GRASP_LIFT_M,
        both_lift_distance_m=params.lift_m,
        pull_y_distance_m=params.pull_y_distance_m,
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

    _report(ctx, state, "running", f"执行单手抓取阶段：{state.stage}")

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
        summary=(f"停止请求 {request.id} 已收到，单手与躯干停止状态待确认" if uncertain else "准备阶段已停止，尚未发送物理动作"),
        evidence_refs=list(dict.fromkeys([*state.evidence_refs, *ctx.recent_evidence()])),
    )
