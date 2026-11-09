"""预定位 → X → Z → 撤辅助手 → Y → 撤主手；按步保存物理动作。"""
from __future__ import annotations

import asyncio
import hashlib
import json
from datetime import timedelta

from semantic_robot_skill_sdk import ActionResult, SkillContext, StopOutcome, StopRequest

from . import controller, solver
from .models import (BothPoses, ContactEvidence, PlaceInput, PlaceResult, PlaceState)

NEXT = {'prepare': 'preposition', 'preposition': 'align_x', 'align_x': 'lower_z',
        'lower_z': 'retract_assist', 'retract_assist': 'push_y',
        'push_y': 'retract_push', 'retract_push': 'verify'}
POSITION = {'align_x': 'x', 'lower_z': 'z'}
FORCE = {'push_y'}
PLACEMENT_REVISION = 6
# 撤手轨迹的【单点超时】与【点数上限】（现场要求：单点 10 秒、点数上限 100）。
# 依据：单手 Ability 对每个轨迹点各等 timeout 秒，且中间点超时不会中止后续点
# （只在循环外检查最后一个点），所以最坏耗时 = 点数 × timeout = 100 × 10 = 1000 秒；
# 点数越多步长越细（0.1 m 撤出量 → 1 mm/点），被压住时越容易逐点脱出。
# 这里显式钉死单点超时：TrajectorySettings.timeout 默认虽为 10，但调用方会传
# 30（实测 retract_motion.timeout=30），只有 skill 侧覆盖才能保证 10 秒。
RETRACT_POINT_CAP = 100
RETRACT_POINT_TIMEOUT = 10.0


class _StageFailed(Exception):
    pass


def _report(ctx, state, status, summary):
    ctx.report(f'stage.{status}', stage=state.stage, stage_status=status,
               summary=summary, evidence_refs=state.evidence_refs)


def _fail(ctx, state, code, message):
    state.failure_code, state.failure_message = code, message
    ctx.checkpoint(state)
    _report(ctx, state, 'failed', message)
    ctx.fail(code, message, state.evidence_refs)
    raise _StageFailed(message)


def _layers(output):
    pending = [(output, 0)] if isinstance(output, dict) else []
    while pending:
        value, depth = pending.pop(0)
        if value.get('status') in {'failed', 'stopped', 'interrupted'} or value.get('error'):
            raise ValueError(str(value.get('error') or value.get('status')))
        yield value
        if depth < 6:
            pending.extend((value[k], depth + 1) for k in ('result', 'output')
                           if isinstance(value.get(k), dict))


def _items(result):
    for layer in _layers(result.output):
        for item in layer.get('observations') or []:
            if isinstance(item, dict):
                yield item
    for item in result.observations:
        yield item.model_dump(mode='json')


def _reading(result, name):
    for item in _items(result):
        value = item.get('value')
        if isinstance(value, dict) and value.get('name') == name:
            item = value
        if item.get('name') == name:
            data = item.get('data')
            if not isinstance(data, dict) or data.get('error'):
                raise ValueError(f'{name} 读取失败或数据缺失：{data}')
            return data
    raise ValueError(f'Ability 未返回 {name}')


def _remember(state, result):
    refs = list(result.evidence_refs)
    for item in _items(result):
        for value in (item, item.get('value', {})):
            if isinstance(value, dict):
                refs.extend(value.get('evidence_refs') or [])
                refs.extend(value.get('artifact_refs') or [])
    state.evidence_refs = list(dict.fromkeys([*state.evidence_refs,
                                            *(ref for ref in refs if isinstance(ref, str))]))


async def _physical(ctx, state, key, action):
    ctx.check_cancelled()
    if key not in state.actions:
        state.actions[key] = action
    state.physical_action_started = True
    ctx.checkpoint(state)
    if key not in state.results:
        result = await ctx.execute(f'place-object-force:{key}', state.actions[key])
        ctx.check_cancelled()
        if result.status == 'succeeded':
            _remember(state, result)
        state.results[key] = result
        ctx.checkpoint(state)
    result = state.results[key]
    if result.status != 'succeeded':
        _fail(ctx, state, 'ACTION_STATE_UNKNOWN' if result.status == 'interrupted'
              else 'PLACE_ACTION_FAILED', result.error_message or f'{action.type}：{result.status}')
    return result


async def _read(ctx, state, name, *, robot_state=False, deadline=None):

    ctx.check_cancelled()

    timeout = 65.0 if name == 'force_data' else 10.0

    if deadline is not None:
        remaining = (deadline - ctx.now()).total_seconds()
        if remaining <= 0:
            _fail(ctx, state, 'PLACE_CONTACT_TIMEOUT', f'{state.stage} 力控超时')
        timeout = min(timeout, remaining)

    state.read_sequence += 1
    ctx.checkpoint(state)

    result = await ctx.execute(f'place-object-force:read:{state.read_sequence}:{name}',
                               controller.read(name, state=robot_state, timeout=timeout))

    ctx.check_cancelled()
    if result.status != 'succeeded':
        _fail(ctx, state, 'PLACE_OBSERVATION_FAILED', result.error_message or f'{name}：{result.status}')
    _remember(state, result)
    return _reading(result, name)


async def _poses(ctx, state, deadline=None):
    if state.base_to_relative is None:
        raise ValueError('末端位姿缺少已保存的 base_link → torso_link4 变换')
    return BothPoses(**{side: solver.pose(await _read(ctx, state, f'ee_pose_{side}', deadline=deadline),
                                        state.base_to_relative) for side in ('left', 'right')})


async def _forces(ctx, state, deadline=None):
    try:
        return solver.forces(await _read(ctx, state, 'force_data', deadline=deadline))
    except (ValueError, KeyError, TypeError) as error:
        _fail(ctx, state, 'PLACE_FORCE_INVALID', f'双侧力反馈不完整或非法：{error}')


def _advance(ctx, state):
    _report(ctx, state, 'completed', f'{state.stage} 完成')
    state.stage = NEXT[state.stage]
    ctx.checkpoint(state)


async def _prepare(ctx, params, state):

    # Step1、torso躯干运动
    if params.torso.enabled:
        await _physical(ctx, state, 'prepare:torso', controller.torso(params))

    if state.base_to_relative is None:
        state.base_to_relative = solver.transform(
            await _read(ctx, state, 'floating_base', robot_state=True), params.relative_frame)
        ctx.checkpoint(state)

    if state.relative_target_m is None:
        box_center = solver.relative_target(params,
            await _read(ctx, state, 'transformer_martix', robot_state=True), state.base_to_relative)
        # relative_target 解出的是箱体中心；末端经托举夹具持箱，减去末端-夹具
        # 偏移（X/Z）后才是双手运动目标，后续规划不再重复减偏移。
        state.relative_target_m = solver.hand_target_m(params, box_center)
        ctx.checkpoint(state)

    if state.initial_poses is None:
        state.initial_poses = await _poses(ctx, state)
        ctx.checkpoint(state)

    if state.preposition_points is None:
        state.preposition_points = solver.preposition(params, state.relative_target_m,
                                                      state.initial_poses)
        ctx.checkpoint(state)

    # 躯干到位后校验反馈；Y 推靠时另取当前力基线。
    state.initial_forces = await _forces(ctx, state)
    _advance(ctx, state)


async def _preposition(ctx, params, state):
    if state.preposition_points is None or state.initial_poses is None:
        raise ValueError('缺少预定位计划或起始双手位姿')

    await _physical(ctx, state, 'preposition:both', controller.both(
        params, state.preposition_points, trajectory=True, motion=params.approach_motion))

    _confirm_height(ctx, params, state, await _poses(ctx, state), state.preposition_points)
    _advance(ctx, state)


def _confirm_height(ctx, params, state, actual, targets):
    left, right = actual.left.z, actual.right.z
    target_left, target_right = targets.left.z, targets.right.z
    gap = abs(left - right)
    tolerance = params.motion.pos_tol
    if max(abs(left - target_left), abs(right - target_right), gap) > tolerance + 1e-9:
        _fail(ctx, state, 'PLACE_HEIGHT_UNCONFIRMED',
              f'{state.stage} 高度未确认：目标 Z 左手={target_left:.6f} m、右手={target_right:.6f} m，'
              f'左手 Z={left:.6f} m，右手 Z={right:.6f} m，'
              f'高度差={gap:.6f} m，容限={tolerance:.6f} m')

async def _position_stage(ctx, params, state):

    stage = state.stage

    if stage == 'lower_z' and 'align_x' not in state.positioned_stages:
        raise ValueError('Z 调整必须在 X 到位之后')

    if stage not in state.position_targets:

        if state.relative_target_m is None or state.initial_poses is None:
            raise ValueError('缺少最终放置目标或起始双手位姿')

        # 先读取当前双手位姿，经 floating_base 由 base_link 转到 torso_link4，
        # 再由当前位置与最终目标的 X/Z 偏移生成保存的绝对目标，Y 锁定起始值。
        current = await _poses(ctx, state)
        state.position_targets[stage] = solver.position_targets(
            state.relative_target_m, current, POSITION[stage], state.initial_poses)
        ctx.checkpoint(state)

    if stage not in state.positioned_stages:
        targets = state.position_targets[stage]
        await _physical(ctx, state, f'{stage}:both', controller.both(params, targets, trajectory=True, motion=params.motion))
        actual = await _poses(ctx, state)
        _confirm_height(ctx, params, state, actual, targets)
        for side in ('left', 'right'):
            if not solver.within(getattr(actual, side), getattr(targets, side),
                                 params.motion.pos_tol, params.motion.orient_tol):
                _fail(ctx, state, 'PLACE_POSE_UNCONFIRMED', f'{stage} 的 {side} 未到达保存目标')
        state.positioned_stages.append(stage)
        ctx.checkpoint(state)
    await _finish_phase(ctx, params, state)


async def _finish_phase(ctx, params, state):
    # 段间等待独立于细步循环间隔；保存截止时间，恢复只等待剩余时间。
    stage = state.stage
    if stage not in state.force_phase_pause_until:
        state.force_phase_pause_until[stage] = ctx.now() + timedelta(seconds=params.force_phase_pause_s)
        ctx.checkpoint(state)
    remaining = max(0.0, (state.force_phase_pause_until[stage] - ctx.now()).total_seconds())
    while remaining > 0:
        ctx.check_cancelled()
        interval = min(remaining, 0.1)
        await asyncio.sleep(interval)
        remaining = max(0.0, remaining - interval)
    ctx.check_cancelled()
    _advance(ctx, state)


async def _force_stage(ctx, params, state):
    """主手沿 Y 力控推靠：直接下发 manipulation.force_align，Ability 内部完成步进与触阈。

    借鉴 grasp_object_unilateral 的力控对齐：skill 不再逐小步下发 move_end_effector_single
    并读 force_data 闭环，而是把阈值/步长/方向交给 manipulation.force_align 动作。
    动作成功即视为接触；接触前后各读一次力作为 baseline 与 final_force。
    """

    stage = state.stage

    if stage in state.contacts:
        await _finish_phase(ctx, params, state)
        return

    settings = params.y_force
    single_side = params.put_mode
    other = 'right' if single_side == 'left' else 'left'
    if other not in state.retracted_sides:
        raise ValueError('Y 推靠前必须完成辅助手撤出')
    if not set(POSITION) <= set(state.positioned_stages):
        raise ValueError('Y 推靠必须在 X/Z 到位后执行')

    sign = -1 if single_side == 'left' else 1
    key = f'{stage}:force'

    if key not in state.contacts:
        baseline = await _forces(ctx, state)
        await _physical(ctx, state, key, controller.force(
            params, single_side, settings, '主手沿 Y 力控推靠'))
        actual = await _poses(ctx, state)
        final = await _forces(ctx, state)
        sides = (single_side,)
        delta = {side: max(abs(getattr(getattr(final, side), axis)
                              - getattr(getattr(baseline, side), axis))
                           for axis in settings.force_coordinate) for side in sides}
        state.contacts[stage] = ContactEvidence(
            movement_axis='y',
            direction='negative' if sign < 0 else 'positive',
            force_coordinate=settings.force_coordinate,
            force_threshold_n=settings.force_threshold_n,
            baseline=baseline, final_force=final, delta_n=delta,
            triggered_sides=list(delta.keys()),
            commanded_travel_m=settings.max_distance_m,
            completed_steps=1, actual_poses=actual)
        ctx.checkpoint(state)

    await _finish_phase(ctx, params, state)


async def _retract(ctx, params, state):

    if state.stage == 'retract_assist':
        if not set(POSITION) <= set(state.positioned_stages):
            raise ValueError('撤辅助手必须在 X/Z 均到位之后')
        side = 'right' if params.put_mode == 'left' else 'left'

    else:
        if 'push_y' not in state.contacts:
            raise ValueError('撤主手必须在 Y 接触之后')
        side = params.put_mode

    if side not in state.retract_plans:
        actual = await _poses(ctx, state)
        state.retract_plans[side] = solver.retract(params, state.relative_target_m, side, getattr(actual, side))
        ctx.checkpoint(state)

    for index, target in enumerate(state.retract_plans[side], 1):
        await _physical(ctx, state, f'retract:{side}:{index}', controller.single(
            params, side, target, trajectory=False, motion=params.retract_motion,
            timeout=RETRACT_POINT_TIMEOUT, point_cap=RETRACT_POINT_CAP))
    actual = await _poses(ctx, state)

    if not solver.within(getattr(actual, side), state.retract_plans[side][-1],
                         params.retract_motion.pos_tol, params.retract_motion.orient_tol):
        _fail(ctx, state, 'PLACE_RETRACTION_UNCONFIRMED', f'{side} 撤出位姿未确认')
    if side not in state.retracted_sides:
        state.retracted_sides.append(side)
    _advance(ctx, state)
    

async def _verify(ctx, params, state):
    if (set(state.positioned_stages) != set(POSITION)
            or set(state.contacts) != FORCE or set(state.retracted_sides) != {'left', 'right'}):
        raise ValueError('完成判据缺少 Z/X 到位、Y 接触或双手撤出')
    final = await _poses(ctx, state)
    for side in ('left', 'right'):
        if not solver.within(getattr(final, side), state.retract_plans[side][-1],
                             params.retract_motion.pos_tol, params.retract_motion.orient_tol):
            _fail(ctx, state, 'PLACE_RETRACTION_UNCONFIRMED', f'{side} 最终撤出位姿未确认')
    final_forces = await _forces(ctx, state)
    ctx.check_cancelled()
    _report(ctx, state, 'completed', 'Z/X 到位、Y 接触和双手撤出均已确认')
    ctx.complete(PlaceResult(object_ref=params.target.object_ref, put_mode=params.put_mode,
        relative_target_m=state.relative_target_m, preposition_points=state.preposition_points,
        contacts=state.contacts, final_poses=final, final_forces=final_forces,
        retracted_sides=state.retracted_sides, evidence_refs=state.evidence_refs))


def _digest(values):
    return hashlib.sha256(json.dumps(values, sort_keys=True).encode()).hexdigest()


def _check_revision(ctx, params, state):
    values = params.model_dump(mode='json')
    digest = _digest(values)
    if state.placement_revision not in (None, PLACEMENT_REVISION):
        _fail(ctx, state, 'PLACEMENT_CHECKPOINT_INCOMPATIBLE', '放置检查点版本不支持当前轨迹流程')
    if state.placement_revision is None:
        history = (state.physical_action_started or state.contacts
                   or state.retract_plans or state.retracted_sides or state.position_targets
                   or state.positioned_stages or state.force_phase_pause_until
                   or state.stage != 'prepare' or state.actions or state.results
                   or state.base_to_relative is not None or state.relative_target_m is not None
                   or state.preposition_points is not None or state.initial_poses is not None
                   or state.initial_forces is not None
                   or state.read_sequence or state.input_digest is not None)
        if history:
            _fail(ctx, state, 'PLACEMENT_CHECKPOINT_INCOMPATIBLE',
                  '旧版放置已开始，不能替换既有动作；请确认物理状态后新建执行')
    if state.input_digest is not None and state.input_digest != digest:
        _fail(ctx, state, 'PLACE_INPUT_CHANGED', '恢复时输入与原执行不一致')
    state.placement_revision = PLACEMENT_REVISION
    state.input_digest = digest
    ctx.checkpoint(state)


async def run(ctx: SkillContext) -> None:
    ctx.check_cancelled()
    params = ctx.input(PlaceInput)
    state = ctx.load_state(PlaceState, default=PlaceState())
    if state.failure_code:
        ctx.fail(state.failure_code, state.failure_message or '本次放置已失败', state.evidence_refs)
        return
    try:
        _check_revision(ctx, params, state)
        _report(ctx, state, 'running', f'执行放置阶段：{state.stage}')
        try:
            if state.stage in FORCE:
                await _force_stage(ctx, params, state)
            elif state.stage in POSITION:
                await _position_stage(ctx, params, state)
            elif state.stage.startswith('retract_'):
                await _retract(ctx, params, state)

            else:

                await {'prepare': _prepare, 'preposition': _preposition, 'verify': _verify}[state.stage](ctx, params, state)

        except (ValueError, KeyError, TypeError, OverflowError) as error:
            _fail(ctx, state, 'PLACE_DATA_INVALID', str(error))
    except _StageFailed:
        return


async def on_stop(ctx: SkillContext, request: StopRequest) -> StopOutcome:
    state = ctx.load_state(PlaceState, default=PlaceState())
    uncertain = state.physical_action_started
    return ctx.stop_outcome(safe=not uncertain, physical_state='unknown' if uncertain else 'no_motion_started',
        requires_intervention=uncertain,
        summary='停止请求已收到，双臂实际停止状态待确认' if uncertain else '尚未发送物理动作',
        evidence_refs=list(dict.fromkeys([*state.evidence_refs, *ctx.recent_evidence()])))
