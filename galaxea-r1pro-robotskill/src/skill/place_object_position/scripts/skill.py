"""预定位 → X → Z → 双手同步撤出；按步保存物理动作。"""
from __future__ import annotations

import asyncio
import hashlib
import json
from datetime import timedelta

from semantic_robot_skill_sdk import ActionResult, SkillContext, StopOutcome, StopRequest

from . import controller, solver
from .models import BothPoses, PlaceInput, PlaceResult, PlaceState

NEXT = {'prepare': 'preposition', 'preposition': 'align_x', 'align_x': 'lower_z',
        'lower_z': 'retract_both', 'retract_both': 'verify'}
POSITION = {'align_x': 'x', 'lower_z': 'z'}
PLACEMENT_REVISION = 1


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
        result = await ctx.execute(f'place-object-position:{key}', state.actions[key])
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


async def _read(ctx, state, name, *, robot_state=False):
    ctx.check_cancelled()
    state.read_sequence += 1
    ctx.checkpoint(state)
    result = await ctx.execute(f'place-object-position:read:{state.read_sequence}:{name}',
                               controller.read(name, state=robot_state, timeout=10.0))
    ctx.check_cancelled()
    if result.status != 'succeeded':
        _fail(ctx, state, 'PLACE_OBSERVATION_FAILED', result.error_message or f'{name}：{result.status}')
    _remember(state, result)
    return _reading(result, name)


async def _poses(ctx, state):
    if state.base_to_relative is None:
        raise ValueError('末端位姿缺少已保存的 base_link → torso_link4 变换')
    return BothPoses(**{side: solver.pose(await _read(ctx, state, f'ee_pose_{side}'),
                                        state.base_to_relative) for side in ('left', 'right')})


def _advance(ctx, state):
    _report(ctx, state, 'completed', f'{state.stage} 完成')
    state.stage = NEXT[state.stage]
    ctx.checkpoint(state)


async def _prepare(ctx, params, state):
    # 躯干到位后再保存坐标变换与双手起始位姿。
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
        await _physical(ctx, state, f'{stage}:both', controller.both(
            params, targets, trajectory=True, motion=params.motion))
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
    if stage not in state.stage_pause_until:
        state.stage_pause_until[stage] = ctx.now() + timedelta(seconds=params.stage_pause_s)
        ctx.checkpoint(state)
    remaining = max(0.0, (state.stage_pause_until[stage] - ctx.now()).total_seconds())
    while remaining > 0:
        ctx.check_cancelled()
        interval = min(remaining, 0.1)
        await asyncio.sleep(interval)
        remaining = max(0.0, remaining - interval)
    ctx.check_cancelled()
    _advance(ctx, state)


async def _retract_both(ctx, params, state):
    if not set(POSITION) <= set(state.positioned_stages):
        raise ValueError('双手撤出必须在 X/Z 均到位之后')
    if state.retract_plan is None:
        if state.retracted_steps:
            raise ValueError('缺少已开始执行的双手撤出计划')
        state.retract_plan = solver.retract_targets(params, await _poses(ctx, state))
        ctx.checkpoint(state)

    for index, targets in enumerate(state.retract_plan, 1):
        if index <= state.retracted_steps:
            continue
        await _physical(ctx, state, f'retract:both:{index}', controller.both(
            params, targets, trajectory=False, motion=params.retract_motion))
        actual = await _poses(ctx, state)
        for side in ('left', 'right'):
            if not solver.within(getattr(actual, side), getattr(targets, side),
                                 params.retract_motion.pos_tol, params.retract_motion.orient_tol):
                _fail(ctx, state, 'PLACE_RETRACTION_UNCONFIRMED',
                      f'双手撤出第 {index} 段的 {side} 位姿未确认')
        state.retracted_steps = index
        ctx.checkpoint(state)
    state.retracted_sides = ['left', 'right']
    _advance(ctx, state)


async def _verify(ctx, params, state):
    if (set(state.positioned_stages) != set(POSITION)
            or state.retract_plan is None or state.retracted_steps != 3
            or set(state.retracted_sides) != {'left', 'right'}):
        raise ValueError('完成判据缺少 X/Z 到位或双手撤出')
    final = await _poses(ctx, state)
    for side in ('left', 'right'):
        if not solver.within(getattr(final, side), getattr(state.retract_plan[-1], side),
                             params.retract_motion.pos_tol, params.retract_motion.orient_tol):
            _fail(ctx, state, 'PLACE_RETRACTION_UNCONFIRMED', f'{side} 最终撤出位姿未确认')
    ctx.check_cancelled()
    _report(ctx, state, 'completed', 'X/Z 到位和双手撤出均已确认')
    ctx.complete(PlaceResult(object_ref=params.target.object_ref,
        relative_target_m=state.relative_target_m, preposition_points=state.preposition_points,
        final_poses=final, retracted_sides=state.retracted_sides,
        evidence_refs=state.evidence_refs))


def _digest(values):
    return hashlib.sha256(json.dumps(values, sort_keys=True).encode()).hexdigest()


def _check_revision(ctx, params, state):
    values = params.model_dump(mode='json')
    digest = _digest(values)
    if state.placement_revision not in (None, PLACEMENT_REVISION):
        _fail(ctx, state, 'PLACEMENT_CHECKPOINT_INCOMPATIBLE', '放置检查点版本不支持当前轨迹流程')
    if state.placement_revision is None:
        history = (state.physical_action_started or state.retract_plan is not None
                   or state.retracted_steps or state.retracted_sides or state.position_targets
                   or state.positioned_stages or state.stage_pause_until
                   or state.stage != 'prepare' or state.actions or state.results
                   or state.base_to_relative is not None or state.relative_target_m is not None
                   or state.preposition_points is not None or state.initial_poses is not None
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
    try:
        state = ctx.load_state(PlaceState, default=PlaceState())
    except (ValueError, KeyError, TypeError) as error:
        ctx.fail('PLACEMENT_CHECKPOINT_INCOMPATIBLE', f'不能用于位置放置的检查点：{error}', [])
        return
    if state.failure_code:
        ctx.fail(state.failure_code, state.failure_message or '本次放置已失败', state.evidence_refs)
        return
    try:
        _check_revision(ctx, params, state)
        _report(ctx, state, 'running', f'执行放置阶段：{state.stage}')
        try:
            if state.stage in POSITION:
                await _position_stage(ctx, params, state)
            elif state.stage == 'retract_both':
                await _retract_both(ctx, params, state)
            else:
                handler = {'prepare': _prepare, 'preposition': _preposition, 'verify': _verify}[state.stage]
                await handler(ctx, params, state)
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
