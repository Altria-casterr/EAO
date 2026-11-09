import asyncio
from copy import deepcopy
from datetime import timedelta
import os
from pathlib import Path

import pytest
import yaml
from pydantic import ValidationError
from semantic_robot_skill_sdk import ActionResult, Observation, SkillCancelled, StopRequest, run_skill

from place_object_force.scripts import controller, solver
from place_object_force.scripts.models import PlaceState
from place_object_force.scripts.skill import on_stop, run
from .support import Context, inputs


def run_all(ctx):
    asyncio.run(run_skill(run, ctx))
    return ctx


def movements(ctx):
    return [(k, a) for k, a in ctx.responder.actions if a.type.startswith('motion.')]


def physical(ctx):
    return [(k, a) for k, a in ctx.responder.actions
            if a.type.startswith(('motion.', 'torso.', 'manipulation.'))]


@pytest.mark.parametrize('side', ['left', 'right'])
def test_flow_aligns_x_then_z_and_records_y_contact(side):
    ctx = run_all(Context(inputs(put_mode=side)))
    assert ctx.status == 'completed'
    assert ctx.result.placed is True and ctx.result.holding_object is None
    assert ctx.result.verification_method == 'force_contact_and_retraction'
    assert [event['stage'] for event in ctx.events if event.get('type') == 'stage.completed'] == [
        'prepare', 'preposition', 'align_x', 'lower_z', 'retract_assist', 'push_y', 'retract_push', 'verify']
    other = 'right' if side == 'left' else 'left'
    assert ctx.result.retracted_sides == [other, side]
    assert all(not action.type.startswith('gripper.')
               for _, action in ctx.responder.actions)
    assert ctx.state.positioned_stages == ['align_x', 'lower_z']
    assert set(ctx.result.contacts) == {'push_y'}
    for stage in ('push_y',):
        contact = ctx.result.contacts[stage]
        assert contact.completed_steps == 1
        assert contact.commanded_travel_m == pytest.approx(inputs().y_force.max_distance_m)
        assert max(contact.delta_n.values()) >= ctx.params.y_force.force_threshold_n
    assert ctx.result.contacts['push_y'].direction == ('negative' if side == 'left' else 'positive')
    assert len(ctx.result.evidence_refs) > 10
    for hand in ('left', 'right'):
        plan = ctx.state.retract_plans[hand]
        assert len(plan) == 3  # z 降 → y 外撤 → 抬升并沿 x 回撤
        start = next(poses[hand]['pose']['position']
                     for key, poses in ctx.responder.physical_before
                     if key.endswith(f'retract:{hand}:1'))
        assert plan[0].z == pytest.approx(start['z'] - ctx.params.retract.lower_m)
        assert plan[1].z == pytest.approx(plan[0].z)                 # y 段保持降后高度
        assert plan[1].x == plan[0].x                                # y 段不改变 X
        assert plan[2].y == pytest.approx(plan[1].y)                 # x 段保持外撤 Y
        # 最终高度相对撤手开始高度 +3 cm，与第一段下降量无关。
        assert plan[2].z == pytest.approx(start['z'] + 0.03)
        assert plan[0].x - plan[2].x == pytest.approx(0.1)           # X 向后回撤
        assert (plan[1].y - plan[0].y) * (1 if hand == 'left' else -1) >= 0.03 - 1e-9


def test_sensor_axis_can_differ_from_y_motion_and_uses_fresh_baseline():
    settings = {**inputs().y_force.model_dump(), 'force_coordinate': ['x']}
    ctx = run_all(Context(inputs(y_force=settings)))
    assert ctx.status == 'completed'
    contact = ctx.result.contacts['push_y']
    assert contact.movement_axis == 'y' and contact.force_coordinate == ('x',)
    assert contact.baseline.left.x == 30.0
    assert ctx.state.initial_forces.left.x == 0.0


@pytest.mark.parametrize('status', ['failed', 'interrupted', 'stopped'])
def test_force_align_failure_is_terminal(status):
    ctx = Context()
    ctx.responder.mutate = lambda k, a, r: (
        ActionResult(status=status, error_message='injected') if k.endswith('push_y:force') else r)
    run_all(ctx)
    assert ctx.failure.code == ('ACTION_STATE_UNKNOWN' if status == 'interrupted' else 'PLACE_ACTION_FAILED')
    assert ctx.state.stage == 'push_y'
    assert 'push_y' not in ctx.state.contacts
    count = len(ctx.responder.actions)
    asyncio.run(run(ctx))
    assert len(ctx.responder.actions) == count


@pytest.mark.parametrize('case', ['missing_side', 'missing_axis', 'nan', 'inf', 'boolean', 'read_error'])
@pytest.mark.parametrize('stage', ['prepare', 'push_y'])
def test_invalid_force_is_terminal_and_no_later_motion(case, stage):
    ctx = Context()
    def mutate(key, action, result):
        if action.parameters.get('observation') == 'force_data' and ctx.state.stage == stage:
            data = result.output['result']['observations'][0]['data']
            if case == 'missing_side': del data['right_arm']
            elif case == 'missing_axis': del data['left_arm']['force_z']
            elif case == 'read_error': data['error'] = 'injected sensor failure'
            else: data['left_arm']['force_z'] = {'nan': float('nan'), 'inf': float('inf'), 'boolean': True}[case]
        return result
    ctx.responder.mutate = mutate
    run_all(ctx)
    assert ctx.failure.code == 'PLACE_FORCE_INVALID'
    assert not any(a.type == 'manipulation.force_align' for _, a in ctx.responder.actions)
    if stage == 'prepare': assert not movements(ctx)


def test_force_align_is_executed_once_even_on_resume():
    ctx = Context()
    execute = ctx.execute
    crashed = False
    async def crashing(k, action):
        nonlocal crashed
        result = await execute(k, action)
        if k.endswith('push_y:force') and not crashed:
            crashed = True
            raise RuntimeError('worker crash')
        return result
    ctx.execute = crashing
    with pytest.raises(RuntimeError, match='worker crash'): run_all(ctx)
    ctx.execute = execute
    run_all(ctx)
    assert ctx.status == 'completed'
    assert sum(k.endswith('push_y:force') for k, _ in physical(ctx)) == 1


@pytest.mark.parametrize('key', ['prepare:torso', 'preposition:both', 'lower_z:both', 'align_x:both',
                                'retract:right:2', 'push_y:force', 'retract:left:3'])
@pytest.mark.parametrize('status', ['failed', 'interrupted', 'stopped'])
def test_physical_failure_and_unknown_state_never_continue(key, status):
    ctx = Context()
    ctx.responder.mutate = lambda k, a, r: ActionResult(status=status, error_message='injected') if k.endswith(key) else r
    run_all(ctx)
    assert ctx.status == 'failed' and ctx.result is None
    assert physical(ctx)[-1][0].endswith(key)
    assert ctx.failure.code == ('ACTION_STATE_UNKNOWN' if status == 'interrupted' else 'PLACE_ACTION_FAILED')
    count = len(ctx.responder.actions)
    asyncio.run(run(ctx))
    assert len(ctx.responder.actions) == count


@pytest.mark.parametrize('key', ['prepare:torso', 'preposition:both', 'lower_z:both', 'align_x:both',
                                'retract:right:2', 'push_y:force', 'retract:left:3'])
def test_crash_after_physical_result_reuses_key(key):
    ctx = Context()
    execute = ctx.execute
    crashed = False
    async def crashing(k, action):
        nonlocal crashed
        result = await execute(k, action)
        if k.endswith(key) and not crashed:
            crashed = True
            raise RuntimeError('worker crash')
        return result
    ctx.execute = crashing
    with pytest.raises(RuntimeError, match='worker crash'): run_all(ctx)
    old_read = ctx.state.read_sequence
    ctx.execute = execute
    run_all(ctx)
    assert ctx.status == 'completed'
    assert sum(k.endswith(key) for k, _ in physical(ctx)) == 1
    assert ctx.state.read_sequence > old_read


def test_force_align_contact_records_baseline_and_final_force():
    ctx = run_all(Context())
    contact = ctx.result.contacts['push_y']
    assert contact.movement_axis == 'y'
    assert contact.baseline.left.y == 31.0
    assert contact.final_force.left.y >= 31.0 + inputs().y_force.force_threshold_n
    assert contact.delta_n['left'] >= inputs().y_force.force_threshold_n
    assert contact.triggered_sides == ['left']


@pytest.mark.parametrize('started', [False, True])
def test_stop_emits_no_motion_and_reports_unknown_after_motion(started):
    ctx = Context()
    ctx.checkpoint(PlaceState(physical_action_started=started))
    ctx.request_stop()
    with pytest.raises(SkillCancelled): asyncio.run(run(ctx))
    outcome = asyncio.run(on_stop(ctx, StopRequest(source='user', reason='stop')))
    assert outcome.safe is (not started)
    assert outcome.requires_intervention is started
    assert outcome.physical_state == ('unknown' if started else 'no_motion_started')
    assert not movements(ctx)


def test_stop_during_force_baseline_never_publishes_force_align():
    ctx = Context()
    def mutate(key, action, result):
        if action.parameters.get('observation') == 'force_data' and ctx.state.stage == 'push_y':
            ctx.request_stop()
        return result
    ctx.responder.mutate = mutate
    with pytest.raises(SkillCancelled): run_all(ctx)
    assert not any(a.type == 'manipulation.force_align' for _, a in ctx.responder.actions)


@pytest.mark.parametrize('side', ['left', 'right'])
def test_helper_retraction_must_be_confirmed_before_y_push(side):
    ctx = Context(inputs(put_mode=side))
    other = 'right' if side == 'left' else 'left'
    def mutate(key, action, result):
        if key.endswith(f'retract:{other}:3'):
            ctx.responder.poses[other]['pose']['position']['z'] -= 0.02
        return result
    ctx.responder.mutate = mutate
    run_all(ctx)
    assert ctx.failure.code == 'PLACE_RETRACTION_UNCONFIRMED'
    assert 'push_y' not in ctx.state.contacts
    assert not any(a.type == 'manipulation.force_align' for _, a in ctx.responder.actions)


def test_semantic_observation_wrapping_is_supported():
    ctx = Context()
    def mutate(key, action, result):
        if action.type.startswith('robot.'):
            item = result.output['result']['observations'][0]
            result.output = {}
            result.observations = [Observation(kind='robot.offline', source='offline', value=item)]
        return result
    ctx.responder.mutate = mutate
    run_all(ctx)
    assert ctx.status == 'completed'


@pytest.mark.parametrize('sign', [-1.0, 1.0])
def test_absolute_force_changes_trigger_contact(sign):
    ctx = Context(inputs(y_force={**inputs().y_force.model_dump(), 'force_coordinate': ['x', 'z']}))
    def mutate(key, action, result):
        # 只在 force_align 执行后的最终力读数上加力，避免污染 baseline。
        if (action.parameters.get('observation') == 'force_data' and ctx.state.stage == 'push_y'
                and 'push_y' in ctx.responder.force_align_done):
            data = result.output['result']['observations'][0]['data']
            data['left_arm']['force_z'] += sign * 3.0
        return result
    ctx.responder.mutate = mutate
    run_all(ctx)
    assert ctx.status == 'completed'
    contact = ctx.result.contacts['push_y']
    assert contact.completed_steps == 1 and contact.triggered_sides == ['left']
    assert contact.delta_n['left'] == 3.0


@pytest.mark.parametrize('case', ['frame', 'quaternion', 'transform'])
def test_invalid_transform_or_pose_fails_without_later_motion(case):
    ctx = Context()
    def mutate(key, action, result):
        name = action.parameters.get('observation')
        if name == 'ee_pose_left':
            data = result.output['result']['observations'][0]['data']
            if case == 'frame': data['header']['frame_id'] = 'camera'
            if case == 'quaternion': del data['pose']['orientation']['w']
        if name == 'floating_base' and case == 'transform':
            result.output['result']['observations'][0]['data']['orientation'] = [0.0]*4
        return result
    ctx.responder.mutate = mutate
    run_all(ctx)
    assert ctx.status == 'failed' and ctx.result is None
    # 起始末端位姿和坐标变换必须在首次手臂运动前有效。
    assert not movements(ctx)


def test_map_target_and_rotated_pose_transform_use_meter_units():
    params = inputs()
    floating = solver.transform({'position': [0.0, 0.0, -0.8], 'orientation': [0.0, 0.0, 0.0, 1.0]})
    assert solver.relative_target(params, {'position': [0.1, -0.2, 0.0], 'orientation': [0.0, 0.0, 0.0, 1.0]}, floating) == pytest.approx((0.6, -0.2, 0.4))
    rotated = solver.transform({'position': [0.0, 0.0, -0.8], 'orientation': [0.0, 0.0, 2**-0.5, 2**-0.5]})
    converted = solver.pose({'header': {'frame_id': 'base_link'}, 'pose': {
        'position': {'x': 1.0, 'y': 0.0, 'z': 1.0}, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}}}, rotated)
    assert (converted.x, converted.y, converted.z) == pytest.approx((0.0, 1.0, 0.2))
    assert converted.qz == pytest.approx(2**-0.5)


def test_input_change_on_resume_fails_before_motion():
    ctx = Context()
    asyncio.run(run(ctx))
    ctx._input = inputs(put_mode='right')
    asyncio.run(run(ctx))
    assert ctx.failure.code == 'PLACE_INPUT_CHANGED'
    assert not movements(ctx)


@pytest.mark.parametrize('patch', [
    {'y_force': {'force_threshold_n': 0.0}},
    {'motion': {'fine_pos_tol': 0.01}}, {'put_mode': 'both'},
    {'relative_frame': 'map'}, {'target': {'position_m': [float('nan'), 0.0, 1.0]}},
    {'y_force': {**inputs().y_force.model_dump(), 'force_coordinate': ['z', 'z']}},
    {'y_force': {**inputs().y_force.model_dump(), 'max_distance_m': 0.0}},
    {'adjustment': {'z_offset_m': -0.02, 'x_offset_m': 0.05}},
])
def test_invalid_parameters_rejected_before_execution(patch):
    with pytest.raises(ValidationError): inputs(**patch)


def test_action_parameters_match_latest_manifests():
    workspace = Path(__file__).resolve().parents[6]
    root = Path(os.environ.get('PLACE_ABILITY_MANIFESTS') or
                workspace / 'semantic-ability/galaxea-r1pro-ability/src/galaxea_r1pro_0/abilities')
    tasks = {}
    for name in ('robot_observation', 'joint_eef_pose_control', 'torso_control'):
        tasks.update({t.get('actionType'): t for t in yaml.safe_load((root/name/'ability.manifest.yaml').read_text())['tasks'] if t.get('actionType')})
    ctx = run_all(Context())
    manifest = yaml.safe_load((Path(__file__).parents[1]/'SKILL.md').read_text().split('---', 2)[1])
    required = {a['type'] for a in manifest['required_actions']}
    observed = {a.type for _, a in ctx.responder.actions}
    assert observed == required
    for _, action in ctx.responder.actions:
        task = tasks[action.type]
        declared = {p['name'] for p in task['params']}
        mandatory = {p['name'] for p in task['params'] if not p.get('optional', False)}
        assert set(action.parameters) <= declared
        assert mandatory <= set(action.parameters)
        assert action.schema_version == task['schemaVersion'] == 2
