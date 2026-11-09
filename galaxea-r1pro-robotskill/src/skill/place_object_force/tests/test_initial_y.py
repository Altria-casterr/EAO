"""放置前保持起始 Y、落箱后单手力控及检查点恢复；全部使用离线替身。"""
import asyncio
from copy import deepcopy

import pytest

from place_object_force.scripts import skill, solver
from solver.capture_point_solver import CapturePointSolver
from .support import Context, inputs
from .test_skill import movements, run_all


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('target_y,box_long,y_offset', [
    (0.0, 0.53, 0.0), (0.3, 0.8, 0.1), (-0.3, 0.2, 0.4),
])
def test_placement_keeps_each_initial_y_without_target_edge_planning(
        side, target_y, box_long, y_offset, monkeypatch):
    target = inputs().target.model_dump(mode='json')
    target['position_m'][1] = target_y
    target['size']['long_m'] = box_long
    ctx = Context(inputs(put_mode=side, target=target, preposition={'y_offset_m': y_offset}))
    initial_y = {'left': 0.317, 'right': -0.243}
    for hand, y in initial_y.items():
        ctx.responder.poses[hand]['pose']['position']['y'] = y

    def forbidden_y_solver(*args, **kwargs):
        raise AssertionError('放置不能调用旧箱缘 Y 安全带求解')

    monkeypatch.setattr(CapturePointSolver, 'get_capture_point_place', forbidden_y_solver)
    run_all(ctx)
    assert ctx.status == 'completed'
    assert ctx.state.relative_target_m[1] == target_y
    placement = [(k, a) for k, a in movements(ctx)
                 if k.endswith(('preposition:both', 'align_x:both', 'lower_z:both'))]
    assert len(placement) == 3
    for _, action in placement:
        for hand, y in initial_y.items():
            assert action.parameters[f'{hand}_target_pose']['target_y'] == y
            assert getattr(ctx.state.initial_poses, hand).y == y
    assert ctx.result.preposition_points.left.y - ctx.result.preposition_points.right.y == pytest.approx(0.56)
    # 辅助手先撤，主手沿实际 Y 推靠，之后才撤主手。
    other = 'right' if side == 'left' else 'left'
    keys = [k for k, a in ctx.responder.actions]
    assert keys.index(f'place-object-force:lower_z:both') < keys.index(f'place-object-force:retract:{other}:1')
    assert keys.index(f'place-object-force:retract:{other}:3') < keys.index('place-object-force:push_y:force')
    assert keys.index('place-object-force:push_y:force') < keys.index(f'place-object-force:retract:{side}:1')
    force_start = next(p[side]['pose']['position']['y']
                       for k, p in ctx.responder.physical_before if k.endswith('push_y:force'))
    assert force_start == initial_y[side]
    actual = getattr(ctx.result.contacts['push_y'].actual_poses, side)
    assert actual.y == pytest.approx(force_start + (-0.002 if side == 'left' else 0.002))


@pytest.mark.parametrize('frame,expected', [
    ('base_link', {'left': 0.22, 'right': 0.07}),
    ('torso_link4', {'left': 0.31, 'right': -0.24}),
])
def test_initial_y_is_saved_in_planning_frame_before_arm_motion(frame, expected):
    ctx = Context()
    ctx.responder.floating = {'position': [0.1, -0.2, -0.8],
                             'orientation': [0.0, 0.0, 2**-0.5, 2**-0.5]}
    for hand, x, y in [('left', 0.42, 0.31), ('right', 0.27, -0.24)]:
        ctx.responder.poses[hand]['header']['frame_id'] = frame
        ctx.responder.poses[hand]['pose']['position'].update(x=x, y=y)
    run_all(ctx)
    assert ctx.status == 'completed'
    for hand, y in expected.items():
        assert getattr(ctx.state.initial_poses, hand).y == pytest.approx(y)
        assert getattr(ctx.result.preposition_points, hand).y == pytest.approx(y)
    trace = ctx.responder.actions
    first_arm = next(i for i, (_, a) in enumerate(trace) if a.type.startswith('motion.'))
    for hand in ('left', 'right'):
        first_pose = next(i for i, (_, a) in enumerate(trace)
                          if a.parameters.get('observation') == f'ee_pose_{hand}')
        assert first_pose < first_arm


@pytest.mark.parametrize('stage', ['preposition', 'align_x'])
def test_feedback_y_drift_does_not_replace_saved_y_for_next_placement(stage):
    ctx = Context()

    def drift(key, action, result):
        if key.endswith(f'{stage}:both'):
            ctx.responder.poses['left']['pose']['position']['y'] += 0.002
            ctx.responder.poses['right']['pose']['position']['y'] -= 0.002
        return result

    ctx.responder.mutate = drift
    run_all(ctx)
    assert ctx.status == 'completed'
    for name in ('align_x', 'lower_z'):
        target = ctx.state.position_targets[name]
        assert target.left.y == ctx.state.initial_poses.left.y == 0.35
        assert target.right.y == ctx.state.initial_poses.right.y == -0.35


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('case', ['bad_frame', 'nan_y', 'missing_orientation'])
def test_bad_initial_pose_blocks_first_arm_motion(side, case):
    ctx = Context()
    data = ctx.responder.poses[side]
    if case == 'bad_frame':
        data['header']['frame_id'] = 'camera'
    elif case == 'nan_y':
        data['pose']['position']['y'] = float('nan')
    else:
        del data['pose']['orientation']['w']
    run_all(ctx)
    assert ctx.failure.code == 'PLACE_DATA_INVALID'
    assert ctx.state.stage == 'prepare'
    assert ctx.state.initial_poses is None
    assert not movements(ctx)


@pytest.mark.parametrize('boundary', ['snapshot', 'preposition_plan'])
def test_prepare_resume_reuses_saved_initial_y_even_when_live_y_changes(boundary):
    ctx = Context()
    checkpoint = ctx.checkpoint

    def crash(state):
        checkpoint(state)
        if (state.initial_poses is not None
                and (state.preposition_points is not None) == (boundary == 'preposition_plan')):
            raise RuntimeError('crash after initial Y checkpoint')

    ctx.checkpoint = crash
    with pytest.raises(RuntimeError, match='crash after initial Y checkpoint'):
        asyncio.run(skill.run(ctx))
    original = deepcopy(ctx.state.initial_poses)
    resumed = Context(ctx.params)
    resumed.checkpoint(ctx.state.model_copy(deep=True))
    resumed._records = deepcopy(ctx._records)
    resumed.responder.poses['left']['pose']['position']['y'] = 0.25
    resumed.responder.poses['right']['pose']['position']['y'] = -0.25
    run_all(resumed)
    assert resumed.status == 'completed'
    assert resumed.state.initial_poses == original
    assert resumed.result.preposition_points.left.y == original.left.y
    assert resumed.result.preposition_points.right.y == original.right.y
    first_arm = next(i for i, (_, a) in enumerate(resumed.responder.actions)
                     if a.type.startswith('motion.'))
    assert not any(a.parameters.get('observation', '').startswith('ee_pose_')
                   for _, a in resumed.responder.actions[:first_arm])


def test_preposition_preserves_existing_xz_offsets_and_fixed_placement_orientation():
    params = inputs()
    initial = solver.BothPoses(**{
        hand: solver.ToolPose(x=0.4, y=y, z=0.5, qx=0.0, qy=0.0, qz=0.0, qw=1.0)
        for hand, y in [('left', 0.317), ('right', -0.243)]
    })
    center = (0.5, 0.0, 0.4)
    actual = solver.preposition(params, center, initial)
    legacy = solver.capture_solver(params).get_capture_point_place(
        dict(zip('xyz', center)), axis='both', put_mode=params.put_mode,
        safe_offset_x=params.preposition.x_offset_m,
        safe_offset_z=params.preposition.z_offset_m,
        safe_offset_y=params.preposition.y_offset_m,
    )
    for hand in ('left', 'right'):
        pose = getattr(actual, hand)
        for axis in ('x', 'z', 'qx', 'qy', 'qz', 'qw'):
            assert getattr(pose, axis) == pytest.approx(legacy[hand][axis])


def test_unversioned_checkpoint_with_saved_initial_pose_cannot_upgrade():
    ctx = Context()
    asyncio.run(skill.run(ctx))
    state = ctx.state.model_copy(deep=True)
    state.placement_revision = None
    resumed = Context(ctx.params)
    resumed.checkpoint(state)
    run_all(resumed)
    assert resumed.failure.code == 'PLACEMENT_CHECKPOINT_INCOMPATIBLE'
    assert not resumed.responder.actions
