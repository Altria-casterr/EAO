"""最终 X/Z 目标、双手高度及崩溃恢复；不依赖力触阈。"""
from copy import deepcopy

import pytest

from place_object_force.scripts import solver
from place_object_force.scripts.models import BothPoses, PlaceState, ToolPose
from .support import Context, inputs
from .test_skill import movements, run_all


@pytest.mark.parametrize('side', ['left', 'right'])
def test_x_then_z_reach_final_box_target_without_inheriting_initial_height_difference(side):
    ctx = Context(inputs(put_mode=side))
    ctx.responder.poses['left']['pose']['position'].update(x=0.49, z=1.32)
    ctx.responder.poses['right']['pose']['position']['z'] = 1.28
    run_all(ctx)
    assert ctx.status == 'completed'
    assert ctx.state.placement_revision == 6
    start = ctx.result.preposition_points
    z, x = ctx.state.position_targets['lower_z'], ctx.state.position_targets['align_x']
    # state.relative_target_m 已在 prepare 减去 grasp_center_offset_m（本例默认零）。
    center = ctx.state.relative_target_m
    # 保留最新预定位参数：x 后退 0.03、z 抬高 0.07，不继承初始位姿与高差。
    for hand in ('left', 'right'):
        initial, lowered, forward = (getattr(p, hand) for p in (start, z, x))
        assert initial.x == pytest.approx(center[0] - 0.03)
        assert initial.z == pytest.approx(center[2] + 0.07)
        assert forward.x == pytest.approx(center[0])
        assert forward.y == initial.y and forward.z == initial.z
        assert lowered.z == pytest.approx(center[2])
        assert lowered.z - forward.z == pytest.approx(-0.07)
        assert lowered.x == forward.x and lowered.y == forward.y
        assert tuple(getattr(forward, k) for k in ('qx', 'qy', 'qz', 'qw')) == tuple(
            getattr(initial, k) for k in ('qx', 'qy', 'qz', 'qw'))
    assert start.left.y == pytest.approx(0.35)
    assert start.right.y == pytest.approx(-0.35)
    assert (x.left.x + x.right.x) / 2 == pytest.approx(center[0])
    assert (z.left.z + z.right.z) / 2 == pytest.approx(center[2])
    assert x.left.x == x.right.x == pytest.approx(center[0])
    assert [key.rsplit(':', 2)[-2] for key, _ in movements(ctx)
            if key.endswith(('align_x:both', 'lower_z:both'))] == ['align_x', 'lower_z']
    for stage in ('align_x', 'lower_z'):
        actions = [a for k, a in movements(ctx) if k.endswith(f'{stage}:both')]
        assert len(actions) == 1
        assert actions[0].type == 'motion.move_end_effector_both_trajectory'
        assert actions[0].parameters['left_pos_tol'] == 0.005
        assert actions[0].parameters['timeout'] == 10.0
        assert actions[0].parameters['max_num_points'] == 100
    assert set(ctx.result.contacts) == {'push_y'}
    assert not any(a.parameters.get('observation') == 'force_data'
                   for stage, (_, a) in zip(ctx.responder.action_stages, ctx.responder.actions)
                   if stage in {'lower_z', 'align_x'})


def test_hand_target_subtracts_tool_offset_xz_only():
    params = inputs(preposition={'grasp_center_offset_m': [0.1, 0.0, 0.0]})
    assert solver.hand_target_m(params, (0.5, 0.0, 0.4)) == pytest.approx((0.4, 0.0, 0.4))
    params_z = inputs(preposition={'grasp_center_offset_m': [0.1, 0.0, 0.02]})
    assert solver.hand_target_m(params_z, (0.5, 0.2, 0.4)) == pytest.approx((0.4, 0.2, 0.38))


def test_position_offsets_measured_from_current_pair_midpoint():
    center = (0.5, 0.0, 0.4)
    identity = dict(qx=0.0, qy=0.0, qz=0.0, qw=1.0)
    # 双手中点 x=0.47、z=0.45，但保留 0.02 的手间高差与 x 间距。
    current = BothPoses(left=ToolPose(x=0.48, y=0.4, z=0.46, **identity),
                        right=ToolPose(x=0.46, y=-0.2, z=0.44, **identity))
    offsets = solver.position_offsets(center, current)
    assert offsets == pytest.approx({'z': -0.05, 'x': 0.03})
    z = solver.position_targets(center, current, 'z', current)
    assert (z.left.z, z.right.z) == pytest.approx((0.41, 0.39))
    assert z.left.x == 0.48 and z.right.x == 0.46
    x = solver.position_targets(center, current, 'x', current)
    assert (x.left.x, x.right.x) == pytest.approx((0.51, 0.49))
    assert x.left.z == 0.46 and x.right.z == 0.44


def test_tool_offset_x01_propagates_to_preposition_and_final_hand_target():
    ctx = Context(inputs(put_mode='left',
                         preposition={'grasp_center_offset_m': [0.1, 0.0, 0.0]}))
    run_all(ctx)
    assert ctx.status == 'completed'
    # 箱心（map→torso 单位变换）= (0.5, 0.0, 0.4)，双手目标 = (0.4, 0.0, 0.4)。
    assert ctx.state.relative_target_m == pytest.approx((0.4, 0.0, 0.4))
    start = ctx.result.preposition_points
    z, x = ctx.state.position_targets['lower_z'], ctx.state.position_targets['align_x']
    for hand in ('left', 'right'):
        initial, lowered, forward = (getattr(p, hand) for p in (start, z, x))
        assert initial.x == pytest.approx(0.37)       # 0.4 - 0.03
        assert initial.z == forward.z == pytest.approx(0.47)  # 0.4 + 0.07
        assert lowered.z == pytest.approx(0.4)
        assert lowered.z - forward.z == pytest.approx(-0.07)
        assert forward.x == pytest.approx(0.4)        # 双手最终 X = 箱心 0.5 - 0.1
        assert lowered.x == forward.x
    # Y 保持起始实测值，不受目标和 X/Z 工具偏移影响。
    assert start.left.y == pytest.approx(0.35) and start.right.y == pytest.approx(-0.35)


def test_z_x_offsets_absorb_live_drift_instead_of_reusing_saved_plan():
    ctx = Context()
    drift_x, drift_z = 0.02, -0.002

    def drift_after_preposition(key, action, result):
        if key.endswith('preposition:both'):
            for hand in ('left', 'right'):
                ctx.responder.poses[hand]['pose']['position']['x'] += drift_x
                ctx.responder.poses[hand]['pose']['position']['z'] += drift_z
        return result

    ctx.responder.mutate = drift_after_preposition
    run_all(ctx)
    assert ctx.status == 'completed'
    center = ctx.state.relative_target_m
    z, x = ctx.state.position_targets['lower_z'], ctx.state.position_targets['align_x']
    plan = ctx.state.preposition_points
    # 偏移从实测位姿中点算出：Z 多补 0.002、X 少走 0.02。
    assert z.left.z - (plan.left.z + drift_z) == pytest.approx(-0.07 - drift_z)
    assert x.left.x - (plan.left.x + drift_x) == pytest.approx(0.03 - drift_x)
    assert x.left.z == pytest.approx(plan.left.z + drift_z)
    assert z.left.x == x.left.x
    # 双手同步平移保持手间关系，最终仍精确到达 relative_target_m。
    assert z.left.z == z.right.z == pytest.approx(center[2])
    assert x.left.x == x.right.x == pytest.approx(center[0])
    assert z.left.y == plan.left.y and x.right.y == plan.right.y


@pytest.mark.parametrize('stage', ['preposition', 'lower_z', 'align_x'])
@pytest.mark.parametrize('errors', [
    (0.006, 0.0), (-0.006, 0.0), (0.006, 0.006), (0.004, -0.004),
    (0.005001, 0.0), (0.002501, -0.002501),
])
def test_height_or_pair_gap_exceeding_tolerance_blocks_all_later_motion(stage, errors):
    ctx = Context()

    def miss_height(key, action, result):
        if key.endswith(f'{stage}:both'):
            for hand, error in zip(('left', 'right'), errors):
                ctx.responder.poses[hand]['pose']['position']['z'] += error
        return result

    ctx.responder.mutate = miss_height
    run_all(ctx)
    assert ctx.failure.code == 'PLACE_HEIGHT_UNCONFIRMED'
    assert ctx.state.stage == stage
    assert stage not in ctx.state.positioned_stages
    assert movements(ctx)[-1][0].endswith(f'{stage}:both')
    assert sum(key.endswith(f'{stage}:both') for key, _ in movements(ctx)) == 1
    assert '高度差=' in ctx.state.failure_message and '容限=0.005000 m' in ctx.state.failure_message
    assert ctx.result is None


@pytest.mark.parametrize('stage,positioned', [
    ('lower_z', []),
    ('retract_assist', []),
    ('retract_assist', ['align_x']),
    ('retract_assist', ['lower_z']),
])
def test_position_preconditions_reject_out_of_order_stages_without_actions(stage, positioned):
    ctx = Context()
    ctx.checkpoint(PlaceState(placement_revision=6, stage=stage, positioned_stages=positioned))
    run_all(ctx)
    assert ctx.failure.code == 'PLACE_DATA_INVALID'
    assert not ctx.responder.actions


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('lower', [0.0, 0.02, 0.05])
def test_retraction_exit_is_three_cm_above_start_independent_of_lowering(side, lower):
    params = inputs(retract={'lower_m': lower})
    current = ToolPose(x=0.5, y=0.4 if side == 'left' else -0.2, z=0.4,
                       qx=0.0, qy=0.0, qz=0.0, qw=1.0)
    plan = solver.retract(params, (0.5, 0.0, 0.4), side, current)
    assert plan[0].z == plan[1].z == pytest.approx(current.z - lower)
    assert plan[2].z == pytest.approx(current.z + 0.03)
    assert plan[2].x == pytest.approx(current.x - params.retract.backward_m)
    assert plan[2].y == plan[1].y
    for target in plan:
        assert (target.qx, target.qy, target.qz, target.qw) == (0.0, 0.0, 0.0, 1.0)


@pytest.mark.parametrize('stage', ['preposition', 'lower_z', 'align_x'])
@pytest.mark.parametrize('errors', [(0.005, 0.0), (-0.005, 0.0), (0.0025, -0.0025), (0.005, 0.005)])
def test_height_and_gap_at_tolerance_boundary_can_continue(stage, errors):
    ctx = Context()

    def boundary_height(key, action, result):
        if key.endswith(f'{stage}:both'):
            for hand, error in zip(('left', 'right'), errors):
                ctx.responder.poses[hand]['pose']['position']['z'] += error
        return result

    ctx.responder.mutate = boundary_height
    run_all(ctx)
    assert ctx.status == 'completed'


@pytest.mark.parametrize('tolerance, succeeds', [(0.002, False), (0.01, True)])
def test_height_guard_uses_configured_motion_position_tolerance(tolerance, succeeds):
    ctx = Context(inputs(motion={'pos_tol': tolerance}))

    def unequal_height(key, action, result):
        if key.endswith('preposition:both'):
            ctx.responder.poses['left']['pose']['position']['z'] += 0.0015
            ctx.responder.poses['right']['pose']['position']['z'] -= 0.0015
        return result

    ctx.responder.mutate = unequal_height
    run_all(ctx)
    assert (ctx.status == 'completed') is succeeds
    if not succeeds:
        assert ctx.failure.code == 'PLACE_HEIGHT_UNCONFIRMED'


@pytest.mark.parametrize('side', ['left', 'right'])
def test_force_align_runs_on_main_hand_only(side):
    ctx = Context(inputs(put_mode=side))
    run_all(ctx)
    assert ctx.status == 'completed'
    forces = [a for k, a in ctx.responder.actions if a.type == 'manipulation.force_align']
    assert len(forces) == 1
    assert forces[0].parameters['axis'] == side
    assert forces[0].parameters['direction'] == ('negative' if side == 'left' else 'positive')
    assert forces[0].parameters['move_coordinate'] == 'y'


@pytest.mark.parametrize('stage', ['lower_z', 'align_x'])
@pytest.mark.parametrize('hand', ['left', 'right'])
@pytest.mark.parametrize('error', ['position', 'orientation'])
def test_unconfirmed_z_x_blocks_later_motion(stage, hand, error):
    ctx = Context()

    def miss_target(key, action, result):
        if key.endswith(f'{stage}:both'):
            pose = ctx.responder.poses[hand]['pose']
            if error == 'position':
                pose['position']['x'] += 0.02
            else:
                pose['orientation']['z'] = 0.2
        return result

    ctx.responder.mutate = miss_target
    run_all(ctx)
    assert ctx.failure.code == 'PLACE_POSE_UNCONFIRMED'
    assert stage not in ctx.state.positioned_stages
    assert movements(ctx)[-1][0].endswith(f'{stage}:both')
    assert ctx.result is None


@pytest.mark.parametrize('stage', ['lower_z', 'align_x'])
@pytest.mark.parametrize('registered', [False, True])
def test_pending_z_x_resume_reuses_absolute_targets_without_adding_offset_again(stage, registered):
    ctx = Context()
    checkpoint = ctx.checkpoint
    crashed = False

    def crashing(state):
        nonlocal crashed
        checkpoint(state)
        key = f'{stage}:both'
        if (stage in state.position_targets and (key in state.actions) is registered
                and key not in state.results and not crashed):
            crashed = True
            raise RuntimeError('crash before fixed move')

    ctx.checkpoint = crashing
    with pytest.raises(RuntimeError, match='crash before fixed move'):
        run_all(ctx)
    target = deepcopy(ctx.state.position_targets[stage])
    ctx.checkpoint = checkpoint
    run_all(ctx)
    assert ctx.status == 'completed'
    assert ctx.state.position_targets[stage] == target
    action = ctx.state.actions[f'{stage}:both']
    assert action.parameters['left_target_pose'] == target.left.action_pose()
    assert sum(key.endswith(f'{stage}:both') for key, _ in movements(ctx)) == 1
