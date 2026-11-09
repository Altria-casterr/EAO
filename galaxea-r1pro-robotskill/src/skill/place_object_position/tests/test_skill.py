import os
from pathlib import Path

import pytest
import yaml
from pydantic import ValidationError

from place_object_position.scripts import solver
from place_object_position.scripts.models import BothPoses, PlaceInput, ToolPose
from .support import Context, inputs, movements, run_all


@pytest.mark.parametrize('torso_enabled', [True, False])
@pytest.mark.parametrize('target_y', [-0.3, 0.0, 0.3])
def test_position_flow_preserves_initial_y_and_retracts_both_without_force(torso_enabled, target_y):
    target = inputs().target.model_dump(mode='json')
    target['position_m'][1] = target_y
    ctx = run_all(Context(inputs(target=target, torso={
        'enabled': torso_enabled, 'positions': [0.0, 0.0, 0.0, 0.0]})))
    assert ctx.status == 'completed'
    assert [e['stage'] for e in ctx.events if e.get('type') == 'stage.completed'] == [
        'prepare', 'preposition', 'align_x', 'lower_z', 'retract_both', 'verify']
    actions = movements(ctx)
    assert len(actions) == 6
    assert [a.type for _, a in actions] == ['motion.move_end_effector_both_trajectory'] * 3 + [
        'motion.move_end_effector_both'] * 3
    for key, action in actions[:3]:
        assert action.parameters['left_target_pose']['target_y'] == 0.317
        assert action.parameters['right_target_pose']['target_y'] == -0.243
    assert [k.rsplit(':', 3)[-3:] for k, _ in actions[-3:]] == [
        ['retract', 'both', '1'], ['retract', 'both', '2'], ['retract', 'both', '3']]
    for _, action in ctx.responder.actions:
        assert action.type != 'manipulation.force_align'
        assert action.parameters.get('observation') != 'force_data'
        assert action.timeout_seconds > 0
    assert ctx.state.skill_name == 'place-object-position'
    assert ctx.state.placement_revision == 1
    assert ctx.state.positioned_stages == ['align_x', 'lower_z']
    assert ctx.state.retracted_steps == 3
    assert ctx.result.retracted_sides == ['left', 'right']
    assert ctx.result.placed and ctx.result.holding_object is None
    assert ctx.result.verification_method == 'position_and_retraction'
    result = ctx.result.model_dump()
    assert not {'contacts', 'final_forces', 'put_mode'} & result.keys()
    assert ctx.result.final_poses.left.y == pytest.approx(0.347)
    assert ctx.result.final_poses.right.y == pytest.approx(-0.273)
    assert ctx.result.final_poses.left.z == pytest.approx(0.43)
    assert (ctx.responder.torso_positions is not None) is torso_enabled
    assert all(k.startswith('place-object-position:') for k, _ in ctx.responder.actions)


def test_prepare_waits_for_torso_and_saves_transformed_initial_pose_before_arm_motion():
    ctx = Context()

    def torso(key, action, result):
        if action.type == 'torso.move_to_posture_all':
            ctx.responder.floating = {'position': [0.1, -0.2, -0.95],
                                     'orientation': [0.0, 0.0, 2**-0.5, 2**-0.5]}
        return result

    ctx.responder.mutate = torso
    ctx.responder.poses['left']['pose']['position']['x'] = 0.42
    ctx.responder.poses['right']['pose']['position']['x'] = 0.27
    run_all(ctx)
    assert ctx.status == 'completed'
    assert ctx.state.initial_poses.left.y == pytest.approx(0.22)
    assert ctx.state.initial_poses.right.y == pytest.approx(0.07)
    assert ctx.state.relative_target_m[2] == pytest.approx(0.25)
    trace = ctx.responder.actions
    assert trace[0][1].type == 'torso.move_to_posture_all'
    first_arm = next(i for i, (_, a) in enumerate(trace) if a.type.startswith('motion.'))
    for name in ['floating_base', 'transformer_martix', 'ee_pose_left', 'ee_pose_right']:
        assert next(i for i, (_, a) in enumerate(trace) if a.parameters.get('observation') == name) < first_arm


def test_tool_offset_and_live_xz_feedback_reach_final_target():
    ctx = Context(inputs(preposition={'grasp_center_offset_m': [0.07, 0.0, 0.02]}))

    def drift(key, action, result):
        if key.endswith('preposition:both'):
            for hand in ('left', 'right'):
                ctx.responder.poses[hand]['pose']['position']['x'] += 0.02
                ctx.responder.poses[hand]['pose']['position']['z'] -= 0.002
                ctx.responder.poses[hand]['pose']['position']['y'] += 0.001
        return result

    ctx.responder.mutate = drift
    run_all(ctx)
    assert ctx.status == 'completed'
    assert ctx.state.relative_target_m == pytest.approx((0.43, 0.0, 0.38))
    for hand in ('left', 'right'):
        initial = getattr(ctx.state.preposition_points, hand)
        x = getattr(ctx.state.position_targets['align_x'], hand)
        z = getattr(ctx.state.position_targets['lower_z'], hand)
        assert initial.x == pytest.approx(0.40)
        assert initial.z == pytest.approx(0.45)
        assert x.x == z.x == pytest.approx(0.43)
        assert x.z == pytest.approx(0.448)
        assert z.z == pytest.approx(0.38)
        assert x.y == z.y == getattr(ctx.state.initial_poses, hand).y


@pytest.mark.parametrize('lower', [0.0, 0.02, 0.05])
@pytest.mark.parametrize('lift', [0.03, 0.15])
@pytest.mark.parametrize('outward', [0.03, 0.1])
def test_paired_retraction_geometry_preserves_each_start_pose(lower, lift, outward):
    start = BothPoses(left=ToolPose(x=0.5, y=0.317, z=0.4, qx=0.1, qy=0.0, qz=0.0, qw=1.0),
                      right=ToolPose(x=0.48, y=-0.243, z=0.402, qx=0.0, qy=0.1, qz=0.0, qw=1.0))
    before = start.model_dump()
    plan = solver.retract_targets(inputs(retract={'lower_m': lower, 'safe_offset_z': lift,
                                                  'minimum_outward_m': outward}), start)
    assert len(plan) == 3
    for side, sign in [('left', 1), ('right', -1)]:
        original = getattr(start, side)
        down, out, exit_pose = [getattr(p, side) for p in plan]
        assert down.z == out.z == pytest.approx(original.z - lower)
        assert down.y == original.y
        assert out.y == exit_pose.y == pytest.approx(original.y + sign * outward)
        assert down.x == out.x == original.x
        assert exit_pose.x == pytest.approx(original.x - 0.1)
        assert exit_pose.z == pytest.approx(original.z + lift)
        for p in (down, out, exit_pose):
            assert [getattr(p, q) for q in ('qx', 'qy', 'qz', 'qw')] == pytest.approx(
                [getattr(original, q) for q in ('qx', 'qy', 'qz', 'qw')])
    assert start.model_dump() == before


@pytest.mark.parametrize('patch', [
    {'y_force': {}}, {'put_mode': 'left'}, {'force_phase_pause_s': 0.0},
    {'motion': {'fine_pos_tol': 0.001}}, {'motion': {'step_timeout_s': 2.0}},
    {'preposition': {'y_offset_m': 0.1}}, {'retract': {'lift_m': 0.15}},
    {'motion': {'num_points': 4}}, {'motion': {'axis_order': 'xxx'}},
    {'torso': {'positions': None}},
])
def test_removed_or_invalid_parameters_rejected(patch):
    with pytest.raises(ValidationError):
        inputs(**patch)


def test_example_schema_and_manifest_match_real_action_contracts():
    root = Path(__file__).resolve().parents[4]
    example = PlaceInput.model_validate_json((root/'inputs/place_object_position.example.json').read_text())
    defaults = inputs(stage_pause_s=0.5)
    assert example.retract.safe_offset_z == 0.15 and defaults.retract.safe_offset_z == 0.03
    assert example.preposition.grasp_center_offset_m == (0.07, 0.0, 0.0)
    assert defaults.torso.timeout == defaults.motion.timeout == defaults.approach_motion.timeout == defaults.retract_motion.timeout == 10.0
    assert set(PlaceInput.model_json_schema()['required']) == {'target'}
    assert defaults.stage_pause_s == 0.5
    workspace = Path(__file__).resolve().parents[6]
    ability = Path(os.environ.get('PLACE_ABILITY_MANIFESTS') or
                   workspace/'semantic-ability/galaxea-r1pro-ability/src/galaxea_r1pro_0/abilities')
    tasks = {}
    for name in ('robot_observation', 'joint_eef_pose_control', 'torso_control'):
        tasks.update({t['actionType']: t for t in yaml.safe_load((ability/name/'ability.manifest.yaml').read_text())['tasks'] if t.get('actionType')})
    ctx = run_all(Context())
    manifest = yaml.safe_load((Path(__file__).parents[1]/'SKILL.md').read_text().split('---', 2)[1])
    assert manifest['name'] == 'place-object-position' and manifest['version'] == '0.1.0'
    assert {a.type for _, a in ctx.responder.actions} == {a['type'] for a in manifest['required_actions']}
    assert {a['type'] for a in manifest['stop_actions']} == {
        'torso.move_to_posture_all', 'motion.move_end_effector_both_trajectory', 'motion.move_end_effector_both'}
    for key, action in ctx.responder.actions:
        task = tasks[action.type]
        declared = {p['name'] for p in task['params']}
        mandatory = {p['name'] for p in task['params'] if not p.get('optional', False)}
        assert mandatory <= set(action.parameters) <= declared
        if ':retract:both:' in key:
            assert 'num_points' not in action.parameters
            assert action.parameters['timeout'] == 10.0
            assert action.timeout_seconds == 25.0
