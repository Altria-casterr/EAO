"""单侧未到位或 Action 异常时，不得继续下一段或报告完成。"""
from copy import deepcopy

import pytest
from semantic_robot_skill_sdk import ActionResult

from .support import Context, movements, run_all


@pytest.mark.parametrize('suffix', ['prepare:torso', 'preposition:both', 'align_x:both',
                                    'lower_z:both', 'retract:both:1', 'retract:both:2', 'retract:both:3'])
@pytest.mark.parametrize('status', ['failed', 'stopped', 'interrupted'])
def test_physical_action_failure_stops_without_retry(suffix, status):
    ctx = Context()
    ctx.responder.mutate = lambda k, a, r: (
        ActionResult(status=status, error_message='offline injected failure') if k.endswith(suffix) else r)
    run_all(ctx)
    assert ctx.status == 'failed' and ctx.result is None
    assert ctx.failure.code == ('ACTION_STATE_UNKNOWN' if status == 'interrupted' else 'PLACE_ACTION_FAILED')
    assert ctx.responder.actions[-1][0].endswith(suffix)
    count = len(ctx.responder.actions)
    run_all(ctx)
    assert len(ctx.responder.actions) == count


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('suffix,axis,expected', [
    ('preposition:both', 'z', 'PLACE_HEIGHT_UNCONFIRMED'),
    ('align_x:both', 'z', 'PLACE_HEIGHT_UNCONFIRMED'),
    ('align_x:both', 'x', 'PLACE_POSE_UNCONFIRMED'),
    ('lower_z:both', 'z', 'PLACE_HEIGHT_UNCONFIRMED'),
    ('retract:both:1', 'z', 'PLACE_RETRACTION_UNCONFIRMED'),
    ('retract:both:2', 'y', 'PLACE_RETRACTION_UNCONFIRMED'),
    ('retract:both:3', 'x', 'PLACE_RETRACTION_UNCONFIRMED'),
])
def test_either_hand_out_of_tolerance_prevents_following_movement(side, suffix, axis, expected):
    ctx = Context()

    def drift(key, action, result):
        if key.endswith(suffix):
            ctx.responder.poses[side]['pose']['position'][axis] += 0.02
        return result

    ctx.responder.mutate = drift
    run_all(ctx)
    assert ctx.failure.code == expected
    assert ctx.result is None
    assert movements(ctx)[-1][0].endswith(suffix)
    if suffix.startswith('retract:'):
        assert ctx.state.retracted_steps == int(suffix[-1]) - 1


@pytest.mark.parametrize('side', ['left', 'right'])
def test_verify_reads_fresh_poses_and_rejects_drift(side):
    ctx = Context()
    checkpoint = ctx.checkpoint

    def drift_at_verify(state):
        checkpoint(state)
        if state.stage == 'verify':
            ctx.responder.poses[side]['pose']['position']['y'] += 0.1

    ctx.checkpoint = drift_at_verify
    run_all(ctx)
    assert ctx.failure.code == 'PLACE_RETRACTION_UNCONFIRMED'
    assert ctx.state.retracted_steps == 3 and len(movements(ctx)) == 6
    assert ctx.result is None


@pytest.mark.parametrize('case', ['failed', 'missing', 'nested_error', 'unknown_frame', 'zero_quaternion'])
def test_bad_observation_prevents_arm_motion(case):
    ctx = Context()

    def invalid(key, action, result):
        if action.parameters.get('observation') != 'ee_pose_left':
            return result
        if case == 'failed':
            return ActionResult(status='failed', error_message='no pose')
        output = deepcopy(result.output)
        if case == 'missing':
            output['result']['observations'] = []
        elif case == 'nested_error':
            output['result']['error'] = 'sensor failure'
        elif case == 'unknown_frame':
            output['result']['observations'][0]['data']['header']['frame_id'] = 'unknown'
        else:
            output['result']['observations'][0]['data']['pose']['orientation'] = dict.fromkeys('xyzw', 0.0)
        return result.model_copy(update={'output': output})

    ctx.responder.mutate = invalid
    run_all(ctx)
    assert ctx.status == 'failed'
    assert ctx.failure.code == ('PLACE_OBSERVATION_FAILED' if case == 'failed' else 'PLACE_DATA_INVALID')
    assert not movements(ctx)
