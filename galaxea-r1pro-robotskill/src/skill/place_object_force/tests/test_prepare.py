"""躯干准备及其后的坐标求解；所有动作均由离线替身响应。"""
import asyncio

import pytest
from pydantic import ValidationError
from semantic_robot_skill_sdk import SkillCancelled, StopRequest

from place_object_force.scripts.skill import on_stop
from .support import Context, inputs
from .test_skill import movements, run_all


def test_torso_completes_before_transform_and_preposition_planning():
    ctx = Context(inputs(torso={'positions': [0.1, 0.2, -0.2, 0.3]}))

    def move_torso(key, action, result):
        if action.type == 'torso.move_to_posture_all':
            ctx.responder.floating['position'][2] = -0.95
        return result

    ctx.responder.mutate = move_torso
    run_all(ctx)
    trace = ctx.responder.actions
    assert ctx.status == 'completed'
    assert trace[0][1].type == 'torso.move_to_posture_all'
    assert ctx.responder.torso_positions == [0.1, 0.2, -0.2, 0.3]
    assert ctx.state.base_to_relative.position[2] == -0.95
    assert ctx.state.relative_target_m[2] == pytest.approx(0.25)
    assert ctx.result.preposition_points.left.z == pytest.approx(0.25 + ctx.params.preposition.z_offset_m)
    names = [action.parameters.get('observation') for _, action in trace]
    first_arm = next(i for i, (_, action) in enumerate(trace) if action.type.startswith('motion.'))
    assert 0 < names.index('floating_base') < names.index('transformer_martix') < first_arm


def test_torso_can_be_explicitly_disabled():
    ctx = run_all(Context(inputs(torso={'enabled': False})))
    assert ctx.status == 'completed'
    assert not any(action.type.startswith('torso.') for _, action in ctx.responder.actions)
    assert ctx.responder.torso_positions is None


@pytest.mark.parametrize('torso', [{}, {'positions': None}, {'positions': [0.0, 0.0]},
                                  {'positions': [0.0, 0.0, float('nan'), 0.0]}])
def test_torso_requires_four_finite_calibrated_positions(torso):
    with pytest.raises(ValidationError):
        inputs(torso=torso)


def test_stop_during_torso_reports_physical_state_unknown_and_prevents_arm_motion():
    ctx = Context()

    def stopping(key, action, result):
        if action.type == 'torso.move_to_posture_all':
            ctx.request_stop()
        return result

    ctx.responder.mutate = stopping
    with pytest.raises(SkillCancelled):
        run_all(ctx)
    outcome = asyncio.run(on_stop(ctx, StopRequest(source='user', reason='stop')))
    assert outcome.safe is False and outcome.requires_intervention is True
    assert outcome.physical_state == 'unknown'
    assert not movements(ctx)
