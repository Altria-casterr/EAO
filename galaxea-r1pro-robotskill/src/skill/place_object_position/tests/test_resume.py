"""真实持久化边界的重入、输入隔离、暂停与停止；不连接硬件。"""
import asyncio
from datetime import timedelta

import pytest
from semantic_robot_skill_sdk import SkillCancelled, StopRequest

from place_object_position.scripts import skill
from place_object_position.scripts.models import PlaceState
from .support import Context, inputs, movements, resumed_context, run_all


@pytest.mark.parametrize('segment', [1, 2, 3])
@pytest.mark.parametrize('boundary', ['registered', 'result_returned', 'progress_saved'])
def test_retraction_resume_reuses_plan_and_never_replays_finished_motion(segment, boundary):
    ctx = Context()
    key = f'retract:both:{segment}'
    checkpoint, execute = ctx.checkpoint, ctx.execute
    crashed = False

    def crashing_checkpoint(state):
        nonlocal crashed
        checkpoint(state)
        should_crash = (boundary == 'registered' and key in state.actions and key not in state.results
                        or boundary == 'progress_saved' and state.retracted_steps == segment)
        if not crashed and should_crash:
            crashed = True
            raise RuntimeError('offline crash')

    async def crashing_execute(action_key, action):
        nonlocal crashed
        result = await execute(action_key, action)
        if not crashed and boundary == 'result_returned' and action_key.endswith(key):
            crashed = True
            raise RuntimeError('offline crash')
        return result

    ctx.checkpoint, ctx.execute = crashing_checkpoint, crashing_execute
    with pytest.raises(RuntimeError, match='offline crash'):
        run_all(ctx)
    saved_plan = [p.model_copy(deep=True) for p in ctx.state.retract_plan]
    resumed = resumed_context(ctx)
    run_all(resumed)
    assert resumed.status == 'completed'
    assert resumed.state.retract_plan == saved_plan
    physical_keys = [k for k, _ in movements(ctx) + movements(resumed)]
    assert len(physical_keys) == len(set(physical_keys)) == 6
    assert resumed.state.retracted_steps == 3


@pytest.mark.parametrize('stage', ['align_x', 'lower_z'])
def test_pause_is_cancellable_and_resume_waits_only_until_saved_deadline(monkeypatch, stage):
    ctx = Context(inputs(stage_pause_s=0.35))
    real_sleep, slept, stopping = asyncio.sleep, [], True

    async def fake_sleep(seconds):
        if ctx.state.stage in ctx.state.stage_pause_until:
            slept.append((ctx.state.stage, seconds))
            ctx.clock += timedelta(seconds=seconds)
            if ctx.state.stage == stage and stopping:
                ctx.request_stop()
        await real_sleep(0)

    monkeypatch.setattr(skill.asyncio, 'sleep', fake_sleep)
    with pytest.raises(SkillCancelled):
        run_all(ctx)
    deadline = ctx.state.stage_pause_until[stage]
    remaining = (deadline - ctx.now()).total_seconds()
    assert stage in ctx.state.positioned_stages
    before = movements(ctx)
    ctx = resumed_context(ctx)
    slept.clear()
    stopping = False
    run_all(ctx)
    assert ctx.status == 'completed'
    assert ctx.state.stage_pause_until[stage] == deadline
    assert sum(s for name, s in slept if name == stage) == pytest.approx(remaining)
    assert all(s <= 0.1 for _, s in slept)
    keys = [k for k, _ in before + movements(ctx)]
    assert len(keys) == len(set(keys)) == 6


def test_changed_input_rejected_before_any_new_action():
    ctx = Context()
    asyncio.run(skill.run(ctx))
    resumed = resumed_context(ctx, inputs(retract={'safe_offset_z': 0.15}))
    run_all(resumed)
    assert resumed.failure.code == 'PLACE_INPUT_CHANGED'
    assert not resumed.responder.actions


@pytest.mark.parametrize('case', ['other_skill', 'other_revision', 'unversioned_history'])
def test_incompatible_checkpoints_rejected_before_actions(case):
    ctx = Context()
    asyncio.run(skill.run(ctx))
    state = ctx.state.model_copy(deep=True)
    if case == 'other_skill':
        state.skill_name = 'place-object-force'
    else:
        state.placement_revision = 99 if case == 'other_revision' else None
    resumed = Context(ctx.params)
    resumed.checkpoint(state)
    run_all(resumed)
    assert resumed.failure.code == 'PLACEMENT_CHECKPOINT_INCOMPATIBLE'
    assert not resumed.responder.actions


@pytest.mark.parametrize('stage', ['lower_z', 'retract_both', 'verify'])
def test_missing_prior_stage_completion_prevents_motion(stage):
    ctx = Context()
    ctx.checkpoint(PlaceState(placement_revision=1, stage=stage))
    run_all(ctx)
    assert ctx.failure.code == 'PLACE_DATA_INVALID'
    assert not ctx.responder.actions


@pytest.mark.parametrize('started', [False, True])
def test_stop_reports_uncertainty_without_sending_movement(started):
    ctx = Context()
    if started:
        asyncio.run(skill.run(ctx))
    before = len(ctx.responder.actions)
    outcome = asyncio.run(skill.on_stop(ctx, StopRequest(source='user', reason='offline test')))
    assert outcome.safe is (not started)
    assert outcome.requires_intervention is started
    assert outcome.physical_state == ('unknown' if started else 'no_motion_started')
    assert len(ctx.responder.actions) == before


def test_stop_before_first_run_sends_no_actions():
    ctx = Context()
    ctx.request_stop()
    with pytest.raises(SkillCancelled):
        run_all(ctx)
    assert not ctx.responder.actions
