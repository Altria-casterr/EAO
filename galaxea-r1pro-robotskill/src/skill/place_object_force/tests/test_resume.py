"""检查真实崩溃边界、现场漂移及段间等待；不连接硬件。"""
import asyncio
from copy import deepcopy
from datetime import timedelta

import pytest
from semantic_robot_skill_sdk import ActionResult, SkillCancelled

from place_object_force.scripts import skill
from place_object_force.scripts.models import PlaceState
from .support import Context, inputs
from .test_skill import movements, run_all


def crash_before_force_align(ctx, *, registered):
    checkpoint = ctx.checkpoint
    crashed = False
    key = 'push_y:force'

    def crashing(state):
        nonlocal crashed
        checkpoint(state)
        if (state.stage == 'push_y' and not crashed
                and (key in state.actions) is registered and key not in state.results):
            crashed = True
            raise RuntimeError('crash before force_align execute')

    ctx.checkpoint = crashing
    with pytest.raises(RuntimeError, match='crash before force_align execute'):
        run_all(ctx)
    ctx.checkpoint = checkpoint


@pytest.mark.parametrize('registered', [False, True])
def test_force_align_resume_after_crash_completes(registered):
    ctx = Context()
    crash_before_force_align(ctx, registered=registered)
    before = len(ctx.responder.actions)
    run_all(ctx)
    assert ctx.status == 'completed'
    assert 'push_y' in ctx.state.contacts
    # 无论崩溃发生在提交前还是提交后，force_align 只执行一次（提交后崩溃复用缓存结果）。
    assert sum(k.endswith('push_y:force') for k, _ in ctx.responder.actions) == 1


def test_registered_but_unconfirmed_force_align_fails_action_state_unknown():
    ctx = Context()
    crash_before_force_align(ctx, registered=True)
    # 恢复时让已注册的 force_align 返回 interrupted，模拟现场未知。
    ctx.responder.mutate = lambda k, a, r: (
        ActionResult(status='interrupted', error_message='lost') if k.endswith('push_y:force') else r)
    run_all(ctx)
    assert ctx.failure.code == 'ACTION_STATE_UNKNOWN'
    assert 'push_y' not in ctx.state.contacts
    # 崩溃发生在 execute 之前，force_align 只在恢复时执行一次并被中断。
    assert sum(k.endswith('push_y:force') for k, _ in ctx.responder.actions) == 1


def test_force_align_crash_after_result_records_contact_on_resume():
    ctx = Context()
    execute = ctx.execute
    crashed = False

    async def crashing(k, action):
        nonlocal crashed
        result = await execute(k, action)
        if k.endswith('push_y:force') and not crashed:
            crashed = True
            raise RuntimeError('crash after force_align result')
        return result

    ctx.execute = crashing
    with pytest.raises(RuntimeError, match='crash after force_align result'):
        run_all(ctx)
    ctx.execute = execute
    run_all(ctx)
    assert ctx.status == 'completed'
    assert 'push_y' in ctx.state.contacts
    # 结果已缓存，恢复时不再重新执行 force_align。
    assert sum(k.endswith('push_y:force') for k, _ in ctx.responder.actions) == 1


@pytest.mark.parametrize('stage', ['preposition', 'lower_z', 'retract_assist', 'verify'])
def test_legacy_physical_checkpoint_cannot_upgrade_or_overwrite_actions(stage):
    ctx = Context()
    ctx.checkpoint(PlaceState(stage=stage, physical_action_started=True))
    original = deepcopy(ctx.state.actions)
    run_all(ctx)
    assert ctx.failure.code == 'PLACEMENT_CHECKPOINT_INCOMPATIBLE'
    assert ctx.state.actions == original
    assert not movements(ctx)


@pytest.mark.parametrize('revision', [None, 1, 2, 3, 4, 5])
def test_legacy_prepare_checkpoint_rejected_without_reusing_old_coordinates(revision):
    ctx = Context()
    asyncio.run(skill.run(ctx))
    state = ctx.state.model_copy(deep=True)
    state.placement_revision = revision
    original = deepcopy(state.preposition_points)
    fresh = Context(ctx.params)
    fresh.checkpoint(state)
    run_all(fresh)
    assert fresh.failure.code == 'PLACEMENT_CHECKPOINT_INCOMPATIBLE'
    assert fresh.state.preposition_points == original
    assert not fresh.responder.actions


@pytest.mark.parametrize('revision', [4, 5])
@pytest.mark.parametrize('stage', ['align_x', 'lower_z', 'retract_assist'])
def test_older_revision_in_progress_cannot_resume_with_new_placement(stage, revision):
    ctx = Context()
    ctx.checkpoint(PlaceState(placement_revision=revision, stage=stage,
                              positioned_stages=['lower_z'], physical_action_started=True))
    run_all(ctx)
    assert ctx.failure.code == 'PLACEMENT_CHECKPOINT_INCOMPATIBLE'
    assert not ctx.responder.actions


def test_unknown_checkpoint_revision_is_terminal():
    ctx = Context()
    ctx.checkpoint(PlaceState(placement_revision=99))
    run_all(ctx)
    assert ctx.failure.code == 'PLACEMENT_CHECKPOINT_INCOMPATIBLE'
    assert not ctx.responder.actions


@pytest.mark.parametrize('stage', ['lower_z', 'align_x', 'push_y'])
def test_force_phase_pause_checks_stop_and_preserves_deadline_on_resume(monkeypatch, stage):
    ctx = Context(inputs(force_phase_pause_s=0.35))
    original_sleep = asyncio.sleep
    slept = []
    stopping = True

    async def fake_sleep(seconds):
        if ctx.state.stage in ctx.state.force_phase_pause_until:
            slept.append((ctx.state.stage, seconds))
            ctx.clock += timedelta(seconds=seconds)
            if ctx.state.stage == stage and stopping:
                ctx.request_stop()
        await original_sleep(0)

    monkeypatch.setattr(skill.asyncio, 'sleep', fake_sleep)
    with pytest.raises(SkillCancelled):
        run_all(ctx)
    deadline = ctx.state.force_phase_pause_until[stage]
    old_count = len(movements(ctx))
    assert (stage in ctx.state.contacts if stage == 'push_y' else stage in ctx.state.positioned_stages)
    assert ctx.state.stage == stage
    assert all(seconds <= 0.1 for _, seconds in slept)
    remaining = (deadline - ctx.now()).total_seconds()
    # 用新 Context 模拟 Worker 恢复，停止锁存属于原执行，不能在原执行内清除。
    resumed = Context(ctx.params)
    resumed.clock = ctx.clock
    resumed.checkpoint(ctx.state.model_copy(deep=True))
    resumed.responder.poses = deepcopy(ctx.responder.poses)
    resumed._records = deepcopy(ctx._records)
    ctx = resumed
    stopping = False
    slept.clear()
    run_all(ctx)
    assert ctx.status == 'completed'
    assert ctx.state.force_phase_pause_until[stage] == deadline
    assert sum(seconds for name, seconds in slept if name == stage) == pytest.approx(remaining)
    assert len(movements(ctx)) > 0
    assert old_count > 0
