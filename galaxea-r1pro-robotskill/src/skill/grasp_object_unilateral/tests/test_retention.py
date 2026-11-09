"""L1 force gate, bounded retry and durable action identity; no robot connection."""
import asyncio
from unittest.mock import AsyncMock
import pytest
from pydantic import ValidationError
from semantic_robot_skill_sdk import ActionResult, run_skill
from grasp_object_unilateral.scripts import skill
from .support import Context, inputs

@pytest.fixture(autouse=True)
def no_wait(monkeypatch):
    monkeypatch.setattr(skill.asyncio, "sleep", AsyncMock())

def run_all(ctx):
    asyncio.run(run_skill(skill.run, ctx))
    return ctx

def force_values(ctx, before, after):
    def mutate(key, action, result):
        if action.parameters.get("observation") == "force_data":
            attempt = int(key.split("attempt:")[1].split(":")[0])
            values = before[attempt] if key.endswith("force-before") else after[attempt]
            data = {ctx._input.grasp_axis + "_arm": dict(zip(("force_x", "force_y", "force_z"), values))}
            return ActionResult(status="succeeded", output={"observations": [{"name": "force_data", "data": data}]})
        return result
    ctx.responder.mutate = mutate

@pytest.mark.parametrize("side", ["left", "right"])
def test_l1_not_l2_and_exact_sampling_positions(side):
    ctx = Context(inputs(grasp_axis=side))
    force_values(ctx, [(-3, 0, 0)], [(-2, 2, -1)])
    run_all(ctx)
    assert ctx.status == "completed"
    check = ctx.state.pull_force_checks[0]
    assert (check.before_l1_n, check.after_l1_n, check.passed) == (3, 5, True)
    keys = [k for k, _ in ctx.responder.actions]
    idx = lambda suffix: next(i for i, k in enumerate(keys) if k.endswith(suffix))
    assert idx("grasp-move") < idx("force-before") < idx("grasp-force")
    assert idx("grasp-pull") < idx("force-after") < idx("other-y-move")
    assert sum(k.endswith(("force-before", "force-after")) for k in keys) == 2

@pytest.mark.parametrize("after", [(1.5, 0, 0), (1.49, 0, 0)])
def test_equal_or_below_threshold_stops_before_auxiliary_when_no_retries(after):
    ctx = Context(inputs(max_grasp_retries=0))
    force_values(ctx, [(1, 0, 0)], [after])
    run_all(ctx)
    assert ctx.status == "failed"
    assert ctx.state.failure_code == "GRASP_RETENTION_FAILED"
    assert not any(":other" in k or k.endswith("lift:up") for k, _ in ctx.responder.actions)


def test_retry_reidentifies_replans_and_resamples_without_torso_or_retreat():
    ctx = Context(inputs(torso={"enabled": True, "positions": [0, 0, 0, 0]}))
    force_values(ctx, [(1, 0, 0), (2, 0, 0)], [(1.4, 0, 0), (3.1, 0, 0)])
    run_all(ctx)
    assert ctx.status == "completed"
    assert ctx.state.grasp_attempt == 1
    assert [c.passed for c in ctx.state.pull_force_checks] == [False, True]
    keys = [k for k, _ in ctx.responder.actions]
    assert sum(k.endswith("prepare:match") for k in keys) == 2
    assert sum(k.endswith("prepare:pose:left") for k in keys) == 2
    assert sum(k.endswith("prepare:torso") for k in keys) == 1
    assert sum(k.endswith("prepare:transform") for k in keys) == 1
    assert any("attempt:0:approach:grasp-pull" in k for k in keys)
    assert any("attempt:1:approach:grasp-pull" in k for k in keys)
    assert not any("attempt:0:approach:other" in k for k in keys)
    after_index = next(i for i,k in enumerate(keys) if "attempt:0:approach:force-after" in k)
    assert keys[after_index + 1].endswith("attempt:1:prepare:match")
    assert "attempt:0:approach:grasp-pull" in ctx.state.results
    assert "attempt:1:approach:grasp-pull" in ctx.state.results


def test_retry_uses_new_baseline_and_exhausts_limit():
    ctx = Context(inputs())
    force_values(ctx, [(1, 0, 0), (2, 0, 0)], [(1, 0, 0), (2.5, 0, 0)])
    run_all(ctx)
    assert ctx.status == "failed"
    assert ctx.state.failure_code == "GRASP_RETENTION_FAILED"
    assert len(ctx.state.pull_force_checks) == 2
    assert not any("approach:other" in k for k, _ in ctx.responder.actions)

@pytest.mark.parametrize("key", ["force-before", "force-after"])
@pytest.mark.parametrize("bad", [None, float("nan"), float("inf"), True, "bad"])
def test_invalid_readings_do_not_retry(key, bad):
    ctx = Context(inputs())
    def mutate(action_key, action, result):
        if action_key.endswith(key):
            return ActionResult(status="succeeded", output={"observations": [{"name":"force_data", "data":{"left_arm":{"force_x":bad,"force_y":0,"force_z":0}}}]})
        return result
    ctx.responder.mutate = mutate
    run_all(ctx)
    assert ctx.status == "failed"
    assert ctx.state.grasp_attempt == 0
    assert not any("approach:other" in k for k, _ in ctx.responder.actions)

@pytest.mark.parametrize("key", ["force-before", "force-after", "grasp-pull"])
@pytest.mark.parametrize("status", ["failed", "interrupted"])
def test_action_failure_does_not_trigger_retry(key, status):
    ctx = Context(inputs())
    ctx.responder.mutate = lambda k,a,r: ActionResult(status=status,error_message="test") if k.endswith(key) else r
    run_all(ctx)
    assert ctx.status == "failed" and ctx.state.grasp_attempt == 0

@pytest.mark.parametrize("key", ["force-before", "force-after"])
def test_resume_reuses_sample_after_action_result_before_checkpoint(key):
    ctx = Context(inputs())
    original = ctx.execute
    crashed = False
    async def execute(k, a):
        nonlocal crashed
        result = await original(k, a)
        if k.endswith(key) and not crashed:
            crashed = True
            raise RuntimeError("crash")
        return result
    ctx.execute = execute
    with pytest.raises(RuntimeError, match="crash"):
        run_all(ctx)
    ctx.execute = original
    run_all(ctx)
    assert ctx.status == "completed"
    assert sum(k.endswith(key) for k, _ in ctx.responder.actions) == 1


def test_resume_after_retry_checkpoint_does_not_replay_previous_attempt():
    ctx = Context(inputs())
    force_values(ctx, [(1,0,0),(1,0,0)], [(1,0,0),(2,0,0)])
    original = ctx.checkpoint
    def checkpoint(state):
        original(state)
        if state.grasp_attempt == 1:
            raise RuntimeError("retry checkpoint crash")
    ctx.checkpoint = checkpoint
    with pytest.raises(RuntimeError, match="retry checkpoint crash"):
        run_all(ctx)
    ctx.checkpoint = original
    run_all(ctx)
    assert ctx.status == "completed"
    keys = [k for k,_ in ctx.responder.actions]
    assert sum("attempt:0:approach:grasp-pull" in k for k in keys) == 1
    assert sum("attempt:1:prepare:match" in k for k in keys) == 1


def test_multiplier_is_configurable_and_zero_baseline_has_no_division():
    for before, after, ratio in [((1,0,0),(1.2,0,0),1.1), ((0,0,0),(0.1,0,0),1.5)]:
        ctx = Context(inputs(pull_force_ratio_threshold=ratio))
        force_values(ctx,[before],[after])
        run_all(ctx)
        assert ctx.status == "completed"

@pytest.mark.parametrize("patch", [
    {"pull_force_ratio_threshold":0}, {"pull_force_ratio_threshold":float("inf")},
    {"max_grasp_retries":-1}, {"max_grasp_retries":1.5},
])
def test_invalid_parameters(patch):
    with pytest.raises(ValidationError):
        inputs(**patch)


def test_resume_after_failed_check_before_retry_transition():
    ctx = Context(inputs())
    force_values(ctx, [(1,0,0),(1,0,0)], [(1,0,0),(2,0,0)])
    original = ctx.checkpoint
    def checkpoint(state):
        original(state)
        if state.grasp_attempt == 0 and state.pull_force_checks:
            raise RuntimeError("saved failed check")
    ctx.checkpoint = checkpoint
    with pytest.raises(RuntimeError, match="saved failed check"):
        run_all(ctx)
    ctx.checkpoint = original
    run_all(ctx)
    assert ctx.status == "completed"
    keys = [k for k,_ in ctx.responder.actions]
    assert len(keys) == len(set(keys))
    assert [c.attempt for c in ctx.state.pull_force_checks] == [0,1]


def test_configured_two_retries_are_bounded():
    ctx = Context(inputs(max_grasp_retries=2))
    force_values(ctx, [(1,0,0)]*3, [(1,0,0)]*3)
    run_all(ctx)
    assert ctx.status == "failed"
    assert ctx.state.failure_code == "GRASP_RETENTION_FAILED"
    assert len(ctx.state.pull_force_checks) == 3
    assert sum(k.endswith("prepare:match") for k,_ in ctx.responder.actions) == 3
    assert not any("approach:other" in k for k,_ in ctx.responder.actions)
