"""stand 的动作顺序、参数、恢复和公共阶段契约。"""
import asyncio

import pytest
from semantic_robot_skill_sdk import ActionResult, run_skill

from grasp_object_bilateral.scripts import controller as bilateral_controller
from grasp_object_bilateral.tests.support import inputs as bilateral_inputs
from grasp_object_unilateral.scripts import controller, skill
from grasp_object_unilateral.scripts.models import GraspState
from .support import Context, inputs


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("stand", [None, {
    "speeds": [0.1, 0.2, 0.3, 0.4], "tol": 0.01, "timeout": 25.0,
}])
def test_stand_runs_one_zeroing_action_between_lift_and_verify(side, stand):
    params = inputs(grasp_axis=side, **({"stand": stand} if stand is not None else {}))
    ctx = Context(params)
    asyncio.run(run_skill(skill.run, ctx))
    assert ctx.status == "completed"
    assert ctx.state.stand_completed is True
    actions = ctx.responder.actions
    stand_actions = [(key, action) for key, action in actions if ":stand:" in key]
    assert len(stand_actions) == 1
    key, action = stand_actions[0]
    assert key == "grasp-object-unilateral:stand:torso"
    assert action.type == "torso.move_to_posture_all"
    settings = stand or {"speeds": [0.5] * 4, "tol": 0.02, "timeout": 10.0}
    assert action.parameters == {"positions": [0.0] * 4, **settings}
    assert action.timeout_seconds == settings["timeout"] + 10
    assert action.model_dump() == bilateral_controller.stand(
        bilateral_inputs(stand=settings)).model_dump()
    assert sum(a.type.startswith("torso.") for _, a in actions) == 1
    keys = [key for key, _ in actions]
    assert keys.index("grasp-object-unilateral:lift:retract") < keys.index(key)
    assert keys.index(key) < keys.index("grasp-object-unilateral:verify:image")
    stages = [event["stage"] for event in ctx.events if event.get("type") == "stage.completed"]
    assert stages == ["prepare", "approach", "lift", "stand", "verify"]


def test_prepare_torso_and_stand_have_separate_targets():
    params = inputs(torso={"enabled": True, "positions": [0, 0.1, 0.2, 0]})
    ctx = Context(params)
    asyncio.run(run_skill(skill.run, ctx))
    assert ctx.status == "completed"
    torso_actions = [(key, action) for key, action in ctx.responder.actions if action.type.startswith("torso.")]
    assert [(key.rsplit(":", 2)[1], a.parameters["positions"]) for key, a in torso_actions] == [
        ("prepare", [0, 0.1, 0.2, 0]), ("stand", [0.0] * 4),
    ]


def test_stand_result_checkpoint_is_reused_before_stage_advance():
    ctx = Context()
    for _ in range(3):
        asyncio.run(skill.run(ctx))
    assert ctx.state.stage == "stand"
    state = ctx.state
    state.actions["stand:torso"] = controller.stand(inputs())
    state.results["stand:torso"] = ActionResult(status="succeeded", physical_effect="confirmed")
    ctx.checkpoint(state)
    asyncio.run(run_skill(skill.run, ctx))
    assert ctx.status == "completed"
    assert ctx.state.stand_completed is True
    assert not any(":stand:" in key for key, _ in ctx.responder.actions)


def test_stand_resume_preserves_saved_action_settings():
    ctx = Context()
    for _ in range(3):
        asyncio.run(skill.run(ctx))
    state = ctx.state
    saved_action = controller.stand(inputs(stand={"speeds": [0.1] * 4, "timeout": 25}))
    state.actions["stand:torso"] = saved_action
    ctx.checkpoint(state)
    asyncio.run(run_skill(skill.run, ctx))
    executed = [action for key, action in ctx.responder.actions if ":stand:" in key]
    assert ctx.status == "completed"
    assert len(executed) == 1
    assert executed[0] == saved_action


@pytest.mark.parametrize("state", [
    GraspState(stage="stand"),
    GraspState(stage="stand", lifted=True),
    GraspState(stage="stand", retracted=True),
    GraspState(stage="verify", lifted=True, retracted=True),
])
def test_stand_and_verify_require_completed_previous_stage(state):
    ctx = Context()
    ctx.checkpoint(state)
    asyncio.run(run_skill(skill.run, ctx))
    assert ctx.status == "failed"
    assert ctx.state.failure_code == "GRASP_DATA_INVALID"
    assert not ctx.responder.actions
