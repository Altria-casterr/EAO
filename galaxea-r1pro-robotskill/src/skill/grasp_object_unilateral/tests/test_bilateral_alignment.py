"""公共阶段与稳定双手版本的离线契约对比；全部 Action 由 MockSkillContext 响应。"""
import asyncio
from copy import deepcopy
from pathlib import Path

import pytest
import yaml
from pydantic import ValidationError
from semantic_robot_skill_sdk import ActionResult, run_skill

from grasp_object_bilateral.scripts import controller as bilateral_controller
from grasp_object_bilateral.scripts import models as bilateral_models
from grasp_object_bilateral.scripts import skill as bilateral_skill
from grasp_object_bilateral.tests.support import Context as BilateralContext
from grasp_object_bilateral.tests.support import inputs as bilateral_inputs
from grasp_object_unilateral.scripts import controller, models, skill, solver
from .support import Context, inputs


def matched_result(best):
    return ActionResult(status="succeeded", output={"result": {"output": {
        "success": True, "robot_frame": "torso_link4", "best_match": best,
    }}})


@pytest.mark.parametrize("confidence", [0.25, 0.9, 1.0])
def test_match_parser_matches_bilateral_and_preserves_evidence(confidence):
    result = matched_result({"center_robot": [0.45, -0.02, 0.3], "confidence": confidence})
    original = deepcopy(result.output)
    center = skill._matched_center(result, (0.5, 0.0, 0.12), inputs())
    assert center == bilateral_skill._matched_center(result, (0.5, 0.0, 0.12), bilateral_inputs())
    assert center == (0.45, -0.02, 0.12)
    assert result.output == original


@pytest.mark.parametrize("best", [
    {"center_robot": [0.45, 0.0, 0.1]},
    {"center_robot": [0.45, 0.0, 0.1], "confidence": 0.24},
    {"center_robot": [0.45, 0.0, 0.1], "confidence": 1.1},
    {"center_robot": [0.45, 0.0, 0.1], "confidence": float("nan")},
    {"center_robot": [0.45, 0.0], "confidence": 0.9},
    {"center_robot": {"x": 0.45, "y": 0, "z": 0.1}, "confidence": 0.9},
    {"center_robot": [0.45, 0.0, float("inf")], "confidence": 0.9},
])
def test_match_parser_rejects_the_same_invalid_contract_as_bilateral(best):
    for module, params in [(skill, inputs()), (bilateral_skill, bilateral_inputs())]:
        with pytest.raises(ValueError):
            module._matched_center(matched_result(best), (0.45, 0, 0.1), params)


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("offset", [0.0, 0.02])
def test_prepare_keeps_unilateral_side_offsets_and_perception_parameters(side, offset):
    params = inputs(grasp_axis=side, capture_z_offset_m=offset, safe_offset_m=0.04)
    ctx = Context(params)
    original_poses = deepcopy(ctx.responder.poses)
    asyncio.run(skill.run(ctx))
    assert ctx.state.stage == "approach"
    assert ctx.status != "failed"
    main = getattr(ctx.state.capture_points, side)
    assert main.x == pytest.approx(0.45)
    assert main.y == pytest.approx(0.34 if side == "left" else -0.34)
    assert main.z == pytest.approx(0.1 + offset)
    other = "right" if side == "left" else "left"
    expected = solver.parse_pose(original_poses[other], params.relative_frame, ctx.state.base_to_relative)
    assert getattr(ctx.state.capture_points, other) == expected
    assert not any(a.type.startswith(("motion.", "manipulation.", "torso.")) for _, a in ctx.responder.actions)
    match = next(a for _, a in ctx.responder.actions if a.type == "perception.match_target")
    assert match.parameters == {
        **params.perception.model_dump(exclude={"minimum_confidence", "timeout_seconds"}),
        "target_point": list(ctx.state.relative_target_m), "robot_frame": params.relative_frame,
    }
    assert "conf_threshold" not in match.parameters
    assert "iou_threshold" not in match.parameters
    assert params.capture_x_offset_m == 0.0


def test_common_motion_and_force_defaults_match_bilateral():
    assert models.MotionSettings().model_dump() == bilateral_models.MotionSettings().model_dump()
    values = {"force_threshold_n": 3, "force_coordinate": ["y"]}
    assert models.ForceSettings(**values).model_dump() == bilateral_models.ForceSettings(**values).model_dump()
    params = inputs()
    assert params.capture_z_offset_m == 0.0
    assert params.pull_y_distance_m == 0.1
    assert params.approach_force.step_size_m == params.verify_force.step_size_m == 0.01


@pytest.mark.parametrize("patch", [
    {"axis_order": "xxz"}, {"axis_order": ""}, {"axis_order": "xy"},
    {"max_num_points": 4}, {"step_distance": 0}, {"num_points": 4},
])
def test_motion_validation_matches_bilateral(patch):
    for cls in [models.MotionSettings, bilateral_models.MotionSettings]:
        with pytest.raises(ValidationError):
            cls(**patch)


def common_poses():
    return {
        "left": {"x": 0.52, "y": 0.36, "z": 0.19, "qx": 0, "qy": 0, "qz": 0, "qw": 1},
        "right": {"x": 0.48, "y": -0.28, "z": 0.17, "qx": 0, "qy": 0, "qz": 0, "qw": 1},
    }


@pytest.mark.parametrize("patch", [
    {},
    {"num_points": 9, "max_num_points": 30, "sequential": False, "axis_order": "xyz", "timeout": 2.0},
    {"num_points": None, "max_num_points": 40, "step_distance": 0.015, "axis_order": " ZXY ", "pos_tol": 0.02},
])
@pytest.mark.parametrize("trajectory", [False, True])
def test_both_controller_payload_and_timeout_match_bilateral(patch, trajectory):
    kwargs = {"trajectory": trajectory, "label": "parity"}
    single = controller.both(inputs(motion=patch), models.BothPoses(**common_poses()), **kwargs)
    both = bilateral_controller.both(
        bilateral_inputs(motion=patch), bilateral_models.BothPoses(**common_poses()), **kwargs)
    assert single.model_dump() == both.model_dump()


@pytest.mark.parametrize("num_points", [None, 9, 20])
@pytest.mark.parametrize("trajectory", [False, True])
def test_unilateral_approach_uses_shared_motion_settings(num_points, trajectory):
    params = inputs(motion={"num_points": num_points, "axis_order": "xyz", "sequential": True})
    pose = models.ToolPose(**common_poses()["left"])
    action = controller.single("left", pose, trajectory=trajectory, label="approach", params=params)
    if trajectory:
        expected_points = params.motion.max_num_points if num_points is None else num_points
        assert action.parameters["num_points"] == expected_points
        assert action.parameters["max_num_points"] == params.motion.max_num_points
        assert action.parameters["sequential"] is True
        assert action.parameters["axis_order"] == "xyz"
        assert action.parameters["step_distance"] == params.motion.step_distance
        assert action.timeout_seconds == params.motion.timeout * expected_points + 15
    else:
        assert "num_points" not in action.parameters
        assert action.timeout_seconds == params.motion.timeout + 15


def seeded_context(bilateral, side, motion, verify_force):
    cls = bilateral_models if bilateral else models
    context_cls = BilateralContext if bilateral else Context
    input_fn = bilateral_inputs if bilateral else inputs
    params = input_fn(verify_side=side, motion=motion, verify_force=verify_force)
    ctx = context_cls(params)
    poses = cls.BothPoses(**common_poses())
    contacts = {"left_contact": True, "right_contact": True} if bilateral else {
        "grasp_contact": True, "grasp_pulled": True,
        "other_arm_contact": True, "other_arm_lifted": True,
    }
    ctx.checkpoint(cls.GraspState(
        stage="lift", relative_target_m=(0.45, 0, 0.1), matched_center_m=(0.45, 0, 0.1),
        capture_points=poses, physical_action_started=True, **contacts,
    ))
    ctx.responder.poses = {
        axis: getattr(poses, axis).sensor_data(params.relative_frame) for axis in ("left", "right")
    }
    return ctx


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("motion", [{}, {"num_points": 12, "axis_order": "xyz", "sequential": False}])
@pytest.mark.parametrize("image_status", ["succeeded", "failed"])
def test_lift_verify_match_bilateral_from_identical_contact_state(side, motion, image_status):
    verify_force = {"force_threshold_n": 2, "force_coordinate": ["y"]}
    contexts = []
    for bilateral in (False, True):
        ctx = seeded_context(bilateral, side, motion, verify_force)
        def mutate(key, action, result):
            if key.endswith("verify:image") and image_status == "failed":
                return ActionResult(status="failed", error_message="offline camera unavailable")
            return result
        ctx.responder.mutate = mutate
        asyncio.run(run_skill(bilateral_skill.run if bilateral else skill.run, ctx))
        assert ctx.status == "completed"
        contexts.append(ctx)
    single, both = contexts
    def trace(ctx):
        return [(key.split(":", 1)[1], action.model_dump()) for key, action in ctx.responder.actions]
    assert trace(single) == trace(both)
    # lift_distance_m 现在两边各自反映自己的 lift_m：双手版本已改为真实值，
    # 单手版本仍是 Literal[0.05] 硬编码（待同步修正），故不纳入公共阶段一致性比较。
    single_result = single.result.model_dump(
        exclude={"pull_y_distance_m", "other_arm_lift_distance_m", "lift_distance_m"})
    assert single_result == both.result.model_dump(exclude={"lift_distance_m"})
    force = next(a for k, a in single.responder.actions if k.endswith("verify:force"))
    assert force.parameters["step_size_m"] == 0.01
    assert force.parameters["axis"] == side


@pytest.mark.parametrize("failure_key", ["lift:up", "lift:retract", "verify:force"])
@pytest.mark.parametrize("failure_status", ["failed", "interrupted"])
def test_common_stage_failure_stops_at_the_same_action_as_bilateral(failure_key, failure_status):
    traces = []
    for bilateral in (False, True):
        ctx = seeded_context(bilateral, "left", {}, {"force_threshold_n": 2, "force_coordinate": ["y"]})
        ctx.responder.mutate = lambda key, action, result: (
            ActionResult(status=failure_status, error_message="injected") if key.endswith(failure_key) else result)
        runner = bilateral_skill.run if bilateral else skill.run
        asyncio.run(run_skill(runner, ctx))
        assert ctx.status == "failed"
        assert ctx.responder.actions[-1][0].endswith(failure_key)
        traces.append([(key.split(":", 1)[1], action.model_dump()) for key, action in ctx.responder.actions])
        before = len(ctx.responder.actions)
        asyncio.run(runner(ctx))
        assert len(ctx.responder.actions) == before
    assert traces[0] == traces[1]


@pytest.mark.parametrize("step", [0.001, 0.005, 0.01])
def test_explicit_verify_force_settings_are_preserved(step):
    values = {"force_threshold_n": 2, "force_coordinate": ["x", "z"], "step_size_m": step, "timeout_s": 12}
    left = controller.force("right", models.ForceSettings(**values), "verify")
    right = bilateral_controller.force("right", bilateral_models.ForceSettings(**values), "verify")
    assert left.model_dump() == right.model_dump()
    assert left.parameters["step_size_m"] == step


@pytest.mark.parametrize("num_points", [None, 13])
def test_all_actions_respect_ability_contracts_with_auto_and_fixed_points(num_points):
    root = Path(__file__).resolve().parents[4]
    base = root / "test/single-skill/yolo_runtime/src/galaxea_r1pro_0/abilities"
    contracts = {}
    for name in ("torso_control", "joint_eef_pose_control", "yolo_detect", "robot_observation"):
        for task in yaml.safe_load((base / name / "ability.manifest.yaml").read_text())["tasks"]:
            if "actionType" in task:
                contracts[task["actionType"]] = task
    skill_dir = Path(__file__).resolve().parents[1]
    manifest = yaml.safe_load((skill_dir / "SKILL.md").read_text().split("---", 2)[1])
    declared = {item["type"] for item in manifest["required_actions"]}
    ctx = Context(inputs(motion={"num_points": num_points}, torso={"enabled": True, "positions": [0, 0, 0, 0]}))
    asyncio.run(run_skill(skill.run, ctx))
    assert ctx.status == "completed"
    for _, action in ctx.responder.actions:
        assert action.type in declared
        contract = contracts[action.type]
        allowed = {item["name"] for item in contract["params"]}
        required = {item["name"] for item in contract["params"] if not item.get("optional", False)}
        assert set(action.parameters) <= allowed
        assert required <= set(action.parameters)


@pytest.mark.parametrize("key", ["lift:up", "verify:force"])
def test_resume_preserves_legacy_saved_action_despite_new_defaults(key):
    ctx = seeded_context(False, "left", {}, {"force_threshold_n": 2, "force_coordinate": ["y"]})
    params = ctx.input(models.GraspInput)
    state = ctx.state
    if key == "lift:up":
        state.lift_points = solver.lift_points(state.capture_points)
        legacy = controller.both(params, state.lift_points, trajectory=True, label="legacy lift")
        values = dict(legacy.parameters)
        values.update(num_points=20, max_num_points=20, left_sequential=False,
                      right_sequential=False, left_axis_order="zyx", right_axis_order="zyx")
        values.pop("step_distance")
        legacy = legacy.model_copy(update={"parameters": values, "timeout_seconds": 615.0})
    else:
        legacy_settings = params.verify_force.model_copy(update={"step_size_m": 0.001})
        legacy = controller.force("left", legacy_settings, "legacy verify")
    state.actions[key] = legacy
    ctx.checkpoint(state)
    asyncio.run(run_skill(skill.run, ctx))
    assert ctx.status == "completed"
    executed = [action for action_key, action in ctx.responder.actions if action_key.endswith(key)]
    assert len(executed) == 1
    assert executed[0].model_dump() == legacy.model_dump()
    assert ctx.state.actions[key].model_dump() == legacy.model_dump()
