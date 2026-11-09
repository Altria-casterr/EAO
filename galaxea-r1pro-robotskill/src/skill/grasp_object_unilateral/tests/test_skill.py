import asyncio
from copy import deepcopy
from pathlib import Path

import pytest
import yaml
from pydantic import ValidationError
from semantic_robot_skill_sdk import ActionResult, Observation, SkillCancelled, StopRequest, run_skill
from grasp_object_unilateral.scripts import controller, solver
from grasp_object_unilateral.scripts.models import BothPoses, GraspInput, GraspState
from grasp_object_unilateral.scripts.skill import on_stop, run
from .support import Context, inputs, pose

ROOT = Path(__file__).resolve().parents[4]
SKILL_DIR = Path(__file__).resolve().parents[1]


def run_all(ctx):
    asyncio.run(run_skill(run, ctx))
    return ctx


def test_complete_five_stage_flow_with_ten_step_approach():
    ctx = run_all(Context())
    assert ctx.status == "completed"
    result = ctx.result
    assert result.holding_object is True
    assert result.verification_method == "single_force_align"
    assert result.pull_y_distance_m == 0.1
    assert result.other_arm_lift_distance_m == 0.1
    actions = ctx.responder.actions
    physical = [(key, action) for key, action in actions if action.type.startswith(("motion.", "manipulation.", "torso."))]
    # approach 9 个物理动作、lift 2 个、stand 1 个、verify 1 个。
    expected_types = [
        # approach
        "motion.move_end_effector_single",            # 1. 抓取侧移到 capture_point
        "manipulation.force_align",                  # 2. 抓取侧力控
        "motion.move_end_effector_single",            # 3. 抓取侧上抬
        "motion.move_end_effector_single_trajectory", # 4. 抓取侧拉箱
        "motion.move_end_effector_single",            # 6. 辅助手 Y 预定位
        "motion.move_end_effector_single",            # 7. 辅助手 Z 到位
        "motion.move_end_effector_single",            # 8. 辅助手 X 到 target
        "manipulation.force_align",                   # 9. 辅助手力控
        "motion.move_end_effector_single_trajectory", # 10. 辅助手上抬
        # lift
        "motion.move_end_effector_both_trajectory",   # lift:up
        "motion.move_end_effector_both_trajectory",   # lift:retract
        # stand
        "torso.move_to_posture_all",                 # stand:torso
        # verify
        "manipulation.force_align",                   # verify:force
    ]
    assert [a.type for _, a in physical] == expected_types
    forces = [a for _, a in physical if a.type == "manipulation.force_align"]
    assert [a.parameters["axis"] for a in forces] == ["left", "right", "left"]
    assert [a.parameters["direction"] for a in forces] == ["negative", "positive", "negative"]
    assert [a.parameters["force_threshold_n"] for a in forces] == [3, 3, 2]
    stages = [e["stage"] for e in ctx.events if e.get("type") == "stage.completed"]
    assert stages == ["prepare", "approach", "lift", "stand", "verify"]
    assert sum(a.type == "robot.capture_rgbd" for _, a in actions) == 1


def test_right_axis_mirrors_pull_direction():
    ctx = run_all(Context(inputs(grasp_axis="right")))
    assert ctx.status == "completed"
    actions = ctx.responder.actions
    # 抓取侧力控方向应是 positive（right）
    grasp_force = next(a for k, a in actions if k.endswith("approach:grasp-force"))
    assert grasp_force.parameters["direction"] == "positive"
    # 辅助手应是 left、方向 negative
    other_force = next(a for k, a in actions if k.endswith("approach:other-force"))
    assert other_force.parameters["axis"] == "left"
    assert other_force.parameters["direction"] == "negative"


def test_pull_point_moves_left_positive_y():
    params = GraspInput.model_validate(inputs())
    grasp_current = solver.parse_pose(pose(0.0, 0.1), "base_link")
    pulled = solver.pull_point(params, grasp_current)
    # left grasp → +y
    assert pulled.y == pytest.approx(grasp_current.y + 0.1)
    assert pulled.x == grasp_current.x and pulled.z == grasp_current.z


def test_base_link_pose_is_transformed_to_torso_frame():
    transform = solver.floating_base_transform(
        {"position": [0.1, -0.2, -1.0], "orientation": [0, 0, 0, 1]}, "torso_link4")
    converted = solver.parse_pose(pose(0.4, 1.2), "torso_link4", transform)
    assert (converted.x, converted.y, converted.z) == pytest.approx((0.55, 0.2, 0.2))
    assert (converted.qx, converted.qy, converted.qz, converted.qw) == pytest.approx((0, 0, 0, 1))


def test_capture_point_uses_grasp_axis():
    params = GraspInput.model_validate(inputs())
    current = BothPoses(left=solver.parse_pose(pose(0.25, 1.2), "base_link"),
                        right=solver.parse_pose(pose(-0.25, 1.2), "base_link"))
    planned = solver.capture_point(params, (0.8, -0.04, 0.05), current)
    # left grasp → y 在箱体左侧
    assert planned.y > 0


@pytest.mark.parametrize("key", [
    "approach:grasp-move", "approach:grasp-force", "approach:grasp-lift",
    "approach:grasp-pull", "approach:other-y-move", "approach:other-z-move",
    "approach:other-x-move",
    "approach:other-force", "approach:other-lift", "lift:up", "lift:retract", "stand:torso", "verify:force",
])
@pytest.mark.parametrize("status", ["failed", "interrupted"])
def test_physical_failure_never_advances_or_retries(key, status):
    ctx = Context()
    ctx.responder.mutate = lambda k, a, r: ActionResult(status=status, error_message="injected") if k.endswith(key) else r
    run_all(ctx)
    assert ctx.status == "failed"
    assert ctx.result is None
    assert ctx.responder.actions[-1][0].endswith(key)
    count = len(ctx.responder.actions)
    asyncio.run(run(ctx))
    assert len(ctx.responder.actions) == count


@pytest.mark.parametrize("case", ["no_match", "wrong_match_frame", "bad_confidence", "bad_pose_frame", "missing_quaternion", "empty_observation", "bad_transform", "unreachable"])
def test_prepare_rejects_invalid_data_before_arm_movement(case):
    ctx = Context()
    def mutate(key, action, result):
        output = result.output
        if action.type == "perception.match_target":
            payload = output["result"]["result"]
            if case == "no_match": payload.update(success=False, best_match=None)
            if case == "wrong_match_frame": payload["robot_frame"] = "map"
            if case == "bad_confidence": payload["best_match"]["confidence"] = float("nan")
            if case == "unreachable": payload["best_match"]["center_robot"][1] = 1.0
        if action.type == "robot.get_sensors_state":
            data = output["result"]["observations"][0]["data"]
            if case == "bad_pose_frame": data["header"]["frame_id"] = "other"
            if case == "missing_quaternion": del data["pose"]["orientation"]["w"]
            if case == "empty_observation": output["result"]["observations"] = []
        if action.type == "robot.get_state" and case == "bad_transform":
            output["result"]["observations"][0]["data"]["orientation"] = [0, 0, 0, 0]
        return result
    ctx.responder.mutate = mutate
    run_all(ctx)
    assert ctx.status == "failed"
    assert not any(a.type.startswith(("motion.", "manipulation.")) for _, a in ctx.responder.actions)


@pytest.mark.parametrize("status", ["failed", "succeeded"])
def test_capture_is_log_only_and_force_runs_once(status):
    ctx = Context()
    ctx.responder.mutate = lambda k, a, r: ActionResult(status=status, observations=[Observation(
        kind="sensor.frame", source="mock", value={"media_type": "image/png"},
        artifact_refs=["artifact://rgb-frame"],
    )] if status == "succeeded" else []) if a.type == "robot.capture_rgbd" else r
    run_all(ctx)
    assert ctx.status == "completed"
    assert ctx.result.image_status == ("available" if status == "succeeded" else "failed")
    assert sum(k.endswith("verify:force") for k, _ in ctx.responder.actions) == 1


@pytest.mark.parametrize("crash_key", ["approach:grasp-move", "approach:other-x-move", "lift:up", "lift:retract", "stand:torso", "verify:force"])
def test_crash_after_physical_result_before_checkpoint_reuses_same_action(crash_key):
    ctx = Context()
    original = ctx.execute
    crashed = False
    async def execute(key, action):
        nonlocal crashed
        result = await original(key, action)
        if key.endswith(crash_key) and not crashed:
            crashed = True
            raise RuntimeError("simulated worker crash")
        return result
    ctx.execute = execute
    with pytest.raises(RuntimeError, match="simulated"):
        run_all(ctx)
    before = deepcopy(ctx.state.capture_points)
    ctx.execute = original
    run_all(ctx)
    assert ctx.status == "completed"
    assert ctx.result.capture_points == before
    assert sum(k.endswith(crash_key) for k, _ in ctx.responder.actions) == 1


def test_failed_result_checkpoint_cannot_be_reinterpreted_as_success():
    ctx = Context()
    asyncio.run(run(ctx))
    state = ctx.state
    state.results["approach:grasp-move"] = ActionResult(status="failed")
    ctx.checkpoint(state)
    run_all(ctx)
    assert ctx.status == "failed"
    assert not any(a.type == "manipulation.force_align" for _, a in ctx.responder.actions)


@pytest.mark.parametrize("stage", ["approach", "stand"])
def test_stop_after_physical_action_is_unknown_and_no_new_motion(stage):
    ctx = Context()
    ctx.checkpoint(GraspState(stage=stage, physical_action_started=True, grasp_contact=True))
    ctx.request_stop()
    with pytest.raises(SkillCancelled):
        asyncio.run(run(ctx))
    result = asyncio.run(on_stop(ctx, StopRequest(source="user", reason="stop")))
    assert not result.safe and result.requires_intervention and result.physical_state == "unknown"
    assert not ctx.responder.actions


@pytest.mark.parametrize("patch", [
    {"torso": {"enabled": True}}, {"approach_force": {"force_threshold_n": 0, "force_coordinate": ["y"]}},
    {"verify_force": {"force_threshold_n": 1, "force_coordinate": []}},
    {"motion": {"num_points": 0}}, {"relative_frame": "  "}, {"grasp_axis": "center"},
    {"stand": {"speeds": [0.5, 0.5, 0.5]}},
    {"stand": {"speeds": [0.5, 0.5, 0.5, 0.5, 0.5]}},
    {"stand": {"speeds": [0.5, 0, 0.5, 0.5]}},
    {"stand": {"speeds": [0.5, float("nan"), 0.5, 0.5]}},
    {"stand": {"timeout": 0}}, {"stand": {"tol": 0}},
    {"stand": {"positions": [0, 0, 0, 0]}}, {"stand": {"enabled": False}},
])
def test_invalid_business_inputs(patch):
    with pytest.raises(ValidationError): inputs(**patch)
