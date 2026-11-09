"""单手 approach 的离线几何、轨迹与恢复测试；不连接 Robot SDK。"""
import asyncio
from copy import deepcopy
import math
import re

import pytest
from pydantic import ValidationError
from semantic_robot_skill_sdk import ActionResult, run_skill

from grasp_object_unilateral.scripts import controller, models, skill, solver
from .support import Context, inputs


def run_all(ctx):
    asyncio.run(run_skill(skill.run, ctx))
    return ctx


def action_for(ctx, suffix):
    return next(action for key, action in ctx.responder.actions if key.endswith(suffix))


def record_actual_contacts(ctx):
    contacts = {}
    snapshots = {}
    respond = ctx.responder.respond

    def tracked(key, action):
        snapshots[re.sub(r"^attempt:\d+:", "", key.split(":", 1)[1])] = deepcopy(ctx.responder.poses)
        result = respond(key, action)
        if key.endswith(("approach:grasp-force", "approach:other-force")):
            axis = action.parameters["axis"]
            pose = ctx.responder.poses[axis]["pose"]
            # 模拟接触后的真实偏差，覆盖 X/Z/姿态，防止测试仅验证理想目标。
            pose["position"]["x"] += 0.011
            pose["position"]["z"] += 0.008
            pose["orientation"] = {"x": 0.1, "y": 0.15, "z": 0.2, "w": math.sqrt(0.9275)}
            contacts[key.split(":", 1)[1]] = deepcopy(ctx.responder.poses[axis])
        return result

    ctx.responder.respond = tracked
    return contacts, snapshots


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("pull_distance", [0.1, 0.2])
def test_both_contact_lifts_use_actual_feedback_not_precontact_targets(side, pull_distance):
    ctx = Context(inputs(grasp_axis=side, pull_y_distance_m=pull_distance))
    contacts, snapshots = record_actual_contacts(ctx)
    run_all(ctx)
    assert ctx.status == "completed"
    for role, delta in [("grasp", 0.05), ("other", 0.1)]:
        contact = contacts[f"approach:{role}-force"]["pose"]
        target = action_for(ctx, f"approach:{role}-lift").parameters["_target_pose"]
        for axis in "xy":
            assert target[f"target_{axis}"] == pytest.approx(contact["position"][axis])
        assert target["target_z"] == pytest.approx(contact["position"]["z"] + delta)
        for axis in "xyzw":
            assert target[axis] == pytest.approx(contact["orientation"][axis])
    current = snapshots["approach:grasp-pull"][side]["pose"]
    pull = action_for(ctx, "approach:grasp-pull").parameters["_target_pose"]
    sign = 1 if side == "left" else -1
    assert pull["target_y"] == pytest.approx(current["position"]["y"] + sign * pull_distance)
    assert pull["target_x"] == pytest.approx(current["position"]["x"])
    assert pull["target_z"] == pytest.approx(current["position"]["z"])


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("pull_distance, safe_offset", [(0.1, 0.05), (0.2, 0.1)])
def test_auxiliary_approach_moves_y_then_z_then_x(side, pull_distance, safe_offset):
    ctx = Context(inputs(grasp_axis=side, pull_y_distance_m=pull_distance))
    _, snapshots = record_actual_contacts(ctx)
    run_all(ctx)
    assert ctx.status == "completed"
    other = "right" if side == "left" else "left"
    before = solver.parse_pose(snapshots["approach:other-y-move"][other], "torso_link4", ctx.state.base_to_relative)
    y_move = action_for(ctx, "approach:other-y-move")
    z_move = action_for(ctx, "approach:other-z-move")
    x_move = action_for(ctx, "approach:other-x-move")
    assert y_move.type == z_move.type == x_move.type == "motion.move_end_effector_single"
    y_pose = y_move.parameters["_target_pose"]
    z_pose = z_move.parameters["_target_pose"]
    target_pose = x_move.parameters["_target_pose"]
    assert y_pose["target_x"] == pytest.approx(before.x)
    assert y_pose["target_z"] == pytest.approx(before.z)
    assert y_pose["target_y"] == pytest.approx(target_pose["target_y"])
    assert z_pose["target_x"] == pytest.approx(y_pose["target_x"])
    assert z_pose["target_y"] == pytest.approx(y_pose["target_y"])
    assert z_pose["target_z"] == pytest.approx(target_pose["target_z"])
    assert target_pose["target_y"] == pytest.approx(z_pose["target_y"])
    assert target_pose["target_z"] == pytest.approx(z_pose["target_z"])
    for action_axis, pose_axis in zip("xyzw", ("qx", "qy", "qz", "qw")):
        assert y_pose[action_axis] == pytest.approx(getattr(before, pose_axis))
    grasp = solver.parse_pose(snapshots["approach:other-y-move"][side], "torso_link4", ctx.state.base_to_relative)
    assert target_pose["target_x"] == pytest.approx(grasp.x)
    assert target_pose["target_z"] == pytest.approx(grasp.z - 0.05)
    sign = -1 if side == "left" else 1
    assert target_pose["target_y"] == pytest.approx(grasp.y + sign * (0.6 + safe_offset))
    assert y_pose != target_pose


@pytest.mark.parametrize("patch", [
    {},
    {"num_points": 12, "max_num_points": 30, "sequential": False, "axis_order": "xyz", "timeout": 2},
    {"num_points": None, "max_num_points": 37, "sequential": True, "axis_order": " ZYX ", "step_distance": 0.013},
])
def test_three_approach_trajectories_use_motion_only(patch):
    params = inputs(motion=patch)
    ctx = run_all(Context(params))
    assert ctx.status == "completed"
    trajectories = [a for key, a in ctx.responder.actions
                    if ":approach:" in key and a.type == "motion.move_end_effector_single_trajectory"]
    assert len(trajectories) == 2
    # 单臂 Ability 对 num_points 执行 int()，None 时显式使用 motion 的上限。
    points = params.motion.num_points if params.motion.num_points is not None else params.motion.max_num_points
    for action in trajectories:
        values = action.parameters
        assert values["num_points"] == points
        assert values["max_num_points"] == params.motion.max_num_points
        assert values["sequential"] == params.motion.sequential
        assert values["axis_order"] == params.motion.axis_order
        assert values["step_distance"] == params.motion.step_distance
        assert values["timeout"] == params.motion.timeout
        assert action.timeout_seconds == params.motion.timeout * points + 15
    assert "approach_motion" not in models.GraspInput.model_fields
    with pytest.raises(ValidationError):
        inputs(approach_motion={})


@pytest.mark.parametrize("side", ["left", "right"])
def test_other_arm_solver_returns_y_z_x_waypoints(side):
    main = models.ToolPose(x=0.54, y=0.4 if side == "left" else -0.4, z=0.21, qx=0, qy=0, qz=0, qw=1)
    other = models.ToolPose(x=0.19, y=-0.31 if side == "left" else 0.31, z=0.48, qx=0.1, qy=0.2, qz=0.3, qw=math.sqrt(0.86))
    solved = solver.other_arm_position(inputs(grasp_axis=side), main, other)
    assert set(solved) == {"y_move_pose", "x_move_pose", "target_pose", "other_axis"}
    y_move, z_move, target = solved["y_move_pose"], solved["x_move_pose"], solved["target_pose"]
    assert solved["other_axis"] == ("right" if side == "left" else "left")
    assert y_move.model_dump(exclude={"y"}) == other.model_dump(exclude={"y"})
    assert y_move.y == z_move.y == target.y
    assert z_move.x == other.x and z_move.z == pytest.approx(main.z - 0.05)
    assert target.x == main.x and target.z == z_move.z
    assert y_move.model_dump(include={"qx", "qy", "qz", "qw"}) == other.model_dump(include={"qx", "qy", "qz", "qw"})
    assert z_move.model_dump(include={"qx", "qy", "qz", "qw"}) == other.model_dump(include={"qx", "qy", "qz", "qw"})
    assert target.model_dump(include={"qx", "qy", "qz", "qw"}) == other.model_dump(include={"qx", "qy", "qz", "qw"})


@pytest.mark.parametrize("role", ["grasp", "other"])
@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("failure", ["failed", "interrupted", "malformed"])
def test_post_contact_pose_failure_prevents_lift_and_all_following_actions(role, side, failure):
    ctx = Context(inputs(grasp_axis=side))
    read_side = side if role == "grasp" else ("right" if side == "left" else "left")
    key = f"approach:{role}-contact:pose:{read_side}"
    def mutate(action_key, action, result):
        if action_key.endswith(key):
            if failure == "malformed":
                return ActionResult(status="succeeded", output={"observations": []})
            return ActionResult(status=failure, error_message="injected read failure")
        return result
    ctx.responder.mutate = mutate
    run_all(ctx)
    assert ctx.status == "failed"
    assert ctx.responder.actions[-1][0].endswith(key)
    assert not any(k.endswith(f"approach:{role}-lift") for k, _ in ctx.responder.actions)
    assert not any(k.endswith("lift:up") for k, _ in ctx.responder.actions)


@pytest.mark.parametrize("key", [
    "approach:grasp-contact:pose:left", "approach:grasp-lift", "approach:grasp-pull",
    "approach:other-y-move", "approach:other-z-move", "approach:other-x-move", "approach:other-contact:pose:right",
    "approach:other-lift",
])
def test_resume_after_action_before_result_checkpoint_reuses_saved_targets(key):
    ctx = Context()
    original = ctx.execute
    crashed = False
    async def interrupted(action_key, action):
        nonlocal crashed
        result = await original(action_key, action)
        if action_key.endswith(key) and not crashed:
            crashed = True
            raise RuntimeError("simulated worker crash")
        return result
    ctx.execute = interrupted
    with pytest.raises(RuntimeError, match="simulated worker crash"):
        run_all(ctx)
    before = deepcopy(ctx.state.actions)
    planned = {name: deepcopy(getattr(ctx.state, name)) for name in (
        "grasp_lift_point", "grasp_pull_point", "other_arm_y_move_pose", "other_arm_x_move_pose", "other_arm_target_pose", "other_arm_lift_point")}
    ctx.execute = original
    run_all(ctx)
    assert ctx.status == "completed"
    assert sum(action_key.endswith(key) for action_key, _ in ctx.responder.actions) == 1
    for action_key, action in before.items():
        assert ctx.state.actions[action_key] == action
    for name, point in planned.items():
        if point is not None:
            assert getattr(ctx.state, name) == point


def test_approach_completes_only_after_all_required_actions_and_flags():
    ctx = Context()
    asyncio.run(skill.run(ctx))
    assert ctx.state.stage == "approach"
    asyncio.run(skill.run(ctx))
    assert ctx.state.stage == "lift"
    state = ctx.state
    assert state.approach_revision == 4
    assert all((state.grasp_contact, state.grasp_lifted, state.grasp_pulled,
                state.other_arm_contact, state.other_arm_lifted))
    for key in ("grasp-move", "grasp-force", "grasp-lift", "grasp-pull",
                "other-y-move", "other-z-move", "other-x-move", "other-force", "other-lift"):
        assert state.results[skill._attempt_key(state, "approach:" + key)].status == "succeeded"
    for name in ("grasp_lift_point", "grasp_pull_point", "other_arm_y_move_pose", "other_arm_x_move_pose", "other_arm_target_pose", "other_arm_lift_point"):
        assert getattr(state, name) is not None


@pytest.mark.parametrize("legacy", ["action", "result", "contact_flag"])
def test_legacy_in_progress_approach_checkpoint_stops_without_reissuing_motion(legacy):
    ctx = Context()
    asyncio.run(skill.run(ctx))
    state = ctx.state
    assert state.approach_revision is None
    if legacy == "action":
        state.actions["approach:grasp-move"] = controller.single(
            "left", state.capture_points.left, trajectory=False, label="legacy", params=inputs())
    elif legacy == "result":
        state.results["approach:grasp-force"] = ActionResult(status="succeeded")
    else:
        state.grasp_contact = True
    state.physical_action_started = True
    ctx.checkpoint(state)
    before = len(ctx.responder.actions)
    asyncio.run(skill.run(ctx))
    assert ctx.status == "failed"
    assert ctx.state.failure_code == "APPROACH_CHECKPOINT_INCOMPATIBLE"
    assert len(ctx.responder.actions) == before


@pytest.mark.parametrize("side", ["left", "right"])
def test_rotated_base_feedback_preserves_transformed_contact_and_auxiliary_pose(side):
    ctx = Context(inputs(grasp_axis=side))
    other = "right" if side == "left" else "left"
    c = math.sqrt(0.5)
    qw = math.sqrt(0.86)
    feedback = {"header": {"frame_id": "base_link"}, "pose": {
        "position": {"x": 0.12, "y": -0.08, "z": 0.2},
        "orientation": {"x": 0.1, "y": 0.2, "z": 0.3, "w": qw},
    }}
    def mutate(key, action, result):
        if key.endswith("prepare:floating"):
            result.output["result"]["observations"][0]["data"]["orientation"] = [0, 0, c, c]
        if key.endswith((f"approach:grasp-contact:pose:{side}",
                         f"approach:other-contact:pose:{other}", f"approach:other:pose:{other}")):
            result.output["result"]["observations"][0]["data"] = deepcopy(feedback)
        return result
    ctx.responder.mutate = mutate
    run_all(ctx)
    assert ctx.status == "completed"
    # 绕 Z 90 度再平移 (0,0,0.8)；四元数按 q_tf * q_feedback 变换。
    expected_q = {"x": -0.1 * c, "y": 0.3 * c, "z": (qw + 0.3) * c, "w": (qw - 0.3) * c}
    for role, lift in (("grasp", 0.05), ("other", 0.1)):
        target = action_for(ctx, f"approach:{role}-lift").parameters["_target_pose"]
        assert target["target_x"] == pytest.approx(0.08)
        assert target["target_y"] == pytest.approx(0.12)
        assert target["target_z"] == pytest.approx(1.0 + lift)
        for axis, value in expected_q.items():
            assert target[axis] == pytest.approx(value)
    y_move = action_for(ctx, "approach:other-y-move").parameters["_target_pose"]
    assert y_move["target_x"] == pytest.approx(0.08)
    assert y_move["target_z"] == pytest.approx(1.0)
    for axis, value in expected_q.items():
        assert y_move[axis] == pytest.approx(value)


def test_old_prepare_checkpoint_without_approach_history_can_start_new_strategy():
    ctx = Context(inputs(torso={"enabled": True, "positions": [0, 0, 0, 0]}))
    asyncio.run(skill.run(ctx))
    assert ctx.state.physical_action_started  # torso alone is not a legacy approach
    assert ctx.state.approach_revision is None
    run_all(ctx)
    assert ctx.status == "completed"
