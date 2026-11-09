import asyncio
from copy import deepcopy
from pathlib import Path

import pytest
import yaml
from pydantic import ValidationError
from semantic_robot_skill_sdk import ActionResult, Observation, SkillCancelled, StopRequest, run_skill
from grasp_object_bilateral.scripts import controller, solver
from grasp_object_bilateral.scripts.models import BothPoses, GraspInput, GraspState
from grasp_object_bilateral.scripts.skill import on_stop, run
from .support import Context, inputs, pose

ROOT = Path(__file__).resolve().parents[4]
SKILL_DIR = Path(__file__).resolve().parents[1]


def run_all(ctx):
    asyncio.run(run_skill(run, ctx))
    return ctx


def test_complete_exact_five_stage_flow_and_actual_contact_lift():
    ctx = run_all(Context())
    assert ctx.status == "completed"
    result = ctx.result
    assert result.holding_object is True
    assert result.verification_method == "single_force_align"
    assert result.image_status == "metadata_only"
    actions = ctx.responder.actions
    physical = [(key, action) for key, action in actions if action.type.startswith(("motion.", "manipulation.", "torso."))]
    assert [a.type for _, a in physical] == [
        "motion.move_end_effector_both_trajectory", "manipulation.force_align", "manipulation.force_align",
        "motion.move_end_effector_both_trajectory", "motion.move_end_effector_both_trajectory",
        "torso.move_to_posture_all",
        "manipulation.force_align",
    ]
    # 两阶段默认按各自 axis_order 顺序插值，省略点数时由 Ability 自动计算。
    approach = physical[0][1].parameters
    assert approach["left_sequential"] is True and approach["right_sequential"] is True
    assert approach["left_axis_order"] == "zyx" and approach["right_axis_order"] == "zyx"
    assert "num_points" not in approach
    assert approach["step_distance"] == 0.05
    assert approach["max_num_points"] == 100
    assert approach["timeout"] == 60.0
    assert physical[0][1].timeout_seconds == pytest.approx(60.0 * 100 + 15)
    for _, action in physical[3:5]:
        assert action.parameters["left_sequential"] is True
        assert action.parameters["right_sequential"] is True
        assert action.parameters["left_axis_order"] == "yzx"
        assert action.parameters["right_axis_order"] == "yzx"
        assert "num_points" not in action.parameters
    assert physical[0][0] == "grasp-object-bilateral:approach:both"
    forces = [a for _, a in physical if a.type == "manipulation.force_align"]
    assert [a.parameters["axis"] for a in forces] == ["left", "right", "left"]
    assert [a.parameters["direction"] for a in forces] == ["negative", "positive", "negative"]
    assert [a.parameters["force_threshold_n"] for a in forces] == [3, 3, 2]
    match = next(a for _, a in actions if a.type == "perception.match_target")
    assert match.parameters["target_point"] == pytest.approx([0.45, 0, 0.1])
    # 抬升量以输入 lift_m 为准（代码默认 0.1），结果字段同步反映该值，不再写死 0.05。
    lift_m = ctx.input(GraspInput).lift_m
    assert result.lift_distance_m == pytest.approx(lift_m)
    for side in ("left", "right"):
        actual = ctx.responder.contact_poses[side]["pose"]["position"]
        lifted = getattr(result.lift_points, side)
        assert lifted.z == pytest.approx(actual["z"] + lift_m)
        assert lifted.y == pytest.approx(actual["y"])
    # 收箱点位由当前上游 move_box_to_center 决定：x = 当前左末端 x - width/4，
    # z = 当前左末端 z（即抬升后的高度）；center_x/z_offset_m 不参与计算。
    center = result.center_points
    assert center.left.x == pytest.approx(0.45 - 0.4 / 4)
    assert center.left.z == pytest.approx(0.15 + lift_m)
    assert center.left.y == pytest.approx(-center.right.y)
    stages = [e["stage"] for e in ctx.events if e.get("type") == "stage.completed"]
    assert stages == ["prepare", "approach", "lift", "stand", "verify"]
    assert sum(a.type == "robot.capture_rgbd" for _, a in actions) == 1


def test_approach_motion_is_configurable_and_scoped_to_approach():
    """approach_motion 可覆盖顺序/插值/点数/单点超时；lift 不受影响。"""
    ctx = run_all(Context(inputs(approach_motion={
        "sequential": False, "axis_order": "xyz", "num_points": 12, "timeout": 4.0,
    })))
    assert ctx.status == "completed"
    approach_key, approach = next((k, a) for k, a in ctx.responder.actions
                                  if k.endswith("approach:both"))
    physical = [a for _, a in ctx.responder.actions
                if a.type.startswith(("motion.", "manipulation.", "torso."))]
    assert approach.type == "motion.move_end_effector_both_trajectory"
    assert approach.parameters["left_sequential"] is False
    assert approach.parameters["left_axis_order"] == "xyz"
    assert approach.parameters["num_points"] == 12
    assert approach.parameters["timeout"] == 4.0
    # Action 总超时按 approach 自己的点数与单点超时推导
    assert approach.timeout_seconds == pytest.approx(4.0 * 12 + 15)
    # lift 两处仍走独立的 motion：30.0 单点超时、自动点数、yzx 顺序插值
    for action in physical[3:5]:
        assert action.parameters["timeout"] == 30.0
        assert "num_points" not in action.parameters
        assert action.parameters["max_num_points"] == 100
        assert action.parameters["left_sequential"] is True
        assert action.parameters["left_axis_order"] == "yzx"


@pytest.mark.parametrize("offset", [-0.05, 0.0, 0.03])
def test_capture_x_offset_reaches_both_approach_targets(offset):
    baseline = run_all(Context())
    ctx = run_all(Context(inputs(capture_x_offset_m=offset)))
    assert ctx.status == "completed"
    approach = next(a for k, a in ctx.responder.actions if k.endswith("approach:both"))
    assert ctx.result.matched_center_m == baseline.result.matched_center_m
    for side in ("left", "right"):
        target = getattr(ctx.result.capture_points, side)
        original = getattr(baseline.result.capture_points, side)
        assert target.x == pytest.approx(original.x + offset)
        assert approach.parameters[f"{side}_target_pose"]["target_x"] == pytest.approx(target.x)
        for key in ("y", "z", "qx", "qy", "qz", "qw"):
            assert getattr(target, key) == pytest.approx(getattr(original, key))
        # 收箱 X 由抬升后的实际末端 X 推出（x - width/4），所以 capture_x_offset_m
        # 会等量传递到收箱目标，不再像旧文档所说“不影响收箱”。
        assert getattr(ctx.result.center_points, side).x == pytest.approx(
            getattr(baseline.result.center_points, side).x + offset)


@pytest.mark.parametrize("lift_m", [0.05, 0.1, 0.15])
def test_lift_m_drives_lift_targets_and_result(lift_m):
    """抬升高度只由输入 lift_m 决定；现场用 0.15，代码默认 0.1，结果字段必须一致。"""
    ctx = run_all(Context(inputs(lift_m=lift_m)))
    assert ctx.status == "completed"
    assert ctx.result.lift_distance_m == pytest.approx(lift_m)
    for side in ("left", "right"):
        actual = ctx.responder.contact_poses[side]["pose"]["position"]
        assert getattr(ctx.result.lift_points, side).z == pytest.approx(actual["z"] + lift_m)
    _, retract = next((k, a) for k, a in ctx.responder.actions if k.endswith("lift:retract"))
    # 收箱高度取自抬升后的实际高度，因此与 lift_m 耦合。
    assert retract.parameters["left_target_pose"]["target_z"] == pytest.approx(0.15 + lift_m)
    assert retract.parameters["right_target_pose"]["target_z"] == pytest.approx(0.15 + lift_m)


def test_center_offsets_are_currently_ignored_by_solver():
    """center_x/z_offset_m 当前不参与计算（上游已注释掉旧公式）。

    这条测试把“参数无效”这一现状显式固定下来：若上游恢复使用这两个参数，
    这里会失败，提醒同时更新 SKILL.md 的说明。
    """
    baseline = run_all(Context())
    ctx = run_all(Context(inputs(center_x_offset_m=0.9, center_z_offset_m=0.9)))
    assert ctx.status == "completed"
    assert ctx.result.center_points == baseline.result.center_points


def test_approach_axis_order_is_normalised():
    ctx = run_all(Context(inputs(approach_motion={"axis_order": "  ZYX  "})))
    _, approach = next((k, a) for k, a in ctx.responder.actions if k.endswith("approach:both"))
    assert approach.parameters["left_axis_order"] == "zyx"
    assert approach.parameters["right_axis_order"] == "zyx"


@pytest.mark.parametrize("num_points", [None, 5, 137, 1000])
def test_trajectory_point_count_selection_for_both_motion_profiles(num_points):
    settings = {"num_points": num_points, "step_distance": 0.013,
                "max_num_points": 73, "timeout": 2.0}
    ctx = run_all(Context(inputs(approach_motion=settings, motion=settings)))
    assert ctx.status == "completed"
    trajectories = [a for _, a in ctx.responder.actions
                    if a.type == "motion.move_end_effector_both_trajectory"]
    assert len(trajectories) == 3
    for action in trajectories:
        assert action.parameters["step_distance"] == 0.013
        assert action.parameters["max_num_points"] == 73
        if num_points is None:
            assert "num_points" not in action.parameters
            expected_budget = 73
        else:
            assert action.parameters["num_points"] == num_points
            expected_budget = num_points
        assert action.timeout_seconds == pytest.approx(2.0 * expected_budget + 15)


@pytest.mark.parametrize("sequential", [True, False])
def test_lift_motion_axis_order_and_interpolation_are_configurable(sequential):
    ctx = run_all(Context(inputs(motion={"axis_order": "  YZX  ", "sequential": sequential})))
    assert ctx.status == "completed"
    for key, action in ctx.responder.actions:
        if key.endswith(("lift:up", "lift:retract")):
            assert action.parameters["left_sequential"] is sequential
            assert action.parameters["right_sequential"] is sequential
            assert action.parameters["left_axis_order"] == "yzx"
            assert action.parameters["right_axis_order"] == "yzx"
        elif key.endswith("approach:both"):
            assert action.parameters["left_sequential"] is True
            assert action.parameters["left_axis_order"] == "zyx"


def test_torso_opt_in_and_verify_right_once():
    ctx = run_all(Context(inputs(torso={"enabled": True, "positions": [0, 0.1, 0.2, 0]}, verify_side="right")))
    assert ctx.status == "completed"
    assert ctx.responder.actions[0][1].type == "torso.move_to_posture_all"
    assert sum(a.type == "torso.move_to_posture_all" for _, a in ctx.responder.actions) == 2
    assert ctx.responder.actions[-1][1].parameters["axis"] == "right"
    assert ctx.result.verification_side == "right"


@pytest.mark.parametrize("stand", [None, {
    "speeds": [0.1, 0.2, 0.3, 0.4], "tol": 0.01, "timeout": 25.0,
}])
def test_stand_runs_one_zeroing_action_between_lift_and_verify(stand):
    ctx = run_all(Context(inputs(**({"stand": stand} if stand is not None else {}))))
    assert ctx.status == "completed"
    assert ctx.state.stand_completed is True
    keys = [key for key, _ in ctx.responder.actions]
    stand_actions = [(key, a) for key, a in ctx.responder.actions if ":stand:" in key]
    assert len(stand_actions) == 1
    key, action = stand_actions[0]
    assert key == "grasp-object-bilateral:stand:torso"
    assert action.type == "torso.move_to_posture_all"
    settings = stand or {"speeds": [0.5, 0.8, 0.7, 0.7], "tol": 0.02, "timeout": 10.0}
    assert action.parameters == {"positions": [0.0] * 4, **settings}
    assert action.timeout_seconds == settings["timeout"] + 10
    assert keys.index("grasp-object-bilateral:lift:retract") < keys.index(key)
    assert keys.index(key) < keys.index("grasp-object-bilateral:verify:image")
    stages = [e["stage"] for e in ctx.events if e.get("type") == "stage.completed"]
    assert stages == ["prepare", "approach", "lift", "stand", "verify"]


def test_stand_result_checkpoint_is_reused_before_stage_advance():
    ctx = Context()
    for _ in range(3):
        asyncio.run(run(ctx))
    assert ctx.state.stage == "stand"
    state = ctx.state
    state.actions["stand:torso"] = controller.stand(ctx.input(GraspInput))
    state.results["stand:torso"] = ActionResult(status="succeeded", physical_effect="confirmed")
    ctx.checkpoint(state)
    run_all(ctx)
    assert ctx.status == "completed"
    assert ctx.state.stand_completed is True
    assert not any(":stand:" in key for key, _ in ctx.responder.actions)


@pytest.mark.parametrize("state", [
    GraspState(stage="stand"),
    GraspState(stage="stand", lifted=True),
    GraspState(stage="stand", retracted=True),
    GraspState(stage="verify", lifted=True, retracted=True),
])
def test_stand_and_verify_require_completed_previous_stage(state):
    ctx = Context()
    ctx.checkpoint(state)
    run_all(ctx)
    assert ctx.status == "failed"
    assert ctx.state.failure_code == "GRASP_DATA_INVALID"
    assert not ctx.responder.actions


def test_base_link_pose_is_transformed_to_torso_frame():
    transform = solver.floating_base_transform(
        {"position": [0.1, -0.2, -1.0], "orientation": [0, 0, 0, 1]}, "torso_link4")
    converted = solver.parse_pose(pose(0.4, 1.2), "torso_link4", transform)
    assert (converted.x, converted.y, converted.z) == pytest.approx((0.55, 0.2, 0.2))
    assert (converted.qx, converted.qy, converted.qz, converted.qw) == pytest.approx((0, 0, 0, 1))


def test_bilateral_capture_points_use_nearest_deterministic_safe_y():
    params = GraspInput.model_validate(inputs())
    current = BothPoses(left=solver.parse_pose(pose(0.25, 1.2), "base_link"),
                        right=solver.parse_pose(pose(-0.25, 1.2), "base_link"))
    planned = solver.capture_points(params, (0.8, -0.04, 0.05), current)
    assert planned.left.y == pytest.approx(0.33)
    assert planned.right.y == pytest.approx(-0.41)


@pytest.mark.parametrize("key", ["approach:both", "approach:left-force", "approach:right-force", "lift:up", "lift:retract", "stand:torso", "verify:force"])
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
    stored = ctx.state.results["verify:image"]
    assert stored.output is None and not stored.observations


@pytest.mark.parametrize("crash_key", ["approach:both", "approach:left-force", "lift:up", "lift:retract", "stand:torso", "verify:force"])
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
    state.results["approach:both"] = ActionResult(status="failed")
    ctx.checkpoint(state)
    run_all(ctx)
    assert ctx.status == "failed"
    assert not any(a.type == "manipulation.force_align" for _, a in ctx.responder.actions)


@pytest.mark.parametrize("stage", ["approach", "stand"])
def test_stop_after_physical_action_is_unknown_and_no_new_motion(stage):
    ctx = Context()
    ctx.checkpoint(GraspState(stage=stage, physical_action_started=True, left_contact=True))
    ctx.request_stop()
    with pytest.raises(SkillCancelled):
        asyncio.run(run(ctx))
    result = asyncio.run(on_stop(ctx, StopRequest(source="user", reason="stop")))
    assert not result.safe and result.requires_intervention and result.physical_state == "unknown"
    assert not ctx.responder.actions


def test_source_solvers_are_copied_without_algorithm_changes():
    source = ROOT / "src/solver"
    for name in ("capture_point_solver", "target_point_solver"):
        assert (source / name / f"{name}.py").is_file()


def test_every_action_matches_current_manifest_and_skill_declarations():
    ctx = run_all(Context(inputs(torso={"enabled": True, "positions": [0, 0, 0, 0]})))
    base = ROOT / "test/single-skill/yolo_runtime/src/galaxea_r1pro_0/abilities"
    contracts = {}
    for name in ("torso_control", "joint_eef_pose_control", "yolo_detect", "robot_observation"):
        for task in yaml.safe_load((base / name / "ability.manifest.yaml").read_text())["tasks"]:
            if "actionType" in task: contracts[task["actionType"]] = task
    manifest = yaml.safe_load((SKILL_DIR / "SKILL.md").read_text().split("---", 2)[1])
    declared = {x["type"] for x in manifest["required_actions"]}
    assert {a.type for _, a in ctx.responder.actions} == declared
    for _, action in ctx.responder.actions:
        contract = contracts[action.type]
        allowed = {p["name"] for p in contract["params"]}
        required = {p["name"] for p in contract["params"] if not p.get("optional", False)}
        assert set(action.parameters) <= allowed
        assert required <= set(action.parameters)


@pytest.mark.parametrize("patch", [
    {"torso": {"enabled": True}}, {"approach_force": {"force_threshold_n": 0, "force_coordinate": ["y"]}},
    {"verify_force": {"force_threshold_n": 1, "force_coordinate": []}},
    {"motion": {"num_points": 0}}, {"relative_frame": "  "},
    {"lift_m": 0.0}, {"lift_m": -0.1}, {"lift_m": float("inf")},
    {"approach_motion": {"axis_order": "xy"}}, {"approach_motion": {"axis_order": "abc"}},
    {"approach_motion": {"axis_order": "xxy"}}, {"approach_motion": {"num_points": 0}},
    {"motion": {"num_points": 1001}}, {"motion": {"max_num_points": 4}},
    {"motion": {"max_num_points": 1001}}, {"motion": {"step_distance": 0}},
    {"capture_x_offset_m": float("nan")}, {"capture_x_offset_m": float("inf")},
    {"capture_x_offset_m": float("-inf")},
    {"stand": {"speeds": [0.5, 0.5, 0.5]}},
    {"stand": {"speeds": [0.5, 0.5, 0.5, 0.5, 0.5]}},
    {"stand": {"speeds": [0.5, 0, 0.5, 0.5]}},
    {"stand": {"speeds": [0.5, float("nan"), 0.5, 0.5]}},
    {"stand": {"timeout": 0}}, {"stand": {"tol": 0}},
    {"stand": {"positions": [0, 0, 0, 0]}}, {"stand": {"enabled": False}},
])
def test_invalid_business_inputs(patch):
    with pytest.raises(ValidationError): inputs(**patch)
