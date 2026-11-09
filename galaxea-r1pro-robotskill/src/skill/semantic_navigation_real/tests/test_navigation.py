"""离线验证真实 Action 参数、阶段分支、物理动作恢复和携物完成条件。"""

from __future__ import annotations

import asyncio
from copy import deepcopy
import math
from pathlib import Path

import pytest
import yaml
from pydantic import ValidationError
from semantic_robot_skill_sdk import ActionResult, MockSkillContext, SkillCancelled, StopRequest, run_skill

from semantic_navigation_real.scripts.controller import (
    build_navigation_action,
    build_verify_action,
)
from semantic_navigation_real.scripts.models import SemanticNavigationInput, SemanticNavigationState
from semantic_navigation_real.scripts.skill import on_stop, run
from semantic_navigation_real.scripts.solver import resolve_navigation_target, resolve_target_position


ROOT = Path(__file__).resolve().parents[4]


def skill_input(carrying: bool = False, mode: str | None = None) -> SemanticNavigationInput:
    selected_mode = mode or ("left" if carrying else "both")
    data = {
        "navigation_purpose": "carry_to_place" if carrying else "approach_grasp",
        "mode": selected_mode,
        "target": {"target_ref": "slot-1" if carrying else "box-1",
                   "position_m": [1.2, 0.8, 0.9], "yaw_rad": 0.0},
        "frame_id": "map", "timeout": 75, "target_point_type": 2,
    }
    if selected_mode != "both":
        data["move_y_m"] = 0.10 if carrying else 0.15
    if carrying:
        data["verify"] = {"carrying_min_load_n": 4.0}
    return SemanticNavigationInput.model_validate(data)


def verify_output(side: str = "left") -> dict:
    # 与指定 Ability VerifyArrivalTask.execute 返回一致，含未进入结果的 raw。
    return {
        "status": "completed", "verdict": "achieved", "holding_object": True,
        "carrying_check": {
            "holding": True, "checked": True, "force_axis": "z", "min_load_n": 4.0,
            "tools": {side: {"force_n": 7.0, "loaded": True}},
            "reasons": [], "errors": [], "raw": {"arm_wrench": {}},
        },
    }


def actions(ctx: MockSkillContext) -> list[dict]:
    return [event for event in ctx.events if event["type"] == "action_started"]


def queue_navigation(ctx: MockSkillContext, status: str = "succeeded") -> None:
    ctx.queue_action("navigation.follow_route", ActionResult(
        status=status, physical_effect="confirmed", evidence_refs=["evidence://navigation"],
    ))


def queue_verify(ctx: MockSkillContext, output: dict | None = None, side: str = "left") -> None:
    ctx.queue_action("navigation.verify_arrival", ActionResult(
        status="succeeded", output=verify_output(side) if output is None else output,
        physical_effect="none", evidence_refs=["evidence://load"],
    ))


def test_action_parameters_match_current_ability_manifest_exactly() -> None:
    ability_root = ROOT / "test/single-skill/yolo_runtime/src/galaxea_r1pro_0/abilities"
    manifests = [
        yaml.safe_load((ability_root / name / "ability.manifest.yaml").read_text())
        for name in ("woosh_navigation",)
    ]
    tasks = {task["taskName"]: task for manifest in manifests for task in manifest["tasks"]}
    inp = skill_input(True)
    navigation = build_navigation_action(resolve_navigation_target(inp))
    verify = build_verify_action(inp.mode, inp.verify)
    for name, action in (("navigate_to_map_pose", navigation), ("verify_arrival", verify)):
        task = tasks[name]
        assert action.type == task["actionType"]
        assert action.schema_version == task["schemaVersion"]
        assert set(action.parameters) == {p["name"] for p in task["params"]} - {"invocation_id"}
    assert navigation.parameters["target_x"] == pytest.approx(1.2 - 0.9126 * 0.65)
    assert navigation.parameters["target_y"] == pytest.approx(0.9)
    assert navigation.parameters["target_orientation"] == pytest.approx([0, 0, 0, 1])
    assert navigation.parameters["timeout"] == 75
    assert navigation.timeout_seconds == 85
    assert verify.parameters == {"tool_refs": ["left"], "carrying_force_axis": "z", "carrying_min_load_n": 4.0}


def test_both_hand_approach_skips_move_and_force_action() -> None:
    ctx = MockSkillContext(skill_input())
    queue_navigation(ctx)
    asyncio.run(run_skill(run, ctx))
    assert ctx.status == "completed"
    assert [event["action"] for event in actions(ctx)] == ["navigation.follow_route"]
    assert ctx.result.holding_object is None
    assert ctx.result.load_verification == "not_required"
    assert ctx.result.target_ref == "box-1"
    assert [e["stage"] for e in ctx.events if e["type"] == "stage.completed"] == ["navigation"]


@pytest.mark.parametrize(("mode", "target_y"), [("left", 0.95), ("right", 0.65)])
def test_single_hand_approach_navigates_to_offset_without_relative_move(mode: str, target_y: float) -> None:
    ctx = MockSkillContext(skill_input(mode=mode))
    queue_navigation(ctx)
    asyncio.run(run_skill(run, ctx))
    assert ctx.status == "completed"
    assert [event["action"] for event in actions(ctx)] == ["navigation.follow_route"]
    assert [e["stage"] for e in ctx.events if e["type"] == "stage.completed"] == ["navigation"]
    assert ctx.result.navigation_target.target_y == pytest.approx(target_y)
    assert "move_succeeded" not in ctx.result.model_dump()


def test_carrying_requires_navigation_and_verify_and_retains_evidence() -> None:
    ctx = MockSkillContext(skill_input(True))
    queue_navigation(ctx)
    queue_verify(ctx)
    asyncio.run(run_skill(run, ctx))
    assert ctx.status == "completed"
    assert [event["action"] for event in actions(ctx)] == [
        "navigation.follow_route", "navigation.verify_arrival",
    ]
    assert [e["stage"] for e in ctx.events if e["type"] == "stage.completed"] == ["navigation", "verify"]
    assert ctx.result.holding_object is True
    assert ctx.result.load_verification == "passed"
    assert ctx.result.evidence_refs == ["evidence://navigation", "evidence://load"]
    assert "raw" not in ctx.result.carrying_check.model_dump()


@pytest.mark.parametrize("status", ["failed", "stopped", "interrupted"])
def test_unsuccessful_navigation_never_verifies_or_retries(status: str) -> None:
    ctx = MockSkillContext(skill_input(True))
    queue_navigation(ctx, status)
    asyncio.run(run_skill(run, ctx))
    assert ctx.status == "failed"
    assert len(actions(ctx)) == 1
    assert ctx.failure.code == ("ACTION_STATE_UNKNOWN" if status == "interrupted" else "NAVIGATION_FAILED")


@pytest.mark.parametrize("case,code", [
    ("not_holding", "CARRYING_LOAD_NOT_CONFIRMED"),
    ("missing_tool", "TOOL_LOAD_UNVERIFIED"),
    ("extra_tool", "TOOL_LOAD_UNVERIFIED"),
    ("missing_force", "TOOL_LOAD_UNVERIFIED"),
    ("unchecked", "TOOL_LOAD_UNVERIFIED"),
    ("sensor_error", "TOOL_LOAD_UNVERIFIED"),
    ("below_threshold", "CARRYING_LOAD_NOT_CONFIRMED"),
    ("wrong_axis", "VERIFY_CONTRACT_MISMATCH"),
    ("wrong_threshold", "VERIFY_CONTRACT_MISMATCH"),
    ("nan", "VERIFY_OUTPUT_INVALID"),
    ("infinity", "VERIFY_OUTPUT_INVALID"),
    ("string_boolean", "VERIFY_OUTPUT_INVALID"),
    ("empty", "VERIFY_OUTPUT_INVALID"),
])
def test_force_check_must_prove_selected_tool_loaded(case: str, code: str) -> None:
    output = verify_output()
    check = output["carrying_check"]
    if case == "not_holding":
        output.update(verdict="not_achieved", holding_object=False)
        check["holding"] = False
    elif case == "missing_tool":
        del check["tools"]["left"]
    elif case == "extra_tool":
        check["tools"]["right"] = {"force_n": 6.0, "loaded": True}
    elif case == "missing_force":
        check["tools"]["left"]["force_n"] = None
    elif case == "unchecked":
        check["checked"] = False
    elif case == "sensor_error":
        check["errors"] = ["sensor unavailable"]
    elif case == "below_threshold":
        check["tools"]["left"]["force_n"] = 1.0
    elif case == "wrong_axis":
        check["force_axis"] = "x"
    elif case == "wrong_threshold":
        check["min_load_n"] = 2.0
    elif case in {"nan", "infinity"}:
        check["tools"]["left"]["force_n"] = float("nan" if case == "nan" else "inf")
    elif case == "string_boolean":
        output["holding_object"] = "true"
    elif case == "empty":
        output = {}
    ctx = MockSkillContext(skill_input(True))
    queue_navigation(ctx)
    queue_verify(ctx, output)
    asyncio.run(run_skill(run, ctx))
    assert ctx.status == "failed"
    assert ctx.failure.code == code
    assert len(actions(ctx)) == 2


def test_verify_action_failure_does_not_complete_or_navigate_again() -> None:
    ctx = MockSkillContext(skill_input(True))
    queue_navigation(ctx)
    ctx.queue_action("navigation.verify_arrival", ActionResult(status="failed", error_message="sensor offline"))
    asyncio.run(run_skill(run, ctx))
    assert ctx.failure.code == "VERIFY_ACTION_FAILED"
    assert len(actions(ctx)) == 2


def test_resume_verify_does_not_repeat_navigation() -> None:
    original = MockSkillContext(skill_input(True))
    queue_navigation(original)
    asyncio.run(run(original))
    assert original.state.stage == "verify"
    restored = MockSkillContext(skill_input(True))
    restored.checkpoint(original.state)
    queue_verify(restored)
    asyncio.run(run_skill(run, restored))
    assert restored.status == "completed"
    assert [event["action"] for event in actions(restored)] == ["navigation.verify_arrival"]
    assert "evidence://navigation" in restored.result.evidence_refs


def test_reentry_after_action_before_checkpoint_reuses_journal() -> None:
    class CrashAfterNavigation(MockSkillContext):
        crash_once = True

        def checkpoint(self, state):
            if state.navigation_succeeded and self.crash_once:
                self.crash_once = False
                raise RuntimeError("simulated worker crash")
            super().checkpoint(state)

    ctx = CrashAfterNavigation(skill_input(True))
    queue_navigation(ctx)
    with pytest.raises(RuntimeError, match="worker crash"):
        asyncio.run(run(ctx))
    assert ctx.state.navigation_succeeded is False
    queue_verify(ctx)
    asyncio.run(run_skill(run, ctx))
    assert ctx.status == "completed"
    assert [event["action"] for event in actions(ctx)].count("navigation.follow_route") == 1


def test_resume_uses_saved_station_without_resolving_again(monkeypatch) -> None:
    import semantic_navigation_real.scripts.skill as module

    inp = skill_input()
    station = resolve_navigation_target(inp)
    ctx = MockSkillContext(inp)
    ctx.checkpoint(SemanticNavigationState(navigation_target=station))
    def unexpected_solver(_):
        raise AssertionError("solver must not run on resume")
    monkeypatch.setattr(module, "resolve_navigation_target", unexpected_solver)
    queue_navigation(ctx)
    asyncio.run(run_skill(run, ctx))
    assert ctx.status == "completed"
    assert ctx.result.navigation_target == station


def test_unreachable_solver_target_fails_before_action() -> None:
    data = skill_input().model_dump()
    data["target"]["position_m"] = [0.5, 0, 3.5]
    ctx = MockSkillContext(SemanticNavigationInput.model_validate(data))
    asyncio.run(run_skill(run, ctx))
    assert ctx.failure.code == "NAV_TARGET_UNREACHABLE"
    assert not actions(ctx)


@pytest.mark.parametrize("case", [
    "no_mode", "no_move_y", "both_with_offset", "carry_both", "no_verify",
    "zero_threshold", "extra_parameter", "nan_target", "no_yaw",
])
def test_invalid_input_is_rejected(case: str) -> None:
    data = deepcopy(skill_input(True).model_dump())
    if case == "no_mode":
        del data["mode"]
    elif case == "no_move_y":
        data["move_y_m"] = None
    elif case == "both_with_offset":
        data = skill_input().model_dump()
        data["move_y_m"] = 0.1
    elif case == "carry_both":
        data["mode"] = "both"
        data["move_y_m"] = None
    elif case == "no_verify":
        data["verify"] = None
    elif case == "zero_threshold":
        data["verify"]["carrying_min_load_n"] = 0
    elif case == "extra_parameter":
        data["maximum_speed_mps"] = 0.5
    elif case == "nan_target":
        data["target"]["position_m"] = [float("nan"), 0, 1]
    elif case == "no_yaw":
        del data["target"]["yaw_rad"]
    with pytest.raises(ValidationError):
        SemanticNavigationInput.model_validate(data)


def test_verify_checkpoint_requires_navigation_success() -> None:
    ctx = MockSkillContext(skill_input(True))
    ctx.checkpoint(SemanticNavigationState(stage="verify"))
    asyncio.run(run_skill(run, ctx))
    assert ctx.failure.code == "INVALID_VERIFY_STATE"
    assert not actions(ctx)


def test_cancelled_execution_starts_no_action() -> None:
    ctx = MockSkillContext(skill_input())
    ctx.request_stop()
    with pytest.raises(SkillCancelled):
        asyncio.run(run(ctx))
    assert not actions(ctx)


@pytest.mark.parametrize(("mode", "navigation_succeeded", "safe"), [
    ("left", False, False), ("left", True, True),
    ("both", False, False), ("both", True, True),
])
def test_stop_adds_no_action_or_fake_confirmation(
    mode: str, navigation_succeeded: bool, safe: bool,
) -> None:
    ctx = MockSkillContext(skill_input(mode=mode))
    ctx.checkpoint(SemanticNavigationState(
        navigation_succeeded=navigation_succeeded,
    ))
    ctx.request_stop()
    outcome = asyncio.run(on_stop(ctx, StopRequest(reason="user stop", source="user")))
    assert outcome.safe is safe
    assert outcome.requires_intervention is not safe
    assert not actions(ctx)


@pytest.mark.parametrize(("mode", "yaw", "sign"), [
    ("left", 0.0, 1), ("right", 0.0, -1),
    ("left", math.pi, -1), ("right", math.pi, 1),
    ("left", -math.pi, -1), ("right", -math.pi, 1),
    ("left", 2 * math.pi, 1), ("right", -2 * math.pi, -1),
    ("left", 1e-7, 1), ("right", math.pi + 1e-7, 1),
])
@pytest.mark.parametrize("distance", [0.10, 0.15, 0.23])
def test_offset_uses_original_center_and_custom_magnitude(mode, yaw, sign, distance):
    data = skill_input(mode=mode).model_dump()
    data["move_y_m"] = distance
    data["target"]["yaw_rad"] = yaw
    inp = SemanticNavigationInput.model_validate(data)
    before = inp.model_dump()
    expected = (1.2, 0.8 + sign * distance, 0.9)
    assert resolve_target_position(inp) == pytest.approx(expected)
    # Repeated solving must not accumulate the offset on the input.
    first = resolve_navigation_target(inp)
    assert resolve_navigation_target(inp) == first
    assert inp.model_dump() == before
    from solver.Nav.navigation import solve_box_navigation_pose
    station = solve_box_navigation_pose(expected, target_yaw_rad=yaw, frame_id="map")
    assert first.target_x == pytest.approx(station.x)
    assert first.target_y == pytest.approx(station.y)


@pytest.mark.parametrize("mode", ["left", "right", "both"])
def test_explicit_zero_disables_offset_and_allows_other_yaw(mode):
    data = skill_input(mode=mode).model_dump()
    data["move_y_m"] = 0
    data["target"]["yaw_rad"] = math.pi / 2
    inp = SemanticNavigationInput.model_validate(data)
    assert resolve_target_position(inp) == inp.target.position_m
    ctx = MockSkillContext(inp)
    queue_navigation(ctx)
    asyncio.run(run_skill(run, ctx))
    assert ctx.status == "completed"
    assert [event["action"] for event in actions(ctx)] == ["navigation.follow_route"]


@pytest.mark.parametrize("yaw", [math.pi / 2, -math.pi / 2, 0.01, math.pi - 0.01])
def test_nonzero_offset_rejects_unsupported_yaw_before_action(yaw):
    data = skill_input(mode="left").model_dump()
    data["target"]["yaw_rad"] = yaw
    ctx = MockSkillContext(SemanticNavigationInput.model_validate(data))
    asyncio.run(run_skill(run, ctx))
    assert ctx.failure.code == "NAV_TARGET_INVALID"
    assert not actions(ctx)


@pytest.mark.parametrize("value", [-0.1, float("nan"), float("inf")])
def test_offset_rejects_invalid_magnitude(value):
    data = skill_input(mode="left").model_dump()
    data["move_y_m"] = value
    with pytest.raises(ValidationError):
        SemanticNavigationInput.model_validate(data)


def test_legacy_input_and_move_checkpoint_are_rejected():
    data = skill_input(mode="left").model_dump()
    data["move_distance_m"] = 0.01
    with pytest.raises(ValidationError):
        SemanticNavigationInput.model_validate(data)
    with pytest.raises(ValidationError):
        SemanticNavigationState(stage="move")
    with pytest.raises(ValidationError):
        SemanticNavigationState(move_succeeded=True)


def test_documented_examples_validate_and_only_declare_used_actions():
    import json
    import re
    text = (ROOT / "src/skill/semantic_navigation_real/SKILL.md").read_text()
    manifest = yaml.safe_load(text.split("---", 2)[1])
    assert manifest["version"] == "0.2.0"
    assert {a["type"] for a in manifest["required_actions"]} == {
        "navigation.follow_route", "navigation.verify_arrival",
    }
    assert {a["type"] for a in manifest["stop_actions"]} == {"navigation.follow_route"}
    for example in re.findall(r"```json\n(.*?)\n```", text, re.DOTALL):
        inp = SemanticNavigationInput.model_validate(json.loads(example))
        ctx = MockSkillContext(inp)
        queue_navigation(ctx)
        if inp.navigation_purpose == "carry_to_place":
            output = verify_output(inp.mode)
            output["carrying_check"]["force_axis"] = inp.verify.carrying_force_axis
            output["carrying_check"]["min_load_n"] = inp.verify.carrying_min_load_n
            queue_verify(ctx, output)
        asyncio.run(run_skill(run, ctx))
        assert ctx.status == "completed"
