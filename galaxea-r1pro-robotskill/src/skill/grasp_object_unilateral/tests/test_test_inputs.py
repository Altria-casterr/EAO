"""测试准备文件和 X 偏移契约；启动脚本测试仅运行临时目录中的假程序。"""
import asyncio
from copy import deepcopy
import json
import os
from pathlib import Path
import subprocess

import pytest
import yaml
from pydantic import ValidationError
from semantic_robot_skill_sdk import run_skill

from grasp_object_bilateral.scripts import solver as bilateral_solver
from grasp_object_bilateral.scripts.models import BothPoses as BilateralPoses
from grasp_object_bilateral.tests.support import inputs as bilateral_inputs
from grasp_object_unilateral.scripts import models, skill, solver
from .support import Context, inputs, pose

ROOT = Path(__file__).resolve().parents[4]


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("offset", [-0.1, 0.0, 0.03])
def test_capture_x_offset_matches_bilateral_without_moving_other_hand(side, offset):
    params = inputs(grasp_axis=side, capture_x_offset_m=offset)
    ctx = Context(params)
    original_poses = deepcopy(ctx.responder.poses)
    asyncio.run(skill.run(ctx))
    assert ctx.state.stage == "approach"
    assert ctx.state.matched_center_m[0] == 0.45
    selected = getattr(ctx.state.capture_points, side)
    assert selected.x == pytest.approx(0.45 + offset)
    assert selected.z == pytest.approx(0.1)
    other = "right" if side == "left" else "left"
    actual = solver.parse_pose(original_poses[other], params.relative_frame, ctx.state.base_to_relative)
    assert getattr(ctx.state.capture_points, other) == actual
    assert not any(a.type.startswith(("motion.", "manipulation.", "torso.")) for _, a in ctx.responder.actions)
    current = models.BothPoses(**{
        axis: solver.parse_pose(original_poses[axis], params.relative_frame, ctx.state.base_to_relative)
        for axis in ("left", "right")
    })
    reference = bilateral_solver.capture_points(
        bilateral_inputs(capture_x_offset_m=offset, capture_z_offset_m=0.0),
        ctx.state.matched_center_m, BilateralPoses.model_validate(current.model_dump()))
    assert selected.model_dump() == getattr(reference, side).model_dump()


@pytest.mark.parametrize("invalid", [float("nan"), float("inf"), float("-inf")])
def test_capture_x_offset_rejects_nonfinite_input(invalid):
    with pytest.raises(ValidationError):
        inputs(capture_x_offset_m=invalid)


def test_prepared_right_hand_input_preserves_bilateral_site_values():
    raw = json.loads((ROOT / "inputs/grasp_object_unilateral.json").read_text())
    baseline = json.loads((ROOT / "inputs/grasp_object_bilateral.json").read_text())
    params = models.GraspInput.model_validate(raw)
    assert params.grasp_axis == "right"
    assert params.pull_y_distance_m == 0.2
    for key in ("target", "relative_frame", "safe_offset_m", "capture_x_offset_m", "capture_z_offset_m",
                "torso", "stand", "approach_force", "verify_side"):
        assert raw[key] == baseline[key]
    assert params.verify_force.force_threshold_n == 1.0
    assert raw["verify_force"] == {**baseline["verify_force"], "force_threshold_n": 1.0}
    assert "approach_motion" not in raw
    assert "approach_motion" not in models.GraspInput.model_fields
    expected_motion = {**baseline["motion"], "pos_tol": 0.05, "speed": 0.03}
    for key, value in expected_motion.items():
        assert raw["motion"][key] == value
    assert raw["motion"]["sequential"] is True
    assert raw["perception"] == {"sensor_id": "head_camera"}
    ctx = Context(params)
    asyncio.run(run_skill(skill.run, ctx))
    assert ctx.status == "completed"
    actions = ctx.responder.actions
    assert next(a for k, a in actions if k.endswith("approach:grasp-move")).parameters["axis"] == "right"
    assert next(a for k, a in actions if k.endswith("approach:grasp-pull")).parameters["axis"] == "right"
    assert next(a for k, a in actions if k.endswith("verify:force")).parameters["axis"] == "left"
    torso_actions = [(key, a) for key, a in actions if a.type.startswith("torso.")]
    assert len(torso_actions) == 1
    assert torso_actions[0][0].endswith("stand:torso")
    assert torso_actions[0][1].parameters["positions"] == [0.0] * 4


def test_unilateral_deployment_has_exact_required_roles_and_local_framework():
    deployment = yaml.safe_load((ROOT / "src/config/grasp-unilateral-deployment.yaml").read_text())
    assert set(deployment["abilities"]) == {"robot", "torso", "perception", "manipulator_motion"}
    assert all(spec["instance_id"] for spec in deployment["abilities"].values())
    assert deployment["ability_framework"]["endpoint"] == "http://127.0.0.1:8080"
    assert deployment["ability_framework"]["managed_by_instance"] is False


def make_fake_launch_tree(tmp_path, preflight_exit=0):
    root = tmp_path / "workspace/semantic-skill/galaxea-r1pro-robotskill"
    scripts = root / "start-shell"
    scripts.mkdir(parents=True)
    source = ROOT / "start-shell/run_grasp_unilateral_skill.sh"
    launcher = scripts / source.name
    launcher.write_text(source.read_text())
    marker = tmp_path / "calls.jsonl"
    (scripts / "run_grasp_bilateral_skill.sh").write_text(
        '#!/usr/bin/env bash\nset -euo pipefail\n'
        '[[ "${GRASP_CHECK_ONLY:-}" == "1" && "${GRASP_SKIP_PREFLIGHT:-}" == "0" ]]\n'
        'printf "preflight\\n" >> "$TEST_CALLS"\n'
        f'exit {preflight_exit}\n')
    for relative in ("src/config/grasp-unilateral-deployment.yaml", "inputs/grasp_object_unilateral.json"):
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("{}")
    fake_python = tmp_path / "python"
    fake_python.write_text('#!/usr/bin/env bash\nexit 0\n')
    fake_python.chmod(0o700)
    fake_pilot = tmp_path / "pilot"
    fake_pilot.write_text('#!/usr/bin/env bash\nprintf "pilot:%s\\n" "$*" >> "$TEST_CALLS"\n')
    fake_pilot.chmod(0o700)
    fake_mktemp = tmp_path / "mktemp"
    fake_mktemp.write_text('#!/usr/bin/env bash\nprintf "%s\\n" "$TEST_OUTPUT"\n')
    fake_mktemp.chmod(0o700)
    env = dict(os.environ, GRASP_PYTHON=str(fake_python), GRASP_PILOT=str(fake_pilot),
               GRASP_SKIP_PREFLIGHT="1", TEST_CALLS=str(marker), TEST_OUTPUT=str(tmp_path / "output"),
               PATH=str(tmp_path) + os.pathsep + os.environ["PATH"])
    return launcher, marker, env


def test_check_only_launcher_never_starts_pilot_even_with_inherited_skip_flag(tmp_path):
    launcher, marker, env = make_fake_launch_tree(tmp_path)
    env["GRASP_CHECK_ONLY"] = "1"
    result = subprocess.run(["bash", str(launcher)], env=env, capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stderr
    assert marker.read_text().splitlines() == ["preflight"]
    assert "未启动 Skill" in result.stdout


def test_failed_preflight_prevents_pilot(tmp_path):
    launcher, marker, env = make_fake_launch_tree(tmp_path, preflight_exit=7)
    env["GRASP_CHECK_ONLY"] = "0"
    result = subprocess.run(["bash", str(launcher)], env=env, capture_output=True, text=True, timeout=10)
    assert result.returncode == 7
    assert marker.read_text().splitlines() == ["preflight"]


def test_explicit_run_path_selects_unilateral_version_not_bilateral(tmp_path):
    launcher, marker, env = make_fake_launch_tree(tmp_path)
    env["GRASP_CHECK_ONLY"] = "0"
    result = subprocess.run(["bash", str(launcher)], env=env, capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stderr
    calls = marker.read_text().splitlines()
    assert calls[0] == "preflight" and len(calls) == 2
    assert "--skill grasp-object-unilateral@0.1.0" in calls[1]
    assert "--skill grasp-object-bilateral" not in calls[1]
    assert "grasp-unilateral-deployment.yaml" in calls[1]
    assert "grasp_object_unilateral.json" in calls[1]
