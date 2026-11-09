"""隔离目录真实 Worker/JSON-RPC 测试；Ability 响应完全由离线 fixture 提供。
单手 skill 不随包分发 solver 副本，因此不验证 scripts/solvers 文件存在。"""
import json
import os
from pathlib import Path
import queue
import subprocess
import sys
import threading
from zipfile import ZipFile

import yaml
import pytest
import semantic_robot_skill_sdk
from semantic_robot_skill_sdk import Action
from grasp_object_unilateral.package_skill import build_package
from .support import Responder, inputs


@pytest.mark.parametrize("retry", [False, True])
def test_isolated_package_worker_runs_all_stages(tmp_path, retry):
    package = build_package(tmp_path / "skill.zip")
    extracted = tmp_path / "isolated"
    with ZipFile(package) as archive:
        names = archive.namelist()
        assert "scripts/skill.py" in names
        assert "scripts/controller.py" in names
        assert not any("tests/" in name or "__pycache__" in name for name in names)
        archive.extractall(extracted)
    manifest = yaml.safe_load((extracted / "SKILL.md").read_text().split("---", 2)[1])
    root = Path(__file__).resolve().parents[4]
    environment = dict(os.environ)
    # 使用当前测试实际加载的 SDK，不依赖工控机固定的目录层级。
    sdk_root = Path(semantic_robot_skill_sdk.__file__).resolve().parent.parent
    environment["PYTHONPATH"] = os.pathsep.join((str(sdk_root), str(root / "src")))
    process = subprocess.Popen(
        [sys.executable, "-m", "semantic_robot_skill_sdk.worker", "--skill-dir", str(extracted)],
        cwd=tmp_path, env=environment, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
        stderr=subprocess.PIPE, text=True, bufsize=1,
    )
    messages = queue.Queue()
    def reader():
        try:
            for line in process.stdout:
                messages.put(json.loads(line))
        except BaseException as error:
            messages.put(error)
    threading.Thread(target=reader, daemon=True).start()
    def receive():
        item = messages.get(timeout=10)
        if isinstance(item, BaseException): raise item
        return item
    def send(item):
        process.stdin.write(json.dumps({"jsonrpc": "2.0", **item}) + "\n")
        process.stdin.flush()
    try:
        assert receive()["method"] == "ready"
        send({"id": "init", "method": "worker.initialize", "params": {
            "name": manifest["name"], "version": manifest["version"], "runtime": manifest["runtime"],
        }})
        initialized = receive()
        assert initialized["result"]["status"] == "initialized"
        assert set(initialized["result"]["input_schema"]["required"]) == {"target", "grasp_axis", "approach_force", "verify_force"}
        params = inputs().model_dump(mode="json")
        send({"id": "validate", "method": "skill.validate_input", "params": {"input": params}})
        assert receive()["result"]["valid"] is True
        send({"id": "run", "method": "skill.run", "params": {
            "execution_id": "offline-unilateral", "robot_ref": "robot://offline",
            "input": params, "checkpoint": None, "log_fields": {},
        }})
        responder = Responder()
        if retry:
            def fail_first_check(key, action, result):
                if "attempt:0:" in key and key.endswith("force-after"):
                    for arm in result.output["result"]["observations"][0]["data"].values():
                        arm["force_x"] = 0.5
                return result
            responder.mutate = fail_first_check
        responses = {}
        completed = []
        terminal = None
        while terminal is None:
            message = receive()
            if message.get("id") == "run":
                terminal = message
                continue
            method = message.get("method")
            values = message.get("params", {})
            if method == "action.start":
                action = Action.model_validate(values["action"])
                action_id = f"action-{len(responses)}"
                responses[action_id] = responder.respond(values["key"], action)
                send({"id": message["id"], "result": {"action_id": action_id}})
            elif method == "action.result":
                send({"id": message["id"], "result": responses[values["action_id"]].model_dump(mode="json")})
            elif method == "event.report":
                if values["event"] == "stage.completed": completed.append(values["stage"])
            else:
                assert method in {"checkpoint", "complete", "log"}, message
        assert terminal["result"]["status"] == "completed", terminal
        result = terminal["result"]["result"]
        assert result["holding_object"] is True
        assert result["lift_distance_m"] == 0.1
        assert result["grasp_attempts"] == (2 if retry else 1)
        assert [c["passed"] for c in result["pull_force_checks"]] == ([False, True] if retry else [True])
        assert result["other_arm_lift_distance_m"] == 0.1
        assert completed == (["prepare", "prepare"] if retry else ["prepare"]) + ["approach", "lift", "stand", "verify"]
        assert sum(a.type == "manipulation.force_align" for _, a in responder.actions) == (4 if retry else 3)
        torso_actions = [(key, a) for key, a in responder.actions if a.type == "torso.move_to_posture_all"]
        assert len(torso_actions) == 1
        assert torso_actions[0][0] == "grasp-object-unilateral:stand:torso"
        assert torso_actions[0][1].parameters["positions"] == [0.0] * 4
        for suffix in ("approach:other-y-move", "approach:other-z-move", "approach:other-x-move"):
            action = next(a for k, a in responder.actions if k.endswith(suffix))
            assert action.type == ("motion.move_end_effector_single_trajectory" if suffix.endswith("other-x-move") else "motion.move_end_effector_single")
        assert "approach_motion" not in initialized["result"]["input_schema"]["properties"]
        send({"id": "shutdown", "method": "worker.shutdown", "params": {}})
        assert receive()["result"]["status"] == "shutting_down"
        process.wait(timeout=5)
        assert process.returncode == 0, process.stderr.read()
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(timeout=5)
        for stream in (process.stdin, process.stdout, process.stderr): stream.close()
