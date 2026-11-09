"""隔离目录真实 Worker/JSON-RPC 测试；Ability 响应完全由离线 fixture 提供。"""
import json
import os
from pathlib import Path
import queue
import subprocess
import sys
import threading
from zipfile import ZipFile

import yaml
from semantic_robot_skill_sdk import Action, ActionResult
from robot_prepare.package_skill import build_package
from robot_prepare.scripts.models import LEFT, RIGHT


def test_isolated_package_worker_runs_all_stages(tmp_path):
    package = build_package(tmp_path / "skill.zip")
    extracted = tmp_path / "isolated"
    with ZipFile(package) as archive:
        names = archive.namelist()
        assert "scripts/skill.py" in names
        assert not any("tests/" in name or "__pycache__" in name for name in names)
        archive.extractall(extracted)
    manifest = yaml.safe_load((extracted / "SKILL.md").read_text().split("---", 2)[1])
    root = Path(__file__).resolve().parents[6]  # SDK 与仓库同属一个工作区（仓库根为 parents[4]）
    environment = dict(os.environ)
    environment["PYTHONPATH"] = str(root / "semantic-skill/robot-skill")
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
        assert not initialized["result"]["input_schema"].get("required")
        params = {}
        send({"id": "validate", "method": "skill.validate_input", "params": {"input": params}})
        assert receive()["result"]["valid"] is True
        send({"id": "run", "method": "skill.run", "params": {
            "execution_id": "offline-prepare", "robot_ref": "robot://offline",
            "input": params, "checkpoint": None, "log_fields": {},
        }})
        actions = []
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
                actions.append(action)
                responses[action_id] = ActionResult(status="succeeded")
                send({"id": message["id"], "result": {"action_id": action_id}})
            elif method == "action.result":
                send({"id": message["id"], "result": responses[values["action_id"]].model_dump(mode="json")})
            elif method == "event.report":
                if values["event"] == "stage.completed": completed.append(values["stage"])
            else:
                assert method in {"checkpoint", "complete", "log"}, message
        assert terminal["result"]["status"] == "completed", terminal
        result = terminal["result"]["result"]
        assert result["ready"] is True
        assert result["left_positions"] == list(LEFT)
        assert result["right_positions"] == list(RIGHT)
        assert completed == ["torso", "arms"]
        assert [a.type for a in actions] == ["torso.move_to_posture_all", "motion.move_to_posture_both"]
        for side in ('left', 'right'):
            assert actions[-1].parameters[f'{side}_joint_names'] == [f'{side}_arm_joint{i}' for i in range(1, 8)]
        send({"id": "shutdown", "method": "worker.shutdown", "params": {}})
        assert receive()["result"]["status"] == "shutting_down"
        process.wait(timeout=5)
        assert process.returncode == 0, process.stderr.read()
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(timeout=5)
        for stream in (process.stdin, process.stdout, process.stderr): stream.close()
