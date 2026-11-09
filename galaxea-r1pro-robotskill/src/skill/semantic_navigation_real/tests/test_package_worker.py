"""完整 ZIP 在独立 Worker 中通过真实 JSON-RPC 管道执行；不连接机器人。"""

from __future__ import annotations

import json
import os
from pathlib import Path
import queue
import subprocess
import sys
import threading
from zipfile import ZipFile

import pytest
import yaml

from semantic_navigation_real.package_skill import build_package


ROOT = Path(__file__).resolve().parents[6]  # SDK 与仓库同属一个工作区（仓库根为 parents[4]）
CATALOG = Path(__file__).resolve().parents[4]


@pytest.mark.parametrize("purpose", ["approach_grasp", "carry_to_place"])
def test_packaged_worker_executes_only_requested_stages(tmp_path: Path, purpose: str) -> None:
    package = build_package(tmp_path / "skill.zip")
    extracted = tmp_path / "isolated-skill"
    with ZipFile(package) as archive:
        assert "solver/Nav/navigation.py" in archive.namelist()
        assert not any("__pycache__" in name or "tests/" in name for name in archive.namelist())
        archive.extractall(extracted)
    manifest = yaml.safe_load((extracted / "SKILL.md").read_text().split("---", 2)[1])
    assert manifest["name"] == "semantic-navigation-real"
    assert manifest["version"] == "0.2.0"
    assert {a["type"] for a in manifest["required_actions"] + manifest["stop_actions"]} == {
        "navigation.follow_route", "navigation.verify_arrival",
    }
    # 仅 SDK 可从源码加载；scripts/solver 必须来自解压包。
    environment = dict(os.environ)
    environment["PYTHONPATH"] = str(ROOT / "semantic-skill/robot-skill")
    process = subprocess.Popen(
        [sys.executable, "-m", "semantic_robot_skill_sdk.worker", "--skill-dir", str(extracted)],
        cwd=tmp_path, env=environment, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
        stderr=subprocess.PIPE, text=True, bufsize=1,
    )
    messages: queue.Queue = queue.Queue()

    def reader() -> None:
        try:
            for line in process.stdout:
                messages.put(json.loads(line))
        except BaseException as error:
            messages.put(error)

    threading.Thread(target=reader, daemon=True).start()

    def receive():
        result = messages.get(timeout=10)
        if isinstance(result, BaseException):
            raise result
        return result

    def send(value):
        process.stdin.write(json.dumps({"jsonrpc": "2.0", **value}) + "\n")
        process.stdin.flush()

    try:
        assert receive()["method"] == "ready"
        send({"id": "init", "method": "worker.initialize", "params": {
            "name": manifest["name"], "version": manifest["version"], "runtime": manifest["runtime"],
        }})
        initialized = receive()
        assert initialized["result"]["status"] == "initialized", initialized
        assert set(initialized["result"]["input_schema"]["required"]) == {
            "navigation_purpose", "mode", "target",
        }
        assert "move_y_m" in initialized["result"]["input_schema"]["properties"]
        assert "move_distance_m" not in initialized["result"]["input_schema"]["properties"]
        example = json.loads((CATALOG / "inputs" / f"semantic_navigation_real.{purpose}.example.json").read_text())
        send({"id": "validate", "method": "skill.validate_input", "params": {"input": example}})
        assert receive()["result"]["valid"] is True
        send({"id": "run", "method": "skill.run", "params": {
            "execution_id": "offline-worker-test", "robot_ref": "robot://offline",
            "input": example, "checkpoint": None, "log_fields": {},
        }})
        action_types: list[str] = []
        action_by_id = {}
        completed_stages = []
        terminal = None
        while terminal is None:
            message = receive()
            if message.get("id") == "run":
                terminal = message
                continue
            method = message.get("method")
            params = message.get("params", {})
            if method == "action.start":
                action = params["action"]
                action_id = f"action-{len(action_types)}"
                action_types.append(action["type"])
                action_by_id[action_id] = action["type"]
                assert action["type"] in {
                    "navigation.follow_route", "navigation.verify_arrival",
                }
                if action["type"] == "navigation.follow_route":
                    assert set(action["parameters"]) == {
                        "target_x", "target_y", "target_orientation", "target_point_type", "frame_id", "timeout",
                    }
                    direction = 1 if example["mode"] == "left" else -1
                    assert action["parameters"]["target_y"] == pytest.approx(
                        example["target"]["position_m"][1] + direction * example["move_y_m"]
                    )
                send({"id": message["id"], "result": {"action_id": action_id}})
            elif method == "action.result":
                result = {"status": "succeeded", "physical_effect": "confirmed", "output": {}}
                if action_by_id[params["action_id"]] == "navigation.verify_arrival":
                    result.update(physical_effect="none", output={
                        "status": "completed", "verdict": "achieved", "holding_object": True,
                        "carrying_check": {
                            "holding": True, "checked": True, "force_axis": example["verify"]["carrying_force_axis"],
                            "min_load_n": example["verify"]["carrying_min_load_n"],
                            "tools": {example["mode"]: {"force_n": 5.0, "loaded": True}},
                            "reasons": [], "errors": [], "raw": {},
                        },
                    })
                send({"id": message["id"], "result": result})
            elif method == "event.report":
                if params["event"] == "stage.completed":
                    completed_stages.append(params["stage"])
            else:
                assert method in {"checkpoint", "complete", "log"}, message
        assert terminal["result"]["status"] == "completed", terminal
        result = terminal["result"]["result"]
        assert result["navigation_succeeded"] is True
        assert "move_succeeded" not in result
        assert result["target_ref"] == example["target"]["target_ref"]
        if purpose == "approach_grasp":
            assert action_types == ["navigation.follow_route"]
            assert completed_stages == ["navigation"]
            assert result["holding_object"] is None
        else:
            assert action_types == [
                "navigation.follow_route", "navigation.verify_arrival",
            ]
            assert completed_stages == ["navigation", "verify"]
            assert result["holding_object"] is True
        send({"id": "shutdown", "method": "worker.shutdown", "params": {}})
        assert receive()["result"]["status"] == "shutting_down"
        process.wait(timeout=5)
        assert process.returncode == 0, process.stderr.read()
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(timeout=5)
        for stream in (process.stdin, process.stdout, process.stderr):
            stream.close()
