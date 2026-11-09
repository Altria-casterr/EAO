"""ManifestRegistry behaviour."""

from __future__ import annotations

from ability_mcp.core.models import Manifest, ParamDef, TaskDef
from ability_mcp.core.registry import ManifestRegistry


def _mk_manifest(name: str, *tasks: tuple[int, str]) -> Manifest:
    return Manifest(
        abilityName=name,
        kind="AtomAbility",
        tasks=[TaskDef(taskType=tt, taskName=tn, params=[]) for tt, tn in tasks],
    )


def test_registry_task_index():
    reg = ManifestRegistry()
    reg.add(_mk_manifest("EndpointControl.Robotics", (0, "MoveToPose"), (1, "MoveJoints")))
    reg.add(_mk_manifest("Perception.Camera", (0, "CaptureOnce")))

    assert set(reg.ability_names()) == {"EndpointControl.Robotics", "Perception.Camera"}
    assert reg.task_type_of("EndpointControl.Robotics", "MoveToPose") == 0
    assert reg.task_type_of("EndpointControl.Robotics", "MoveJoints") == 1
    assert reg.task_type_of("Perception.Camera", "CaptureOnce") == 0
    assert reg.task_type_of("Missing", "X") is None

    tasks_pairs = list(reg.tasks())
    assert ("EndpointControl.Robotics", reg.get("EndpointControl.Robotics").tasks[0]) in tasks_pairs


def test_registry_filter_glob():
    reg = ManifestRegistry()
    reg.add(_mk_manifest("EndpointControl.Robotics", (0, "MoveToPose")))
    reg.add(_mk_manifest("EndpointControl.Simulator", (0, "Step")))
    reg.add(_mk_manifest("Perception.Camera", (0, "CaptureOnce")))

    reg.filter("EndpointControl.*")
    assert set(reg.ability_names()) == {
        "EndpointControl.Robotics",
        "EndpointControl.Simulator",
    }
    # task index also pruned
    assert reg.task_type_of("Perception.Camera", "CaptureOnce") is None


def test_registry_filter_noop_on_star():
    reg = ManifestRegistry()
    reg.add(_mk_manifest("X", (0, "A")))
    reg.filter("*")
    assert reg.ability_names() == ["X"]
    reg.filter("")
    assert reg.ability_names() == ["X"]
