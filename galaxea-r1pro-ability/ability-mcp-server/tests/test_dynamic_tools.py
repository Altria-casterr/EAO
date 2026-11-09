"""Tests for `_build_task_tool_fn`: dynamically compiled per-task handlers.

Regression cover for the bug where every dynamic tool was registered with a
single `args: Dict` parameter, which FastMCP could not bind client kwargs
into — the agent's `position=..., orientation=...` call was silently lost.
"""

from __future__ import annotations

import asyncio
import inspect
from typing import Any, Dict, List

import pytest

from ability_mcp.core.models import ParamDef, TaskDef
from ability_mcp.native import tools as tools_mod


class FakeClient:
    """Stand-in FrameworkClient — only needs to keep the reference alive;
    _dispatch_ability_task is patched out so none of its methods run."""


@pytest.fixture
def captured(monkeypatch):
    """Patch module-level `_dispatch_ability_task` so the generated tool
    fn hits our capture instead of hitting a real framework. The patch
    stays in place for the whole test (monkeypatch auto-restores at
    teardown)."""
    collected: List[Dict[str, Any]] = []

    async def _fake_dispatch(*, client, ability_name, task_name, task_type, args, timeout):
        collected.append(args)
        return {"task_id": "task-fake", "status": "completed", "payload": args}

    monkeypatch.setattr(tools_mod, "_dispatch_ability_task", _fake_dispatch)
    return collected


def _build(task: TaskDef):
    return tools_mod._build_task_tool_fn(
        task=task,
        binding={
            "ability": "EndpointControl.Robotics",
            "task_name": task.taskName,
            "task_type": task.taskType,
        },
        client=FakeClient(),
        task_timeout=5.0,
    )


def _make_move_to_pose() -> TaskDef:
    return TaskDef(
        taskType=0,
        taskName="MoveToPose",
        summary="move end effector",
        params=[
            ParamDef(name="position", type="Point3d", optional=False),
            ParamDef(name="orientation", type="Quaternion", optional=False),
            ParamDef(name="velocity_scale", type="number", optional=True),
            ParamDef(name="acceleration_scale", type="number", optional=True),
        ],
    )


def test_generated_signature_matches_manifest_params(captured):
    """inspect.signature must reflect the manifest, not `args`."""
    fn = _build(_make_move_to_pose())

    sig = inspect.signature(fn)
    assert list(sig.parameters.keys()) == [
        "position",
        "orientation",
        "velocity_scale",
        "acceleration_scale",
    ]
    # required params have no default
    assert sig.parameters["position"].default is inspect.Parameter.empty
    assert sig.parameters["orientation"].default is inspect.Parameter.empty
    # optional params default to None
    assert sig.parameters["velocity_scale"].default is None
    assert sig.parameters["acceleration_scale"].default is None
    # coroutine
    assert inspect.iscoroutinefunction(fn)


def test_call_with_kwargs_forwards_full_args_dict(captured):
    """The core regression: calling fn(position=..., orientation=...)
    must arrive at the dispatcher as {"position": ..., "orientation": ...}
    rather than an empty dict."""
    fn = _build(_make_move_to_pose())

    pos = {"x": 0.3, "y": 0.1, "z": 0.2}
    quat = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    result = asyncio.run(fn(position=pos, orientation=quat, velocity_scale=0.3))

    assert len(captured) == 1
    assert captured[0] == {
        "position": pos,
        "orientation": quat,
        "velocity_scale": 0.3,
    }
    # acceleration_scale was left as default None → omitted from the dict
    assert "acceleration_scale" not in captured[0]
    # dispatch result propagated
    assert result["status"] == "completed"
    assert result["payload"]["position"] == pos


def test_no_params_task_has_empty_signature(captured):
    """Zero-param tasks like GetCurrentPose should still round-trip."""
    fn = _build(TaskDef(taskType=4, taskName="GetCurrentPose", params=[]))
    sig = inspect.signature(fn)
    assert len(sig.parameters) == 0
    asyncio.run(fn())
    assert captured == [{}]


def test_param_name_sanitisation_preserves_original_key(captured):
    """Param names containing dots/dashes/keywords get rewritten in the
    Python signature, but the ORIGINAL manifest name must still be what
    ends up in the args dict so the ability process receives it."""
    task = TaskDef(
        taskType=7,
        taskName="Weird",
        params=[
            ParamDef(name="foo.bar", type="string", optional=False),
            ParamDef(name="class", type="string", optional=False),  # py keyword
            ParamDef(name="_hidden", type="string", optional=False),
        ],
    )
    fn = _build(task)

    sig = inspect.signature(fn)
    names = list(sig.parameters.keys())
    # sanitised: dot → underscore, keyword → `class_`, leading underscore → prefixed
    assert "foo_bar" in names
    assert "class_" in names
    # leading-underscore name gets p-prefixed until no longer starting with _
    assert not any(n.startswith("_") for n in names)

    # Call with the sanitised names
    kwargs = {n: "x" for n in names}
    asyncio.run(fn(**kwargs))
    # … but the dispatcher receives ORIGINAL manifest keys
    assert "foo.bar" in captured[0]
    assert "class" in captured[0]
    assert "_hidden" in captured[0]


def test_typed_hints_make_it_to_signature(captured):
    """Manifest types map to Python type annotations; agents / FastMCP
    infer a baseline schema from them (we still override with the precise
    manifest schema after registration)."""
    task = TaskDef(
        taskType=3,
        taskName="SetGripper",
        params=[
            ParamDef(name="opening", type="number", optional=False),
            ParamDef(name="force", type="number", optional=True),
            ParamDef(name="label", type="string", optional=True),
            ParamDef(name="enabled", type="boolean", optional=True),
        ],
    )
    fn = _build(task)
    sig = inspect.signature(fn)
    # required numeric - real float type (not a string), thanks to
    # dont_inherit=True on compile()
    assert sig.parameters["opening"].annotation is float
    # optional numeric still float at heart (via `float | None`)
    assert sig.parameters["force"].default is None
    assert sig.parameters["label"].default is None
    assert sig.parameters["enabled"].default is None
