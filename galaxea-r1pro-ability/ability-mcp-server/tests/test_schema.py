"""Unit tests for schema.py (no framework required)."""

from __future__ import annotations

from ability_mcp.core.models import ParamDef, TaskDef
from ability_mcp.core.schema import build_input_schema, tool_name


def test_build_input_schema_empty_params():
    task = TaskDef(taskType=0, taskName="GetCurrentPose", params=[])
    schema = build_input_schema(task)
    assert schema == {"type": "object", "properties": {}}


def test_build_input_schema_primitives():
    task = TaskDef(
        taskType=1,
        taskName="SetGripper",
        params=[
            ParamDef(name="opening", type="number", optional=False, description="dim"),
            ParamDef(name="force", type="number", optional=True, description="N"),
            ParamDef(name="label", type="string"),
        ],
    )
    schema = build_input_schema(task)
    assert schema["type"] == "object"
    assert schema["required"] == ["opening", "label"]
    assert schema["properties"]["opening"] == {"type": "number", "description": "dim"}
    assert schema["properties"]["force"] == {"type": "number", "description": "N"}
    assert schema["properties"]["label"] == {"type": "string"}


def test_build_input_schema_composite_types():
    task = TaskDef(
        taskType=0,
        taskName="MoveToPose",
        params=[
            ParamDef(name="position", type="Point3d", description="target"),
            ParamDef(name="orientation", type="Quaternion"),
        ],
    )
    schema = build_input_schema(task)
    pos = schema["properties"]["position"]
    assert pos["type"] == "object"
    assert set(pos["properties"].keys()) == {"x", "y", "z"}
    assert pos["description"] == "target"
    quat = schema["properties"]["orientation"]
    assert set(quat["properties"].keys()) == {"x", "y", "z", "w"}
    assert schema["required"] == ["position", "orientation"]


def test_build_input_schema_unknown_type_permissive():
    task = TaskDef(
        taskType=0,
        taskName="Foo",
        params=[ParamDef(name="weird", type="MyCustomType")],
    )
    schema = build_input_schema(task)
    # Unknown types fall through to {} so MCP clients accept anything
    assert schema["properties"]["weird"] == {}


def test_tool_name_canonicalisation():
    assert (
        tool_name("EndpointControl.Robotics", "MoveToPose")
        == "endpoint_control_robotics__move_to_pose"
    )
    assert tool_name("Perception.Camera2D", "CaptureOnce") == "perception_camera2d__capture_once"
    assert tool_name("foo-bar", "baz_qux") == "foo_bar__baz_qux"
