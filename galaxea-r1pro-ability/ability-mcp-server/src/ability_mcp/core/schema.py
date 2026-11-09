"""Convert manifest task definitions into MCP tool input schemas."""

from __future__ import annotations

import re
from typing import Any, Dict

from .models import TaskDef


# Built-in + common robotics composite types. Extend as needed.
BUILTIN_TYPES: Dict[str, Dict[str, Any]] = {
    "string": {"type": "string"},
    "str": {"type": "string"},
    "int": {"type": "integer"},
    "integer": {"type": "integer"},
    "number": {"type": "number"},
    "float": {"type": "number"},
    "double": {"type": "number"},
    "bool": {"type": "boolean"},
    "boolean": {"type": "boolean"},
    "array": {"type": "array"},
    "list": {"type": "array"},
    "object": {"type": "object"},
    "dict": {"type": "object"},
    "map": {"type": "object"},
    # robotics composite shapes
    "Point3d": {
        "type": "object",
        "properties": {
            "x": {"type": "number"},
            "y": {"type": "number"},
            "z": {"type": "number"},
        },
        "required": ["x", "y", "z"],
    },
    "Vector3": {
        "type": "object",
        "properties": {
            "x": {"type": "number"},
            "y": {"type": "number"},
            "z": {"type": "number"},
        },
        "required": ["x", "y", "z"],
    },
    "Quaternion": {
        "type": "object",
        "properties": {
            "x": {"type": "number"},
            "y": {"type": "number"},
            "z": {"type": "number"},
            "w": {"type": "number"},
        },
        "required": ["x", "y", "z", "w"],
    },
    "Euler": {
        "type": "object",
        "properties": {
            "roll": {"type": "number"},
            "pitch": {"type": "number"},
            "yaw": {"type": "number"},
        },
        "required": ["roll", "pitch", "yaw"],
    },
    "Pose": {
        "type": "object",
        "properties": {
            "position": {
                "type": "object",
                "properties": {
                    "x": {"type": "number"},
                    "y": {"type": "number"},
                    "z": {"type": "number"},
                },
            },
            "orientation": {
                "type": "object",
                "properties": {
                    "x": {"type": "number"},
                    "y": {"type": "number"},
                    "z": {"type": "number"},
                    "w": {"type": "number"},
                },
            },
        },
    },
}


def _type_to_schema(type_name: str) -> Dict[str, Any]:
    """Look up a type name in BUILTIN_TYPES, case-sensitive for composites
    and case-insensitive for scalars. Unknown types fall through to a
    permissive `{}` (no constraint), which MCP clients treat as "any"."""
    if type_name in BUILTIN_TYPES:
        return dict(BUILTIN_TYPES[type_name])  # copy
    low = type_name.lower()
    if low in BUILTIN_TYPES:
        return dict(BUILTIN_TYPES[low])
    return {}


def build_input_schema(task: TaskDef) -> Dict[str, Any]:
    """Turn `task.params` into an MCP-compatible JSON Schema object.

    Returns `{"type": "object", "properties": {...}, "required": [...]}`.
    Even an empty param list produces `{"type": "object", "properties": {}}`.
    """
    props: Dict[str, Any] = {}
    required: list[str] = []
    for p in task.params:
        s = _type_to_schema(p.type)
        if p.description:
            s = {**s, "description": p.description}
        props[p.name] = s
        if not p.optional:
            required.append(p.name)
    out: Dict[str, Any] = {"type": "object", "properties": props}
    if required:
        out["required"] = required
    return out


_NON_ID = re.compile(r"[^a-zA-Z0-9]+")


def _snake(s: str) -> str:
    s = _NON_ID.sub("_", s)
    # CamelCase boundary (lowercase -> uppercase); exclude digits so that
    # "Camera2D" stays "camera2d" rather than "camera2_d".
    s = re.sub(r"([a-z])([A-Z])", r"\1_\2", s)
    # Acronym boundary (e.g. HTTPServer -> HTTP_Server).
    s = re.sub(r"([A-Z]+)([A-Z][a-z])", r"\1_\2", s)
    return s.strip("_").lower()


def tool_name(ability_name: str, task_name: str) -> str:
    """Generate an MCP tool name from `(abilityName, taskName)`.

    Example:
        EndpointControl.Robotics + MoveToPose
          -> endpoint_control_robotics__move_to_pose
    """
    a = _snake(ability_name)
    t = _snake(task_name)
    return f"{a}__{t}"
