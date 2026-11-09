"""RobotObservation 专用功能函数层（观测数据读取/序列化）。"""

from __future__ import annotations

from dataclasses import asdict
from typing import Any


def _output(result: Any) -> Any:
    """从 CommandResult 提取 output.data。"""

    return (result.output or {}).get("data")


def _snapshot_dict(state: Any) -> dict[str, Any]:
    """把 RobotState 转换为可序列化 dict（asdict 会处理嵌套 dataclass）。"""

    try:
        return asdict(state)

    except TypeError:
        return {
            "robot_id": state.robot_id,
            "backend": getattr(state.backend, "value", str(state.backend)),
            "connected": state.connected,
            "mode": state.mode,
            "base_pose": asdict(state.base_pose),
            "joints": asdict(state.joints),
            "held_resources": list(state.held_resources),
        }
