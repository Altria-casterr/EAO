"""RobotObservation 业务 Task：机器人状态与传感器读取（借鉴参考骨架 _observation）。

每次传入 observation 字段指定读取其中一个观测项；
同步返回 {"status": "completed", "observations": [...]}，observations 为单项观测，
条目统一为 {"source": "state"|"sensor", "name": ..., "data": ...}，
由 R1ProAbilityService.register_completed 登记为执行记录的可观测数据。

get_robot_state 同时支持两种入参（与 gripper/joint_control 一致，按 semantic_flag 区分）：
- 直接调用：``{"observation": "snapshot"}`` 读取单项状态观测
- semantic 调用（semantic robot.get_state）：``{"semantic_flag": true}``，
  无必填参数，返回 semantic Observation（kind=robot.state，schema_version=2）。
"""

from __future__ import annotations

import logging
from datetime import datetime, timezone
from typing import Any, Callable, Mapping

from ..utils.utils_RobotObservation import _output, _snapshot_dict
from .base import AbilityTask

logger = logging.getLogger(__name__)


def _semantic_observation(
    kind: str,
    subject_ref: str | None,
    value: Mapping[str, Any],
    *,
    confidence: float | None = None,
) -> dict[str, Any]:
    """构造 semantic Observation 兼容条目。"""

    return {
        "kind": kind,
        "schema_version": 2,
        "subject_ref": subject_ref,
        "source": "galaxea-r1pro",
        "observed_at": datetime.now(timezone.utc).isoformat(),
        "confidence": confidence,
        "value": dict(value),
        "artifact_refs": [],
        "evidence_refs": [f"evidence://ability/galaxea-r1pro/{kind}"],
    }


_STATE_READERS: dict[str, Callable[[Any], Any]] = {}
_SENSOR_READERS: dict[str, Callable[[Any], Any]] = {}


def _register_readers() -> None:
    """延迟注册 state / sensor 读取器（避免 import 期构造 SDK 依赖）。"""

    if _STATE_READERS:
        return

    _STATE_READERS.update({
        "snapshot": lambda sdk: _snapshot_dict(sdk.state.snapshot()),
        "torso_status": lambda sdk: _output(sdk.state.get_torso_status()),
        "floating_base": lambda sdk: _output(sdk.state.get_floating_base()),
        "transformer_martix": lambda sdk: _output(
            sdk.state.get_transformer_martix(
                source_frame="map", target_frame="body",
            )
        ),
    })

    _SENSOR_READERS.update({
        "ee_pose_left": lambda sdk: _output(sdk.sensors.get_ee_pose("left")),
        "ee_pose_right": lambda sdk: _output(sdk.sensors.get_ee_pose("right")),
        "gripper_state_left": lambda sdk: _output(sdk.sensors.get_gripper_state("left")),
        "gripper_state_right": lambda sdk: _output(sdk.sensors.get_gripper_state("right")),
        "joint_state_left": lambda sdk: _output(sdk.sensors.get_joint_state("left")),
        "joint_state_right": lambda sdk: _output(sdk.sensors.get_joint_state("right")),
        "force_data": lambda sdk: _output(sdk.sensors.get_force_data()),
        "torso_positions": lambda sdk: _output(sdk.sensors.get_torso_positions()),
        "chassis_feedback": lambda sdk: _output(sdk.sensors.get_chassis_feedback()),
        "localization": lambda sdk: _output(sdk.sensors.get_localization()),
    })


def _read_one(
    sdk: Any,
    readers: dict[str, Callable[[Any], Any]],
    source: str,
    name: str,
) -> dict[str, Any]:
    """读取单个观测项，返回 observations 条目；未知项抛 ValueError。"""

    reader = readers.get(name)
    if reader is None:
        raise ValueError(f"未知观测项: {name}（可选: {list(readers)}）")
    try:
        data = reader(sdk)
    except Exception as exc:  # 单项读取失败以 error 形式返回，不阻塞执行
        logger.warning("观测项 %s 读取失败: %s", name, exc)
        data = {"error": str(exc)}
    return {"source": source, "name": name, "data": data}


class GetRobotStateTask(AbilityTask):
    """机器人状态读取（taskType 0）。

    直接调用：按 observation 字段读取单项状态观测；
    semantic 调用（semantic_flag=True，robot.get_state）：无必填参数，
    返回 semantic robot.state 观测（robot_id/connected/mode/held_resources）。
    """

    name = "get_robot_state"
    summary = "读取机器人状态（semantic robot.get_state 返回 robot.state 观测；直接调用按 observation 读取单项）"
    task_type = 0

    def execute(self, input_data: Mapping[str, Any]) -> dict[str, Any]:

        _register_readers()
        observation = str(input_data.get("observation"))

        if not observation:
            raise ValueError("observation 必须提供")

        item = _read_one(self.sdk, _STATE_READERS, "state", observation)
        
        if observation == "snapshot":
            item["data"] = _semantic_observation(
                "robot.state", item["data"]["robot_id"], item["data"], confidence=1.0)

        return {"status": "completed", "observations": [item]}

class GetSensorsStateTask(AbilityTask):
    """传感器数据单项读取（taskType 1）。"""

    name = "get_sensors_state"
    summary = "读取指定传感器观测项（末端位姿/夹爪/关节/力数据）"
    task_type = 1

    def execute(self, input_data: Mapping[str, Any]) -> dict[str, Any]:
        _register_readers()
        observation = input_data.get("observation")
        if not observation:
            raise ValueError("observation 必须提供")
        observation = str(observation)
        item = _read_one(self.sdk, _SENSOR_READERS, "sensor", observation)
        return {"status": "completed", "observations": [item]}


class CaptureRGBDTask(AbilityTask):
    """RGB-D 图像采集（taskType 2，借鉴参考骨架 _observation 的 CaptureRGBD）。"""

    name = "capture_rgbd"
    summary = "采集指定传感器 RGB 与深度图像"
    task_type = 2

    def execute(self, input_data: Mapping[str, Any]) -> dict[str, Any]:
        sensor_id = input_data.get("sensor_id")
        if not sensor_id:
            raise ValueError("sensor_id 必须提供")
        sensor_id = str(sensor_id)

        aixs = input_data.get("axis", "rgb")
        
        if aixs not in ["rgb", "depth", "both"]:
            raise ValueError("axis 必须为 rgb/depth 或 both")

        observations = []

        if aixs == "rgb":
            rgb = self.sdk.sensors.capture_rgb(sensor_id)
            observations.append({
                "source": "sensor",
                "name": "rgb",
                "sensor_id": rgb.sensor_id,
                "encoding": rgb.encoding,
                "frame_id": rgb.frame_id})

        elif aixs == "depth":
            depth = self.sdk.sensors.capture_depth(sensor_id)
            observations.append({
                "source": "sensor",
                "name": "depth",
                "sensor_id": depth.sensor_id,
                "encoding": depth.encoding,
                "frame_id": depth.frame_id})

        else:
            rgb = self.sdk.sensors.capture_rgb(sensor_id)
            depth = self.sdk.sensors.capture_depth(sensor_id)
            observations.append({
                "source": "sensor",
                    "name": "rgb",
                    "sensor_id": rgb.sensor_id,
                    "encoding": rgb.encoding,
                    "frame_id": rgb.frame_id})
            observations.append({
                "source": "sensor",
                    "name": "depth",
                    "sensor_id": depth.sensor_id,
                    "encoding": depth.encoding,
                    "frame_id": depth.frame_id})
        return {"status": "completed", "observations": observations}
