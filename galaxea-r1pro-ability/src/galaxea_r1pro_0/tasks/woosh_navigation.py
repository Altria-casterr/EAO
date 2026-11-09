"""WooshNavigation 业务 Task：地图导航（借鉴 Atom WooshNavigation Galaxea_R1Pro）。

navigate_to_map_pose：SDK 内部同步等待导航完成（解锁刹车、等待/超时/紧急停止闭环），
Task 用返回的 CommandResult 构造 CommandHandle 登记执行记录（可 Get/Stop）；
失败/取消直接抛异常。
stop：主动停止运动和导航（紧急停止 + 零速发布），返回 CommandHandle。
"""

from __future__ import annotations

import logging
from typing import Any, Mapping

from galaxea_r1pro import CommandHandle, CommandStatus
from ..utils.utils_WooshNav import quat_to_yaw_deg
import math
from .base import AbilityTask

logger = logging.getLogger(__name__)

class NavigateToMapPoseTask(AbilityTask):
    """地图导航（taskType 0）：导航到地图坐标系下的指定位置。"""

    name = "navigate_to_map_pose"
    summary = "导航到地图坐标系下的目标位置（同步等待完成）"
    task_type = 0

    def execute(self, input_data: Mapping[str, Any]) -> CommandHandle:
        sdk = self.sdk

        # Step1、解析输入参数（semantic navigation.follow_route 的 target 契约），切分是Sem调用还是直接调用

        target_x = float(input_data.get("target_x", 0.0))
        target_y = float(input_data.get("target_y", 0.0))
        target_orientation = input_data.get("target_orientation", [0.0, 0.0, 0.0, 1.0])
        target_yaw = quat_to_yaw_deg(target_orientation)

        target_point_type = int(input_data.get("target_point_type", 0))
        frame_id = str(input_data.get("frame_id", "map"))
        timeout = float(input_data.get("timeout", 60.0))

        logger.info(
            "地图导航目标: pos=(%.3f,%.3f) yaw=%.1f frame=%s target_point_type=%s timeout=%.1fs",
            target_x, target_y, target_yaw, frame_id, target_point_type, timeout,
        )

        result = sdk.woosh_navigation.navigate_to_map_pose(
            x=target_x, y=target_y, yaw=target_yaw, frame_id=frame_id, target_point_type=target_point_type, timeout=timeout)

        if result.status is CommandStatus.FAILED:
            raise RuntimeError(f"地图导航失败: {result.error_message or '未知错误'}")
        if result.status is CommandStatus.CANCELLED:
            raise RuntimeError("地图导航被取消")

        # 用 SDK 返回的 CommandResult 构造 CommandHandle 登记执行记录（可 Get/Stop）
        return CommandHandle(
            result.command_id,
            sdk.profile.robot_id,
            "woosh_navigation",
            "navigate_to_map_pose",
        )

class VerifyArrivalTask(AbilityTask):
    """持物检查（taskType 2）：按腕部力传感判断手中是否仍有搬运物体。"""

    name = "verify_arrival"
    summary = "按腕部力传感检查手中是否仍有搬运物体（指定方向分量超阈值）"
    task_type = 2

    # 默认用哪个方向的力判负载
    DEFAULT_FORCE_AXIS = "z"
    # 允许的取力方向：三分量之一，或 norm（合力）
    FORCE_AXES = ("x", "y", "z", "norm")
    FORCE_COMPONENTS = ("x", "y", "z")
    # 负载腕力阈值（牛，占位值，需按真机标定）
    DEFAULT_MIN_LOAD_N = 3.0

    @staticmethod
    def _tool_sides(tool_refs: Any) -> list[str]:
        """把 tool_refs 规整成 left/right；未声明时默认检查左右两侧。"""

        refs = (tool_refs,) if isinstance(tool_refs, str) else (tool_refs or ())
        sides: list[str] = []
        for value in refs:
            side = str(value).strip().rstrip("/").rsplit("/", 1)[-1].strip().lower()
            if side and side not in sides:
                sides.append(side)
        return sides or ["left", "right"]

    def _read_wrench(self) -> tuple[Any, list[str]]:
        """读取腕力数据；读取失败时返回错误信息而不是中断任务。"""

        try:
            output = self.sdk.sensors.get_force_data().output or {}
            return output.get("data"), []
        except Exception as exc:
            return {"error": str(exc)}, [str(exc)]

    @staticmethod
    def _side_force(wrench: Any, side: str, axis: str) -> tuple[float | None, str | None]:
        """解析单侧腕力，返回指定分量或三轴合力。"""

        if not isinstance(wrench, Mapping):
            return None, "腕力数据缺失"
        arm = wrench.get(f"{side}_arm")

        if not isinstance(arm, Mapping):
            return None, f"缺少 {side}_arm 腕力"

        try:
            names = VerifyArrivalTask.FORCE_COMPONENTS if axis == "norm" else (axis,)

            if any(arm.get(f"force_{name}") is None for name in names):
                return None, f"{side}_arm 腕力分量缺失"

            values = {name: float(arm[f"force_{name}"]) for name in names}

        except (TypeError, ValueError) as exc:
            return None, f"{side}_arm 腕力解析失败: {exc}"

        if axis == "norm":
            return math.sqrt(sum(value * value for value in values.values())), None

        return values[axis], None

    def _check_holding(
        self,
        tool_refs: list[str] | str | None = None,
        *,
        force_axis: str = DEFAULT_FORCE_AXIS,
        min_load_n: float = DEFAULT_MIN_LOAD_N) -> dict[str, Any]:
        """按腕力判断手中是否有搬运物体，返回判定与原始证据。"""

        if force_axis not in self.FORCE_AXES:
            raise ValueError(f"carrying_force_axis 必须是 {self.FORCE_AXES} 之一：{force_axis!r}")
        sides = self._tool_sides(tool_refs)
        load_threshold = float(min_load_n)

        wrench_raw, errors = self._read_wrench()

        tools: dict[str, Any] = {}
        for side in sides:
            force, reason = self._side_force(wrench_raw, side, force_axis)
            tools[side] = {
                "force_n": force,
                "loaded": force is not None and abs(force) >= load_threshold,
            }
            if reason:
                errors.append(reason)

        loaded_ok = all(item["loaded"] for item in tools.values())
        checked = all(item["force_n"] is not None for item in tools.values())

        reasons: list[str] = []
        if not loaded_ok:
            reasons.append(f"{force_axis} 方向腕力未达负载阈值（{load_threshold}N）")
        if not checked:
            reasons.append("部分腕力读取失败，无法确认持有状态")

        return {
            "holding": bool(loaded_ok and checked), 
            "checked": bool(checked),
            "force_axis": force_axis,
            "min_load_n": load_threshold,
            "tools": tools,
            "reasons": reasons,
            "errors": errors,
            "raw": {"arm_wrench": wrench_raw},
        }

    def execute(self, input_data: Mapping[str, Any]) -> dict[str, Any]:
        tool_refs = input_data.get("tool_refs")
        force_axis = self._force_axis(input_data)
        min_load_n = float(input_data.get("carrying_min_load_n", self.DEFAULT_MIN_LOAD_N))
        check = self._check_holding(tool_refs, force_axis=force_axis, min_load_n=min_load_n)
        holding = bool(check["holding"])

        # Step3、返回执行结果（verdict 只反映持物检查，不含到达判定）
        logger.info(
            "持物检查: holding=%s checked=%s 侧别=%s 方向=%s 阈值=%.1fN 原因=%s",
            holding,
            check["checked"],
            list(check["tools"]),
            check["force_axis"],
            min_load_n,
            "；".join(check["reasons"]) or "无",
        )
        return {
            "status": "completed",
            "verdict": "achieved" if holding else "not_achieved",
            "holding_object": holding,
            "carrying_check": check,
        }

    @classmethod
    def _force_axis(cls, input_data: Mapping[str, Any]) -> str:
        return str(input_data.get("carrying_force_axis") or cls.DEFAULT_FORCE_AXIS).strip().lower()


class StopTask(AbilityTask):
    """主动停止（taskType 1）：置位紧急停止并连续发布零速停止底盘运动。"""

    name = "stop"
    summary = "停止所有运动与导航（紧急停止 + 零速发布）"
    task_type = 1

    def execute(self, input_data: Mapping[str, Any]) -> CommandHandle:
        handle = self.sdk.woosh_navigation.stop()
        logger.info("导航停止指令已发布（紧急停止置位）")
        return handle
