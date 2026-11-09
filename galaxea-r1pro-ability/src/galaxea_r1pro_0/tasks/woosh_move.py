"""WooshMove 业务 Task：底盘单轴相对运动控制（借鉴 Atom WooshMove Galaxea_R1Pro）。

move_robot：单轴相对运动（x/y/z），采用"持续刷新速度命令 + 定位优先 + 积分兜底"的闭环，
到位容差 / 超时 / 刷新间隔由输入控制；到位或超时前都会先停止底盘。
入参为扁平参数：target_x / target_y / target_orientation（四元数，仅取 yaw 映射到 z 轴），
三者中恰好一个非零；stop：主动停止底盘运动（紧急停止 + 零速发布）。

符号约定（重要）：utils_WooshMove 中定位测量值对 x/y 已取负以适配底盘方向，
即"测量值符号与指令符号相反"，因此到位判定只比较幅值，不做方向校验。
"""

from __future__ import annotations

import logging
import time
from typing import Any, Mapping

from galaxea_r1pro import CommandHandle

from ..utils.utils_WooshMove import (
    INVALID_LOCALIZATION,
    _get_localization_value,
    _integrate_feedback,
    _record_initial_localization,
)
from ..utils.utils_WooshNav import quat_to_yaw_deg
from .base import AbilityTask

logger = logging.getLogger(__name__)


def _read_localization(sdk: Any) -> Any:
    """读取定位输出的 data 字段（不可用时返回 None）。"""

    result = sdk.sensors.get_localization()
    return (getattr(result, "output", None) or {}).get("data")


class MoveRobotTask(AbilityTask):
    """单轴相对运动（taskType 0），含到位闭环（定位优先 + 积分兜底）。"""

    name = "move_robot"
    summary = "控制底盘沿指定轴相对运动（x/y/z，含到位闭环）"
    task_type = 0

    @staticmethod
    def _parse_target(input_data: Mapping[str, Any]) -> tuple[str, float]:
        """从扁平入参解析单轴目标，返回 (轴, 目标值)；x/y 单位米，z 单位度。"""

        target_x = float(input_data.get("target_x", 0.0))
        target_y = float(input_data.get("target_y", 0.0))
        orientation = input_data.get("target_orientation", [0.0, 0.0, 0.0, 1.0])

        if len(orientation) != 4:
            raise ValueError(f"target_orientation 必须有 4 个元素：{orientation}")

        yaw_deg = quat_to_yaw_deg(orientation)

        # 单轴相对运动：x/y/yaw(z) 三个分量中恰好一个非零
        candidates = {"x": target_x, "y": target_y, "z": yaw_deg}
        active = {name: value for name, value in candidates.items() if abs(value) > 1e-6}
        if len(active) != 1:
            logger.error("[MoveRobot] 需要 x/y/z 中恰好一个非零: candidates=%s", candidates)
            raise ValueError(
                f"语义相对运动要求 x/y/z 三个方向中恰好一个非零（单轴运动），"
                f"收到 dx={target_x:.3f} dy={target_y:.3f} yaw={yaw_deg:.3f}"
            )

        axis, target_value = next(iter(active.items()))
        logger.warning(
            "[MoveRobot] 单轴解析完成: axis=%s target=%.4f (x=%.4f y=%.4f yaw=%.4f)",
            axis, target_value, target_x, target_y, yaw_deg)

        return axis, target_value

    def execute(self, input_data: Mapping[str, Any]) -> CommandHandle:
        sdk = self.sdk
        logger.warning("[MoveRobot] 收到原始入参: keys=%s", sorted(input_data.keys()))

        axis, target_value = self._parse_target(input_data)

        speed = float(input_data.get("speed", 0.5))
        tolerance = float(input_data.get("tolerance", 0.02 if axis in ("x", "y") else 0.5))
        timeout = float(input_data.get("timeout", 10.0))
        refresh_interval = float(input_data.get("refresh_interval", 0.1))
        settle_time = float(input_data.get("settle_time", 0.2))
        logger.warning(
            "[MoveRobot] 闭环参数: axis=%s target=%.4f speed=%.4f tolerance=%.4f "
            "timeout=%.2fs refresh_interval=%.3fs settle_time=%.3fs",
            axis, target_value, speed, tolerance, timeout, refresh_interval, settle_time)

        initial = _record_initial_localization(_read_localization(sdk))
        if initial is None:
            logger.warning("[MoveRobot] 初始定位不可用，进入积分模式兜底")

        start = time.time()
        last_refresh = 0.0
        last_update = start
        current = 0.0
        refresh_count = 0
        poll_count = 0
        handle = None

        try:
            while time.time() - start < timeout:
                poll_count += 1
                now = time.time()

                if handle is None or now - last_refresh >= refresh_interval:
                    handle = sdk.woosh_move.move_robot(
                        axis=axis, target_value=target_value, speed=speed)
                    last_refresh = now
                    refresh_count += 1

                loc_value = _get_localization_value(initial, _read_localization(sdk), axis)

                if loc_value != INVALID_LOCALIZATION:
                    final_value = float(loc_value)
                else:
                    current += _integrate_feedback(sdk, axis, max(now - last_update, 0.0))
                    last_update = now
                    final_value = current

                # 只比较幅值：测量值符号与指令符号相反（见模块 docstring），
                # 直接做差会导致反向永远收敛不了，故与 Atom 参考实现保持一致。
                if abs(abs(final_value) - abs(target_value)) <= tolerance:
                    logger.warning(
                        "[MoveRobot] 到位成功: axis=%s target=%.4f final=%.4f tolerance=%.4f "
                        "polls=%d refreshes=%d elapsed=%.3fs",
                        axis, target_value, final_value, tolerance,
                        poll_count, refresh_count, now - start)
                    return handle

                time.sleep(min(refresh_interval, 0.05))
        except RuntimeError as exc:
            # 运动期间被紧急停止（如 StopTask 置位）或其他底层运行时错误：
            # 带上真实原因抛出，避免与"超时"混淆；finally 仍会停止底盘。
            logger.error("[MoveRobot] 相对运动中断: %s", exc)
            raise RuntimeError(f"{axis}轴相对运动中断：{exc}") from exc
        finally:
            logger.warning(
                "[MoveRobot] 停止底盘: axis=%s settle_time=%.3fs refreshes=%d polls=%d",
                axis, settle_time, refresh_count, poll_count)
            sdk.woosh_move.stop()
            time.sleep(settle_time)

        logger.warning(
            "[MoveRobot] 相对运动超时: axis=%s target=%.4f tolerance=%.4f timeout=%.2fs elapsed=%.3fs",
            axis, target_value, tolerance, timeout, time.time() - start)
        raise RuntimeError(
            f"{axis}轴相对运动超时（{timeout}s）：目标 {target_value}，容差 {tolerance}"
        )


class StopTask(AbilityTask):
    """主动停止（taskType 1）：置位紧急停止并连续发布零速停止底盘运动。"""

    name = "stop"
    summary = "停止底盘运动（紧急停止 + 零速发布）"
    task_type = 1

    def execute(self, input_data: Mapping[str, Any]) -> CommandHandle:
        logger.warning("[StopRobot] 收到主动停止请求: keys=%s", sorted(input_data.keys()))

        handle = self.sdk.woosh_move.stop()
        # SDK stop() 只发零速、不置位标志；这里补齐以阻止后续 move_robot（对齐 Atom stop 语义，
        # 且与 Atom 一致：置位后需重启进程/重新初始化才能再次运动）
        self.sdk.woosh_move._emergency_stop = True

        logger.warning(
            "[StopRobot] 底盘停止指令已发布: command_id=%s handle_type=%s",
            getattr(handle, "command_id", None), type(handle).__name__)
        return handle
