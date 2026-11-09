"""EndEffector 业务 Task：夹爪控制（借鉴 Mock-Gripper + Atom Gripper_Control）。

Task 只依赖 self.sdk；解锁刹车 / 断连自愈由 Robot SDK 内部处理。
参数对齐 Atom：position/speed/effort（底层）+ tol/timeout/loop_period（反馈闭环）。
流程：发布目标 → 轮询 sdk.sensors.get_gripper_state 直到 check_reached 通过或超时。
物理命令返回 CommandHandle，由 R1ProAbilityService 登记执行记录并输出；
参数错误/超时直接抛异常，不再输出 success/failure 字典。
"""

from __future__ import annotations

import logging
import time
from typing import Any, Mapping

from galaxea_r1pro import CommandHandle

from ..utils.utils_common import check_reached
from .base import AbilityTask

logger = logging.getLogger(__name__)

# 真机 tote_clamp 全行程（米），用于把 semantic 米制开度换算为百分制。
# 这是真机标定点：不同工具/行程需现场确认后调整。
TRAVEL_M = 0.039

_TOOL_SIDE = {"left": "left", "right": "right"}


def _tool_side(tool_ref: str) -> str:
    """semantic tool_ref（component://tool/left）→ 夹爪侧。"""

    side = str(tool_ref).rsplit("/", 1)[-1].lower()
    if side not in _TOOL_SIDE:
        logger.warning(
            "[_tool_side] 无法从 tool_ref 解析夹爪侧: tool_ref=%r, 解析值=%r, 合法值=%s",
            tool_ref,
            side,
            list(_TOOL_SIDE.keys()),
        )
        raise ValueError(f"无法从 tool_ref 解析夹爪侧：{tool_ref}")
    logger.warning("[_tool_side] 解析夹爪侧成功: tool_ref=%r -> side=%s", tool_ref, side)
    return side


def _gripper_state(sdk, hand: str) -> float | None:
    """读取夹爪实际开度（无反馈时返回 None）。"""

    logger.warning("[_gripper_state] 读取夹爪实际开度: hand=%s", hand)
    result = sdk.sensors.get_gripper_state(hand)
    data = (result.output or {}).get("data")
    if data is None:
        logger.warning(
            "[_gripper_state] 夹爪反馈数据为空: hand=%s, result.output=%s",
            hand,
            result.output,
        )
    else:
        logger.warning("[_gripper_state] 夹爪反馈读取成功: hand=%s, current=%.3f", hand, float(data))
    return data


class HandleSingleGripperTask(AbilityTask):
    """单手夹爪控制（taskType 0），含反馈闭环。"""

    name = "HandleSingleGripper"
    summary = "控制单侧夹爪移动到目标开度（含反馈闭环）"
    task_type = 0

    def execute(self, input_data: Mapping[str, Any]) -> CommandHandle:

        sdk = self.sdk
        hand = str(input_data.get("hand", "left"))
        position = float(input_data.get("position", 50.0))
        speed = float(input_data.get("speed", 0.5))
        effort = float(input_data.get("effort", 0.3))
        tol = float(input_data.get("tol", 5.0))
        timeout = float(input_data.get("timeout", 10.0))
        loop_period = float(input_data.get("loop_period", 0.1))

        logger.warning(
            "[HandleSingleGripper] 参数解析完成: hand=%s, position=%.3f, speed=%.3f, effort=%.3f, tol=%.3f, timeout=%.3fs, loop_period=%.3fs",
            hand,
            position,
            speed,
            effort,
            tol,
            timeout,
            loop_period,
        )

        if hand not in ("left", "right"):
            logger.warning("[HandleSingleGripper] 无效的夹爪侧参数: hand=%r", hand)
            raise ValueError(f"无效的夹爪侧: {hand!r}")

        # ① 发布目标指令
        logger.warning(
            "[HandleSingleGripper] 开始发布单手夹爪目标指令: hand=%s position=%.3f speed=%.3f effort=%.3f",
            hand,
            position,
            speed,
            effort,
        )
        handle = sdk.gripper.control_single_gripper(
            arm_side=hand,
            position=position,
            speed=speed,
            effort=effort,
        )
        logger.warning(
            "[HandleSingleGripper] 单手夹爪目标指令发布完成: hand=%s, command_id=%s",
            hand,
            getattr(handle, "command_id", None),
        )

        # ② 反馈闭环：轮询实际开度直到 check_reached 通过或超时
        start = time.time()
        poll_count = 0
        last_reported_current = None
        while time.time() - start < timeout:
            poll_count += 1
            current = _gripper_state(sdk, hand)

            if current is None:
                elapsed = time.time() - start
                remaining = timeout - elapsed
                if poll_count == 1 or poll_count % 5 == 0:
                    logger.warning(
                        "[HandleSingleGripper] 第 %d 次轮询: hand=%s 反馈仍为空, elapsed=%.2fs, remaining≈%.2fs",
                        poll_count,
                        hand,
                        elapsed,
                        max(0.0, remaining),
                    )
                time.sleep(loop_period)
                continue

            if last_reported_current != current and (
                poll_count == 1 or poll_count % 3 == 0
            ):
                diff = abs(float(current) - position)
                elapsed = time.time() - start
                logger.warning(
                    "[HandleSingleGripper] 第 %d 次轮询: hand=%s current=%.3f target=%.3f diff=%.3f tol=%.3f elapsed=%.2fs",
                    poll_count,
                    hand,
                    float(current),
                    position,
                    diff,
                    tol,
                    elapsed,
                )
                last_reported_current = current

            if check_reached(
                [current], [position], [tol], description=f"{hand}夹爪到位"
            ):
                elapsed = time.time() - start
                logger.warning(
                    "[HandleSingleGripper] 夹爪到位成功: hand=%s final_current=%.3f target=%.3f polls=%d elapsed=%.2fs",
                    hand,
                    float(current),
                    position,
                    poll_count,
                    elapsed,
                )
                return handle

            time.sleep(loop_period)

        elapsed_total = time.time() - start
        logger.warning(
            "[HandleSingleGripper] 单手夹爪控制超时: hand=%s target_position=%.3f tol=%.3f timeout=%.3fs elapsed=%.2fs total_polls=%d",
            hand,
            position,
            tol,
            timeout,
            elapsed_total,
            poll_count,
        )
        raise RuntimeError(f"单手夹爪控制超时（{timeout}s）：目标 position={position}")


class HandleBothGripperTask(AbilityTask):
    """双手夹爪控制（taskType 1），左右并行发布并做双向反馈闭环。"""

    name = "HandleBothGripper"
    summary = "控制左右夹爪同时移动到目标开度（含反馈闭环）"
    task_type = 1

    def execute(self, input_data: Mapping[str, Any]) -> CommandHandle:
        sdk = self.sdk

        hand = str(input_data.get("hand", "both"))
        left_position = float(input_data.get("left_position", 50.0))
        right_position = float(input_data.get("right_position", 50.0))
        left_speed = float(input_data.get("left_speed", 0.5))
        right_speed = float(input_data.get("right_speed", 0.5))
        left_effort = float(input_data.get("left_effort", 0.3))
        right_effort = float(input_data.get("right_effort", 0.3))
        left_tol = right_tol = float(input_data.get("tol", 5.0))
        left_timeout = right_timeout = float(input_data.get("timeout", 10.0))
        left_loop_period = right_loop_period = float(input_data.get("loop_period", 0.1))

        logger.warning(
            "[HandleBothGripper] 参数解析完成: hand=%s, "
            "left=(pos=%.3f,spd=%.3f,eff=%.3f) right=(pos=%.3f,spd=%.3f,eff=%.3f) "
            "tol=%.3f/%.3f timeout=%.3f/%.3fs loop_period=%.3f/%.3fs",
            hand,
            left_position,
            left_speed,
            left_effort,
            right_position,
            right_speed,
            right_effort,
            left_tol,
            right_tol,
            left_timeout,
            right_timeout,
            left_loop_period,
            right_loop_period,
        )

        if hand != "both":
            logger.warning("[HandleBothGripper] 无效的控制模式: hand=%r (仅支持 both)", hand)
            raise ValueError("无效的控制模式，仅支持 both")

        # ① 左右并行发布目标指令
        logger.warning(
            "[HandleBothGripper] 开始并行发布双手夹爪目标: left(pos=%.3f,spd=%.3f,eff=%.3f) right(pos=%.3f,spd=%.3f,eff=%.3f)",
            left_position,
            left_speed,
            left_effort,
            right_position,
            right_speed,
            right_effort,
        )
        handles = sdk.gripper.control_both_gripper(
            left_position=left_position,
            right_position=right_position,
            left_speed=left_speed,
            right_speed=right_speed,
            left_effort=left_effort,
            right_effort=right_effort,
        )
        left_cmd_id = getattr(handles[0], "command_id", None) if handles else None
        right_cmd_id = getattr(handles[1], "command_id", None) if len(handles) > 1 else None
        logger.warning(
            "[HandleBothGripper] 双手夹爪目标指令发布完成: left_command_id=%s right_command_id=%s",
            left_cmd_id,
            right_cmd_id,
        )

        # ② 双向反馈闭环：两侧都 check_reached 通过才算成功
        max_timeout = max(left_timeout, right_timeout)
        poll_period = min(left_loop_period, right_loop_period)
        start = time.time()
        poll_count = 0
        last_reported_pair = None
        while time.time() - start < max_timeout:
            poll_count += 1
            left = _gripper_state(sdk, "left")
            right = _gripper_state(sdk, "right")

            left_none = left is None
            right_none = right is None
            if left_none or right_none:
                elapsed = time.time() - start
                remaining = max_timeout - elapsed
                if poll_count == 1 or poll_count % 5 == 0:
                    logger.warning(
                        "[HandleBothGripper] 第 %d 次轮询: 反馈缺失 left=%s right=%s elapsed=%.2fs remaining≈%.2fs",
                        poll_count,
                        "None" if left_none else f"{float(left):.3f}",
                        "None" if right_none else f"{float(right):.3f}",
                        elapsed,
                        max(0.0, remaining),
                    )
                time.sleep(poll_period)
                continue

            pair_key = (round(float(left), 3), round(float(right), 3))
            if last_reported_pair != pair_key and (
                poll_count == 1 or poll_count % 3 == 0
            ):
                left_diff = abs(float(left) - left_position)
                right_diff = abs(float(right) - right_position)
                elapsed = time.time() - start
                logger.warning(
                    "[HandleBothGripper] 第 %d 次轮询: "
                    "left=(cur=%.3f/tgt=%.3f/diff=%.3f/tol=%.3f) "
                    "right=(cur=%.3f/tgt=%.3f/diff=%.3f/tol=%.3f) elapsed=%.2fs",
                    poll_count,
                    float(left),
                    left_position,
                    left_diff,
                    left_tol,
                    float(right),
                    right_position,
                    right_diff,
                    right_tol,
                    elapsed,
                )
                last_reported_pair = pair_key

            if check_reached(
                [left, right],
                [left_position, right_position],
                [left_tol, right_tol],
                description="双手夹爪到位",
            ):
                elapsed = time.time() - start
                logger.warning(
                    "[HandleBothGripper] 双手夹爪到位成功: "
                    "left_final=%.3f/tgt=%.3f right_final=%.3f/tgt=%.3f polls=%d elapsed=%.2fs",
                    float(left),
                    left_position,
                    float(right),
                    right_position,
                    poll_count,
                    elapsed,
                )
                return handles[0]

            time.sleep(poll_period)

        elapsed_total = time.time() - start
        logger.warning(
            "[HandleBothGripper] 双手夹爪控制超时: max_timeout=%.3fs elapsed=%.2fs total_polls=%d "
            "left_target=%.3f/tol=%.3f right_target=%.3f/tol=%.3f",
            max_timeout,
            elapsed_total,
            poll_count,
            left_position,
            left_tol,
            right_position,
            right_tol,
        )
        raise RuntimeError(
            f"双手夹爪控制超时（{max_timeout}s）：left目标={left_position} right目标={right_position}"
        )
