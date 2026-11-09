"""Torso_Control 业务 Task：躯干四关节控制（借鉴 Atom Torso_Control）。

四关节：发布目标 → 轮询 sdk.sensors.get_torso_positions 直到 check_reached 通过或超时。
单关节：joint_id 从 0 开始（0-3），与底层 SDK 索引一致。
每个 Task 同时支持两种入参（与 joint_control/gripper 一致，按 semantic_flag 区分）：
- 直接调用：positions（四关节）+ 速度/力矩；或 joint_id + target_pos（单关节）
物理命令返回 CommandHandle，由 R1ProAbilityService 登记执行记录并输出；
参数错误/超时直接抛异常，不输出 success/failure 字典。
"""

from __future__ import annotations

import logging
import time
from typing import Any, Mapping

from galaxea_r1pro import CommandHandle

from ..utils.utils_common import check_reached
from .base import AbilityTask

logger = logging.getLogger(__name__)
TORSO_JOINT_COUNT = 4

def _torso_positions(sdk) -> list | None:
    """读取躯干四关节实际位置（无反馈时返回 None）。"""

    result = sdk.sensors.get_torso_positions()
    data = (result.output or {}).get("data")
    return data if isinstance(data, list) else None

class ControlTorsoJointsTask(AbilityTask):
    """躯干四关节同时控制（taskType 0），含反馈闭环。
    """

    name = "control_torso_joints"
    summary = "控制躯干四关节移动到目标位置或命名姿态（含反馈闭环）"
    task_type = 0

    def execute(self, input_data: Mapping[str, Any]) -> CommandHandle:
        sdk = self.sdk

        positions = [float(value) for value in input_data.get("positions", [])]

        speeds = input_data.get("speeds")
        if speeds is not None:
            speeds = [float(value) for value in speeds]

        effort = input_data.get("effort")
        if effort is not None:
            effort = [float(value) for value in effort]
        names = list(input_data.get("names", []))

        tol = float(input_data.get("tol", 0.02))
        timeout = float(input_data.get("timeout", 10.0))
        loop_period = float(input_data.get("loop_period", 0.1))

        # ① 发布目标指令
        handle = sdk.torso.control_torso_joints(
            positions=positions,
            speeds=speeds,
            effort=effort,
            names=names,
        )
        logger.info("躯干关节目标发布: positions=%s speeds=%s", positions, speeds)

        # ② 反馈闭环：轮询实际位置直到 check_reached 通过或超时
        start = time.time()
        while time.time() - start < timeout:
            current = _torso_positions(sdk)
            if current is not None and check_reached(
                current[:len(positions)],
                positions,
                [tol] * len(positions),
                description="躯干关节到位",
            ):
                return handle
            time.sleep(loop_period)

        raise RuntimeError(f"躯干关节控制超时（{timeout}s）：目标 position={positions}")

class ControlSingleTorsoJointTask(AbilityTask):
    """躯干单关节控制（taskType 1），其他关节保持当前位置，含反馈闭环。

    semantic 调用（semantic_flag=True，semantic torso.rotate）：angle_rad（弧度）
    作用于旋转关节（joint_id 固定为 0）。
    """

    name = "control_single_torso_joint"
    summary = "控制躯干单个关节移动到目标位置（含反馈闭环；semantic 支持 torso.rotate 旋转）"
    task_type = 1

    def execute(self, input_data: Mapping[str, Any]) -> CommandHandle:
        sdk = self.sdk

        joint_id = int(input_data.get("joint_id", 0))

        if joint_id < 0 or joint_id >= TORSO_JOINT_COUNT:
            raise ValueError(f"关节 ID 必须为 0-{TORSO_JOINT_COUNT - 1}")

        target_pos = float(input_data.get("target_pos"))
        if target_pos is None:
            raise ValueError("target_pos 必须提供")
        speed = float(input_data.get("speed", 1.5))
        tol = float(input_data.get("tol", 0.02))
        timeout = float(input_data.get("timeout", 8.0))
        loop_period = float(input_data.get("loop_period", 0.1))
        effort = input_data.get("effort")
        if effort is not None:
            effort = [float(value) for value in effort]
        names = list(input_data.get("names", []))

        # ① 发布目标指令（joint_id 从 0 开始，与底层 SDK 索引一致）
        handle = sdk.torso.control_single_torso_joint(
            joint_id=joint_id,
            target_pos=target_pos,
            speed=speed,
            effort=effort,
            names=names,
        )
        logger.info("躯干单关节目标发布: joint_id=%s target_pos=%s speed=%s", joint_id, target_pos, speed)

        # ② 反馈闭环：轮询实际位置直到 check_reached 通过或超时
        start = time.time()
        while time.time() - start < timeout:
            current = _torso_positions(sdk)
            if current is not None and check_reached(
                [current[joint_id]],
                [target_pos],
                [tol],
                description=f"躯干关节{joint_id}到位",
            ):
                return handle
            time.sleep(loop_period)

        raise RuntimeError(f"躯干单关节控制超时（{timeout}s）：joint_id={joint_id} target_pos={target_pos}")
