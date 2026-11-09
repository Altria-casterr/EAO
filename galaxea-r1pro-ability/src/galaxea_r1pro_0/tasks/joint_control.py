"""Joint_Control 业务 Task：手臂关节控制（借鉴 Atom Joint_Control）。

单臂/双臂：发布目标 → 轮询 sdk.sensors.get_joint_state 直到 check_reached 通过或超时。
每个 Task 同时支持两种入参（与 gripper 一致，按 semantic_flag 区分）：
- 直接调用：arm_side/positions/joint_names（单臂），left/right_*（双臂）；
- semantic 调用（semantic motion.move_to_posture）：
  * 单臂（motion.move_to_posture_single）：{"semantic_flag": true, "position_m": [7], "arm_side": "left"|"right"}
  * 双臂（motion.move_to_posture_both）：{"semantic_flag": true, "left_positions": [7], "right_positions": [7]}
  position_m 是手臂 7 关节目标角度（弧度），顺序固定为 joint1..joint7。
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

def _joint_state(sdk, arm_side: str) -> list | None:
    """读取手臂关节实际位置（无反馈时返回 None）。"""

    result = sdk.sensors.get_joint_state(arm_side)
    data = (result.output or {}).get("data")
    return data if isinstance(data, list) else None


class ControlSingleArmJointTask(AbilityTask):
    """单臂关节控制（taskType 0），含反馈闭环。

    semantic 调用（semantic_flag=True）：position_m 为手臂 7 关节角度数组，
    臂侧由 arm_side 指定（默认 left）。joint_names 按 arm_side 自动补齐。
    """

    name = "control_single_arm_joint"
    summary = "控制单臂关节移动到目标位置（含反馈闭环）"
    task_type = 0

    def execute(self, input_data: Mapping[str, Any]) -> CommandHandle:
        sdk = self.sdk

        arm_side = str(input_data.get("arm_side", "left"))
        if arm_side not in ("left", "right"):
            raise ValueError(f"无效的手臂侧: {arm_side!r}")

        positions = [float(value) for value in input_data.get("positions", [])]
        joint_names = list(input_data.get("joint_names", []))
        if not positions or not joint_names or len(positions) != len(joint_names):
            raise ValueError("positions 与 joint_names 必须非空且等长")

        velocities = input_data.get("velocities", None)
        effort = input_data.get("effort", None)

        tol = float(input_data.get("tol", 0.01))
        timeout = float(input_data.get("timeout", 10.0))
        loop_freq = float(input_data.get("loop_freq", 0.2))

        # ① 发布目标指令
        handle = sdk.joint.control_single_arm_joint(
            arm_side=arm_side,
            positions=positions,
            joint_names=joint_names,
            velocities=velocities,
            effort=effort,
        )
        logger.info("单臂关节目标发布: %s name=%s position=%s", arm_side, joint_names, positions)

        # ② 反馈闭环：轮询实际关节位置直到 check_reached 通过或超时
        start = time.time()
        while time.time() - start < timeout:
            current = _joint_state(sdk, arm_side)
            if current is not None and check_reached(
                current[: len(positions)],
                positions,
                [tol] * len(positions),
                description=f"{arm_side}单臂关节到位",
            ):
                return handle
            time.sleep(loop_freq)

        raise RuntimeError(f"单臂关节控制超时（{timeout}s）：目标 position={positions}")


class ControlBothArmsJointTask(AbilityTask):
    """双臂关节控制（taskType 1），左右并行发布并做双向反馈闭环。

    semantic 调用（semantic_flag=True）：left_positions/right_positions 各为
    7 关节角度数组（joint1..joint7），joint_names 按两侧自动补齐。
    """

    name = "control_both_arms_joint"
    summary = "控制双臂关节同时移动到目标位置（含反馈闭环）"
    task_type = 1

    def execute(self, input_data: Mapping[str, Any]) -> CommandHandle:
        sdk = self.sdk
        
        left_positions = [float(value) for value in input_data.get("left_positions", [])]
        right_positions = [float(value) for value in input_data.get("right_positions", [])]
        left_joint_names = list(input_data.get("left_joint_names", []))
        right_joint_names = list(input_data.get("right_joint_names", []))
        if (not left_positions or not right_positions or len(left_positions) != len(left_joint_names)
                or len(right_positions) != len(right_joint_names)):
                raise ValueError("双臂 positions 与 joint_names 必须非空且等长")

        left_velocities = input_data.get("left_velocities", None)
        right_velocities = input_data.get("right_velocities", None)
        left_effort = input_data.get("left_effort", None)
        right_effort = input_data.get("right_effort", None)

        tol = float(input_data.get("tol", 0.01))
        timeout = float(input_data.get("timeout", 10.0))
        loop_freq = float(input_data.get("loop_freq", 5.0))

        # ① 左右并行发布目标指令
        handles = sdk.joint.control_both_arms_joint(
            left_positions=left_positions,
            right_positions=right_positions,
            left_joint_names=left_joint_names,
            right_joint_names=right_joint_names,
            left_velocities=left_velocities,
            right_velocities=right_velocities,
            left_effort=left_effort,
            right_effort=right_effort,
        )
        logger.info(
            "双臂关节目标发布: left=%s right=%s",
            left_positions,
            right_positions,
        )

        # ② 双向反馈闭环：两侧都 check_reached 通过才算成功
        n_joints = len(left_positions) + len(right_positions)
        start = time.time()
        while time.time() - start < timeout:
            left = _joint_state(sdk, "left")
            right = _joint_state(sdk, "right")
            if left is not None and right is not None and check_reached(
                left[: len(left_positions)] + right[: len(right_positions)],
                left_positions + right_positions,
                [tol] * n_joints,
                description="双臂关节到位",
            ):
                return handles[0]
            time.sleep(loop_freq)

        raise RuntimeError(
            f"双臂关节控制超时（{timeout}s）：left目标={left_positions} right目标={right_positions}"
        )
