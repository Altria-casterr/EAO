"""EEF Pose 业务 Task：末端执行器位姿控制（借鉴 Atom Joint_EEFPose_Control Galaxea_R1Pro_S2）。

私有辅助函数（位姿读取、浮动基座转换、目标构造、到位检查、参数解析）
统一放在 utils/utils_JointEEFPoseControl.py；发布 + 闭环由 Task 用
_target_pose / _arm_reached 自行组合。
物理命令返回 CommandHandle，由 R1ProAbilityService 登记执行记录并输出；
参数错误/超时直接抛异常，不输出 success/failure 字典。
"""

from __future__ import annotations

import logging
import time
from typing import Any, Mapping

from galaxea_r1pro import CommandHandle

from ..utils.utils_JointEEFPoseControl import (
    _arm_reached,
    _require_pose_values,
    _target_pose,
    current_pose_in_frame,
    force_align_arm,
    calculate_cartesian_num_points,
    generate_cartesian_trajectory,

)
from .base import AbilityTask

logger = logging.getLogger(__name__)

class ControlSingleArmTask(AbilityTask):
    """单臂末端位姿控制（taskType 0），含反馈闭环与坐标系转换。"""

    name = "control_single_arm"
    summary = "控制单臂末端移动到目标位姿（含反馈闭环与坐标系转换）"
    task_type = 0

    def execute(self, input_data: Mapping[str, Any]) -> CommandHandle:
        sdk = self.sdk

        axis = str(input_data.get("axis", "left"))
        if axis not in ("left", "right"):
            raise ValueError(f"无效的轴: {axis!r}")
        
        pose = _require_pose_values(input_data, "")
        frame_id = str(input_data.get("frame_id", "torso_link4"))
        pos_tol = float(input_data.get("pos_tol", 0.05))
        orient_tol = float(input_data.get("orient_tol", 0.01))
        timeout = float(input_data.get("timeout", 10.0))
        loop_freq = float(input_data.get("loop_freq", 1.0))

        # ① 发布目标位姿
        handle = sdk.eef.control_single_arm(
            x=pose["target_x"], y=pose["target_y"], z=pose["target_z"],
            roll=pose["roll"], pitch=pose["pitch"], yaw=pose["yaw"],
            frame_id=frame_id, axis=axis,
        )
        logger.info(
            "%s臂末端目标发布: pos=(%.3f,%.3f,%.3f) rpy=(%.1f,%.1f,%.1f) frame=%s",
            axis, pose["target_x"], pose["target_y"], pose["target_z"],
            pose["roll"], pose["pitch"], pose["yaw"], frame_id,
        )
        target = _target_pose(pose["target_x"], pose["target_y"], pose["target_z"], pose["roll"], pose["pitch"], pose["yaw"])

        # ② 反馈闭环：轮询当前位姿直到到位或超时；每轮持续重发目标位姿，
        start = time.time()
        while time.time() - start < timeout:
            if _arm_reached(sdk, axis, target, frame_id, pos_tol, orient_tol):
                return handle
            time.sleep(loop_freq)
            handle = sdk.eef.control_single_arm(
                x=pose["target_x"], y=pose["target_y"], z=pose["target_z"],
                roll=pose["roll"], pitch=pose["pitch"], yaw=pose["yaw"],
                frame_id=frame_id, axis=axis,
            )

        raise RuntimeError(
            f"{axis}臂末端控制超时（{timeout}s）：目标 pos=({pose['target_x']:.3f},{pose['target_y']:.3f},{pose['target_z']:.3f}) frame={frame_id}"
        )

class ControlBothArmTask(AbilityTask):
    """双臂末端位姿控制（taskType 1），左右并行发布并做双向反馈闭环。"""

    name = "control_both_arm"
    summary = "控制双臂末端同时移动到目标位姿（含反馈闭环与坐标系转换）"
    task_type = 1

    def execute(self, input_data: Mapping[str, Any]) -> CommandHandle:
        sdk = self.sdk

        left = _require_pose_values(input_data, "left")
        right = _require_pose_values(input_data, "right")
        left_frame_id = str(input_data.get("left_frame_id", "1"))
        right_frame_id = str(input_data.get("right_frame_id", "1"))
        left_pos_tol = float(input_data.get("left_pos_tol", 0.05))
        left_orient_tol = float(input_data.get("left_orient_tol", 0.01))
        right_pos_tol = float(input_data.get("right_pos_tol", 0.05))
        right_orient_tol = float(input_data.get("right_orient_tol", 0.01))
        timeout = float(input_data.get("timeout", 10.0))
        loop_freq = float(input_data.get("loop_freq", 1.0))

        # ① 左右并行发布目标位姿
        handles = sdk.eef.control_both_arm(
            left_x=left["target_x"], left_y=left["target_y"], left_z=left["target_z"],
            left_roll=left["roll"], left_pitch=left["pitch"], left_yaw=left["yaw"],
            right_x=right["target_x"], right_y=right["target_y"], right_z=right["target_z"],
            right_roll=right["roll"], right_pitch=right["pitch"], right_yaw=right["yaw"],
            left_frame_id=left_frame_id, right_frame_id=right_frame_id,
        )
        logger.info("双臂末端目标发布: left=%s right=%s", left, right)

        left_target = _target_pose(left["target_x"], left["target_y"], left["target_z"], left["roll"], left["pitch"], left["yaw"])
        right_target = _target_pose(right["target_x"], right["target_y"], right["target_z"], right["roll"], right["pitch"], right["yaw"])

        # ② 双向反馈闭环：两侧都到位才算成功；每轮持续重发双臂目标位姿，
        #    避免单次发布丢失导致某侧机械臂不动（对齐旧工程）。
        start = time.time()
        while time.time() - start < timeout:
            left_ok = _arm_reached(sdk, "left", left_target, left_frame_id, left_pos_tol, left_orient_tol)
            right_ok = _arm_reached(sdk, "right", right_target, right_frame_id, right_pos_tol, right_orient_tol)
            if left_ok and right_ok:
                return handles[0]
            time.sleep(loop_freq)
            handles = sdk.eef.control_both_arm(
                left_x=left["target_x"], left_y=left["target_y"], left_z=left["target_z"],
                left_roll=left["roll"], left_pitch=left["pitch"], left_yaw=left["yaw"],
                right_x=right["target_x"], right_y=right["target_y"], right_z=right["target_z"],
                right_roll=right["roll"], right_pitch=right["pitch"], right_yaw=right["yaw"],
                left_frame_id=left_frame_id, right_frame_id=right_frame_id,
            )

        raise RuntimeError(f"双臂末端控制超时（{timeout}s）：left={left} right={right}")

class ForceAlignTask(AbilityTask):
    """力反馈精调（manipulation.force_align，借鉴 Atom move_arm_with_force_feedback）"""

    name = "force_align"
    summary = "单臂力反馈精调：沿指定轴步进逼近，监测力变化触阈即停（同步闭环）"
    task_type = 2

    def execute(self, input_data: Mapping[str, Any]) -> CommandHandle:
        sdk = self.sdk

        axis = str(input_data.get("axis", "left"))
        if axis not in ("left", "right"):
            raise ValueError("axis 必须为 left/right")

        move_coordinate = str(input_data.get("move_coordinate", "y")).strip()
        if move_coordinate not in ("x", "y", "z"):
            raise ValueError("move_coordinate 必须为 x/y/z")

        raw_force_axes = input_data.get("force_coordinate") or ("x", "y", "z")

        force_coordinates = tuple(str(value).strip().lower() for value in raw_force_axes)
        if not force_coordinates or any(value not in ("x", "y", "z") for value in force_coordinates):
            raise ValueError("force_coordinate 必须是 x/y/z 的非空集合")
        
        if input_data.get("force_threshold_n", None) is None:
            raise ValueError("force_threshold_n 必须指定")
        force_threshold = float(input_data.get("force_threshold_n"))

        direction = str(input_data.get("direction", "negative")).strip()
        if direction not in ("negative", "positive"):
            raise ValueError("direction 必须为 negative/positive")
        step_size = float(input_data.get("step_size_m", 0.01))
        loop_freq = float(input_data.get("loop_freq_s", 0.1))
        timeout = float(input_data.get("timeout_s", 10.0))

        return force_align_arm(
            sdk, axis,
            move_coordinate=move_coordinate, force_coordinates=force_coordinates,
            force_threshold=force_threshold, step_size=step_size, direction=direction,
            loop_freq=loop_freq, timeout=timeout,
        )

class ControlSingleArmTrajectoryTask(AbilityTask):
    """单臂末端笛卡尔轨迹控制（taskType 3），逐点发布并反馈到位。"""

    name = "control_single_arm_trajectory"
    summary = "按笛卡尔五次多项式轨迹逐点控制单臂（含反馈闭环）"
    task_type = 3

    def execute(self, input_data: Mapping[str, Any]) -> CommandHandle:

        sdk = self.sdk
        axis = str(input_data.get("axis", "left"))
        if axis not in ("left", "right"):
            raise ValueError(f"无效的轴: {axis!r}")

        target_pose = _require_pose_values(input_data, "")
        frame_id = str(input_data.get("frame_id", "torso_link4"))
        num_points = int(input_data.get("num_points", 100))
        speed = float(input_data.get("speed", 0.05))
        axis_order = str(input_data.get("axis_order", "zyx"))
        sequential = bool(input_data.get("sequential", True))
        step_distance = float(input_data.get("step_distance", 0.01))
        max_num_points = int(input_data.get("max_num_points", 100))
        pos_tol = float(input_data.get("pos_tol", 0.05))
        orient_tol = float(input_data.get("orient_tol", 0.01))
        timeout = float(input_data.get("timeout", 10.0))
        loop_freq = float(input_data.get("loop_freq", 1.0))

        # 起点必须换算到与目标相同的坐标系，否则插值路径会整体偏移
        start_pose = current_pose_in_frame(sdk, axis, frame_id)
        if start_pose is None:
            raise RuntimeError(f"{axis}臂轨迹控制启动失败：无法读取当前末端位姿")

        trajectory = generate_cartesian_trajectory(
            start_pose, target_pose,
            num_points=num_points, speed=speed,
            axis_order=axis_order, sequential=sequential,
            step_distance=step_distance, max_num_points=max_num_points,
        )
        logger.info("%s臂笛卡尔轨迹生成：%d 个点", axis, len(trajectory))

        handle: CommandHandle | None = None
        for idx, point in enumerate(trajectory):
            # ① 发布当前轨迹点
            handle = sdk.eef.control_single_arm(
                x=point["x"], y=point["y"], z=point["z"],
                roll=point["roll"], pitch=point["pitch"], yaw=point["yaw"],
                frame_id=frame_id, axis=axis,
            )
            logger.info("%s臂轨迹点 %d/%d 已发布: %s", axis, idx + 1, len(trajectory), point)
            target = _target_pose(point["x"], point["y"], point["z"], point["roll"], point["pitch"], point["yaw"])

            # ② 反馈：等待该点到位；每轮持续重发该轨迹点
            start = time.time()
            reached = False
            while time.time() - start < timeout:
                if _arm_reached(sdk, axis, target, frame_id, pos_tol, orient_tol):
                    reached = True
                    break
                time.sleep(loop_freq)
                handle = sdk.eef.control_single_arm(
                    x=point["x"], y=point["y"], z=point["z"],
                    roll=point["roll"], pitch=point["pitch"], yaw=point["yaw"],
                    frame_id=frame_id, axis=axis,
                )

        if not reached:
            raise RuntimeError(f"{axis}臂轨迹执行失败，停止在点 {idx + 1}（超时 {timeout}s）")

        return handle  # 主记录：最后一个轨迹点的 CommandHandle


class ControlBothArmsTrajectoryTask(AbilityTask):
    """双臂末端笛卡尔轨迹控制（taskType 4），左右同步逐点发布并反馈到位（借鉴 Atom control_both_arms_trajectory）。"""

    name = "control_both_arms_trajectory"
    summary = "按笛卡尔五次多项式轨迹同步控制双臂（含反馈闭环）"
    task_type = 4

    def execute(self, input_data: Mapping[str, Any]) -> CommandHandle:
        sdk = self.sdk
        left_target = _require_pose_values(input_data, "left")
        right_target = _require_pose_values(input_data, "right")
        left_frame_id = str(input_data.get("left_frame_id", "1"))
        right_frame_id = str(input_data.get("right_frame_id", "1"))
        left_axis_order = str(input_data.get("left_axis_order", "zyx"))
        left_sequential = bool(input_data.get("left_sequential", True))
        right_axis_order = str(input_data.get("right_axis_order", "zyx"))
        right_sequential = bool(input_data.get("right_sequential", True))
        num_points = input_data.get("num_points")
        num_points = int(num_points) if num_points is not None else None
        speed = float(input_data.get("speed", 0.05))
        step_distance = float(input_data.get("step_distance", 0.01))
        max_num_points = int(input_data.get("max_num_points", 100))
        left_pos_tol = float(input_data.get("left_pos_tol", 0.05))
        left_orient_tol = float(input_data.get("left_orient_tol", 0.01))
        right_pos_tol = float(input_data.get("right_pos_tol", 0.05))
        right_orient_tol = float(input_data.get("right_orient_tol", 0.01))
        timeout = float(input_data.get("timeout", 10.0))
        loop_freq = float(input_data.get("loop_freq", 1.0))

        # 起点必须换算到与目标相同的坐标系（frame 名不一致时 _current_pose 的原始值会造成整条路径偏移）
        left_start = current_pose_in_frame(sdk, "left", left_frame_id)
        right_start = current_pose_in_frame(sdk, "right", right_frame_id)
        if left_start is None or right_start is None:
            raise RuntimeError("双臂轨迹控制启动失败：无法读取当前末端位姿")
        
        # 统一点数，保证左右同步（借鉴 Atom control_both_arms_trajectory）
        if num_points is None:
            left_num = calculate_cartesian_num_points(left_start, left_target, step_distance, max_num_points)
            right_num = calculate_cartesian_num_points(right_start, right_target, step_distance, max_num_points)
            num_points = max(left_num, right_num)
            logger.info("双臂同步轨迹统一点数：%d（左臂建议%d，右臂建议%d）", num_points, left_num, right_num)

        left_traj = generate_cartesian_trajectory(
            left_start, left_target,
            num_points=num_points, speed=speed,
            axis_order=left_axis_order, sequential=left_sequential,
            step_distance=step_distance, max_num_points=max_num_points,
        )
        right_traj = generate_cartesian_trajectory(
            right_start, right_target,
            num_points=num_points, speed=speed,
            axis_order=right_axis_order, sequential=right_sequential,
            step_distance=step_distance, max_num_points=max_num_points,
        )
        logger.info(
            "双臂笛卡尔轨迹生成：%d 个点；左臂 起点=%s 终点=%s；右臂 起点=%s 终点=%s",
            len(left_traj), left_traj[0], left_traj[-1], right_traj[0], right_traj[-1],
        )

        last_handles: list[CommandHandle] | None = None
        for idx, (l_pose, r_pose) in enumerate(zip(left_traj, right_traj)):
            # ① 左右并行发布当前轨迹点
            handles = sdk.eef.control_both_arm(
                left_x=l_pose["x"], left_y=l_pose["y"], left_z=l_pose["z"],
                left_roll=l_pose["roll"], left_pitch=l_pose["pitch"], left_yaw=l_pose["yaw"],
                right_x=r_pose["x"], right_y=r_pose["y"], right_z=r_pose["z"],
                right_roll=r_pose["roll"], right_pitch=r_pose["pitch"], right_yaw=r_pose["yaw"],
                left_frame_id=left_frame_id, right_frame_id=right_frame_id,
            )
            logger.info(
                "双臂轨迹点 %d/%d 已发布: left=%s right=%s",
                idx + 1, len(left_traj), l_pose, r_pose,
            )
            left_target_pose = _target_pose(
                l_pose["x"], l_pose["y"], l_pose["z"],
                l_pose["roll"], l_pose["pitch"], l_pose["yaw"],
            )
            right_target_pose = _target_pose(
                r_pose["x"], r_pose["y"], r_pose["z"],
                r_pose["roll"], r_pose["pitch"], r_pose["yaw"],
            )

            # ② 反馈：等待该点左右都到位；每轮持续重发该轨迹点
            start = time.time()
            reached = False
            while time.time() - start < timeout:
                left_ok = _arm_reached(sdk, "left", left_target_pose, left_frame_id, left_pos_tol, left_orient_tol)
                right_ok = _arm_reached(sdk, "right", right_target_pose, right_frame_id, right_pos_tol, right_orient_tol)
                if left_ok and right_ok:
                    reached = True
                    break
                time.sleep(loop_freq)
                handles = sdk.eef.control_both_arm(
                    left_x=l_pose["x"], left_y=l_pose["y"], left_z=l_pose["z"],
                    left_roll=l_pose["roll"], left_pitch=l_pose["pitch"], left_yaw=l_pose["yaw"],
                    right_x=r_pose["x"], right_y=r_pose["y"], right_z=r_pose["z"],
                    right_roll=r_pose["roll"], right_pitch=r_pose["pitch"], right_yaw=r_pose["yaw"],
                    left_frame_id=left_frame_id, right_frame_id=right_frame_id,
                )

        if not reached:
            raise RuntimeError(f"双臂轨迹执行失败，停止在点 {idx + 1}（超时 {timeout}s）")

        last_handles = handles

        return last_handles[0]  # 主记录：最后一点的左臂 CommandHandle







