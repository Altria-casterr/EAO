"""JointEEFPoseControl 专用功能函数层（借鉴 Atom Joint_EEFPose_Control/Utils）。

包含 EEF 位姿工具（check/transform）与 Task 复用的私有辅助函数
（位姿读取、浮动基座、目标构造、到位检查、参数解析）。
轨迹生成与 Atom CartesianPolynomialTrajectoryGeneratorFunc 对齐：
calculate_cartesian_num_points ↔ calculate_num_points，
generate_cartesian_trajectory ↔ generate_cartesian_trajectory（五次多项式）。
"""

from __future__ import annotations

import logging
import math
import time
import copy
from typing import Any, Mapping

import numpy as np
from scipy.spatial.transform import Rotation as R

from galaxea_r1pro_0.utils.utils_common import euler_to_quaternionx

logger = logging.getLogger(__name__)

POSE_KEYS = ("target_x", "target_y", "target_z", "x", "y", "z", "w")

def check_pose_reached(
    current_pose: Mapping[str, Any],
    target_pose: Mapping[str, Any],
    pos_tol: float,
    orient_tol: float,
    description: str = "末端位姿到达") -> bool:
    """判断当前末端位姿是否到达目标（位置欧氏距离 + 四元数姿态角差，借鉴 Atom check_stop）。

    Args:
        current_pose: 当前位姿 {"position": {"x","y","z"}, "orientation": {"x","y","z","w"}}
        target_pose: 目标位姿（同结构）
        pos_tol: 位置容差（米）
        orient_tol: 姿态容差（弧度）
        description: 描述信息（用于日志）

    Returns:
        bool: 是否到达目标位姿
    """

    try:
        pos_current = np.array([
            current_pose["position"]["x"],
            current_pose["position"]["y"],
            current_pose["position"]["z"],
        ], dtype=float)
        pos_target = np.array([
            target_pose["position"]["x"],
            target_pose["position"]["y"],
            target_pose["position"]["z"],
        ], dtype=float)
        quat_current = np.array([
            current_pose["orientation"]["x"],
            current_pose["orientation"]["y"],
            current_pose["orientation"]["z"],
            current_pose["orientation"]["w"],
        ], dtype=float)
        quat_target = np.array([
            target_pose["orientation"]["x"],
            target_pose["orientation"]["y"],
            target_pose["orientation"]["z"],
            target_pose["orientation"]["w"],
        ], dtype=float)
    except (KeyError, TypeError):
        logger.error("%s：位姿数据格式非法（当前=%s 目标=%s）", description, current_pose, target_pose)
        return False

    pos_error = float(np.linalg.norm(pos_current - pos_target))

    norm_current = float(np.linalg.norm(quat_current))
    norm_target = float(np.linalg.norm(quat_target))
    if norm_current == 0.0 or norm_target == 0.0:
        logger.error("%s：四元数模长为 0", description)
        return False
    dot = float(np.clip(np.dot(quat_current / norm_current, quat_target / norm_target), -1.0, 1.0))
    orient_error = 2.0 * math.acos(abs(dot))

    logger.info("%s误差：位置=%.4fm | 姿态=%.4frad", description, pos_error, orient_error)
    reached = pos_error <= pos_tol and orient_error <= orient_tol
    if reached:
        logger.info("%s已到达目标位姿", description)
    return reached

def transform_pose(
    pose: Mapping[str, Any],
    transform: Mapping[str, Any]) -> dict[str, dict[str, float]]:
    """将 pose 按 transform（position + orientation 四元数）变换到目标坐标系（借鉴 Atom transform_pose）。

    Args:
        pose: 源位姿 {"position": {"x","y","z"}, "orientation": {"x","y","z","w"}}
        transform: {"position": (x, y, z), "orientation": (qx, qy, qz, qw)}

    Returns:
        dict: 变换后的位姿（同 pose 结构）
    """

    pos = np.array([pose["position"]["x"], pose["position"]["y"], pose["position"]["z"]], dtype=float)
    quat = np.array([
        pose["orientation"]["x"],
        pose["orientation"]["y"],
        pose["orientation"]["z"],
        pose["orientation"]["w"],
    ], dtype=float)
    t_pos = np.array(transform["position"], dtype=float)
    t_quat = np.array(transform["orientation"], dtype=float)

    rot = R.from_quat(t_quat)
    new_pos = rot.apply(pos) + t_pos
    new_quat = (rot * R.from_quat(quat)).as_quat()

    return {
        "position": {"x": float(new_pos[0]), "y": float(new_pos[1]), "z": float(new_pos[2])},
        "orientation": {
            "x": float(new_quat[0]),
            "y": float(new_quat[1]),
            "z": float(new_quat[2]),
            "w": float(new_quat[3]),
        },
    }

# ==================== Task 复用的私有辅助函数 ====================
def _ee_pose(sdk, axis: str) -> dict | None:
    """读取单臂末端当前位姿（无反馈时返回 None）。"""

    result = sdk.sensors.get_ee_pose(axis)
    data = (result.output or {}).get("data")
    return data if isinstance(data, dict) else None

def _floating_base(sdk) -> dict | None:
    """读取浮动基座逆变换（frame_id 不一致时用于坐标系转换）。"""

    result = sdk.state.get_floating_base()
    data = (result.output or {}).get("data")
    return data if isinstance(data, dict) else None

def _target_pose(x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> dict:
    """按 SDK 相同转换（yxz 顺序，角度制）构造目标位姿。"""

    qx, qy, qz, qw = (float(value) for value in euler_to_quaternionx([roll, pitch, yaw], seq="yxz"))
    return {
        "position": {"x": x, "y": y, "z": z},
        "orientation": {"x": qx, "y": qy, "z": qz, "w": qw},
    }

def _quat_to_rpy_deg(orientation_xyzw: Any) -> tuple[float, float, float]:
    """semantic 四元数(xyzw) → galaxea roll/pitch/yaw(度, YXZ 顺序)。

    galaxea 的 eef_control 用 ``R.from_euler("YXZ", [roll, pitch, yaw], degrees=False)``
    构造四元数；这里取它的逆，得到可直接透传的 roll/pitch/yaw。
    """

    try:
        values = [float(value) for value in orientation_xyzw]

    except (TypeError, ValueError) as exc:
        raise ValueError(f"orientation_xyzw 必须是四元数: {orientation_xyzw}") from exc

    if len(values) != 4:
        raise ValueError(f"orientation_xyzw 必须包含 4 个元素: {orientation_xyzw}")

    roll, pitch, yaw = R.from_quat(values).as_euler("YXZ", degrees=True)
    return float(roll), float(pitch), float(yaw)

def _arm_reached(
    sdk,
    axis: str,
    target_pose: dict,
    frame_id: str,
    pos_tol: float,
    orient_tol: float) -> bool:
    """检查单臂当前位姿是否到位（frame_id 不一致时先做坐标系转换）。"""

    current = _ee_pose(sdk, axis)
    if current is None:
        return False
    current_pose = current.get("pose")
    if not current_pose:
        return False
    if str(current.get("header", {}).get("frame_id", "")).strip() != frame_id:
        transform = _floating_base(sdk)
        if transform is not None:
            current_pose = transform_pose(current_pose, transform)
    return check_pose_reached(current_pose, target_pose, pos_tol, orient_tol, f"{axis}臂末端到位")

def _require_pose_values(input_data: Mapping[str, Any], prefix: str) -> dict[str, float]:
    """从输入中取出 {prefix}x/{prefix}y/... 位姿值；x/y/z 必填，roll/pitch/yaw 默认 0（度）。"""

    pose = input_data.get(f"{prefix}_target_pose") or {}
    if not pose:
        raise ValueError(f"{prefix}_target_pose 必须包含 position 和 orientation")
    
    missing = [key for key in POSE_KEYS if key not in pose]
    if missing:
        raise ValueError(f"{prefix}缺少必填位姿参数: {[key for key in missing]}")

    orientation = [pose.get("x"), pose.get("y"), pose.get("z"), pose.get("w")]
    roll, pitch, yaw = _quat_to_rpy_deg(orientation)

    return{
        "target_x": float(pose.get("target_x")),
        "target_y": float(pose.get("target_y")),
        "target_z": float(pose.get("target_z")),
        "roll": roll,
        "pitch": pitch,
        "yaw": yaw,
    }

# ==================== 力反馈精调（force_align / pull_and_grab 复用） ====================
def _force_data(sdk, axis: str) -> dict | None:
    """读取左右臂力/力矩数据（无反馈时返回 None）。

    数据形如 {"left_arm": {"force_x": .., "force_y": .., "force_z": .., ...},
             "right_arm": {...}}。
    """

    result = sdk.sensors.get_force_data()
    data = (result.output or {}).get("data")
    return data[f"{axis}_arm"] if isinstance(data, dict) and f"{axis}_arm" in data else None

def _current_pose(sdk, axis: str) -> dict | None:

    current = _ee_pose(sdk, axis)
    if current is None:
        return None
    pose = current.get("pose") or {}
    position = pose.get("position") or {}
    orientation = pose.get("orientation") or {}
    try:
        values: dict[str, Any] = {
            "target_x": float(position["x"]),
            "target_y": float(position["y"]),
            "target_z": float(position["z"]),
        }
    except (KeyError, TypeError, ValueError):
        return None
    roll, pitch, yaw = _quat_to_rpy_deg(
        (
            orientation.get("x", 0.0),
            orientation.get("y", 0.0),
            orientation.get("z", 0.0),
            orientation.get("w", 1.0),
        )
    )
    values.update({"roll": roll, "pitch": pitch, "yaw": yaw})
    values["frame_id"] = str(current.get("header", {}).get("frame_id", "")).strip() or "torso_link4"
    return values

def _max_force_change(
    current: Mapping[str, Any],
    baseline: Mapping[str, Any],
    axes: tuple[str, ...]) -> float:
    """监测轴集合上力反馈相对基线的最大绝对变化量（牛）。"""

    change = 0.0
    for axis in axes:
        key = f"force_{axis}"
        try:
            change = max(change, abs(float(current.get(key, 0.0)) - float(baseline.get(key, 0.0))))
        except (TypeError, ValueError):
            continue
    return change

def force_align_arm(
    sdk,
    axis: str,
    *,
    move_coordinate: str,
    force_coordinates: tuple[str, ...],
    force_threshold: float,
    step_size: float,
    direction: str,
    loop_freq: float,
    timeout: float):
    """单臂沿 move_coordinate 轴步进逼近，监测轴力变化达到阈值即停（借鉴 Atom
    move_arm_with_force_feedback）。

    起始位姿取自当前末端反馈，姿态保持不变；每个周期先查力变化、未触阈再步进
    并重发目标。成功返回最后一个 CommandHandle；超时抛 RuntimeError。
    """

    sign = 1.0 if direction == "positive" else -1.0
    move_key = f"target_{move_coordinate}"

    pose = _current_pose(sdk, axis)

    if pose is None:
        raise RuntimeError(f"{axis}臂力反馈精调启动失败：无法读取当前末端位姿")

    frame_id = pose.pop("frame_id")

    def _publish():
        return sdk.eef.control_single_arm(
            x=pose["target_x"], y=pose["target_y"], z=pose["target_z"],
            roll=pose["roll"], pitch=pose["pitch"], yaw=pose["yaw"],
            frame_id=frame_id, axis=axis,
        )

    start = time.time()
    # 先重发当前位姿（保证 handle 有效、真机侧目标不丢失），再采样基线力
    handle = _publish()

    while time.time() - start < timeout:
        force_init = copy.deepcopy(_force_data(sdk, axis))

        if force_init is not None and isinstance(force_init, dict):
            break
        time.sleep(loop_freq)

    if force_init is None:
        raise RuntimeError(f"{axis}臂力反馈精调启动失败：无法读取力反馈数据")

    logger.info(
        "%s臂力反馈精调开始：沿 %s 轴 %s 方向步进 %.3fm，监测 [%s] 轴力变化阈值 %.1fN",
        axis, move_coordinate, direction, step_size,
        ",".join(force_coordinates), force_threshold)
        
    while time.time() - start < timeout:
        force_current = copy.deepcopy(_force_data(sdk, axis))

        if force_current is not None and isinstance(force_current, dict):
            change = _max_force_change(force_current, force_init, force_coordinates)
            if change >= force_threshold:
                logger.info(
                    "%s臂力反馈精调接触停止：力变化 %.2fN ≥ 阈值 %.2fN",
                    axis, change, force_threshold,
                )
                return handle
        pose[move_key] += sign * step_size
        handle = _publish()
        logger.info("%s臂 %s 轴步进至 %.3f", axis, move_coordinate, pose[move_key])
        time.sleep(loop_freq)

    raise RuntimeError(
        f"{axis}臂力反馈精调超时（{timeout}s）：沿 {move_coordinate} 轴 {direction} 步进未接触"
    )

#-----------------------------------轨迹插值算法------------------------------------------------------------
def _fifth_order_polynomial(s: float, s0: float, s1: float) -> float:
    """五次多项式插值核心公式（对齐 Atom CartesianPolynomialTrajectoryGeneratorFunc.fifth_order_polynomial）。

    a0=s0, a1=0, a2=0, a3=10(s1-s0), a4=-15(s1-s0), a5=6(s1-s0)。
    """

    return s0 + (s1 - s0) * (10.0 * s**3 - 15.0 * s**4 + 6.0 * s**5)


def calculate_cartesian_num_points(
    start_pose: Mapping[str, Any],
    target_pose: Mapping[str, Any],
    step_distance: float = 0.01,
    max_num_points: int = 100,
) -> int:
    """自动计算笛卡尔轨迹点数（对齐 Atom CartesianPolynomialTrajectoryGeneratorFunc.calculate_num_points）。

    Args:
        start_pose: 起始位姿 {"target_x","target_y","target_z",...}
        target_pose: 目标位姿（同结构）
        step_distance: 步进距离（米），<=0 时按 0.01 处理
        max_num_points: 点数上限

    Returns:
        int: 轨迹点数（下限 5，上限 max_num_points）
    """

    dx = abs(float(target_pose["target_x"]) - float(start_pose["target_x"]))
    dy = abs(float(target_pose["target_y"]) - float(start_pose["target_y"]))
    dz = abs(float(target_pose["target_z"]) - float(start_pose["target_z"]))

    if step_distance <= 0:
        step_distance = 0.01

    nx = max(1, int(np.ceil(dx / step_distance)))
    ny = max(1, int(np.ceil(dy / step_distance)))
    nz = max(1, int(np.ceil(dz / step_distance)))

    num_points = max(nx, ny, nz)
    num_points = max(num_points, 5)
    num_points = min(num_points, max_num_points)

    logger.info(
        "自动计算插值点数：dx=%.3fm, dy=%.3fm, dz=%.3fm → 各轴点数(nx=%d, ny=%d, nz=%d) → 最终点数=%d (间隔=%sm，上限=%d)",
        dx, dy, dz, nx, ny, nz, num_points, step_distance, max_num_points,
    )
    return num_points

def generate_cartesian_trajectory(
    start_pose: Mapping[str, Any],
    target_pose: Mapping[str, Any],
    num_points: int | None = None,
    speed: float = 0.05,
    axis_order: str = "xyz",
    sequential: bool = True,
    step_distance: float = 0.01,
    max_num_points: int = 100,
) -> list[dict[str, float]]:
    """生成笛卡尔五次多项式轨迹（对齐 Atom CartesianPolynomialTrajectoryGeneratorFunc.generate_cartesian_trajectory）。

    Args:
        start_pose: 起始位姿 {"target_x","target_y","target_z","roll","pitch","yaw"}（角度单位：度）
        target_pose: 目标位姿（同结构）
        num_points: 轨迹点数；None 时按 step_distance 自动计算
        speed: 运动速度（保留参数，用于信息日志）
        axis_order: 顺序插值时逐轴执行顺序（如 "xyz" / "zyx"）
        sequential: 是否逐轴顺序插值（False 时六维同步插值）
        step_distance: 自动计算点数时的步进距离（米）
        max_num_points: 点数上限

    Returns:
        list[dict]: 每个轨迹点 {"x","y","z","roll","pitch","yaw"}
    """

    # 入参用 target_x/target_y/target_z 键；内部插值与输出轨迹点统一用 x/y/z
    src_keys = ("target_x", "target_y", "target_z", "roll", "pitch", "yaw")
    keys = ("x", "y", "z", "roll", "pitch", "yaw")
    start = dict(zip(keys, (float(start_pose[key]) for key in src_keys)))
    target = dict(zip(keys, (float(target_pose[key]) for key in src_keys)))

    if num_points is None:
        num_points = calculate_cartesian_num_points(start_pose, target_pose, step_distance, max_num_points)
    if num_points < 5:
        num_points = 5

    s_list = np.linspace(0.0, 1.0, num_points)
    logger.info(
        "生成笛卡尔五次多项式轨迹：点数=%d，速度=%s，轴顺序=%s，顺序插值=%s",
        num_points,
        speed,
        axis_order,
        sequential,
    )

    if not sequential:
        values = {
            key: [_fifth_order_polynomial(s, start[key], target[key]) for s in s_list]
            for key in keys
        }
        return [
            {key: float(values[key][i]) for key in keys}
            for i in range(num_points)
        ]

    axis_order = axis_order.lower()
    time_ranges = {
        axis_order[0]: (0.0, 1.0 / 3.0),
        axis_order[1]: (1.0 / 3.0, 2.0 / 3.0),
        axis_order[2]: (2.0 / 3.0, 1.0),
    }
    current_xyz = {key: start[key] for key in ("x", "y", "z")}
    trajectory: list[dict[str, float]] = []

    for s in s_list:
        pose_xyz = {key: start[key] for key in ("x", "y", "z")}
        for axis_key in axis_order:
            t_start, t_end = time_ranges[axis_key]
            if t_start <= s <= t_end:
                s_axis = (s - t_start) / (t_end - t_start)
                pose_xyz[axis_key] = _fifth_order_polynomial(
                    s_axis, current_xyz[axis_key], target[axis_key]
                )
                current_xyz[axis_key] = pose_xyz[axis_key]
            elif s > t_end:
                pose_xyz[axis_key] = target[axis_key]
                current_xyz[axis_key] = target[axis_key]

        trajectory.append({
            "x": pose_xyz["x"],
            "y": pose_xyz["y"],
            "z": pose_xyz["z"],
            "roll": _fifth_order_polynomial(s, start["roll"], target["roll"]),
            "pitch": _fifth_order_polynomial(s, start["pitch"], target["pitch"]),
            "yaw": _fifth_order_polynomial(s, start["yaw"], target["yaw"]),
        })
    return trajectory
