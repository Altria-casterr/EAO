"""读取子处理层：Galaxea_R1Pro 全部 get 函数的统一实现。

按控制模块分组（joint_control / gripper_control / torso_control /
force_feedback / eef_control / woosh_move / woosh_navigation /
coordinate_transformation），每个模块的 handler 与其映射条目放在一起，
便于审查。

实现借鉴 skill_library/Atom 各 Galaxea_R1Pro main.py 的 get 函数：
在固定超时时间内循环检测反馈话题是否出现最新消息，收到后解析返回；
超时未收到抛 MessageNotFoundError。handler 默认只接收 ros；
标注 accepts_target 的 handler 额外接收 target 参数（如坐标变换 get_*）。
"""

from __future__ import annotations

import copy
import logging
import time
from functools import partial
from typing import Optional

from scipy.spatial.transform import Rotation as R

from ..utils.Connect_Ros import ConnectRos
from ..utils.errors import MessageNotFoundError
from ..utils.utils_common import (
    compute_chain_transform,
    find_transform_path,
    inverse_transform,
    normalize_frame_name,
)

logger = logging.getLogger(__name__)

# 读取型命令映射：handler 为本模块中的处理函数
# handler(ros_connection) -> 处理后的反馈数据
GALAXEA_R1PRO_GET_COMMANDS: dict[tuple[str, str], dict] = {}

_DEFAULT_INTERVAL = 0.05

# 固定等待超时（借鉴 Atom：关节/夹爪/躯干/末端 5s，力反馈 30s）
_WAIT_JOINT = 5.0
_WAIT_GRIPPER = 5.0
_WAIT_TORSO = 5.0
_WAIT_FORCE = 30.0
_WAIT_EE = 5.0
_WAIT_CHASSIS = 5.0
_WAIT_TRANSFORM = 5.0


# ==================== 通用等待机制 ====================

def _wait_for_topic(ros: ConnectRos, topic: str, timeout: float = _WAIT_JOINT, interval: float = _DEFAULT_INTERVAL) -> Optional[dict]:
    """循环检测指定话题是否已有最新消息；超时返回 None。"""

    start = time.time()
    while time.time() - start < timeout:
        message = ros.latest_messages.get(topic)
        if message:
            return message
        time.sleep(interval)
    logger.error("等待话题 %s 消息超时（%.1fs）", topic, timeout)
    raise MessageNotFoundError(
        topic=topic,
        reason=MessageNotFoundError.MESSAGE_TIMEOUT,
        timeout=timeout,
        hint="请检查话题发布端是否在线、是否已 connect() 注册该话题",
    )


# ==================== joint_control ====================
def get_joint_state(ros: ConnectRos, axis: str) -> Optional[list]:
    """循环检测并读取单臂关节位置（左/右由 axis 指定，借鉴 Atom Joint_Control）。"""

    message = _wait_for_topic(ros, f"/hdas/feedback_arm_{axis}", _WAIT_JOINT)
    position = (message or {}).get("position")
    return list(position) if position else None

GALAXEA_R1PRO_GET_COMMANDS[("joint_control", "get_left_joint_state")] = {"handler": partial(get_joint_state, axis="left")}
GALAXEA_R1PRO_GET_COMMANDS[("joint_control", "get_right_joint_state")] = {"handler": partial(get_joint_state, axis="right")}


# ==================== gripper_control ====================
def get_gripper_state(ros: ConnectRos, axis: str) -> Optional[float]:
    """获取单侧夹爪当前状态位置（左/右由 axis 指定，借鉴 Atom Gripper_Control）。"""

    message = _wait_for_topic(ros, f"/hdas/feedback_gripper_{axis}", _WAIT_GRIPPER)
    position = (message or {}).get("position")
    return float(position[0]) if position else None

GALAXEA_R1PRO_GET_COMMANDS[("gripper_control", "get_left_gripper_state")] = {"handler": partial(get_gripper_state, axis="left")}
GALAXEA_R1PRO_GET_COMMANDS[("gripper_control", "get_right_gripper_state")] = {"handler": partial(get_gripper_state, axis="right")}


# ==================== torso_control ====================
def get_torso_positions(ros: ConnectRos) -> Optional[list]:
    """获取躯干四关节当前位置（借鉴 Atom Torso_Control.get_torso_positions）。"""

    message = _wait_for_topic(ros, "/hdas/feedback_torso", _WAIT_TORSO)
    position = (message or {}).get("position")
    if position and len(position) >= 4:
        return [float(value) for value in position[:4]]
    return None

def get_torso_status(ros: ConnectRos) -> Optional[dict]:
    """获取躯干状态反馈（借鉴 Atom Torso_Control.get_torso_status）。"""

    return _wait_for_topic(ros, "/hdas/feedback_status_torso", _WAIT_TORSO)

GALAXEA_R1PRO_GET_COMMANDS[("torso_control", "get_torso_positions")] = {"handler": get_torso_positions}
GALAXEA_R1PRO_GET_COMMANDS[("torso_control", "get_torso_status")] = {"handler": get_torso_status}


# ==================== force_feedback ====================
def get_force_data(ros: ConnectRos) -> dict:
    """循环检测并获取左右臂力/力矩数据（借鉴 Atom ForceFeedback.get_force_data）。

    左右臂分别等待 wrench 话题出现消息后解析；超时抛 MessageNotFoundError。
    """

    def _parse(wrench_message: Optional[dict]) -> dict:
        wrench = (wrench_message or {}).get("wrench")
        if not wrench:
            return {}
        force = wrench.get("force", {})
        torque = wrench.get("torque", {})
        return {
            "force_x": float(force.get("x", 0.0)),
            "force_y": float(force.get("y", 0.0)),
            "force_z": float(force.get("z", 0.0)),
            "torque_x": float(torque.get("x", 0.0)),
            "torque_y": float(torque.get("y", 0.0)),
            "torque_z": float(torque.get("z", 0.0)),
        }

    left = _wait_for_topic(ros, "/hdas/feedback_left_arm_wrench", _WAIT_FORCE)
    right = _wait_for_topic(ros, "/hdas/feedback_right_arm_wrench", _WAIT_FORCE)
    return {
        "left_arm": _parse(left),
        "right_arm": _parse(right),
    }
GALAXEA_R1PRO_GET_COMMANDS[("force_feedback", "get_force_data")] = {"handler": get_force_data}

# ==================== eef_control ====================
def get_ee_pose(ros: ConnectRos, axis: str) -> Optional[dict]:
    """循环检测并读取单臂末端执行器当前位姿（左/右由 axis 指定）。

    返回 {"header": {"frame_id": ...}, "pose": ...}：header 仅保留 frame_id，
    便于 main 判断坐标系是否一致并进行坐标变换。
    """

    message = _wait_for_topic(ros, f"/motion_control/pose_ee_arm_{axis}", _WAIT_EE)
    pose = (message or {}).get("pose")
    if not pose:
        return None
    header = (message or {}).get("header") or {}
    return {"header": {"frame_id": header.get("frame_id")}, "pose": pose}

def get_floating_base(ros: ConnectRos) -> Optional[dict]:
    """循环检测并读取浮动基座标变换（借鉴 Atom Joint_EEFPose_Control.get_trans_martix）。

    返回 base_link → 浮动基座的逆变换：{"position": (x, y, z), "orientation": (qx, qy, qz, qw)}，
    由 scipy Rotation 求四元数逆并旋转取反得到。
    """

    message = _wait_for_topic(ros, "/motion_control/pose_floating_base", _WAIT_TRANSFORM)
    pose = (message or {}).get("pose")
    if not pose:
        return None
    position = (
        float(pose["position"]["x"]),
        float(pose["position"]["y"]),
        float(pose["position"]["z"]),
    )
    orientation = (
        float(pose["orientation"]["x"]),
        float(pose["orientation"]["y"]),
        float(pose["orientation"]["z"]),
        float(pose["orientation"]["w"]),
    )
    rot = R.from_quat(orientation)
    inv_rot = rot.inv()
    inv_quat = inv_rot.as_quat()
    inv_pos = -inv_rot.apply(position)
    return {"position": tuple(inv_pos), "orientation": tuple(inv_quat)}

GALAXEA_R1PRO_GET_COMMANDS[("eef_control", "get_left_ee_pose")] = {"handler": partial(get_ee_pose, axis="left")}
GALAXEA_R1PRO_GET_COMMANDS[("eef_control", "get_right_ee_pose")] = {"handler": partial(get_ee_pose, axis="right")}
GALAXEA_R1PRO_GET_COMMANDS[("eef_control", "get_floating_base")] = {"handler": get_floating_base}


# ==================== woosh_move ====================
def get_chassis_feedback(ros: ConnectRos) -> dict:
    """循环检测并读取底盘轮速/轮角反馈（借鉴 Atom WooshMove._feedback_callback）。

    轮速/轮角按 WHEEL_INDEX 顺序（左前/右前/后轮）取 velocity/position 前三个值。
    """

    message = _wait_for_topic(ros, "/hdas/feedback_chassis", _WAIT_CHASSIS)
    velocity = (message or {}).get("velocity") or [0.0, 0.0, 0.0]
    position = (message or {}).get("position") or [0.0, 0.0, 0.0]
    return {
        "feedback_received": bool(message),
        "wheel_speeds": {
            "left_front": float(velocity[0]),
            "right_front": float(velocity[1]),
            "rear": float(velocity[2]),
        },
        "wheel_angles": {
            "left_front": float(position[0]),
            "right_front": float(position[1]),
            "rear": 0.0,
        },
    }


def get_localization(ros: ConnectRos) -> dict:
    """循环检测并读取定位结果（借鉴 Atom WooshMove._localization_callback）。

    返回 position_global/local、orientation_global/local 及可用性标志。
    """

    message = _wait_for_topic(ros, "/localization/localization_results", _WAIT_CHASSIS)
    position = (message or {}).get("position", {})
    orientation = (message or {}).get("orientation", {})
    pos_global = position.get("position_global", {})
    pos_local = position.get("position_local", {})
    ori_global = orientation.get("euler_global", {})
    ori_local = orientation.get("euler_local", {})
    status_available = int((message or {}).get("status", {}).get("available", 0))
    pos_available = int(position.get("available", 0))
    ori_available = int(orientation.get("available", 0))
    return {
        "localization_received": bool(message),
        "localization_available": bool(pos_available == 1 and ori_available == 1 and status_available == 1),
        "position_global": {"x": pos_global.get("x"), "y": pos_global.get("y"), "available": pos_available},
        "position_local": {"x": pos_local.get("x"), "y": pos_local.get("y"), "available": int(pos_local.get("available", 0))},
        "orientation_global": {"yaw": ori_global.get("yaw"), "available": ori_available},
        "orientation_local": {"yaw": ori_local.get("yaw"), "available": int(ori_local.get("available", 0))},
        "status_available": status_available,
    }


GALAXEA_R1PRO_GET_COMMANDS[("woosh_move", "get_chassis_feedback")] = {"handler": get_chassis_feedback}
GALAXEA_R1PRO_GET_COMMANDS[("woosh_move", "get_localization")] = {"handler": get_localization}

# ==================== woosh_navigation ====================
def get_navigation_status(ros: ConnectRos) -> dict:
    """读取当前导航状态快照（借鉴 Atom WooshNavigation.get_current_status，立即返回）。"""

    return copy.deepcopy(ros.navigation_state or {})
    
GALAXEA_R1PRO_GET_COMMANDS[("woosh_navigation", "get_current_status")] = {"handler": get_navigation_status}


# ==================== coordinate_transformation ====================
def _build_tf_cache(message: Optional[dict]) -> dict:
    """把 /tf 消息的 transforms 解析为 {child_frame: {parent_frame, translation, rotation}} 缓存（对齐 Atom _tf_callback）。"""

    cache: dict = {}
    for transform in (message or {}).get("transforms", []) or []:
        parent_frame = normalize_frame_name(transform.get("header", {}).get("frame_id"))
        child_frame = normalize_frame_name(transform.get("child_frame_id"))
        if parent_frame is None or child_frame is None:
            continue
        tf_transform = transform.get("transform", {})
        translation = tf_transform.get("translation", {})
        rotation = tf_transform.get("rotation", {})
        cache[child_frame] = {
            "parent_frame": parent_frame,
            "translation": {
                "x": float(translation.get("x")),
                "y": float(translation.get("y")),
                "z": float(translation.get("z")),
            },
            "rotation": {
                "x": float(rotation.get("x", 0.0)),
                "y": float(rotation.get("y", 0.0)),
                "z": float(rotation.get("z", 0.0)),
                "w": float(rotation.get("w", 0.0)),
            },
        }
    return cache

def _get_transform_martix(ros: ConnectRos, source_frame: str, target_frame: str) -> dict:
    """保留旧私有调用的 ROS 字段格式；公开接口使用累积 TF 缓存。"""
    from ..utils.tf_buffer import TransformBuffer
    buffer = getattr(ros, 'tf_buffer', None)
    if buffer is None:
        buffer = TransformBuffer()
        for topic in ('/tf_static', '/tf'):
            buffer.update(ros.latest_messages.get(topic) or {}, static=topic == '/tf_static')
    try:
        matrix = buffer.lookup(source_frame, target_frame)
    except LookupError as exc:
        raise MessageNotFoundError(topic='/tf + /tf_static',
            reason=MessageNotFoundError.MESSAGE_NOT_FOUND, hint=str(exc)) from exc
    return {'parent_frame': normalize_frame_name(target_frame),
            'translation': dict(zip('xyz', map(float, matrix[:3, 3]))),
            'rotation': dict(zip('xyzw', map(float, R.from_matrix(matrix[:3, :3]).as_quat())))}


def get_transformer_martix(ros: ConnectRos, source_frame: str = "map", target_frame: str = "body",
                           timeout: float = 15.0, max_age_s: float | None = None) -> Optional[dict]:
    """累积 /tf 与 /tf_static，读取最新连通变换；仅支持停稳后的采集。

    max_age_s 是动态边距最后更新的本机单调时钟时间；静态边不失效。
    此接口不提供按图像时间戳查询的历史 TF。
    """
    import math
    if not math.isfinite(timeout) or timeout < 0:
        raise ValueError("TF timeout 必须为非负有限数")
    if max_age_s is not None and (not math.isfinite(max_age_s) or max_age_s <= 0):
        raise ValueError("TF max_age_s 必须为正的有限数")
    source = normalize_frame_name(source_frame)
    target = normalize_frame_name(target_frame)
    if not source or not target:
        raise ValueError("TF 坐标系不能为空")
    deadline = time.monotonic() + timeout
    while True:
        try:
            matrix = ros.tf_buffer.lookup(source, target, max_age_s)
            return {"position": tuple(map(float, matrix[:3, 3])),
                    "orientation": tuple(map(float, R.from_matrix(matrix[:3, :3]).as_quat()))}
        except LookupError as exc:
            if time.monotonic() >= deadline:
                raise MessageNotFoundError(topic="/tf + /tf_static",
                    reason=MessageNotFoundError.MESSAGE_TIMEOUT, timeout=timeout,
                    hint=str(exc)) from exc
            time.sleep(min(_DEFAULT_INTERVAL, max(0.0, deadline-time.monotonic())))


GALAXEA_R1PRO_GET_COMMANDS[("coordinate_transformation", "get_transformer_martix")] = {"handler": get_transformer_martix, "accepts_target": True}
