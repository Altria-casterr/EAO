"""消息转换层：Galaxea_R1Pro 命令映射与 ROS 消息构造。

按控制模块分组（joint_control / gripper_control / torso_control /
eef_control / woosh_move / woosh_navigation），每个模块的 build_*
消息构造函数与其映射条目放在一起，便于审查。

- ``GALAXEA_R1PRO_PUBLISH_COMMANDS``：SDK 操作 → 发布话题映射
  （kind=publisher 发布消息 / kind=action 发送 Action goal）；
- ``build_*``：按 SDK target 参数构造对应 ROS 消息。

ROS 连接本身由 ``utils/Connect_Ros.py`` 负责；本层只做消息转换。
"""

from __future__ import annotations

import math

from ..utils.utils_common import create_timestamp_message

# 执行型命令映射：publisher=话题发布 / action=Action goal
GALAXEA_R1PRO_PUBLISH_COMMANDS: dict[tuple[str, str], dict] = {}

# ==================== joint_control ====================
def build_joint_state_message(target: dict) -> dict:
    """构造 sensor_msgs/JointState 消息（手臂关节）。"""

    return {
        "header": create_timestamp_message(),
        "name": [str(name) for name in (target.get("names") or [])],
        "position": [float(value) for value in (target.get("positions") or [])],
        "velocity": [float(value) for value in (target.get("velocities") or [])],
        "effort": [float(value) for value in (target.get("efforts") or [])],
    }
GALAXEA_R1PRO_PUBLISH_COMMANDS[("joint_control", "control_single_arm_joint")] = {
    "kind": "publisher",
    "topic": lambda target: f"/motion_target/target_joint_state_arm_{target['arm_side']}",
    "message_type": "sensor_msgs/msg/JointState",
    "builder": build_joint_state_message,
}

# ==================== gripper_control ====================
def build_gripper_message(target: dict) -> dict:
    """构造夹爪 JointState 消息（SDK 使用米，旧 Galaxea 控制按需求换算）。"""

    side = str(target.get("side") or "left")
    position = float(target.get("position", 0.0))
    velocity = float(target.get("speed", 0.5))
    effort = float(target.get("effort") or 0.3)
    return {
        "header": create_timestamp_message(),
        "name": [f"gripper_{side}_joint"],
        "position": [position],
        "velocity": [velocity],
        "effort": [effort],
    }
GALAXEA_R1PRO_PUBLISH_COMMANDS[("gripper_control", "control_single_gripper")] = {
    "kind": "publisher",
    "topic": lambda target: f"/motion_target/target_position_gripper_{target['side']}",
    "message_type": "sensor_msgs/msg/JointState",
    "builder": build_gripper_message,
}

# ==================== torso_control ====================
def build_torso_message(target: dict) -> dict:
    """构造躯干 JointState 消息（4 关节，name 可传入，借鉴 Atom Torso_Control._create_msg）。"""

    return {
        "header": create_timestamp_message(),
        "name": [str(name) for name in (target.get("names") or [])],
        "position": [float(value) for value in (target.get("positions") or [])],
        "velocity": [float(value) for value in (target.get("velocities") or [])],
        "effort": [float(value) for value in (target.get("efforts") or [])],
    }
GALAXEA_R1PRO_PUBLISH_COMMANDS[("torso_control", "control_torso_joints")] = {
    "kind": "publisher",
    "topic": "/motion_target/target_joint_state_torso",
    "message_type": "sensor_msgs/msg/JointState",
    "builder": build_torso_message,
}

# ==================== eef_control ====================
def build_pose_stamped_message(target: dict) -> dict:
    """构造 geometry_msgs/PoseStamped 消息。"""

    pose = target["target"]
    return {
        "header": create_timestamp_message(frame_id=pose.frame_id),
        "pose": {
            "position": {
                "x": float(pose.position[0]),
                "y": float(pose.position[1]),
                "z": float(pose.position[2]),
            },
            "orientation": {
                "x": float(pose.orientation_xyzw[0]),
                "y": float(pose.orientation_xyzw[1]),
                "z": float(pose.orientation_xyzw[2]),
                "w": float(pose.orientation_xyzw[3]),
            },
        },
    }
GALAXEA_R1PRO_PUBLISH_COMMANDS[("eef_control", "control_single_arm")] = {
    "kind": "publisher",
    "topic": lambda target: f"/motion_target/target_pose_arm_{target['axis']}",
    "message_type": "geometry_msgs/msg/PoseStamped",
    "builder": build_pose_stamped_message,
}

# ==================== woosh_move ====================
def build_twist_stamped_message(target: dict) -> dict:
    """构造 geometry_msgs/TwistStamped 消息（底盘速度控制）。"""

    return {
        "header": create_timestamp_message(frame_id="base_link"),
        "twist": {
            "linear": {"x": float(target.get("dx", 0.0)), "y": float(target.get("dy", 0.0)), "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": math.radians(float(target.get("dyaw", 0.0)))},
        },
    }

def build_brake_message(target: dict) -> dict:
    """构造 std_msgs/Bool 刹车消息（woosh_move / woosh_navigation 停止共用）。

    target["brake"] 为 False 时发布解锁（brake=False），默认 True 为刹车。
    """
    return {"data": bool(target.get("brake", True))}

GALAXEA_R1PRO_PUBLISH_COMMANDS[("woosh_move", "move_robot")] = {"kind": "publisher","topic": "/motion_target/target_speed_chassis","message_type": "geometry_msgs/msg/TwistStamped","builder": build_twist_stamped_message}

GALAXEA_R1PRO_PUBLISH_COMMANDS[("woosh_move", "unlock_brake")] = {"kind": "publisher","topic": "/motion_target/brake_mode","message_type": "std_msgs/msg/Bool","builder": build_brake_message}

# ==================== woosh_navigation ====================
def build_navigation_goal(target: dict) -> dict:
    """构造 system_manager_msg/NavigationTask action goal。"""

    yaw = target.get("yaw")
    if yaw is not None:
        yaw_rad = math.radians(float(yaw))
        qx, qy, qz, qw = 0.0, 0.0, math.sin(yaw_rad / 2), math.cos(yaw_rad / 2)
    else:
        qx = float(target.get("qx") or 0.0)
        qy = float(target.get("qy") or 0.0)
        qz = float(target.get("qz") or 0.0)
        qw = float(target.get("qw") if target.get("qw") is not None else 1.0)
    frame_id = str(target.get("frame_id") or "map")
    return {
        "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": frame_id},
        "pose": {
            "position": {"x": float(target.get("x", 0.0)), "y": float(target.get("y", 0.0)), "z": float(target.get("z", 0.0))},
            "orientation": {"x": qx, "y": qy, "z": qz, "w": qw},
        },
        "frame_id": frame_id,
        "target_point_type": int(target.get("target_point_type", 0)),
    }

GALAXEA_R1PRO_PUBLISH_COMMANDS[("woosh_navigation", "navigate_to_map_pose")] = {"kind": "action",
    "topic": "/system_manager/navi/action", "message_type": "system_manager_msg/action/NavigationTask", "builder": build_navigation_goal}

