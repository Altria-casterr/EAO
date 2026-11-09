"""Ability 角色枚举（与 manifest abilityName 对应）。"""

from __future__ import annotations

from enum import StrEnum


class AbilityRole(StrEnum):
    base_motion = "base_motion"
    upper_body_motion = "upper_body_motion"
    woosh_move = "woosh_move"
    woosh_navigation = "woosh_navigation"
    robot_observation = "robot_observation"
    joint_eef_pose_control = "joint_eef_pose_control"
    torso_control = "torso_control"
    joint_control = "joint_control"
    gripper_controller = "gripper_controller"
    yolo_detect = "yolo_detect"
    grasp_planning = "grasp_planning"
    policy_control = "policy_control"
