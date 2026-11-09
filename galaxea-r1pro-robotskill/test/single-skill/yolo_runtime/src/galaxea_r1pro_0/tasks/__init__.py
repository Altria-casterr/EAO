"""业务 Task 注册：每个 Task 写完后在这里给出角色映射（顶层 tasks/ 目录）。"""

from __future__ import annotations

from galaxea_r1pro import R1ProSDK

from ..service.execution import ExecutionRegistry
from ..service.models import ModelProfileRegistry
from .registry import TaskRegistry
from .base import AbilityTask

from .gripper import HandleBothGripperTask, HandleSingleGripperTask
from .joint_control import (
    ControlBothArmsJointTask,
    ControlSingleArmJointTask,
)
from .joint_eef_pose_control import (
    ControlBothArmTask,
    ControlBothArmsTrajectoryTask,
    ControlSingleArmTask,
    ControlSingleArmTrajectoryTask,
    ForceAlignTask,
)
from .robot_observation import (
    CaptureRGBDTask,
    GetRobotStateTask,
    GetSensorsStateTask,
)
from .yolo_detect import (
    EstimateObjectPoseTask,
    GeneratePointCloudTask,
    MatchTargetTask,
    YoloDetectTask,
)
from .torso_control import ControlSingleTorsoJointTask, ControlTorsoJointsTask
from .woosh_move import MoveRobotTask, StopTask as WooshMoveStopTask
from .woosh_navigation import (
    NavigateToMapPoseTask,
    StopTask as WooshNavigationStopTask,
    VerifyArrivalTask,
)

# 角色 → Task 类（顺序即 manifest 中业务 task 的顺序；taskType 由 Task 类声明）
ROLE_TASK_SPECS: dict[str, tuple[type[AbilityTask], ...]] = {
    "woosh_move": (MoveRobotTask, WooshMoveStopTask),
    "woosh_navigation": (NavigateToMapPoseTask, WooshNavigationStopTask, VerifyArrivalTask),
    "robot_observation": (GetRobotStateTask, GetSensorsStateTask, CaptureRGBDTask),
    "yolo_detect": (
        GeneratePointCloudTask,
        YoloDetectTask,
        MatchTargetTask,
        EstimateObjectPoseTask,
    ),
    "joint_eef_pose_control": (
        ControlSingleArmTask,
        ControlBothArmTask,
        ForceAlignTask,
        ControlSingleArmTrajectoryTask,
        ControlBothArmsTrajectoryTask,
    ),
    "torso_control": (ControlTorsoJointsTask, ControlSingleTorsoJointTask),
    "joint_control": (
        ControlSingleArmJointTask,
        ControlBothArmsJointTask,
    ),
    "gripper_controller": (HandleSingleGripperTask, HandleBothGripperTask),
}


def role_task_types(role: str) -> tuple[tuple[int, str], ...]:
    """返回 (task_type, task_name) 列表，供 entrypoint 注册框架 task 使用。"""

    return tuple(
        (task_cls.task_type, task_cls.name)
        for task_cls in ROLE_TASK_SPECS.get(role, ())
    )


def build_role_tasks(
    role: str,
    sdk: R1ProSDK,
    executions: ExecutionRegistry | None = None,
    models: ModelProfileRegistry | None = None,
    handler=None,
) -> TaskRegistry:
    """按角色实例化业务 Task（invoke 只引用 TaskRegistry，不引用 SDK）。"""

    registry = TaskRegistry()
    for task_cls in ROLE_TASK_SPECS.get(role, ()):
        registry.add(task_cls(sdk, executions, models, handler))
    return registry


__all__ = ["ROLE_TASK_SPECS", "build_role_tasks", "role_task_types"]
