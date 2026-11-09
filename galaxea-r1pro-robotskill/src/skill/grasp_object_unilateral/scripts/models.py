"""业务输入与持久检查点。位置单位米，四元数顺序 xyzw。"""
from __future__ import annotations

import math
from typing import Annotated, Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator
from semantic_robot_skill_sdk import Action, ActionResult

Finite = Annotated[float, Field(allow_inf_nan=False)]
Positive = Annotated[float, Field(gt=0, allow_inf_nan=False)]
Point = tuple[Finite, Finite, Finite]
Side = Literal["left", "right"]
Stage = Literal["prepare", "approach", "lift", "stand", "verify"]


class StrictModel(BaseModel):
    model_config = ConfigDict(extra="forbid")


class BoxSize(StrictModel):
    long_m: Positive  # solver 用于相对坐标系 Y 向跨度
    width_m: Positive
    height_m: Positive


class Target(StrictModel):
    object_ref: str = Field(min_length=1)
    position_m: Point
    size: BoxSize
    # RobotObservation.transformer_martix 当前固定读取 map → body。
    world_frame: Literal["map"] = "map"


class TorsoSettings(StrictModel):
    enabled: bool = False
    positions: tuple[Finite, Finite, Finite, Finite] | None = None
    timeout: Positive = 10.0
    tol: Positive = 0.02
    speeds: tuple[Positive, Positive, Positive, Positive] = (0.5, 0.8, 0.7, 0.7)

    @model_validator(mode="after")
    def require_positions(self):
        if self.enabled and self.positions is None:
            raise ValueError("启用躯干调整必须提供已标定的四关节 positions")
        return self


class StandSettings(StrictModel):
    """stand 固定四关节回零，仅开放速度、容差和超时。"""

    speeds: tuple[Positive, Positive, Positive, Positive] = (0.5, 0.8, 0.7, 0.7)
    tol: Positive = 0.02
    timeout: Positive = 10.0


class ForceSettings(StrictModel):
    force_threshold_n: Positive
    force_coordinate: tuple[Literal["x", "y", "z"], ...] = Field(min_length=1, max_length=3)
    step_size_m: Positive = 0.01
    loop_freq_s: Positive = 0.1
    timeout_s: Positive = 3.0


class MotionSettings(StrictModel):
    pos_tol: Positive = 0.05
    orient_tol: Positive = 0.01
    timeout: Positive = 30.0  # 轨迹动作里是「单点」超时；整体上限 ≈ timeout × num_points
    loop_freq: Positive = 0.1
    # 全部轨迹共用 motion；None 时双手由 Ability 自动算点，
    # 单臂因 Ability 的 int(num_points) 契约而显式使用 max_num_points。
    num_points: int | None = Field(default=None, ge=5, le=1000)
    max_num_points: int = Field(default=100, ge=5, le=1000)
    speed: Positive = 0.03  # 上游仅用于日志；不代表真实限速
    # 与双手 Skill 的 lift 运动档一致；sequential=False 时忽略 axis_order。
    sequential: bool = True
    axis_order: str = "yzx"
    step_distance: Positive = 0.05

    @field_validator("axis_order")
    @classmethod
    def axis_order_permutation(cls, value: str) -> str:
        value = value.strip().lower()
        if sorted(value) != ["x", "y", "z"]:
            raise ValueError("axis_order 必须是 x/y/z 的一个排列（如 zyx）：Ability 对非法值会抛 IndexError/KeyError")
        return value


class PerceptionSettings(StrictModel):
    sensor_id: str = Field(default="head_camera", min_length=1)
    minimum_confidence: Finite = Field(default=0.25, ge=0, le=1)
    top_k: int = Field(default=2, ge=1)
    gray_box_only: bool = False
    delta_threshold_x: Positive = 0.2
    delta_threshold_y: Positive = 0.25
    delta_threshold_z: Positive = 0.2
    timeout_seconds: Positive = 120.0


class GraspInput(StrictModel):
    target: Target
    grasp_axis: Side  # 抓取侧：单手抓箱由该侧手先接近并拉箱
    approach_force: ForceSettings
    verify_force: ForceSettings
    verify_side: Side = "left"
    pull_force_ratio_threshold: Positive = 1.5
    max_grasp_retries: int = Field(default=1, ge=0)
    pull_y_distance_m: Positive = 0.1  # 抓取侧沿 y 拉箱位移（米），远离辅助手方向
    # 必须与现场标定的 TargetPointSolver 输出坐标约定一致。
    relative_frame: str = Field(default="torso_link4", min_length=1)
    torso: TorsoSettings = Field(default_factory=TorsoSettings)
    stand: StandSettings = Field(default_factory=StandSettings)
    motion: MotionSettings = Field(default_factory=MotionSettings)
    perception: PerceptionSettings = Field(default_factory=PerceptionSettings)
    safe_offset_m: Positive = 0.07
    capture_x_offset_m: Finite = 0.0  # 与双手 Skill 一致，仅平移抓取侧预备点 X，单位米。
    capture_z_offset_m: Finite = 0.0
    center_x_offset_m: Finite = 0.2
    center_z_offset_m: Finite = 0.2
    # 仅用于双手 lift 段（skill 的 lift:up）；单手抓取侧/辅助手两段抬升用的是
    # skill.GRASP_LIFT_M（0.10 米），改这个字段不会影响那两段。
    lift_m: Positive = 0.1

    @field_validator("relative_frame")
    @classmethod
    def frame(cls, value):
        value = value.strip().strip("/")
        if not value:
            raise ValueError("relative_frame 不能为空")
        return value


class ToolPose(StrictModel):
    x: Finite
    y: Finite
    z: Finite
    qx: Finite
    qy: Finite
    qz: Finite
    qw: Finite

    @model_validator(mode="after")
    def quaternion(self):
        norm = math.hypot(self.qx, self.qy, self.qz, self.qw)
        if norm < 1e-8:
            raise ValueError("末端四元数不能为零")
        for key in ("qx", "qy", "qz", "qw"):
            setattr(self, key, getattr(self, key) / norm)
        return self

    def action_pose(self) -> dict[str, float]:
        return dict(target_x=self.x, target_y=self.y, target_z=self.z,
                    x=self.qx, y=self.qy, z=self.qz, w=self.qw)

    def sensor_data(self, frame: str) -> dict:
        return {"header": {"frame_id": frame}, "pose": {
            "position": {"x": self.x, "y": self.y, "z": self.z},
            "orientation": {"x": self.qx, "y": self.qy, "z": self.qz, "w": self.qw},
        }}


class BothPoses(StrictModel):
    """单手 skill 中语义为：抓取侧 + 辅助手两端。"""
    left: ToolPose
    right: ToolPose

    def for_axis(self, axis: Side) -> ToolPose:
        return getattr(self, axis)

    def other(self, axis: Side) -> ToolPose:
        return getattr(self, "right" if axis == "left" else "left")


class FrameTransform(StrictModel):
    source_frame: str
    target_frame: str
    position: Point
    orientation: tuple[Finite, Finite, Finite, Finite]

    @model_validator(mode="after")
    def normalize_quaternion(self):
        norm = math.hypot(*self.orientation)
        if norm < 1e-8:
            raise ValueError("坐标变换四元数不能为零")
        self.orientation = tuple(value / norm for value in self.orientation)
        return self

    def transform_data(self) -> dict:
        return {"position": list(self.position), "orientation": list(self.orientation)}


class PullForceCheck(StrictModel):
    attempt: int = Field(ge=0)
    side: Side
    before_xyz_n: Point
    after_xyz_n: Point
    before_l1_n: Finite
    after_l1_n: Finite
    ratio_threshold: Positive
    passed: bool


class GraspState(StrictModel):
    grasp_attempt: int = Field(default=0, ge=0)
    force_before_xyz_n: Point | None = None
    force_after_xyz_n: Point | None = None
    pull_force_checks: list[PullForceCheck] = Field(default_factory=list)
    stage: Stage = "prepare"
    actions: dict[str, Action] = Field(default_factory=dict)
    results: dict[str, ActionResult] = Field(default_factory=dict)
    relative_target_m: Point | None = None
    base_to_relative: FrameTransform | None = None
    matched_center_m: Point | None = None
    capture_points: BothPoses | None = None
    lift_points: BothPoses | None = None
    center_points: BothPoses | None = None
    physical_action_started: bool = False
    # 旧版 approach 已开始的检查点不能混用新目标/动作类型；必须人工确认后新建执行。
    approach_revision: int | None = None
    grasp_contact: bool = False  # 抓取侧力控对齐完成
    grasp_lifted: bool = False  # 抓取侧基于接触后实际位姿上抬完成
    grasp_pulled: bool = False  # 抓取侧沿 y 拉箱完成
    grasp_lift_point: ToolPose | None = None
    grasp_pull_point: ToolPose | None = None
    other_arm_contact: bool = False  # 辅助手力控对齐完成
    other_arm_lifted: bool = False  # 辅助手上抬完成
    other_arm_y_move_pose: ToolPose | None = None  # 辅助手 Y 向预定位
    other_arm_x_move_pose: ToolPose | None = None  # 辅助手 Y、Z 到位后的 X 向目标
    other_arm_target_pose: ToolPose | None = None  # 辅助手最终目标
    other_arm_lift_point: ToolPose | None = None
    lifted: bool = False
    retracted: bool = False
    stand_completed: bool = False
    verified: bool = False
    image_status: Literal["pending", "available", "metadata_only", "failed"] = "pending"
    image_refs: list[str] = Field(default_factory=list)
    evidence_refs: list[str] = Field(default_factory=list)
    failure_code: str | None = None
    failure_message: str | None = None


class GraspResult(StrictModel):
    grasp_attempts: int = Field(default=1, ge=1)
    pull_force_checks: list[PullForceCheck] = Field(default_factory=list)
    object_ref: str
    relative_frame: str
    matched_center_m: Point
    holding_object: Literal[True] = True
    verification_method: Literal["single_force_align"] = "single_force_align"
    verification_side: Side
    force_threshold_n: Positive
    # 三个抬升量如实上报（此前写死 0.05/0.1，与实际运动不符，属误报）：
    lift_distance_m: Positive = 0.1              # 抓取侧单手上抬（skill.GRASP_LIFT_M 0.10）
    other_arm_lift_distance_m: Positive = 0.1    # 辅助手上抬（同 skill.GRASP_LIFT_M）
    both_lift_distance_m: Positive = 0.1        # 双手 lift 段（输入 lift_m）
    pull_y_distance_m: Positive
    retracted: Literal[True] = True
    capture_points: BothPoses
    lift_points: BothPoses
    center_points: BothPoses
    image_status: str
    image_refs: list[str]
    evidence_refs: list[str]
