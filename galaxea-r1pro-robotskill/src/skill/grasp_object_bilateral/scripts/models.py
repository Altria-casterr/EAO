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
    pos_tol: Positive = 0.03
    orient_tol: Positive = 0.01
    timeout: Positive = 30.0  # 轨迹动作里是「单点」超时；整体上限 ≈ timeout × num_points
    loop_freq: Positive = 0.1
    # None 表示不指定点数，由 Ability 按 step_distance 自动计算。
    num_points: int | None = Field(default=None, ge=5, le=1000)
    max_num_points: int = Field(default=100, ge=5, le=1000)
    speed: Positive = 0.05  # 上游仅用于日志；不代表真实限速
    # 仅 trajectory 类动作生效：sequential=False 时 Ability 走六维同步插值，
    # axis_order 会被完全忽略（见 utils_JointEEFPoseControl.generate_cartesian_trajectory）。
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


class ApproachMotionSettings(MotionSettings):
    """approach 阶段专用运动档。

    默认开启逐轴顺序插值并按 zyx 执行（先 Z 调高、再 Y 侧向、最后 X 前伸），
    单点超时默认为 60 秒；与 lift 的携箱动作解耦，lift 仍使用 motion
    （默认 sequential=True、yzx 逐轴插值、单点超时 30 秒）。
    """

    sequential: bool = True
    axis_order: str = "zyx"
    timeout: Positive = 60.0


class PerceptionSettings(StrictModel):
    sensor_id: str = Field(default="head_camera", min_length=1)
    minimum_confidence: Finite = Field(default=0.25, ge=0, le=1)
    conf_threshold: Finite = Field(default=0.5, ge=0, le=1)
    iou_threshold: Finite = Field(default=0.5, ge=0, le=1)
    top_k: int = Field(default=2, ge=1)
    gray_box_only: bool = False
    delta_threshold_x: Positive = 0.2
    delta_threshold_y: Positive = 0.25
    delta_threshold_z: Positive = 0.2
    timeout_seconds: Positive = 120.0


class GraspInput(StrictModel):
    target: Target
    approach_force: ForceSettings
    verify_force: ForceSettings
    verify_side: Side = "left"
    # 必须与现场标定的 TargetPointSolver 输出坐标约定一致。
    relative_frame: str = Field(default="torso_link4", min_length=1)
    torso: TorsoSettings = Field(default_factory=TorsoSettings)
    stand: StandSettings = Field(default_factory=StandSettings)
    motion: MotionSettings = Field(default_factory=MotionSettings)
    # approach 阶段的独立运动档；不传时用本类默认（trajectory + zyx 顺序插值）。
    approach_motion: ApproachMotionSettings = Field(default_factory=ApproachMotionSettings)
    perception: PerceptionSettings = Field(default_factory=PerceptionSettings)
    safe_offset_m: Positive = 0.07
    capture_x_offset_m: Finite = 0.0  # 抓取目标 X 偏移，单位米，允许负值。
    capture_z_offset_m: Finite = 0.05
    # 以下两个 offset 目前未被 move_box_to_center 使用（上游已注释掉旧公式），
    # 改它们不会改变点位；保留字段以免破坏既有输入文件，详见 SKILL.md。
    center_x_offset_m: Finite = 0.2
    center_z_offset_m: Finite = 0.2
    # 抬升段末端指令位移（米）。代码默认 0.1，现场输入 0.15；Skills 不使用箱高推算。
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
    left: ToolPose
    right: ToolPose


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


class GraspState(StrictModel):
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
    left_contact: bool = False
    right_contact: bool = False
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
    object_ref: str
    relative_frame: str
    matched_center_m: Point
    holding_object: Literal[True] = True
    verification_method: Literal["single_force_align"] = "single_force_align"
    verification_side: Side
    force_threshold_n: Positive
    # 本次实际使用的 lift_m（现场 0.15），不是固定 5 厘米；由 skill.py 显式传入。
    lift_distance_m: Positive
    retracted: Literal[True] = True
    capture_points: BothPoses
    lift_points: BothPoses
    center_points: BothPoses
    image_status: str
    image_refs: list[str]
    evidence_refs: list[str]
