"""米制输入、可恢复检查点与力接触证据；四元数顺序 xyzw。"""
from __future__ import annotations

import math
from datetime import datetime
from typing import Annotated, Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator
from semantic_robot_skill_sdk import Action, ActionResult

Finite = Annotated[float, Field(strict=True, allow_inf_nan=False)]
Positive = Annotated[float, Field(strict=True, gt=0, allow_inf_nan=False)]
Nonnegative = Annotated[float, Field(strict=True, ge=0, allow_inf_nan=False)]
Point = tuple[Finite, Finite, Finite]
Side = Literal['left', 'right']
Coordinate = Literal['x', 'y', 'z']
ContactStage = Literal['align_x', 'lower_z', 'push_y']
PositionStage = Literal['align_x', 'lower_z']
ForceStage = Literal['push_y']
Stage = Literal['prepare', 'preposition', 'align_x', 'lower_z',
                'retract_assist', 'push_y', 'retract_push', 'verify']


class StrictModel(BaseModel):
    model_config = ConfigDict(extra='forbid', allow_inf_nan=False, str_strip_whitespace=True)


class BoxSize(StrictModel):
    long_m: Positive
    width_m: Positive
    height_m: Positive


class Target(StrictModel):
    object_ref: str = Field(min_length=1)
    world_frame: Literal['map'] = 'map'
    position_m: Point
    size: BoxSize


class TorsoSettings(StrictModel):
    enabled: bool = True
    positions: tuple[Finite, Finite, Finite, Finite] | None = None
    timeout: Positive = 10.0
    tol: Positive = 0.02
    speeds: tuple[Positive, Positive, Positive, Positive] = (0.5, 0.8, 0.7, 0.7)

    @model_validator(mode='after')
    def require_positions(self):
        if self.enabled and self.positions is None:
            raise ValueError('启用躯干调整必须提供已标定的四关节 positions')
        return self


class ForceSettings(StrictModel):
    force_threshold_n: Positive = 1.5
    force_coordinate: tuple[Coordinate, ...] = Field(default=('x',), min_length=1, max_length=3)
    max_distance_m: Positive = 0.01
    timeout_s: Positive = 30.0
    step_size_m: Positive = 0.01
    loop_freq_s: Positive = 0.1

    @model_validator(mode='after')
    def unique_axes(self):
        if len(set(self.force_coordinate)) != len(self.force_coordinate):
            raise ValueError('力监测轴不能重复')
        return self


class TrajectorySettings(StrictModel):
    pos_tol: Positive = 0.005
    orient_tol: Positive = 0.01
    timeout: Positive = 10.0
    loop_freq: Positive = 0.1
    num_points: int | None = Field(default=None, strict=True, ge=5, le=1000)
    max_num_points: int = Field(default=100, strict=True, ge=5, le=1000)
    speed: Positive = 0.05  # 当前 trajectory 只把 speed 用于日志。
    sequential: bool = True
    axis_order: str = 'yzx'
    step_distance: Positive = 0.05

    @field_validator('axis_order')
    @classmethod
    def axis_order_permutation(cls, value):
        value = value.strip().lower()
        if sorted(value) != ['x', 'y', 'z']:
            raise ValueError('axis_order 必须是 x/y/z 的一个排列（如 zyx）')
        return value


class MotionSettings(TrajectorySettings):
    fine_pos_tol: Positive = 0.0005
    step_timeout_s: Positive = 2.0


class ApproachMotionSettings(TrajectorySettings):
    # 对齐最新双手 Skill 的独立接近轨迹档。
    pos_tol: Positive = 0.03
    timeout: Positive = 10.0
    axis_order: str = 'zyx'


class RetractionMotionSettings(TrajectorySettings):
    # 单手轨迹使用最新单手参数约定；保留放置撤手的 5 mm 到位精度。
    speed: Positive = 0.03


class PrepositionSettings(StrictModel):
    x_offset_m: Finite = -0.03
    y_offset_m: Nonnegative = 0.1  # 兼容旧输入；放置阶段保持起始 Y，此参数不再参与计算。
    z_offset_m: Nonnegative = 0.07
    # 末端与托举夹具的偏移（torso_link4，米）：双手持箱目标 = 箱心 - 本偏移，
    # 仅 X/Z 生效，须按实际工具标定；Y 固定为零（箱缘几何仍用箱心 Y）。
    grasp_center_offset_m: Point = (0.0, 0.0, 0.0)


class RetractSettings(StrictModel):
    lower_m: Nonnegative = 0.02
    minimum_outward_m: Positive = 0.03
    lift_m: Positive = 0.15
    backward_m: Nonnegative = 0.1  # 第三步沿 X 负方向回撤量（米），默认 0.1
    safe_offset_z: Nonnegative = 0.03  # 第三段目标高度相对撤手开始高度的增量（米）


class PlaceInput(StrictModel):
    target: Target
    y_force: ForceSettings = Field(default_factory=ForceSettings)
    put_mode: Side = 'left'
    relative_frame: Literal['torso_link4'] = 'torso_link4'
    force_phase_pause_s: Nonnegative = 0.5
    torso: TorsoSettings = Field(default_factory=TorsoSettings)
    preposition: PrepositionSettings = Field(default_factory=PrepositionSettings)
    retract: RetractSettings = Field(default_factory=RetractSettings)
    motion: MotionSettings = Field(default_factory=MotionSettings)
    approach_motion: ApproachMotionSettings = Field(default_factory=ApproachMotionSettings)
    retract_motion: RetractionMotionSettings = Field(default_factory=RetractionMotionSettings)

    @model_validator(mode='after')
    def fine_tolerance(self):
        if self.motion.fine_pos_tol > self.y_force.step_size_m / 2:
            raise ValueError('motion.fine_pos_tol 不得超过 Y 力控步长的一半')
        return self


class ToolPose(StrictModel):
    x: Finite
    y: Finite
    z: Finite
    qx: Finite
    qy: Finite
    qz: Finite
    qw: Finite

    @model_validator(mode='after')
    def normalize(self):
        length = math.hypot(self.qx, self.qy, self.qz, self.qw)
        if not math.isfinite(length) or length < 1e-8:
            raise ValueError('末端四元数模长必须有限且不能为零')
        for name in ('qx', 'qy', 'qz', 'qw'):
            setattr(self, name, getattr(self, name) / length)
        return self

    def action_pose(self):
        return dict(target_x=self.x, target_y=self.y, target_z=self.z,
                    x=self.qx, y=self.qy, z=self.qz, w=self.qw)

    def sensor_data(self, frame: str):
        return {'header': {'frame_id': frame}, 'pose': {
            'position': dict(x=self.x, y=self.y, z=self.z),
            'orientation': dict(x=self.qx, y=self.qy, z=self.qz, w=self.qw),
        }}


class BothPoses(StrictModel):
    left: ToolPose
    right: ToolPose


class FrameTransform(StrictModel):
    # 默认值仅用于解析旧检查点；版本判定在任何新运动之前执行。
    source_frame: str = 'base_link'
    target_frame: str = 'torso_link4'
    position: Point
    orientation: tuple[Finite, Finite, Finite, Finite]

    @model_validator(mode='after')
    def normalize_quaternion(self):
        length = math.hypot(*self.orientation)
        if not math.isfinite(length) or length < 1e-8:
            raise ValueError('坐标变换四元数模长必须有限且不能为零')
        self.orientation = tuple(value / length for value in self.orientation)
        return self

    def transform_data(self):
        return {'position': list(self.position), 'orientation': list(self.orientation)}


class ForceVector(StrictModel):
    x: Finite
    y: Finite
    z: Finite


class ForceSample(StrictModel):
    left: ForceVector
    right: ForceVector


class ContactEvidence(StrictModel):
    movement_axis: Coordinate
    direction: Literal['negative', 'positive']
    force_coordinate: tuple[Coordinate, ...]
    force_threshold_n: Positive
    baseline: ForceSample
    final_force: ForceSample
    delta_n: dict[Side, Finite]
    triggered_sides: list[Side] = Field(min_length=1)
    commanded_travel_m: Nonnegative
    completed_steps: int = Field(ge=0)
    actual_poses: BothPoses


class PlaceState(StrictModel):
    stage: Stage = 'prepare'
    placement_revision: int | None = None
    input_digest: str | None = None
    base_to_relative: FrameTransform | None = None
    relative_target_m: Point | None = None
    initial_poses: BothPoses | None = None  # 躯干到位后、首次手臂运动前的位姿（torso_link4）。
    initial_forces: ForceSample | None = None
    preposition_points: BothPoses | None = None
    position_targets: dict[PositionStage, BothPoses] = Field(default_factory=dict)
    positioned_stages: list[PositionStage] = Field(default_factory=list)
    contacts: dict[ContactStage, ContactEvidence] = Field(default_factory=dict)
    force_phase_pause_until: dict[ContactStage, datetime] = Field(default_factory=dict)
    retract_plans: dict[Side, list[ToolPose]] = Field(default_factory=dict)
    retracted_sides: list[Side] = Field(default_factory=list)
    actions: dict[str, Action] = Field(default_factory=dict)
    results: dict[str, ActionResult] = Field(default_factory=dict)
    # 读操作每次用新 key，恢复后的现场判断不能复用旧传感器结果。
    read_sequence: int = Field(default=0, ge=0)
    physical_action_started: bool = False
    evidence_refs: list[str] = Field(default_factory=list)
    failure_code: str | None = None
    failure_message: str | None = None


class PlaceResult(StrictModel):
    object_ref: str
    put_mode: Side
    relative_frame: Literal['torso_link4'] = 'torso_link4'
    relative_target_m: Point
    placed: Literal[True] = True
    holding_object: None = None
    verification_method: Literal['force_contact_and_retraction'] = 'force_contact_and_retraction'
    preposition_points: BothPoses
    contacts: dict[ForceStage, ContactEvidence]
    final_poses: BothPoses
    final_forces: ForceSample
    retracted_sides: list[Side]
    evidence_refs: list[str] = Field(default_factory=list)
