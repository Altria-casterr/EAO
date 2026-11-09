"""EEF Skill 输入：一次只执行一种动作，数值在 Pilot 前校验。"""
from __future__ import annotations

import math
from typing import Annotated, Literal, Union

from pydantic import BaseModel, ConfigDict, Field, RootModel, model_validator

Finite = Annotated[float, Field(allow_inf_nan=False)]
Positive = Annotated[float, Field(gt=0, allow_inf_nan=False)]
AxisOrder = Literal["xyz", "xzy", "yxz", "yzx", "zxy", "zyx"]


class StrictModel(BaseModel):
    model_config = ConfigDict(extra="forbid", populate_by_name=True)


class TargetPose(StrictModel):
    target_x: Finite
    target_y: Finite
    target_z: Finite
    x: Finite
    y: Finite
    z: Finite
    w: Finite

    @model_validator(mode="after")
    def normalize_quaternion(self):
        norm = math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w)
        if not math.isfinite(norm) or norm < 1e-8:
            raise ValueError("姿态四元数必须非零且有限")
        self.x /= norm
        self.y /= norm
        self.z /= norm
        self.w /= norm
        return self


class SinglePoseParams(StrictModel):
    axis: Literal["left", "right"]
    target_pose: TargetPose = Field(alias="_target_pose")
    frame_id: str = Field(min_length=1)
    pos_tol: Positive = 0.05
    orient_tol: Positive = 0.01
    timeout: Positive = 10.0
    loop_freq: Positive = 1.0


class BothPoseParams(StrictModel):
    left_target_pose: TargetPose
    right_target_pose: TargetPose
    left_frame_id: str = Field(min_length=1)
    right_frame_id: str = Field(min_length=1)
    left_pos_tol: Positive = 0.05
    left_orient_tol: Positive = 0.01
    right_pos_tol: Positive = 0.05
    right_orient_tol: Positive = 0.01
    timeout: Positive = 10.0
    loop_freq: Positive = 1.0


class SingleTrajectoryParams(SinglePoseParams):
    num_points: int | None = Field(default=None, ge=5)
    speed: Positive = 0.05
    axis_order: AxisOrder = "zyx"
    sequential: bool = True
    step_distance: Positive = 0.01
    max_num_points: int = Field(default=100, ge=5, le=1000)

    @model_validator(mode="after")
    def check_point_limit(self):
        if self.num_points is not None and self.num_points > self.max_num_points:
            raise ValueError("num_points 不得超过 max_num_points")
        return self


class BothTrajectoryParams(BothPoseParams):
    num_points: int | None = Field(default=None, ge=5)
    speed: Positive = 0.05
    left_axis_order: AxisOrder = "zyx"
    right_axis_order: AxisOrder = "zyx"
    left_sequential: bool = True
    right_sequential: bool = True
    step_distance: Positive = 0.01
    max_num_points: int = Field(default=100, ge=5, le=1000)

    @model_validator(mode="after")
    def check_point_limit(self):
        if self.num_points is not None and self.num_points > self.max_num_points:
            raise ValueError("num_points 不得超过 max_num_points")
        return self


class ForceAlignParams(StrictModel):
    axis: Literal["left", "right"]
    move_coordinate: Literal["x", "y", "z"]
    direction: Literal["negative", "positive"]
    force_coordinate: tuple[Literal["x", "y", "z"], ...] = Field(min_length=1)
    force_threshold_n: Positive
    step_size_m: Positive
    max_distance_m: Positive
    loop_freq_s: Positive = 0.1
    timeout_s: Positive = 10.0

    @model_validator(mode="after")
    def check_distance(self):
        if self.step_size_m > self.max_distance_m:
            raise ValueError("step_size_m 不得超过 max_distance_m")
        return self


class SinglePoseRequest(StrictModel):
    task: Literal["single_pose"]
    params: SinglePoseParams


class BothPoseRequest(StrictModel):
    task: Literal["both_pose"]
    params: BothPoseParams


class SingleTrajectoryRequest(StrictModel):
    task: Literal["single_trajectory"]
    params: SingleTrajectoryParams


class BothTrajectoryRequest(StrictModel):
    task: Literal["both_trajectory"]
    params: BothTrajectoryParams


class ForceAlignRequest(StrictModel):
    task: Literal["force_align"]
    params: ForceAlignParams


RequestUnion = Annotated[
    Union[SinglePoseRequest, BothPoseRequest, SingleTrajectoryRequest, BothTrajectoryRequest, ForceAlignRequest],
    Field(discriminator="task"),
]


class EEFInput(RootModel[RequestUnion]):
    pass


class EEFState(StrictModel):
    stage: Literal["pending", "completed"] = "pending"
    invocation_id: str | None = None
    evidence_refs: list[str] = Field(default_factory=list)


class EEFResult(StrictModel):
    task: str
    invocation_id: str | None = None
    ability_status: str
    ability_output: dict = Field(default_factory=dict)
    evidence_refs: list[str] = Field(default_factory=list)


def action_timeout(request) -> float:
    params = request.params
    if request.task in ("single_trajectory", "both_trajectory"):
        return (params.num_points or params.max_num_points) * params.timeout + 15.0
    if request.task == "force_align":
        return params.timeout_s + 10.0
    return params.timeout + 10.0
