from typing import Any, Literal
from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator


class YoloInput(BaseModel):
    model_config = ConfigDict(extra='forbid')
    mode: Literal['detect', 'point_cloud', 'both', 'match', 'pose'] = 'detect'
    sensor_id: str = 'head_camera'
    conf_threshold: float = Field(default=0.25, ge=0, le=1, allow_inf_nan=False)
    iou_threshold: float = Field(default=0.7, ge=0, le=1, allow_inf_nan=False)
    timeout_seconds: float = Field(default=120, gt=0, le=240, allow_inf_nan=False)
    require_objects: bool = False
    gray_box_only: bool = Field(default=False, strict=True)
    top_k: int = Field(default=2, ge=1, strict=True)
    target_point: tuple[float, float, float] | None = None
    target_input_frame: str = 'map'
    target_frame: str = 'base_link'
    trans_flag: bool = True
    rosbridge_host: str = '172.16.70.147'
    rosbridge_port: int = Field(default=9091, ge=1, le=65535)
    transform_timeout_s: float = Field(default=5, gt=0, le=30, allow_inf_nan=False)
    transform_max_age_s: float = Field(default=1, gt=0, le=5, allow_inf_nan=False)
    reference_points: list[tuple[float, float, float]] | None = None
    reference_frame: str | None = None
    reference_dir: str | None = None
    reference_camera: dict[str, Any] | None = None
    reference_target_point: tuple[float, float, float] | None = None
    reference_class_name: str | None = None
    yaw_range: tuple[float, float] = (-90, 90)
    search_step: float = Field(default=2, gt=0, allow_inf_nan=False)
    focus_step: float = Field(default=0.5, gt=0, allow_inf_nan=False)
    enable_roll: bool = False
    enable_pitch: bool = False
    enable_yaw: bool = True

    @model_validator(mode='after')
    def validate_target(self):
        import math
        if self.mode in ('match', 'pose') and self.target_point is None:
            raise ValueError('match/pose 模式必须提供 target_point，单位米')
        if self.target_point is not None and self.mode not in ('match', 'pose'):
            raise ValueError('传 target_point 时请使用 match/pose 模式，避免忽略目标点')
        if self.target_point is not None and not all(math.isfinite(x) for x in self.target_point):
            raise ValueError('target_point 必须是有限数')
        for key in ('target_input_frame','target_frame'):
            value=getattr(self,key).strip().strip('/')
            if not value:raise ValueError(key+' 不能为空')
            setattr(self,key,value)
        if not self.trans_flag and self.target_input_frame != self.target_frame:
            raise ValueError('关闭转换时，输入目标必须已在 target_frame 坐标系')
        if self.mode == 'pose':
            if self.reference_points is not None:
                if self.reference_dir or self.reference_camera is not None:
                    raise ValueError('参考点云和参考 RGB-D 只能选择一种')
                if len(self.reference_points) < 3 or not all(math.isfinite(v) for p in self.reference_points for v in p):
                    raise ValueError('reference_points 要求至少三个有限三维点')
                if (self.reference_frame or '').strip().strip('/') != self.target_frame:
                    raise ValueError('参考点云必须与 target_frame 相同')
                self.reference_frame = self.target_frame
            elif not self.reference_dir or not self.reference_camera:
                raise ValueError('pose 需要 reference_points 或 reference_dir + reference_camera')
            if self.reference_target_point and not all(math.isfinite(v) for v in self.reference_target_point):
                raise ValueError('reference_target_point 必须有限')
            if not all(math.isfinite(v) for v in self.yaw_range) or self.yaw_range[0] >= self.yaw_range[1]:
                raise ValueError('yaw_range 必须有限且递增')
            if not (self.enable_roll or self.enable_pitch or self.enable_yaw):
                raise ValueError('姿态估计至少开启一个旋转轴')
        return self

    @field_validator('sensor_id')
    @classmethod
    def sensor_not_empty(cls, value):
        if not value.strip():
            raise ValueError('sensor_id 不能为空')
        return value.strip()


class YoloState(BaseModel):
    next_index: int = Field(default=0, ge=0)
    readings: list[dict[str, Any]] = Field(default_factory=list)
    evidence_refs: list[str] = Field(default_factory=list)


class YoloResult(BaseModel):
    overall_status: Literal['succeeded', 'partial', 'failed']
    readings: list[dict[str, Any]]
    evidence_refs: list[str] = Field(default_factory=list)
