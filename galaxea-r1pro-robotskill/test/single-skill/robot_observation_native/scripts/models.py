from typing import Any, Literal
from pydantic import BaseModel, ConfigDict, Field, model_validator

StateName = Literal['snapshot', 'torso_status', 'floating_base', 'transformer_martix']
SensorName = Literal['ee_pose_left', 'ee_pose_right', 'gripper_state_left', 'gripper_state_right', 'joint_state_left', 'joint_state_right', 'force_data', 'torso_positions', 'chassis_feedback', 'localization']


class ObservationInput(BaseModel):
    model_config = ConfigDict(extra='forbid')
    state_observations: list[StateName] = Field(default_factory=lambda: ['snapshot'])
    sensor_observations: list[SensorName] = Field(default_factory=lambda: ['joint_state_left', 'joint_state_right', 'torso_positions'])
    capture_rgbd: bool = True
    sensor_id: str = 'head_camera'
    timeout_seconds: float = Field(default=30.0, gt=0, allow_inf_nan=False)

    @model_validator(mode='after')
    def valid_selection(self):
        if not self.state_observations and not self.sensor_observations and not self.capture_rgbd:
            raise ValueError('至少选择一个观测项')
        if self.capture_rgbd and not self.sensor_id.strip():
            raise ValueError('采集 RGB-D 时 sensor_id 不能为空')
        return self


class ObservationState(BaseModel):
    next_index: int = Field(default=0, ge=0)
    readings: list[dict[str, Any]] = Field(default_factory=list)
    evidence_refs: list[str] = Field(default_factory=list)


class ObservationResult(BaseModel):
    overall_status: Literal['succeeded', 'partial', 'failed']
    readings: list[dict[str, Any]]
    evidence_refs: list[str] = Field(default_factory=list)
