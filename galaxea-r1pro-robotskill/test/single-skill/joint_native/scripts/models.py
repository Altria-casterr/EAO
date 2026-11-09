"""手臂关节目标输入；角度和容差单位为弧度。"""

from __future__ import annotations

from typing import Annotated, Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator


FiniteFloat = Annotated[float, Field(allow_inf_nan=False)]
SevenAngles = tuple[FiniteFloat, FiniteFloat, FiniteFloat, FiniteFloat, FiniteFloat, FiniteFloat, FiniteFloat]
ArmNames = tuple[str, ...]


def _check_names(names: ArmNames, side: str) -> ArmNames:
    if names == (f"{side}_arm",):
        return names
    if len(names) != 7 or any(not name.strip() for name in names) or len(set(names)) != 7:
        raise ValueError(f"{side} 关节名称须为单个臂组名称 {side}_arm，或 7 个非空且不重复的关节名称")
    return names


class SingleArmParams(BaseModel):
    model_config = ConfigDict(extra="forbid")

    arm_side: Literal["left", "right"] = "left"
    positions: SevenAngles
    joint_names: ArmNames
    velocities: SevenAngles | None = None
    effort: SevenAngles | None = None
    tol: FiniteFloat = Field(default=0.01, gt=0)
    timeout: FiniteFloat = Field(default=10.0, gt=0)
    loop_freq: FiniteFloat = Field(default=0.2, gt=0)

    @field_validator("joint_names")
    @classmethod
    def valid_names(cls, names: ArmNames, info) -> ArmNames:
        return _check_names(names, info.data.get("arm_side", "left"))


class BothArmsParams(BaseModel):
    model_config = ConfigDict(extra="forbid")

    left_positions: SevenAngles
    right_positions: SevenAngles
    left_joint_names: ArmNames
    right_joint_names: ArmNames
    left_velocities: SevenAngles | None = None
    right_velocities: SevenAngles | None = None
    left_effort: SevenAngles | None = None
    right_effort: SevenAngles | None = None
    tol: FiniteFloat = Field(default=0.01, gt=0)
    timeout: FiniteFloat = Field(default=10.0, gt=0)
    loop_freq: FiniteFloat = Field(default=0.2, gt=0)

    @field_validator("left_joint_names", "right_joint_names")
    @classmethod
    def valid_names(cls, names: ArmNames, info) -> ArmNames:
        return _check_names(names, "left" if info.field_name.startswith("left") else "right")


class JointNativeInput(BaseModel):
    model_config = ConfigDict(extra="forbid")

    single: SingleArmParams
    both: BothArmsParams


class JointNativeState(BaseModel):
    stage: Literal["single", "both"] = "single"
    invocation_ids: dict[str, str] = Field(default_factory=dict)
    evidence_refs: list[str] = Field(default_factory=list)


class JointNativeResult(BaseModel):
    single_arm_side: Literal["left", "right"]
    single_positions: SevenAngles
    left_positions: SevenAngles
    right_positions: SevenAngles
    invocation_ids: dict[str, str] = Field(default_factory=dict)
    evidence_refs: list[str] = Field(default_factory=list)
