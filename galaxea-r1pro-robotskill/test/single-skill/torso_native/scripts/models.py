"""躯干原生动作的输入、检查点和结果；位置和容差单位均为弧度。"""

from __future__ import annotations

from typing import Annotated, Literal

from pydantic import BaseModel, Field


FiniteFloat = Annotated[float, Field(allow_inf_nan=False)]


class SingleTorsoParams(BaseModel):
    joint_id: int = Field(ge=0, le=3)
    target_pos: FiniteFloat = Field(allow_inf_nan=False)
    speed: FiniteFloat = Field(default=1.5, gt=0, allow_inf_nan=False)
    effort: tuple[FiniteFloat, FiniteFloat, FiniteFloat, FiniteFloat] | None = None
    names: tuple[str, str, str, str] | None = None
    tol: FiniteFloat = Field(default=0.02, gt=0, allow_inf_nan=False)
    timeout: FiniteFloat = Field(default=8.0, gt=0, allow_inf_nan=False)
    loop_period: FiniteFloat = Field(default=0.1, gt=0, allow_inf_nan=False)


class AllTorsoParams(BaseModel):
    positions: tuple[FiniteFloat, FiniteFloat, FiniteFloat, FiniteFloat]
    speeds: tuple[FiniteFloat, FiniteFloat, FiniteFloat, FiniteFloat] | None = None
    effort: tuple[FiniteFloat, FiniteFloat, FiniteFloat, FiniteFloat] | None = None
    names: tuple[str, str, str, str] | None = None
    tol: FiniteFloat = Field(default=0.02, gt=0, allow_inf_nan=False)
    timeout: FiniteFloat = Field(default=10.0, gt=0, allow_inf_nan=False)
    loop_period: FiniteFloat = Field(default=0.1, gt=0, allow_inf_nan=False)


class TorsoNativeInput(BaseModel):
    single: SingleTorsoParams
    all: AllTorsoParams


class TorsoNativeState(BaseModel):
    stage: Literal["single", "all"] = "single"
    invocation_ids: dict[str, str] = Field(default_factory=dict)
    evidence_refs: list[str] = Field(default_factory=list)


class TorsoNativeResult(BaseModel):
    single_joint_id: int
    single_target_pos: float
    all_positions: tuple[float, float, float, float]
    invocation_ids: dict[str, str] = Field(default_factory=dict)
    evidence_refs: list[str] = Field(default_factory=list)
