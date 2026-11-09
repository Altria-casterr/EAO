"""gripper_native 的输入、状态和结果。

两段流程，每段使用自己独立的输入/参数模型，互不共用字段：
1. set_opening_single —— 单侧夹爪（SetOpeningSingleParams）
2. set_opening_both   —— 双侧夹爪（SetOpeningBothParams）

注意 position 系列均为百分制（0-100），不是米。
"""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, Field

StageName = Literal["set_opening_single", "set_opening_both"]


class SetOpeningSingleParams(BaseModel):
    """单侧夹爪段的独立模型：既是 skill 输入，也直接作为 Action 参数。

    对应 ability task：HandleSingleGripper（gripper.set_opening_single）。
    """

    hand: Literal["left", "right"] = "left"
    position: float = Field(default=50.0, ge=0.0, le=100.0)
    speed: float = Field(default=0.5, ge=0.0, le=1.0)
    effort: float = Field(default=0.3, ge=0.0, le=1.0)


class SetOpeningBothParams(BaseModel):
    """双侧夹爪段的独立模型：既是 skill 输入，也直接作为 Action 参数。

    对应 ability task：HandleBothGripper（gripper.set_opening_both）。
    """

    hand: Literal["both"] = "both"
    left_position: float = Field(default=50.0, ge=0.0, le=100.0)
    right_position: float = Field(default=50.0, ge=0.0, le=100.0)
    left_speed: float = Field(default=0.5, ge=0.0, le=1.0)
    right_speed: float = Field(default=0.5, ge=0.0, le=1.0)
    left_effort: float = Field(default=0.3, ge=0.0, le=1.0)
    right_effort: float = Field(default=0.3, ge=0.0, le=1.0)


class GripperNativeInput(BaseModel):
    """两段输入：每段挂自己的独立模型，互不共用。"""

    single: SetOpeningSingleParams = Field(default_factory=SetOpeningSingleParams)
    both: SetOpeningBothParams = Field(default_factory=SetOpeningBothParams)


class GripperNativeState(BaseModel):
    stage: StageName = "set_opening_single"
    invocation_ids: dict[str, str] = Field(default_factory=dict)


class GripperNativeResult(BaseModel):
    single_hand: str
    single_target_position: float
    both_left_position: float
    both_right_position: float
    evidence_refs: list[str] = Field(default_factory=list)
