"""woosh_move_native 的输入、状态和结果。

对应新栈 WooshMove.Demo 的语义 task：
    navigation.relative_move  →  taskName: move_robot   （单轴相对移动）
    navigation.stop           →  taskName: stop         （停止底盘）

新版 ability 的入参是**扁平参数**：target_x / target_y / target_orientation（不再有
semantic_flag + target.pose 嵌套结构）。skill 输入仍是友好的「单轴 + 位移」形式，
翻译在 controller 完成。
"""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, Field, field_validator

StageName = Literal["relative_move"]

Axis = Literal["x", "y", "z"]


class WooshMoveInput(BaseModel):
    """单轴相对移动输入。

    axis / value 语义（与 ability 的 target_x / target_y / yaw 一一对应）：
        x —— 前后，value 单位为米（base_link 前方为正）
        y —— 左右，value 单位为米（base_link 左方为正）
        z —— 原地自转，value 单位为度
    """

    axis: Axis = "x"
    value: float
    speed: float = Field(default=0.1, gt=0)
    tolerance: float | None = Field(default=None, gt=0)
    timeout: float = Field(default=10.0, gt=0)

    @field_validator("value")
    @classmethod
    def _value_must_be_nonzero(cls, value: float) -> float:
        """value 为 0 时 ability 会判为非单轴运动并报错，这里提前拦下。"""

        if abs(value) < 1e-9:
            raise ValueError(
                "value 必须非零：ability 要求 target_x/target_y/yaw 中恰好一个方向非零（单轴运动）"
            )
        return value


class WooshMoveState(BaseModel):
    stage: StageName = "relative_move"
    invocation_id: str | None = None


class WooshMoveResult(BaseModel):
    axis: Axis
    value: float
    evidence_refs: list[str] = Field(default_factory=list)


class RelativeMoveParameters(BaseModel):
    """navigation.relative_move（taskName: move_robot）的扁平入参。

    三个方向 target_x / target_y / yaw(由 target_orientation 换算) 中恰好一个非零。
    未选中的方向显式送 0，避免依赖 ability 侧默认值。
    """

    target_x: float = 0.0
    target_y: float = 0.0
    target_orientation: list[float] = Field(
        default=[0.0, 0.0, 0.0, 1.0], min_length=4, max_length=4
    )
    speed: float
    tolerance: float
    timeout: float


class StopParameters(BaseModel):
    """navigation.stop（taskName: stop）的入参：无业务参数，invocation_id 由 Pilot 注入。"""
