"""nav_native 的输入、状态和结果。

对应新栈 WooshNavigation.Demo 的两个业务 task，每个 taskName 一个独立模型：

    navigation.follow_route   →  taskName: navigate_to_map_pose  （NavigateParams）
    navigation.verify_arrival →  taskName: verify_arrival        （VerifyParams）
    navigation.stop           →  taskName: stop                  （StopParameters）

入参为**扁平原生格式**（target_x / target_y / target_orientation，无嵌套 target.pose）：
target_orientation 是 [qx, qy, qz, qw]，ability 只用其中的 yaw 分量作为导航朝向；
target_x / target_y 单位为米，坐标系由 frame_id 指定（默认 map）。
"""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, Field

StageName = Literal["navigate", "verify"]
ForceAxis = Literal["x", "y", "z", "norm"]


class NavigateParams(BaseModel):
    """导航段的独立模型：既是 skill 输入，也直接作为 Action 参数。

    对应 ability task：navigate_to_map_pose（navigation.follow_route）。
    """

    target_x: float = 0.0
    target_y: float = 0.0
    target_orientation: list[float] = Field(
        default=[0.0, 0.0, 0.0, 1.0], min_length=4, max_length=4
    )
    target_point_type: int = 0
    frame_id: str = "map"
    timeout: float = Field(default=60.0, gt=0.0)


class VerifyParams(BaseModel):
    """复核段的独立模型：对应 ability task verify_arrival。

    注意：该 ability task 当前是**腕部力传感持物检查**，并不做位姿/距离到位校验，
    因此这里的参数与到达距离无关，只有检查侧别与载荷阈值。
    """

    tool_refs: list[str] = Field(default_factory=list)
    carrying_force_axis: ForceAxis = "z"
    carrying_min_load_n: float = Field(default=3.0, gt=0.0)


class StopParameters(BaseModel):
    """navigation.stop（taskName: stop）的入参：无业务参数，invocation_id 由 Pilot 注入。"""


class NavNativeInput(BaseModel):
    """两段输入：每段挂自己的独立模型，互不共用。"""

    navigate: NavigateParams = Field(default_factory=NavigateParams)
    verify: VerifyParams = Field(default_factory=VerifyParams)


class NavNativeState(BaseModel):
    stage: StageName = "navigate"


class NavNativeResult(BaseModel):
    target_x: float
    target_y: float
    frame_id: str
    # verify_arrival 的 verdict：achieved / not_achieved，含义是腕力持物判定，不是到位判定
    verify_verdict: str
    holding_object: bool
    evidence_refs: list[str] = Field(default_factory=list)
