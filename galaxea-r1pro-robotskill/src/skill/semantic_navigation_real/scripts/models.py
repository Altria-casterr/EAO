"""模型提供业务目标；Action 参数逐字段对齐 WooshNavigation manifest。"""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, StrictBool, model_validator


NavigationPurpose = Literal["approach_grasp", "carry_to_place"]
ManipulationMode = Literal["left", "right", "both"]
StageName = Literal["navigation", "verify"]
ForceAxis = Literal["x", "y", "z", "norm"]


class InputModel(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False, str_strip_whitespace=True)


class BoxTarget(InputModel):
    target_ref: str = Field(min_length=1, description="抓取箱子或放置目标的业务引用")
    position_m: tuple[float, float, float] = Field(
        description="frame_id 中未经偏移的原始箱子中心 XYZ；放置模式为箱子期望放置中心，不是手中箱子当前位置"
    )
    yaw_rad: float = Field(description="frame_id 中的绝对作业朝向，弧度；机器人到位后朝向该方向")


class NavigateParameters(InputModel):
    """navigate_to_map_pose 的六个原生参数；目标由 solver 计算。"""

    target_x: float
    target_y: float
    target_orientation: tuple[float, float, float, float]
    target_point_type: int = Field(default=0, strict=True)
    frame_id: str = Field(default="map", min_length=1)
    timeout: float = Field(default=60.0, gt=0)


class VerifyInput(InputModel):
    """模型只提供力检测条件；待检测工具由 mode 唯一确定。"""

    carrying_force_axis: ForceAxis = "z"
    carrying_min_load_n: float = Field(gt=0, description="现场标定的单侧负载阈值，N；禁止模型猜测")


class VerifyParameters(VerifyInput):
    """verify_arrival 原生参数。"""

    tool_refs: list[str]


class SemanticNavigationInput(InputModel):
    navigation_purpose: NavigationPurpose = Field(
        description="approach_grasp 接近抓取；carry_to_place 单手携物前往放置位置"
    )
    mode: ManipulationMode = Field(description="作业手模式：left、right 或 both")
    move_y_m: float | None = Field(
        default=None,
        ge=0,
        description="导航求解前施加到原始目标地图 Y 的偏移幅值，米；方向由 mode/yaw 决定，0 禁用偏移",
    )
    target: BoxTarget
    frame_id: str = Field(default="map", min_length=1, description="输入箱子和输出底盘目标共用的地图坐标系")
    target_point_type: int = Field(default=0, strict=True)
    timeout: float = Field(default=60.0, gt=0, description="透传导航 Ability 的超时秒数")
    verify: VerifyInput | None = Field(default=None, description="携物模式必填；检测 mode 指定侧的腕力")

    @model_validator(mode="after")
    def validate_mode(self) -> "SemanticNavigationInput":
        if self.mode == "both":
            if self.navigation_purpose == "carry_to_place":
                raise ValueError("carry_to_place 只允许 left 或 right 模式")
            if self.move_y_m not in (None, 0):
                raise ValueError("both 模式不偏移，move_y_m 只能省略或为 0")
        elif self.move_y_m is None:
            raise ValueError("left/right 模式必须提供 move_y_m，0 表示不偏移")
        if self.navigation_purpose == "carry_to_place" and self.verify is None:
            raise ValueError("carry_to_place 必须提供 verify，包含现场标定的 carrying_min_load_n")
        if self.navigation_purpose == "approach_grasp" and self.verify is not None:
            raise ValueError("approach_grasp 不执行力检测，请省略 verify")
        return self


class ForceToolResult(BaseModel):
    model_config = ConfigDict(extra="ignore", allow_inf_nan=False)
    force_n: float | None
    loaded: StrictBool


class CarryingCheck(BaseModel):
    """保留 Ability 的判断与数值，不把 raw 传感大对象写入检查点。"""

    model_config = ConfigDict(extra="ignore", allow_inf_nan=False)
    holding: StrictBool
    checked: StrictBool
    force_axis: ForceAxis
    min_load_n: float = Field(gt=0)
    tools: dict[str, ForceToolResult]
    reasons: list[str] = Field(default_factory=list)
    errors: list[str] = Field(default_factory=list)


class VerifyOutput(BaseModel):
    model_config = ConfigDict(extra="ignore")
    status: Literal["completed"]
    verdict: Literal["achieved", "not_achieved"]
    holding_object: StrictBool
    carrying_check: CarryingCheck


class SemanticNavigationState(InputModel):
    stage: StageName = "navigation"
    navigation_target: NavigateParameters | None = None
    navigation_succeeded: bool = False
    evidence_refs: list[str] = Field(default_factory=list)


class SemanticNavigationResult(InputModel):
    navigation_purpose: NavigationPurpose
    mode: ManipulationMode
    target_ref: str
    navigation_succeeded: Literal[True] = True
    navigation_target: NavigateParameters
    holding_object: bool | None
    load_verification: Literal["not_required", "passed"]
    carrying_check: CarryingCheck | None = None
    evidence_refs: list[str] = Field(default_factory=list)
