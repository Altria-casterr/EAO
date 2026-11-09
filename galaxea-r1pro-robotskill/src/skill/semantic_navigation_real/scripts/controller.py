"""构造地图导航和到位力检测 Action。"""

from semantic_robot_skill_sdk import Action

from .models import (
    ManipulationMode,
    NavigateParameters,
    VerifyInput,
    VerifyParameters,
)


NAVIGATION_ACTION_TYPE = "navigation.follow_route"
VERIFY_ACTION_TYPE = "navigation.verify_arrival"


def build_navigation_action(parameters: NavigateParameters) -> Action:
    return Action.from_model(
        action_type=NAVIGATION_ACTION_TYPE,
        parameters=parameters,
        timeout_seconds=parameters.timeout + 10.0,
        label="导航到箱子作业站位（含路径规划和到位检测）",
    )


def build_verify_action(mode: ManipulationMode, verify_input: VerifyInput) -> Action:
    if mode == "both":
        raise ValueError("carry_to_place 不支持 both 模式")
    parameters = VerifyParameters(
        tool_refs=[mode],
        carrying_force_axis=verify_input.carrying_force_axis,
        carrying_min_load_n=verify_input.carrying_min_load_n,
    )
    return Action.from_model(
        action_type=VERIFY_ACTION_TYPE,
        parameters=parameters,
        timeout_seconds=15.0,
        label=f"检测{mode}侧腕力，确认仍携带箱子",
    )
