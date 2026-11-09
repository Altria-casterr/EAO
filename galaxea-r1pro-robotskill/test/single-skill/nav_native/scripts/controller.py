"""nav_native 的 Action 构造（新栈原生扁平参数 + 自定义 actionType）。

每个 taskName 独立构造：各自的参数模型直接作为 Action 参数，互不共用、互不影响。
"""

from __future__ import annotations

from semantic_robot_skill_sdk import Action

from .models import NavigateParams, StopParameters, VerifyParams

# navigate_to_map_pose 在 ability 内部是同步阻塞的等待闭环，真正的超时由入参 timeout 决定；
# Action 信封的 timeout 留出余量，避免 pilot 在 ability 返回结果前先判超时。
_NAVIGATE_TIMEOUT_MARGIN_S = 10.0


def build_navigate_action(params: NavigateParams) -> Action:
    """构造 navigation.follow_route（taskName: navigate_to_map_pose）。"""

    return Action.from_model(
        action_type="navigation.follow_route",
        parameters=params,
        timeout_seconds=params.timeout + _NAVIGATE_TIMEOUT_MARGIN_S,
        label=f"地图导航到 ({params.target_x}, {params.target_y})",
    )


def build_verify_arrival_action(params: VerifyParams) -> Action:
    """构造 navigation.verify_arrival（taskName: verify_arrival）。"""

    return Action.from_model(
        action_type="navigation.verify_arrival",
        parameters=params,
        timeout_seconds=15,
        label="复核持物状态（腕力）",
    )


def build_stop_action() -> Action:
    """构造 navigation.stop（taskName: stop）：零速发布 + 急停置位。"""

    return Action.from_model(
        action_type="navigation.stop",
        parameters=StopParameters(),
        timeout_seconds=10,
        label="停止底盘运动与导航",
    )
