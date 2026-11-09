"""woosh_move_native 的 Action 构造。

把「单轴 + 位移」翻译成 ability 的扁平入参，并保证
target_x / target_y / yaw 中恰好一个非零（ability 的单轴约束）：

    x → target_x=value, target_y=0, orientation=[0,0,0,1]  → 仅 x 非零
    y → target_x=0, target_y=value, orientation=[0,0,0,1]  → 仅 y 非零
    z → target_x=0, target_y=0, orientation=绕 Z 的 yaw 四元数 → 仅 yaw 非零
"""

from __future__ import annotations

import math

from semantic_robot_skill_sdk import Action

from .models import RelativeMoveParameters, StopParameters, WooshMoveInput

# ability 侧 move_robot 的到位容差默认值（x/y 为米，z 为度）
_DEFAULT_TOLERANCE = {"x": 0.02, "y": 0.02, "z": 0.5}
# Action 超时相对 ability 内部闭环超时的余量（秒），保证 ability 先超时并返回失败
_ACTION_TIMEOUT_MARGIN_S = 5.0


def _axis_targets(skill_input: WooshMoveInput) -> tuple[float, float, list[float]]:
    """单轴位移 → (target_x, target_y, target_orientation)，确保只有对应分量非零。"""

    if skill_input.axis == "x":
        return skill_input.value, 0.0, [0.0, 0.0, 0.0, 1.0]
    if skill_input.axis == "y":
        return 0.0, skill_input.value, [0.0, 0.0, 0.0, 1.0]

    # z：绕 Z 轴自转，value 为度 → 四元数 [0, 0, sin(θ/2), cos(θ/2)]
    half = math.radians(skill_input.value) / 2.0
    return 0.0, 0.0, [0.0, 0.0, math.sin(half), math.cos(half)]


def build_relative_move_action(skill_input: WooshMoveInput) -> Action:
    """构造 navigation.relative_move（taskName: move_robot）。"""

    target_x, target_y, orientation = _axis_targets(skill_input)
    tolerance = (
        skill_input.tolerance
        if skill_input.tolerance is not None
        else _DEFAULT_TOLERANCE[skill_input.axis]
    )

    return Action.from_model(
        action_type="navigation.relative_move",
        parameters=RelativeMoveParameters(
            target_x=target_x,
            target_y=target_y,
            target_orientation=orientation,
            speed=skill_input.speed,
            tolerance=tolerance,
            timeout=skill_input.timeout,
        ),
        timeout_seconds=skill_input.timeout + _ACTION_TIMEOUT_MARGIN_S,
        label=f"底盘相对移动：{skill_input.axis}={skill_input.value}",
    )


def build_stop_action() -> Action:
    """构造 navigation.stop（taskName: stop）：紧急停止 + 零速发布。"""

    return Action.from_model(
        action_type="navigation.stop",
        parameters=StopParameters(),
        timeout_seconds=10,
        label="停止底盘运动",
    )
