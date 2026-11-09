"""只产生现有 schema-v2 Action，不连接 Robot SDK 或 ROS。"""
from semantic_robot_skill_sdk import Action

from .models import BothPoses, ForceSettings, PlaceInput, ToolPose, TrajectorySettings


def read(name: str, *, state=False, timeout=65.0) -> Action:
    return Action(type='robot.get_state' if state else 'robot.get_sensors_state',
                  parameters={'observation': name}, timeout_seconds=timeout, label=f'读取 {name}')


def torso(params: PlaceInput) -> Action:
    return Action(type='torso.move_to_posture_all', parameters={
        'positions': list(params.torso.positions), 'timeout': params.torso.timeout,
        'tol': params.torso.tol, 'speeds': list(params.torso.speeds),
    }, timeout_seconds=params.torso.timeout + 10, label='调整躯干四关节')


def both(params: PlaceInput, targets: BothPoses, *, trajectory=False,
         fine_tol=None, timeout=None, motion: TrajectorySettings | None = None) -> Action:
    settings = params.motion if motion is None else motion
    limit = settings.timeout if timeout is None else timeout
    tolerance = settings.pos_tol if fine_tol is None else fine_tol
    values = dict(left_target_pose=targets.left.action_pose(), right_target_pose=targets.right.action_pose(),
                  left_frame_id=params.relative_frame, right_frame_id=params.relative_frame,
                  left_pos_tol=tolerance, right_pos_tol=tolerance,
                  left_orient_tol=settings.orient_tol, right_orient_tol=settings.orient_tol,
                  timeout=limit, loop_freq=min(settings.loop_freq, limit))
    point_budget = 1
    if trajectory:
        values.update(max_num_points=settings.max_num_points, speed=settings.speed,
                      left_sequential=settings.sequential, right_sequential=settings.sequential,
                      left_axis_order=settings.axis_order, right_axis_order=settings.axis_order,
                      step_distance=settings.step_distance)
        if settings.num_points is not None:
            values['num_points'] = settings.num_points
        point_budget = settings.num_points or settings.max_num_points
    return Action(type='motion.move_end_effector_both_trajectory' if trajectory
                  else 'motion.move_end_effector_both', parameters=values,
                  timeout_seconds=limit * point_budget + 15,
                  label='双手预定位轨迹' if trajectory else '双手同步位置调整')


def single(params: PlaceInput, side: str, target: ToolPose, *, trajectory=False,
           fine_tol=None, timeout=None, motion: TrajectorySettings | None = None,
           point_cap: int | None = None) -> Action:
    settings = params.motion if motion is None else motion
    limit = settings.timeout if timeout is None else timeout
    values = {
        'axis': side, '_target_pose': target.action_pose(), 'frame_id': params.relative_frame,
        'pos_tol': settings.pos_tol if fine_tol is None else fine_tol,
        'orient_tol': settings.orient_tol, 'timeout': limit, 'loop_freq': min(settings.loop_freq, limit),
    }
    point_budget = 1
    if trajectory:
        # 当前单手 Ability 对 num_points 执行 int()，自动档必须显式使用上限。
        point_budget = settings.num_points or settings.max_num_points
        # 点数直接决定耗时上界：Ability 对【每个轨迹点】各等 timeout 秒，且中间点
        # 超时不会中止后续点（只检查最后一个点），所以最坏耗时 = 点数 × timeout，
        # 同时 Action 级超时按同样的乘积计算。封顶把最坏耗时压到 point_cap × timeout，
        # 避免手臂被压住时长时间保持危险姿态。
        if point_cap is not None:
            point_budget = min(point_budget, point_cap)
        values.update(num_points=point_budget, max_num_points=settings.max_num_points,
                      speed=settings.speed, sequential=settings.sequential,
                      axis_order=settings.axis_order, step_distance=settings.step_distance)
    return Action(type='motion.move_end_effector_single_trajectory' if trajectory
                  else 'motion.move_end_effector_single', parameters=values,
                  timeout_seconds=limit * point_budget + 15, label=f'{side} 单手移动')


def force(params: PlaceInput, side: str, settings: ForceSettings, label: str) -> Action:
    # 沿用抓取约定：left→negative、right→positive，move_coordinate 固定 y。
    return Action(type='manipulation.force_align', parameters={
        'axis': side, 'move_coordinate': 'y',
        'force_coordinate': list(settings.force_coordinate),
        'force_threshold_n': settings.force_threshold_n,
        'direction': 'negative' if side == 'left' else 'positive',
        'step_size_m': settings.step_size_m,
        'loop_freq_s': settings.loop_freq_s,
        'timeout_s': settings.timeout_s,
    }, timeout_seconds=settings.timeout_s + 15, label=label)
