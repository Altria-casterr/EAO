"""只产生现有 schema-v2 Action，不连接 Robot SDK 或 ROS。"""
from semantic_robot_skill_sdk import Action

from .models import BothPoses, PlaceInput, TrajectorySettings


def read(name: str, *, state=False, timeout=10.0) -> Action:
    return Action(type='robot.get_state' if state else 'robot.get_sensors_state',
                  parameters={'observation': name}, timeout_seconds=timeout, label=f'读取 {name}')


def torso(params: PlaceInput) -> Action:
    return Action(type='torso.move_to_posture_all', parameters={
        'positions': list(params.torso.positions), 'timeout': params.torso.timeout,
        'tol': params.torso.tol, 'speeds': list(params.torso.speeds),
    }, timeout_seconds=params.torso.timeout + 10, label='调整躯干四关节')


def both(params: PlaceInput, targets: BothPoses, *, trajectory=False,
         motion: TrajectorySettings | None = None) -> Action:
    settings = params.motion if motion is None else motion
    limit = settings.timeout
    tolerance = settings.pos_tol
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
                  label='双手轨迹' if trajectory else '双手同步位置调整')
