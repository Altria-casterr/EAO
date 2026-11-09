"""只组装现有 Ability manifest 声明的参数；不直接调用 Robot SDK。"""
from semantic_robot_skill_sdk import Action
from .models import BothPoses, ForceSettings, GraspInput, MotionSettings


def read_state(name: str) -> Action:
    return Action(type="robot.get_state", parameters={"observation": name},
                  timeout_seconds=10, label=f"读取 {name}")


def read_pose(side: str) -> Action:
    return Action(type="robot.get_sensors_state", parameters={"observation": f"ee_pose_{side}"},
                  timeout_seconds=10, label=f"读取{side}末端位姿")


def torso(params: GraspInput) -> Action:
    return Action(type="torso.move_to_posture_all", parameters={
        "positions": list(params.torso.positions), "timeout": params.torso.timeout,
        "tol": params.torso.tol, "speeds": list(params.torso.speeds),
    }, timeout_seconds=params.torso.timeout + 10, label="调整躯干四关节")


def stand(params: GraspInput) -> Action:
    return Action(type="torso.move_to_posture_all", parameters={
        "positions": [0.0, 0.0, 0.0, 0.0], **params.stand.model_dump(mode="json"),
    }, timeout_seconds=params.stand.timeout + 10, label="站起：躯干四关节回零")


def match_target(params: GraspInput, target) -> Action:
    values = params.perception.model_dump(exclude={"timeout_seconds", "minimum_confidence"})

    return Action(type="perception.match_target", parameters={
        **values, "target_point": list(target), "robot_frame": params.relative_frame,
        
    }, timeout_seconds=params.perception.timeout_seconds, label="匹配世界目标对应的箱子")


def both(params: GraspInput, poses: BothPoses, *, trajectory: bool, label: str,
         motion: MotionSettings | None = None) -> Action:
    """组装双臂位姿/轨迹动作。

    motion 不传时使用 params.motion（lift 的携箱轨迹）；approach 传入
    params.approach_motion 以单独控制点数、单点超时与插值顺序。
    """
    settings = params.motion if motion is None else motion
    values = {
        "left_target_pose": poses.left.action_pose(), "right_target_pose": poses.right.action_pose(),
        "left_frame_id": params.relative_frame, "right_frame_id": params.relative_frame,
        "left_pos_tol": settings.pos_tol, "right_pos_tol": settings.pos_tol,
        "left_orient_tol": settings.orient_tol, "right_orient_tol": settings.orient_tol,
        "timeout": settings.timeout, "loop_freq": settings.loop_freq,
    }
    point_budget = 1
    if trajectory:
        values.update(max_num_points=settings.max_num_points,
                      speed=settings.speed, left_sequential=settings.sequential, right_sequential=settings.sequential,
                      left_axis_order=settings.axis_order, right_axis_order=settings.axis_order, step_distance=settings.step_distance)
        if settings.num_points is not None:
            values["num_points"] = settings.num_points
            point_budget = settings.num_points
        else:
            # 实际起点由 Ability 读取；以自动点数上限预留完整等待预算。
            point_budget = settings.max_num_points
    # timeout 是单点超时；自动点数模式按上限预留 Action 总预算。
    return Action(
        type="motion.move_end_effector_both_trajectory" if trajectory else "motion.move_end_effector_both",
        parameters=values,
        timeout_seconds=settings.timeout * point_budget + 15,
        label=label,
    )


def force(side: str, settings: ForceSettings, label: str) -> Action:
    return Action(type="manipulation.force_align", parameters={
        **settings.model_dump(mode="json"), "axis": side, "move_coordinate": "y",
        "direction": "negative" if side == "left" else "positive",
    }, timeout_seconds=settings.timeout_s + 10, label=label)


def capture(params: GraspInput) -> Action:
    return Action(type="robot.capture_rgbd", parameters={
        "sensor_id": params.perception.sensor_id, "axis": "rgb",
    }, timeout_seconds=15, label="记录收箱后的 RGB 图像")
