"""只组装现有 Ability manifest 声明的参数；不直接调用 Robot SDK。"""
from semantic_robot_skill_sdk import Action
from .models import BothPoses, ForceSettings, GraspInput, ToolPose


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


def single(side: str, pose: ToolPose, *, trajectory: bool, label: str,
           params: GraspInput) -> Action:
    """构造单手 motion action：trajectory=True 走 move_end_effector_single_trajectory。"""
    motion = params.motion
    values = {
        "axis": side,
        "_target_pose": pose.action_pose(),
        "frame_id": params.relative_frame,
        "pos_tol": motion.pos_tol,
        "orient_tol": motion.orient_tol,
        "timeout": motion.timeout,
        "loop_freq": motion.loop_freq,
    }
    # 单臂 Ability 会对 num_points 执行 int()，不能传 None；未指定时使用
    # motion.max_num_points 作为显式点数和等待预算，不伪称底层已自动算点。
    num_points = motion.num_points if motion.num_points is not None else motion.max_num_points
    if trajectory:
        values.update(num_points=num_points, max_num_points=motion.max_num_points,
                      speed=motion.speed, axis_order=motion.axis_order, sequential=motion.sequential,
                      step_distance=motion.step_distance)
    return Action(
        type="motion.move_end_effector_single_trajectory" if trajectory else "motion.move_end_effector_single",
        parameters=values, timeout_seconds=motion.timeout * (num_points if trajectory else 1) + 15,
        label=label,
    )


def both(params: GraspInput, poses: BothPoses, *, trajectory: bool, label: str) -> Action:
    """lift 的双手动作参数与稳定双手 Skill 一致；单手 approach 不调用此函数。"""
    settings = params.motion
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
            # 与双手 Skill 一致：自动点数按上限预留完整等待预算。
            point_budget = settings.max_num_points
    return Action(
        type="motion.move_end_effector_both_trajectory" if trajectory else "motion.move_end_effector_both",
        parameters=values,
        timeout_seconds=settings.timeout * point_budget + 15,
        label=label,
    )


def force(side: str, settings: ForceSettings, label: str) -> Action:
    # 沿用双手约定：left→negative、right→positive。
    return Action(type="manipulation.force_align", parameters={
        **settings.model_dump(mode="json"), "axis": side, "move_coordinate": "y",
        "direction": "negative" if side == "left" else "positive",
    }, timeout_seconds=settings.timeout_s + 10, label=label)


def capture(params: GraspInput) -> Action:
    return Action(type="robot.capture_rgbd", parameters={
        "sensor_id": params.perception.sensor_id, "axis": "rgb",
    }, timeout_seconds=15, label="记录收箱后的 RGB 图像")


def read_force() -> Action:
    return Action(type="robot.get_sensors_state", parameters={"observation": "force_data"},
                  timeout_seconds=10, label="读取主抓手三轴力反馈")
