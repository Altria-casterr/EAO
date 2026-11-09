"""适配已有两个 solver，显式统一单位、坐标系和数据形状。单手抓箱版本。"""
import numpy as np
from pydantic import TypeAdapter
from .models import BothPoses, FrameTransform, GraspInput, Point, ToolPose
from solver.capture_point_solver import CapturePointSolver
from solver.target_point_solver import TargetPointSolver


def point(value) -> tuple[float, float, float]:
    return TypeAdapter(Point).validate_python(value)


def validate_transform(data: dict) -> dict:
    # 原 solver 对零四元数退化为单位矩阵；在 Skill 边界拒绝这种缺失数据。
    position, orientation = data["position"], data["orientation"]
    xyz = point([position[k] for k in "xyz"] if isinstance(position, dict) else position)
    quat = [orientation[k] for k in ("x", "y", "z", "w")] if isinstance(orientation, dict) else orientation

    if len(quat) != 4:
        raise ValueError("坐标变换四元数必须包含四个元素")

    pose = ToolPose(x=xyz[0], y=xyz[1], z=xyz[2], qx=quat[0], qy=quat[1], qz=quat[2], qw=quat[3])

    return {"position": [pose.x, pose.y, pose.z], "orientation": [pose.qx, pose.qy, pose.qz, pose.qw]}


def relative_target(params: GraspInput, transform: dict, floating_base: dict) -> Point:

    # TODO 需要传入 xyz 坐标
    return point(TargetPointSolver().solve(
        params.target.position_m, validate_transform(transform), validate_transform(floating_base),
        trans_flag=True, mm_scale=1.0,
    ))


def floating_base_transform(data: dict, target_frame: str) -> FrameTransform:
    """SDK floating_base 是 base_link 点到 torso_link4 的逆变换。"""

    if target_frame != "torso_link4":
        raise ValueError(f"floating_base 只能用于 base_link → torso_link4，收到 {target_frame!r}")

    value = validate_transform(data)

    return FrameTransform(
        source_frame="base_link", target_frame=target_frame,
        position=tuple(value["position"]), orientation=tuple(value["orientation"]),
    )


def _quat_multiply(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return np.asarray([
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ], dtype=np.float64)


def parse_pose(data: dict, frame: str, base_to_relative: FrameTransform | None = None) -> ToolPose:
    actual = str(data.get("header", {}).get("frame_id", "")).strip().strip("/")
    position, quat = data["pose"]["position"], data["pose"]["orientation"]
    xyz = np.asarray([position[key] for key in "xyz"], dtype=np.float64)
    orientation = np.asarray([quat[key] for key in ("x", "y", "z", "w")], dtype=np.float64)
    if actual != frame:
        transform = base_to_relative
        if transform is None or actual != transform.source_frame or frame != transform.target_frame:
            raise ValueError(f"末端坐标系 {actual!r} 无法转换到 solver 坐标系 {frame!r}")
        rotation = TargetPointSolver._quat_to_rotation(transform.orientation)
        xyz = rotation @ xyz + np.asarray(transform.position, dtype=np.float64)
        orientation = _quat_multiply(transform.orientation, orientation)
    return ToolPose(x=xyz[0], y=xyz[1], z=xyz[2],
                    qx=orientation[0], qy=orientation[1], qz=orientation[2], qw=orientation[3])


def capture_solver(params: GraspInput) -> CapturePointSolver:
    size = params.target.size
    return CapturePointSolver(
        box_long=size.long_m,
        box_height=size.height_m,
        box_width=size.width_m,
        randomize_capture_y=False,
    )

def capture_point(params: GraspInput, center: Point, current: BothPoses) -> ToolPose:
    """单手抓取点：调用 CapturePointSolver.get_capture_point(axis=grasp_axis)，
    返回抓取侧 ToolPose（xyz + 当前四元数）。"""
    result = capture_solver(params).get_capture_point(
        current.left.sensor_data(params.relative_frame),
        current.right.sensor_data(params.relative_frame),
        dict(x=center[0] + params.capture_x_offset_m, y=center[1], z=center[2]),
        axis=params.grasp_axis,
        z_lift=params.capture_z_offset_m,
        safe_offset=params.safe_offset_m,
    )
    if not isinstance(result, dict) or "x" not in result:
        raise ValueError("单手 capture_point 求解失败或返回 None")
    return ToolPose(x=result["x"], y=result["y"], z=result["z"],
                    qx=result["qx"], qy=result["qy"], qz=result["qz"], qw=result["qw"])


def pull_point(params: GraspInput, grasp_current: ToolPose) -> ToolPose:
    """抓取侧沿 y 拉箱：left→+y（远离右侧辅助手），right→-y（远离左侧辅助手）。
    保留当前 x/z 与四元数。"""
    direction = 1.0 if params.grasp_axis == "left" else -1.0
    new_y = grasp_current.y + direction * params.pull_y_distance_m
    return grasp_current.model_copy(update={"y": new_y})


def lift_point(pose: ToolPose, lift_m: float = 0.05) -> ToolPose:
    """单手上抬：z += lift_m，保留其他字段。

    注意：默认值 0.05 仅为历史兼容，本 Skill 的两个调用点都显式传
    skill.GRASP_LIFT_M（0.10）；新增调用点必须显式传参，不要依赖此默认值。"""
    return pose.model_copy(update={"z": pose.z + lift_m})


# 辅助手最终 Z 相对主抓手当前 Z 的下降量（米）；与 skill.GRASP_LIFT_M 各自独立。
OTHER_ARM_Z_LIFT_M = 0.10


def other_arm_position(params: GraspInput, grasp_current: ToolPose,
                       other_current: ToolPose) -> dict[str, ToolPose | str]:
    """返回辅助手沿 Y、Z、X 依次运动的三个目标。
    safe_y_move 使用 pull_y_distance_m 的一半；最终 Z 比主抓手当前高度低
    z_lift（本 Skill 调用点显式传 OTHER_ARM_Z_LIFT_M = 0.10 米；共享 solver 自身
    默认 0.1，勿依赖）。
    不修改双手 Skill 共用的 CapturePointSolver。"""
    result = capture_solver(params).get_other_arm_position(
        axis=params.grasp_axis,
        grasp_data=grasp_current.sensor_data(params.relative_frame),
        other_data=other_current.sensor_data(params.relative_frame),
        init_x=other_current.x,
        safe_y_move=params.pull_y_distance_m / 2,
        z_lift=OTHER_ARM_Z_LIFT_M,
    )
    if not isinstance(result, dict) or "y_move" not in result or "target" not in result:
        raise ValueError("辅助手位置求解失败或返回 None")
    target = result["target"]
    other_axis = result["axis"]
    target_pose = ToolPose(x=target["x"], y=target["y"], z=target["z"],
                           qx=target["qx"], qy=target["qy"], qz=target["qz"], qw=target["qw"])
    y_move_pose = other_current.model_copy(update={"y": target_pose.y})
    # 完成 Y 后先调 Z，X 仍保持预定位值；最后由 target_pose 单独改变 X。
    z_move_pose = other_current.model_copy(update={"y": target_pose.y, "z": target_pose.z})
    return {
        "y_move_pose": y_move_pose,
        "x_move_pose": z_move_pose,
        "target_pose": target_pose,
        "other_axis": other_axis,
    }


def lift_points(current: BothPoses, lift_m: float = 0.05) -> BothPoses:
    """lift 阶段：双手同时 z+lift_m。

    注意：默认值 0.05 仅为历史兼容，调用点显式传 params.lift_m（默认 0.1）。"""
    return BothPoses(**{side: getattr(current, side).model_copy(update={"z": getattr(current, side).z + lift_m})
                        for side in ("left", "right")})


def center_points(params: GraspInput, current: BothPoses) -> BothPoses:
    """verify 阶段前携箱收回：沿用 move_box_to_center。"""
    result = capture_solver(params).move_box_to_center(
        current.left.sensor_data(params.relative_frame),
        current.right.sensor_data(params.relative_frame),
        end_pose_z_offset=params.center_z_offset_m, end_pose_x_offset=params.center_x_offset_m,
    )

    if result is None:
        raise ValueError("move_box_to_center 没有返回双手目标")
    return BothPoses(**{side: result[side]["target"] for side in ("left", "right")})
