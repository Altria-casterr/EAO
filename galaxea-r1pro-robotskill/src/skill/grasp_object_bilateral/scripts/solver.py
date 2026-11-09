"""适配已有两个 solver，显式统一单位、坐标系和数据形状。"""
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
    # randomize_capture_y=False：两侧 Y 取最靠近箱缘的安全点（确定性），与后续
    # force_align 的最大行程一致；姿态由 solver 固定 RPY 给出，不使用当前末端四元数。
    # box_height 传进去只保留兼容性——当前上游 move_box_to_center 已不使用它。
    return CapturePointSolver(
        box_long=size.long_m,
        box_height=size.height_m,
        box_width=size.width_m,
        randomize_capture_y=False,
    )


def capture_points(params: GraspInput, center: Point, current: BothPoses) -> BothPoses:
    """双手预备点：X 用匹配中心 + capture_x_offset_m，Y 取最近安全点，Z 加 capture_z_offset_m。"""
    return BothPoses.model_validate(capture_solver(params).get_capture_point(
        current.left.sensor_data(params.relative_frame), current.right.sensor_data(params.relative_frame),
        dict(x=center[0] + params.capture_x_offset_m, y=center[1], z=center[2]),
        axis="both", z_lift=params.capture_z_offset_m,
        safe_offset=params.safe_offset_m,
    ))


def lift_points(current: BothPoses, lift_m: float = 0.1) -> BothPoses:
    """Z 方向平移 lift_m 米。默认值与 GraspInput.lift_m 一致；调用方应显式传入。"""
    return BothPoses(**{side: getattr(current, side).model_copy(update={"z": getattr(current, side).z + lift_m})
                        for side in ("left", "right")})


def center_points(params: GraspInput, current: BothPoses) -> BothPoses:
    # 注意：center_z_offset_m / center_x_offset_m 会作为 end_pose_*_offset 传给上游，
    # 但当前上游 move_box_to_center 已注释掉旧公式、不再引用这两个形参，
    # 实际点位是 x=当前左末端x-width/4、z=当前左末端z（抬升后的高度）。
    result = capture_solver(params).move_box_to_center(
        current.left.sensor_data(params.relative_frame), current.right.sensor_data(params.relative_frame),
        end_pose_z_offset=params.center_z_offset_m, end_pose_x_offset=params.center_x_offset_m,
    )

    if result is None:
        raise ValueError("move_box_to_center 没有返回双手目标")
    return BothPoses(**{side: result[side]["target"] for side in ("left", "right")})
