"""纯数值求解；沿用现场 TargetPointSolver 的 map 转换约定。"""
import math

import numpy as np
from pydantic import TypeAdapter
from solver.capture_point_solver import CapturePointSolver
from solver.capture_point_solver.capture_point_solver import (
    CAPTURE_EULER_SEQ, LEFT_CAPTURE_RPY, RIGHT_CAPTURE_RPY,
)
from solver.target_point_solver import TargetPointSolver

from .models import BothPoses, ForceSample, FrameTransform, PlaceInput, Point, ToolPose


def transform(data: dict, target_frame: str = 'torso_link4') -> FrameTransform:
    if target_frame != 'torso_link4':
        raise ValueError(f'floating_base 只能用于 base_link → torso_link4，收到 {target_frame!r}')
    position, orientation = data['position'], data['orientation']
    xyz = [position[k] for k in 'xyz'] if isinstance(position, dict) else position
    quat = [orientation[k] for k in 'xyzw'] if isinstance(orientation, dict) else orientation

    if len(xyz) != 3 or len(quat) != 4:
        raise ValueError('变换需要完整 XYZ 和 xyzw 四元数')

    pose = ToolPose(x=xyz[0], y=xyz[1], z=xyz[2],
                    qx=quat[0], qy=quat[1], qz=quat[2], qw=quat[3])

    return FrameTransform(source_frame='base_link', target_frame=target_frame,
                          position=(pose.x, pose.y, pose.z),
                          orientation=(pose.qx, pose.qy, pose.qz, pose.qw))


def relative_target(params: PlaceInput, map_transform: dict, floating: FrameTransform):
    if floating.source_frame != 'base_link' or floating.target_frame != params.relative_frame:
        raise ValueError('floating_base 的源/目标坐标系与放置求解坐标系不一致')
    world = transform(map_transform)

    return TypeAdapter(Point).validate_python(TargetPointSolver().solve(
        params.target.position_m,
        world.transform_data(), floating.transform_data(),
        trans_flag=True, mm_scale=1.0,
    ))


def pose(data: dict, floating: FrameTransform, frame: str = 'torso_link4') -> ToolPose:
    position, orientation = data['pose']['position'], data['pose']['orientation']
    original = ToolPose(x=position['x'], y=position['y'], z=position['z'],
                        qx=orientation['x'], qy=orientation['y'],
                        qz=orientation['z'], qw=orientation['w'])
    actual_frame = str(data.get('header', {}).get('frame_id', '')).strip().strip('/')
    if actual_frame == frame:
        return original
    if actual_frame != floating.source_frame or frame != floating.target_frame:
        raise ValueError(f'末端坐标系 {actual_frame!r} 无法转换到 solver 坐标系 {frame!r}')
    rotation = TargetPointSolver._quat_to_rotation(floating.orientation)
    xyz = rotation @ np.asarray([original.x, original.y, original.z]) + np.asarray(floating.position)
    lx, ly, lz, lw = floating.orientation
    rx, ry, rz, rw = original.qx, original.qy, original.qz, original.qw
    return ToolPose(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]),
                    qx=lw*rx+lx*rw+ly*rz-lz*ry, qy=lw*ry-lx*rz+ly*rw+lz*rx,
                    qz=lw*rz+lx*ry-ly*rx+lz*rw, qw=lw*rw-lx*rx-ly*ry-lz*rz)


def forces(data: dict) -> ForceSample:
    return ForceSample.model_validate({side: {axis: data[f'{side}_arm'][f'force_{axis}']
                                             for axis in 'xyz'} for side in ('left', 'right')})


def shifted(pose: ToolPose, **offsets) -> ToolPose:

    values = pose.model_dump()

    for axis, distance in offsets.items():
        values[axis] += distance

    return ToolPose.model_validate(values)


def capture_solver(params: PlaceInput) -> CapturePointSolver:
    size = params.target.size
    return CapturePointSolver(
        box_long=size.long_m,
        box_height=size.height_m,
        box_width=size.width_m,
        randomize_capture_y=False,
    )


def preposition(params: PlaceInput, center: Point, initial: BothPoses) -> BothPoses:
    """X/Z 使用放置目标与预定位偏移，Y 保持左右手各自的起始值。

    不再计算箱缘 Y 安全带或按箱长推出另一只手；放置四元数沿用共享求解器约定。
    """
    capture = capture_solver(params)
    targets = {}
    for side, rpy in (('left', LEFT_CAPTURE_RPY), ('right', RIGHT_CAPTURE_RPY)):
        qx, qy, qz, qw = capture.euler_to_quaternion(list(rpy), CAPTURE_EULER_SEQ)
        targets[side] = ToolPose(
            x=center[0] + params.preposition.x_offset_m,
            y=getattr(initial, side).y,
            z=center[2] + params.preposition.z_offset_m,
            qx=qx, qy=qy, qz=qz, qw=qw,
        )
    return BothPoses(**targets)


def step_targets(start: BothPoses, axis: str, distance: float, side: str | None) -> BothPoses:
    return BothPoses(**{hand: shifted(getattr(start, hand), **{axis: distance})
                       if side is None or hand == side else getattr(start, hand)
                       for hand in ('left', 'right')})


def hand_target_m(params: PlaceInput, box_center: Point) -> Point:
    """箱体中心目标 → 双手持箱目标（torso_link4，米）。

    末端经托举夹具持箱，双手目标 = 箱心 - grasp_center_offset_m（末端与夹具
    的偏移），只作用于 X/Z；Y 保持箱心，供撤手箱缘 Y 几何
    继续使用。
    """

    offset = params.preposition.grasp_center_offset_m
    return (box_center[0] - offset[0], box_center[1], box_center[2] - offset[2])


def position_offsets(center: Point, current: BothPoses) -> dict[str, float]:
    """由持箱目标与实测双手中点计算 Z/X 移动偏移量（torso_link4，米）。

    center 已是减去 grasp_center_offset_m 的双手目标，不能再重复减偏移。
    """

    return {
        'z': center[2] - (current.left.z + current.right.z) / 2,
        'x': center[0] - (current.left.x + current.right.x) / 2,
    }


def position_targets(center: Point, current: BothPoses, axis: str,
                     initial: BothPoses) -> BothPoses:
    """先读实测双手位姿（经 floating_base 转到 torso_link4），再按当前位置与
    持箱目标的偏移生成绝对目标：双手同步平移该轴，保持手间相对
    X/Z 相对位置与各自姿态不变，Y 固定为各手保存的起始值。只支持 Z/X。"""

    if axis not in ('x', 'z'):
        raise ValueError(f'最终位置调整只支持 Z/X，收到 {axis!r}')
    targets = step_targets(current, axis, position_offsets(center, current)[axis], None)
    return BothPoses(**{
        side: getattr(targets, side).model_copy(update={'y': getattr(initial, side).y})
        for side in ('left', 'right')
    })


def retract(params: PlaceInput, center: Point, side: str, current: ToolPose) -> list[ToolPose]:
    """复用共享 CapturePointSolver.move_to_retract_point：降 Z → 外撤 Y → 抬升并回撤 X。

    三个点顺序 z / y / x 直接来自共享求解器，与 Put_Down/Agibot_G2 参考实现一致；
    最后一段目标 Z = 撤手开始时的 Z + safe_offset_z（默认 0.03 米）；
    X 回撤 backward_m，Y 保持外撤位置。lift_m 保留但不参与当前计算。
    """

    current_data = {
        'header': {'frame_id': params.relative_frame},
        'pose': {
            'position': {'x': current.x, 'y': current.y, 'z': current.z},
            'orientation': {
                'x': current.qx, 'y': current.qy,
                'z': current.qz, 'w': current.qw,
            },
        },
    }
    result = capture_solver(params).move_to_retract_point(
        current_data,
        box_middle_y=center[1],
        axis=side,
        offset_z=params.retract.lower_m,
        save_offset_y=params.retract.minimum_outward_m,
        save_offset_x=params.retract.backward_m,
        safe_offset_z=params.retract.safe_offset_z,
    )
    if not isinstance(result, dict) or not all(k in result for k in ('z', 'y', 'x')):
        raise ValueError('撤手点求解失败或返回 None')
    return [ToolPose(**result[k]) for k in ('z', 'y', 'x')]


def within(actual: ToolPose, target: ToolPose, pos_tol: float, orient_tol: float) -> bool:
    position_error = math.dist((actual.x, actual.y, actual.z), (target.x, target.y, target.z))
    dot = sum(getattr(actual, k) * getattr(target, k) for k in ('qx', 'qy', 'qz', 'qw'))
    angle = 2 * math.acos(min(1.0, abs(dot)))
    return position_error <= pos_tol + 1e-9 and angle <= orient_tol + 1e-9
