"""按相机配置解析标定；T_target_source 将 source 中的点映射到 target。"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from typing import Any, Mapping
import numpy as np
from scipy.spatial.transform import Rotation

GEOMETRY_KEYS = ('camera_intrinsics', 'camera_extrinsics', 'transform_camera_to_robot')


def merge_camera_input(cameras, data):

    sensor = str(data.get('sensor_id') or 'head_camera')
    selected = cameras.get(sensor, {})

    if not isinstance(selected, Mapping):
        raise ValueError(f'相机 {sensor} 配置必须是对象')

    result = deepcopy(dict(selected))

    if not any(key in data for key in ('camera_extrinsics', 'transform_camera_to_robot')):
        # 静态标定绑定两端坐标系，修改名字不能当成执行了坐标转换。
        bound_frames = {}

        extrinsics = selected.get('camera_extrinsics') or {}

        if selected.get('transform_camera_to_robot') is not None:
            bound_frames = {'camera_frame': '', 'robot_frame': 'base_link'}

        elif extrinsics:

            bound_frames = {'camera_frame': ''}

            if 'camera_to_base' in extrinsics or 'tcp_to_base' in extrinsics:
                bound_frames['base_frame'] = 'base_link'

        for key, default in bound_frames.items():
            if key in data and frame_name(data[key] or default) != frame_name(selected.get(key) or default):
                raise ValueError(f'修改标定绑定的 {key} 时必须同时提供对应的新外参')

    if 'camera_extrinsics' in data or 'transform_camera_to_robot' in data:
        result.pop('camera_extrinsics', None)
        result.pop('transform_camera_to_robot', None)

    if any(k in data for k in ('fx', 'fy', 'cx', 'cy')):
        result.pop('camera_intrinsics', None)
        
    result.update(deepcopy(dict(data)))
    result['sensor_id'] = sensor
    return result


def read_intrinsics(data):
    source = data.get('camera_intrinsics', data)
    if not isinstance(source, Mapping) or any(k not in source for k in ('fx', 'fy', 'cx', 'cy')):
        raise ValueError('请配置 camera_intrinsics: fx、fy、cx、cy（对应对齐后的 RGB-D 分辨率）')
    try:
        values = {k: float(source[k]) for k in ('fx', 'fy', 'cx', 'cy')}
    except (TypeError, ValueError) as exc:
        raise ValueError('相机内参尚未填写为有效数值') from exc
    if not np.isfinite(list(values.values())).all() or values['fx'] <= 0 or values['fy'] <= 0:
        raise ValueError('相机内参必须有限，fx/fy 必须为正')
    return values


def pose_matrix(pose):
    """兼容旧配置：[毫米平移, XYZ 弧度]，旋转为 Rz @ Ry @ Rx。"""
    values = np.asarray(pose, dtype=float)
    if values.shape != (6,) or not np.isfinite(values).all():
        raise ValueError('外参必须是六个有限数 [x_mm,y_mm,z_mm,rx,ry,rz]')
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_euler('xyz', values[3:]).as_matrix()
    matrix[:3, 3] = values[:3] * 0.001
    return matrix


def transform_matrix(data):
    if not isinstance(data, Mapping):
        raise ValueError('变换必须包含 position 和 orientation')
    position = np.asarray(data.get('position'), dtype=float)
    orientation = np.asarray(data.get('orientation'), dtype=float)
    if position.shape != (3,) or orientation.shape != (4,) or not np.isfinite(position).all() or not np.isfinite(orientation).all() or np.linalg.norm(orientation) < 1e-8:
        raise ValueError('变换要求有限的 position[3]（米）和非零 orientation[4]（xyzw）')
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(orientation).as_matrix()
    matrix[:3, 3] = position
    return matrix


def frame_name(value):
    return str(value or '').strip().strip('/')


@dataclass(frozen=True)
class CameraGeometry:
    intrinsics: dict
    depth_scale: float
    camera_frame: str
    robot_frame: str
    matrix: np.ndarray
    source: str

    @property
    def transform(self):
        return self.matrix[:3, :3], self.matrix[:3, 3]

    def metadata(self):
        return {
            'camera_frame': self.camera_frame, 'robot_frame': self.robot_frame,
            'intrinsics': self.intrinsics, 'depth_scale': self.depth_scale,
            'transform_source': self.source,
            'transform_camera_to_robot': {
                'position': self.matrix[:3, 3].tolist(),
                'orientation': Rotation.from_matrix(self.matrix[:3, :3]).as_quat().tolist(),
            },
        }


def resolve_geometry(sdk, data, image_frame='', *, allow_dynamic=True):
    intrinsics = read_intrinsics(data)
    if 'depth_scale' not in data:
        raise ValueError('请配置 depth_scale（毫米深度为 0.001，米深度为 1）')
    scale = float(data['depth_scale'])
    if not np.isfinite(scale) or scale <= 0:
        raise ValueError('depth_scale 必须为正的有限数')
    camera = frame_name(data.get('camera_frame') or image_frame)
    if not camera:
        raise ValueError('缺少相机光学坐标系 camera_frame')
    if image_frame and frame_name(image_frame) != camera:
        raise ValueError(f'图像坐标系 {image_frame} 与标定坐标系 {camera} 不一致')
    robot = frame_name(data.get('robot_frame') or 'base_link')
    base = frame_name(data.get('base_frame') or 'base_link')
    if not robot or not base:
        raise ValueError('base_frame 与 robot_frame 不能为空')

    def lookup(source, target):
        if source == target:
            return np.eye(4)
        if not allow_dynamic:
            raise ValueError('参考图必须提供拍摄时完整外参，不能读取当前动态 TF')
        timeout = float(data.get('transform_timeout_s', 2.0))
        age = float(data.get('transform_max_age_s', 1.0))
        if not np.isfinite([timeout, age]).all() or timeout <= 0 or age <= 0:
            raise ValueError('TF 等待时间和最大年龄必须为正的有限数')
        result = sdk.state.get_transformer_martix(source_frame=source, target_frame=target,
                                                 timeout=timeout, max_age_s=age)
        value = (result.output or {}).get('data')
        if value is None:
            raise ValueError(f'没有有效 TF: {source} → {target}')
        return transform_matrix(value)

    explicit = data.get('transform_camera_to_robot')
    extrinsics = data.get('camera_extrinsics')

    if explicit is not None and extrinsics is not None:
        raise ValueError('完整变换与 camera_extrinsics 只能选择一种')

    if explicit is not None:
        matrix, source = transform_matrix(explicit), 'explicit_camera_to_robot'

    else:

        if not isinstance(extrinsics, Mapping):
            raise ValueError('缺少 camera_extrinsics 或 transform_camera_to_robot，不能使用默认标定')

        direct, mounted = 'camera_to_base' in extrinsics, 'camera_to_tcp' in extrinsics

        if direct == mounted:
            raise ValueError('camera_to_base 与 camera_to_tcp 必须且只能提供一种')

        if direct:
            if 'tcp_to_base' in extrinsics or 'tcp_frame' in extrinsics:
                raise ValueError('camera_to_base 已是完整链路，请删除多余的 TCP 变换')
            matrix, source = pose_matrix(extrinsics['camera_to_base']), 'static_camera_to_base'
            
        else:
            snapshot = extrinsics.get('tcp_to_base')
            tcp = frame_name(extrinsics.get('tcp_frame'))
            if snapshot is not None and tcp:
                raise ValueError('tcp_to_base 与 tcp_frame 只能选择一种')
            if snapshot is not None:
                tcp_to_base, source = pose_matrix(snapshot), 'explicit_tcp_snapshot'
            elif tcp:
                tcp_to_base, source = lookup(tcp, base), 'live_mount_tf'
            else:
                raise ValueError('camera_to_tcp 还需要 tcp_frame 或拍摄时的 tcp_to_base')
            matrix = tcp_to_base @ pose_matrix(extrinsics['camera_to_tcp'])
        if base != robot:
            matrix = lookup(base, robot) @ matrix
            source += '+base_to_robot_tf'
    return CameraGeometry(intrinsics, scale, camera, robot, matrix, source)
