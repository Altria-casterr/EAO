"""YOLO 检测、fast_voxel 点云与按相机配置解析的坐标变换。"""
from __future__ import annotations

from typing import Any, Mapping
import cv2
import numpy as np

from ..abilities.yolo_detect.camera_geometry import resolve_geometry
from ..utils.point_cloud_filter import CloudFilter
from ..utils.utils_yolo import PointCloudRotationMatcher, YoloUtils
from .base import AbilityTask


def _point(value, name):
    point = np.asarray(value, dtype=float)
    if point.shape != (3,) or not np.isfinite(point).all():
        raise ValueError(f'{name} 必须是三个有限数，单位米')
    return point


def _thresholds(data):
    values = [float(data.get('delta_threshold_' + axis, default))
              for axis, default in zip('xyz', (0.2, 0.25, 0.2))]
    if not np.isfinite(values).all() or min(values) < 0:
        raise ValueError('匹配阈值必须为非负有限数，单位米')
    return values


def _selection_options(data):
    gray_only = data.get('gray_box_only', False)
    top_k = data.get('top_k', 2)
    if type(gray_only) is not bool:
        raise ValueError('gray_box_only 必须是布尔值')
    if type(top_k) is not int or top_k < 1:
        raise ValueError('top_k 必须是正整数')
    return gray_only, top_k


def _clouds(depth_frame, objects, geometry, data):
    depth = np.asarray(depth_frame.data)
    if depth.ndim == 3:
        depth = depth[..., 0]
    config = data.get('point_cloud_filter') or {}
    CloudFilter(**dict(config))  # 参数错误不能被当作空目标跳过。
    max_points = data.get('max_points', 5000)
    gray_only, top_k = _selection_options(data)
    candidates = []
    valid_depth = np.isfinite(depth) & (depth > 0)
    for index, obj in enumerate(objects):
        if gray_only and obj.class_name != "gray_box":
            continue
        if obj.mask is None:
            continue
        mask = np.asarray(obj.mask) > 0
        if mask.shape != depth.shape:
            raise ValueError('掩码和深度图尺寸不一致')
        if bool(data.get('refine_mask', True)):
            mask = YoloUtils.refine_mask_by_depth(depth, mask,
                       depth_band_tolerance=0.2 / geometry.depth_scale) > 0
        valid = mask & valid_depth
        if not valid.any():
            continue
        # Range to camera origin, not optical-axis Z or distance to robot origin.
        # Compute scalar ranges directly; do not build/filter full XYZ clouds here.
        v, u = np.nonzero(valid)
        intrinsics = geometry.intrinsics
        z = depth[valid].astype(np.float64) * geometry.depth_scale
        ray_norm = np.sqrt(1.0 + ((u-intrinsics['cx'])/intrinsics['fx'])**2
                           + ((v-intrinsics['cy'])/intrinsics['fy'])**2)
        camera_distance_m = float(np.median(z * ray_norm))
        median_depth_m = float(np.median(z))
        candidates.append((camera_distance_m, index, obj, mask, median_depth_m))
    candidates.sort(key=lambda item: (item[0], item[1]))
    output = []
    for camera_distance_m, index, obj, mask, median_depth_m in candidates[:top_k]:
        cloud = YoloUtils.generate_point_cloud(depth, mask, geometry.intrinsics,
                    geometry.depth_scale, geometry.transform, max_points=max_points,
                    refine_mask=False, filter_config=config)
        output.append({'index': index, 'camera_distance_m': camera_distance_m,
                       'median_depth_m': median_depth_m, 'class_id': obj.class_id,
                       'class_name': obj.class_name, 'confidence': obj.confidence,
                       **cloud})
    return output


def _public_object(obj):
    return {
        'index': obj['index'], 'class_id': obj['class_id'],
        'class_name': obj['class_name'], 'confidence': obj['confidence'],
        'median_depth_m': obj['median_depth_m'],
        'camera_distance_m': obj['camera_distance_m'],
        'center_robot': obj['center_robot'].tolist(),
        'center_method': obj['center_method'],
        'bounds_robot': {key: value.tolist() for key, value in obj['bounds_robot'].items()},
        'filter_stats': obj['filter_stats'],
    }


def _capture(task, input_data):
    if task.handler is None:
        raise RuntimeError('YOLO perception handler 未初始化')
    _selection_options(input_data)
    data = task.handler.prepare(input_data)
    rgb, depth, objects, geometry = task.handler.segment_with_geometry(data)
    return data, rgb, depth, geometry, _clouds(depth, objects, geometry, data)


class GeneratePointCloudTask(AbilityTask):
    name, task_type = 'generate_point_cloud', 0
    summary = '使用 fast_voxel 过滤点云并返回目标坐标系中的包围盒中心'

    def execute(self, input_data: Mapping[str, Any]) -> dict[str, Any]:
        data, rgb, depth, geometry, objects = _capture(self, input_data)
        return {'status': 'completed', 'result': {
            'sensor_id': data['sensor_id'], **geometry.metadata(),
            'image_shape': list(np.asarray(rgb.data).shape),
            'rgb': {'encoding': rgb.encoding, 'frame_id': rgb.frame_id},
            'depth': {'encoding': depth.encoding, 'frame_id': depth.frame_id},
            'objects': [_public_object(obj) for obj in objects],
        }}


class YoloDetectTask(AbilityTask):
    name, task_type = 'yolo_detect', 1
    summary = '获取 RGB-D 图像并执行 YOLO 检测与分割'

    def execute(self, input_data: Mapping[str, Any]) -> dict[str, Any]:
        if self.handler is None:
            raise RuntimeError('YOLO perception handler 未初始化')
        _selection_options(input_data)
        rgb, depth, detected = self.handler.segment(input_data)
        objects = []
        for index, obj in enumerate(detected):
            item = {'index': index, 'class_id': obj.class_id, 'class_name': obj.class_name,
                    'confidence': obj.confidence, 'box_xyxy': list(obj.box_xyxy),
                    'has_mask': obj.mask is not None}
            if input_data.get('include_masks', True) and obj.mask is not None:
                item['mask'] = np.asarray(obj.mask, dtype=np.uint8).tolist()
            objects.append(item)
        return {'status': 'completed', 'result': {
            'sensor_id': str(input_data.get('sensor_id') or 'head_camera'),
            'image_shape': list(np.asarray(rgb.data).shape),
            'rgb': {'encoding': rgb.encoding, 'frame_id': rgb.frame_id},
            'depth': {'encoding': depth.encoding, 'frame_id': depth.frame_id},
            'metadata': dict(getattr(rgb, 'metadata', {}) or {}), 'objects': objects,
        }}


class MatchTargetTask(AbilityTask):
    name, task_type = 'match_target', 2
    summary = '将目标点匹配到过滤后物体的包围盒中心'

    def execute(self, input_data: Mapping[str, Any]) -> dict[str, Any]:
        target = _point(input_data.get('target_point'), 'target_point')
        data, _, _, geometry, objects = _capture(self, input_data)
        matched = YoloUtils.match_target([_public_object(obj) for obj in objects], target, _thresholds(data))
        return {'status': 'completed', 'result': {**matched, **geometry.metadata()}}


class EstimateObjectPoseTask(AbilityTask):
    name, task_type = 'estimate_object_pose', 3
    summary = '比较参考与目标点云，估计相对单轴旋转'

    def execute(self, input_data: Mapping[str, Any]) -> dict[str, Any]:
        target_point = _point(input_data.get('target_point'), 'target_point')
        data, _, _, geometry, objects = _capture(self, input_data)
        matched = YoloUtils.match_target([_public_object(obj) for obj in objects], target_point, _thresholds(data))
        if matched['best_match'] is None:
            raise ValueError('未找到满足阈值的目标物体')
        target = next(obj for obj in objects if obj['index'] == matched['best_match']['index'])
        reference, reference_meta = self._reference(data, geometry.robot_frame)
        target_points = target['points_robot']
        target_center = target['center_robot']
        reference_center = (reference.min(axis=0) + reference.max(axis=0)) / 2
        if len(reference) < 3 or len(target_points) < 3:
            raise ValueError("参考与目标点云都至少需要三个有效点")
        matcher = PointCloudRotationMatcher(
            yaw_range=tuple(data.get('yaw_range', (-90.0, 90.0))),
            search_step=float(data.get('search_step', 2.0)),
            focus_step=float(data.get('focus_step', 0.5)),
            max_points=int(data.get('matcher_max_points', 15000)),
            shape_bin_size=float(data.get('shape_bin_size', 0.05)),
            enable_roll=bool(data.get('enable_roll', False)),
            enable_pitch=bool(data.get('enable_pitch', False)),
            enable_yaw=bool(data.get('enable_yaw', True)),
        )
        result = matcher.load_clouds((reference-reference_center)*1000,
                                     (target_points-target_center)*1000).estimate_pose()
        result.update(reference_center=reference_center.tolist(), target_center=target_center.tolist(),
                      reference_object=reference_meta, target_object=matched['best_match'],
                      target_point=target_point.tolist(), center_method='bounding_box', **geometry.metadata())
        return {'status': 'completed', 'result': result}

    def _reference(self, data, robot_frame):
        from types import SimpleNamespace
        from ..utils.point_cloud_filter import filter_point_cloud
        if data.get('reference_points') is not None:
            if str(data.get('reference_frame', '')).strip('/') != robot_frame:
                raise ValueError('reference_points 必须用 reference_frame 明确声明与目标相同的坐标系')
            points = np.asarray(data['reference_points'], dtype=float)
            if points.ndim != 2 or points.shape[1] != 3 or len(points) < 3:
                raise ValueError('reference_points 必须是至少三个点的 N×3 数组，单位米')
            keep, stats = filter_point_cloud(points, data.get('point_cloud_filter'))
            return points[keep], {'source': 'reference_points', 'robot_frame': robot_frame, 'filter_stats': stats}
        reference_data = data.get('reference_camera')
        if not isinstance(reference_data, Mapping):
            raise ValueError('参考图需要独立的 reference_camera 标定（拍摄时的完整外参）')
        reference_data = dict(reference_data)
        reference_data.setdefault('robot_frame', robot_frame)
        geometry = resolve_geometry(self.sdk, reference_data, allow_dynamic=False)
        if geometry.robot_frame != robot_frame:
            raise ValueError('参考图与目标点云的 robot_frame 必须相同')
        color_path, depth_path = YoloUtils.reference_image_paths(data.get('reference_dir'))
        color = cv2.imread(str(color_path), cv2.IMREAD_COLOR)
        depth = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED)
        if color is None or depth is None:
            raise ValueError(f'参考图读取失败: {color_path} / {depth_path}')
        if color.shape[:2] != depth.shape[:2]:
            raise ValueError('参考 RGB 和深度图尺寸不一致')
        model_data = dict(data)
        for key in ('conf_threshold', 'iou_threshold', 'imgsz', 'device', 'class_names'):
            if 'reference_' + key in data:
                model_data[key] = data['reference_' + key]
        detected = self.handler.segment_image(color, model_data)
        cloud_data = dict(data)
        cloud_data['max_points'] = 0  # 保留全部过滤点，中心不受后续采样影响。
        cloud_data['refine_mask'] = data.get('reference_refine_mask', data.get('refine_mask', True))
        objects = _clouds(SimpleNamespace(data=depth), detected, geometry, cloud_data)
        if data.get('reference_class_name') is not None:
            objects = [obj for obj in objects if obj['class_name'] == data['reference_class_name']]
        if data.get('reference_class_id') is not None:
            objects = [obj for obj in objects if obj['class_id'] == int(data['reference_class_id'])]
        if not objects:
            raise ValueError('参考图中未找到指定的有效物体')
        if data.get('reference_target_point') is not None:
            point = _point(data['reference_target_point'], 'reference_target_point')
            matched = YoloUtils.match_target([_public_object(obj) for obj in objects], point, _thresholds(data))
            if matched['best_match'] is None:
                raise ValueError('筛选后的参考物体均不在 reference_target_point 匹配阈值内，请检查 top_k、标签和参考目标')
            obj = next(obj for obj in objects if obj['index'] == matched['best_match']['index'])
        elif len(objects) == 1:
            obj = objects[0]
        else:
            raise ValueError('参考图含多个候选物体，请指定 reference_target_point 或唯一的参考类别')
        return obj['points_robot'], {**_public_object(obj), **geometry.metadata(),
                                    'reference_color_path': str(color_path), 'reference_depth_path': str(depth_path)}
