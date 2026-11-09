from semantic_robot_skill_sdk import Action
from .models import YoloInput


def build_actions(params: YoloInput):
    common = dict(sensor_id=params.sensor_id, conf_threshold=params.conf_threshold,
                  iou_threshold=params.iou_threshold, gray_box_only=params.gray_box_only,
                  top_k=params.top_k)
    actions = []
    if params.mode in ('detect', 'both'):
        actions.append(('detect', Action(type='perception.yolo_detect',
            parameters={**common, 'include_masks': False},
            timeout_seconds=params.timeout_seconds, label='YOLO 检测')))
    if params.mode in ('point_cloud', 'both'):
        actions.append(('point_cloud', Action(type='perception.generate_point_cloud',
            parameters=common, timeout_seconds=params.timeout_seconds,
            label='YOLO 点云与标定验证')))
    if params.mode == 'match':
        actions.append(('match', Action(type='perception.match_target',
            parameters={**common, 'target_point':list(params.target_point), 'robot_frame':params.target_frame},
            timeout_seconds=params.timeout_seconds, label='地图目标匹配')))
    if params.mode == 'pose':
        pose = params.model_dump(include={
            'reference_points', 'reference_frame', 'reference_dir', 'reference_camera',
            'reference_target_point', 'reference_class_name', 'yaw_range', 'search_step',
            'focus_step', 'enable_roll', 'enable_pitch', 'enable_yaw'}, exclude_none=True)
        actions.append(('pose', Action(type='perception.estimate_object_pose',
            parameters={**common, **pose, 'target_point':list(params.target_point),
                        'robot_frame':params.target_frame},
            timeout_seconds=params.timeout_seconds, label='参考点云姿态估计')))
    return actions
