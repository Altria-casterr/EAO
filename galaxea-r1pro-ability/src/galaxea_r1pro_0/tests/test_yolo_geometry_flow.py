from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from galaxea_r1pro_0.abilities.yolo_detect.camera_geometry import merge_camera_input, resolve_geometry
from galaxea_r1pro_0.abilities.yolo_detect.handler import ObjectPerceptionHandler
from galaxea_r1pro_0.abilities.yolo_detect.yolo_client import DetectedObject
from galaxea_r1pro_0.tasks.yolo_detect import GeneratePointCloudTask, MatchTargetTask, EstimateObjectPoseTask
from galaxea_r1pro_0.utils.point_cloud_filter import filter_point_cloud
from galaxea_r1pro_0.utils.utils_yolo import YoloUtils


def calibration(**extra):
    return dict(camera_frame='optical', camera_intrinsics=dict(fx=100, fy=100, cx=10, cy=10),
                depth_scale=0.001, **extra)


def direct(**extra):
    return calibration(camera_extrinsics={'camera_to_base': [1000, 0, 0, 0, 0, 0]}, **extra)


def sdk_with_tf():
    state = Mock()
    state.get_transformer_martix.return_value = SimpleNamespace(output={'data': {
        'position': [1, 2, 3], 'orientation': [0, 0, np.sin(np.pi/4), np.cos(np.pi/4)]}})
    return SimpleNamespace(state=state)


def test_direct_transform_units_and_no_double_tf():
    sdk = sdk_with_tf()
    result = resolve_geometry(sdk, direct(), 'optical')
    np.testing.assert_allclose(result.matrix[:3, 3], [1, 0, 0])
    sdk.state.get_transformer_martix.assert_not_called()


def test_mount_chain_uses_current_tf_and_noncommuting_rotation():
    sdk = sdk_with_tf()
    data = calibration(camera_extrinsics={'camera_to_tcp': [1000, 0, 0, 0, 0, 0], 'tcp_frame': 'left_tcp'})
    first = resolve_geometry(sdk, data, 'optical')
    np.testing.assert_allclose(first.matrix[:3, 3], [1, 3, 3])
    assert sdk.state.get_transformer_martix.call_args.kwargs == dict(
        source_frame='left_tcp', target_frame='base_link', timeout=2.0, max_age_s=1.0)
    sdk.state.get_transformer_martix.return_value.output['data']['position'] = [2, 2, 3]
    second = resolve_geometry(sdk, data, 'optical')
    np.testing.assert_allclose(second.matrix[:3, 3], [2, 3, 3])


def test_explicit_snapshot_chain_and_output_frame_conversion():
    sdk = sdk_with_tf()
    data = calibration(camera_extrinsics={'camera_to_tcp': [1000, 0, 0, 0, 0, 0],
        'tcp_to_base': [0, 1000, 0, 0, 0, np.pi/2]})
    result = resolve_geometry(sdk, data)
    np.testing.assert_allclose(result.matrix[:3, 3], [0, 2, 0], atol=1e-10)
    sdk.state.get_transformer_martix.assert_not_called()
    result = resolve_geometry(sdk, direct(robot_frame='torso_link4'))
    np.testing.assert_allclose(result.matrix[:3, 3], [1, 3, 3], atol=1e-10)


def test_explicit_camera_to_robot_replaces_config_without_stale_chain():
    config = {'head_camera': direct()}
    merged = merge_camera_input(config, {'transform_camera_to_robot': {'position': [4, 5, 6], 'orientation': [0, 0, 0, 1]}})
    assert 'camera_extrinsics' not in merged
    sdk = sdk_with_tf()
    np.testing.assert_allclose(resolve_geometry(sdk, merged).matrix[:3, 3], [4, 5, 6])
    sdk.state.get_transformer_martix.assert_not_called()
    assert 'camera_extrinsics' in config['head_camera']


@pytest.mark.parametrize('data', [
    {}, calibration(),
    calibration(camera_extrinsics={'camera_to_base': [0]*6, 'camera_to_tcp': [0]*6}),
    calibration(camera_extrinsics={'camera_to_base': [0]*6, 'tcp_to_base': [0]*6}),
    calibration(camera_extrinsics={'camera_to_tcp': [0]*6}),
    calibration(transform_camera_to_robot={'position': [0,0,0], 'orientation': [0,0,0,0]}),
])
def test_missing_or_ambiguous_calibration_fails(data):
    with pytest.raises(ValueError):
        resolve_geometry(sdk_with_tf(), data)


def test_missing_tf_and_wrong_optical_frame_fail():
    sdk = sdk_with_tf()
    sdk.state.get_transformer_martix.return_value.output = {'data': None}
    with pytest.raises(ValueError, match='TF'):
        resolve_geometry(sdk, calibration(camera_extrinsics={'camera_to_tcp': [0]*6, 'tcp_frame': 'tcp'}))
    with pytest.raises(ValueError, match='不一致'):
        resolve_geometry(sdk, direct(), 'another_optical')


def test_camera_switch_uses_independent_calibrations():
    cameras = {'head': direct(), 'wrist': calibration(camera_extrinsics={'camera_to_base': [0, 2000, 0, 0, 0, 0]})}
    first = resolve_geometry(None, merge_camera_input(cameras, {'sensor_id': 'head'}))
    second = resolve_geometry(None, merge_camera_input(cameras, {'sensor_id': 'wrist'}))
    np.testing.assert_allclose(first.matrix[:3,3], [1,0,0])
    np.testing.assert_allclose(second.matrix[:3,3], [0,2,0])


def test_filter_keeps_main_cluster_and_removes_background():
    rng = np.random.default_rng(7)
    points = np.concatenate([rng.normal([0,0,1], .02, (800,3)), rng.normal([1,1,2], .02, (100,3)), [[np.nan,0,0]]])
    keep, stats = filter_point_cloud(points)
    assert 790 <= len(keep) <= 800
    assert np.all(keep < 800)
    assert stats['mode_used'] == 'fast_voxel'
    assert stats['voxel_count'] < stats['knn_point_count']


def test_fallback_and_small_cloud():
    points = np.array([[0,0,0],[1,0,0],[2,0,0]], dtype=float)
    keep, stats = filter_point_cloud(points)
    assert len(keep) == 3
    assert stats['mode_used'] == 'legacy_dbscan_fallback'
    with pytest.raises(ValueError, match='聚类'):
        filter_point_cloud(points, {'fast_fallback': False})
    with pytest.raises(ValueError):
        filter_point_cloud(np.zeros((0,3)))
    assert len(filter_point_cloud(np.zeros((10,3)))[0]) == 10


def test_bbox_is_computed_before_sampling_and_depth_units_match():
    depth = np.full((40,40), 1000., dtype=float)
    mask = np.ones_like(depth)
    intrinsics = dict(fx=100, fy=100, cx=20, cy=20)
    kwargs = dict(mask=mask, intrinsics=intrinsics, transform=(np.eye(3), np.zeros(3)))
    full = YoloUtils.generate_point_cloud(depth, depth_scale=.001, max_points=0, **kwargs)
    small = YoloUtils.generate_point_cloud(depth*.001, depth_scale=1, max_points=5, **kwargs)
    np.testing.assert_allclose(full['center_robot'], small['center_robot'])
    np.testing.assert_allclose(full['center_robot'], [-.005,-.005,1])
    np.testing.assert_allclose(full['bounds_robot']['max'], small['bounds_robot']['max'])
    assert len(small['points_robot']) == 5
    np.testing.assert_allclose(small['points_camera'], small['points_robot'])


def handler_fixture():
    sdk = sdk_with_tf()
    rgb = SimpleNamespace(data=np.zeros((30,30,3), dtype=np.uint8), frame_id='optical', encoding='bgr8')
    depth = SimpleNamespace(data=np.full((30,30), 1000, dtype=np.uint16), frame_id='optical', encoding='16UC1')
    sdk.sensors = SimpleNamespace(capture_rgb=Mock(return_value=rgb), capture_depth=Mock(return_value=depth))
    handler = ObjectPerceptionHandler(sdk, cameras={'head_camera': direct()})
    obj = DetectedObject(0, 'box', .99, [0,0,29,29], np.ones((30,30), dtype=bool))
    handler._client = Mock(return_value=SimpleNamespace(segment_image=Mock(return_value=[obj])))
    return sdk, handler


def test_generate_and_match_task_share_center_and_metadata():
    sdk, handler = handler_fixture()
    cloud = GeneratePointCloudTask(sdk, handler=handler).execute({})['result']
    assert cloud['robot_frame'] == 'base_link'
    obj = cloud['objects'][0]
    assert obj['center_method'] == 'bounding_box'
    match = MatchTargetTask(sdk, handler=handler).execute({'target_point': obj['center_robot']})['result']
    assert match['success']
    assert match['best_match']['distance'] == 0
    assert match['transform_camera_to_robot'] == cloud['transform_camera_to_robot']


def test_reference_requires_own_static_calibration_and_never_reads_live_tf():
    sdk, handler = handler_fixture()
    task = EstimateObjectPoseTask(sdk, handler=handler)
    with pytest.raises(ValueError, match='独立'):
        task._reference(direct(), 'base_link')
    with pytest.raises(ValueError, match='参考图'):
        task._reference({'reference_camera': calibration(camera_extrinsics={'camera_to_tcp': [0]*6, 'tcp_frame': 'tcp'})}, 'base_link')
    sdk.state.get_transformer_martix.assert_not_called()
    with pytest.raises(ValueError, match='reference_frame'):
        task._reference({'reference_points': [[0,0,0],[1,0,0],[0,1,0]]}, 'base_link')


def test_estimate_task_with_explicit_reference_points():
    sdk, handler = handler_fixture()
    yy, xx = np.indices((30,30))
    points = np.column_stack(((xx.ravel()-10)/100+1, (yy.ravel()-10)/100, np.ones(900)))
    result = EstimateObjectPoseTask(sdk, handler=handler).execute({
        'target_point': [1.045,.045,1], 'reference_points': points.tolist(), 'reference_frame': 'base_link',
        'yaw_range': [-5,5], 'search_step': 1.0,
    })['result']
    assert result['success']
    assert result['center_method'] == 'bounding_box'
    np.testing.assert_allclose(result['reference_center'], result['target_center'], atol=1e-12)


def test_reference_image_uses_recorded_calibration(tmp_path):
    import cv2
    sdk, handler = handler_fixture()
    task = EstimateObjectPoseTask(sdk, handler=handler)
    cv2.imwrite(str(tmp_path/'zero_color_s_box.png'), np.zeros((30,30,3), dtype=np.uint8))
    cv2.imwrite(str(tmp_path/'zero_depth_s_box.png'), np.full((30,30),1000,dtype=np.uint16))
    reference_camera = direct()
    reference_camera['camera_extrinsics']['camera_to_base'][0] = 2000
    points, metadata = task._reference({'reference_dir': str(tmp_path), 'reference_camera': reference_camera}, 'base_link')
    with pytest.raises(ValueError, match='reference_target_point'):
        task._reference({'reference_dir': str(tmp_path), 'reference_camera': reference_camera,
                         'reference_target_point': [100,100,100]}, 'base_link')
    np.testing.assert_allclose(metadata['center_robot'], [2.045,.045,1])
    assert metadata['transform_source'] == 'static_camera_to_base'
    sdk.state.get_transformer_martix.assert_not_called()


def test_service_injects_camera_config_and_result_is_json_serializable():
    import json
    from galaxea_r1pro_0.service.ability_service import R1ProAbilityService
    sdk, handler = handler_fixture()
    service = R1ProAbilityService('yolo_detect', sdk, cameras={'head_camera': direct()})
    service.handler._client = handler._client
    result = service.invoke('generate_point_cloud', {'invocation_id':'cloud1'})
    assert result['status'] == 'completed'
    json.dumps(result)
    assert service.handler.cameras['head_camera']['camera_frame'] == 'optical'


def test_runtime_config_defaults_to_models_json_even_without_cr(tmp_path, monkeypatch):
    from pathlib import Path
    from galaxea_r1pro_0.service import entrypoint
    from galaxea_r1pro_0.service.models import ModelProfileRegistry
    monkeypatch.setattr(entrypoint, 'find_cr', lambda _: None)
    monkeypatch.setenv('ROBOT_PROFILE_PATH', 'override.json')
    monkeypatch.delenv('MODEL_REGISTRY_PATH', raising=False)
    monkeypatch.chdir(tmp_path)
    config = entrypoint.RuntimeConfig.load('YoloDetect.Demo')
    assert config.robot_profile_path == 'override.json'
    assert Path(config.model_registry_path).name == 'models.example.json'
    registry = ModelProfileRegistry.load(config.model_registry_path)
    assert 'head_camera' in registry.cameras
    assert registry.resolve('yolo_detect', None).name == 'fake-perception-default'


def test_models_json_priority_and_camera_flow(tmp_path, monkeypatch):
    import json, yaml
    from galaxea_r1pro_0.service import entrypoint
    from galaxea_r1pro_0.service.models import ModelProfileRegistry
    from galaxea_r1pro_0.service.ability_service import R1ProAbilityService
    models = tmp_path/'models.json'
    models.write_text(json.dumps({'cameras':{'head_camera':direct()}}))
    cr = tmp_path/'ability.yaml'
    cr.write_text(yaml.safe_dump({'spec':{'config':{
        'robotProfilePath':'robot.json', 'modelRegistryPath':str(models),
        'cameras':{'head_camera':{'invalid':'old CR camera ignored'}}}}}))
    monkeypatch.setattr(entrypoint, 'find_cr', lambda _: cr)
    monkeypatch.delenv('ROBOT_PROFILE_PATH', raising=False)
    monkeypatch.delenv('MODEL_REGISTRY_PATH', raising=False)
    config = entrypoint.RuntimeConfig.load('YoloDetect.Demo')
    assert config.model_registry_path == str(models)
    registry = ModelProfileRegistry.load(config.model_registry_path)
    sdk, handler = handler_fixture()
    service = R1ProAbilityService('yolo_detect',sdk, models=registry, cameras=registry.cameras)
    service.handler._client = handler._client
    result = service.invoke('generate_point_cloud',{'invocation_id':'json-calibration'})
    assert result['status'] == 'completed'
    assert service.handler.prepare({})['camera_extrinsics'] == direct()['camera_extrinsics']
    monkeypatch.setenv('MODEL_REGISTRY_PATH', 'env-models.json')
    assert entrypoint.RuntimeConfig.load('YoloDetect.Demo').model_registry_path == 'env-models.json'


@pytest.mark.parametrize('cameras', [None, [], {'head_camera':42}])
def test_invalid_camera_config_in_models_json_fails(tmp_path, cameras):
    import json
    from galaxea_r1pro_0.service.models import ModelProfileRegistry, ModelProfileError
    path = tmp_path/'models.json'
    path.write_text(json.dumps({'cameras':cameras}))
    with pytest.raises(ModelProfileError, match='cameras'):
        ModelProfileRegistry.load(path)


def test_unsynchronized_rgbd_is_rejected_before_inference():
    sdk, handler = handler_fixture()
    sdk.sensors.capture_rgb.return_value.metadata={'ros_stamp':{'sec':100}}
    sdk.sensors.capture_depth.return_value.metadata={'ros_stamp':{'sec':101}}
    with pytest.raises(ValueError,match='时间差'):
        handler.segment({})
    handler._client.assert_not_called()


@pytest.mark.parametrize('change', [{'robot_frame':'map'}, {'camera_frame':'other'}])
def test_complete_transform_cannot_be_relabelled(change):
    config = calibration(transform_camera_to_robot={'position':[0,0,0], 'orientation':[0,0,0,1]})
    with pytest.raises(ValueError, match='新外参'):
        merge_camera_input({'head_camera':config}, change)
    replacement = {**change, 'transform_camera_to_robot':{'position':[1,0,0], 'orientation':[0,0,0,1]}}
    assert merge_camera_input({'head_camera':config}, replacement)['transform_camera_to_robot']['position'][0] == 1


def test_static_base_cannot_be_relabelled_but_output_can_be_transformed():
    with pytest.raises(ValueError, match='base_frame'):
        merge_camera_input({'head_camera':direct()}, {'base_frame':'map'})
    data = merge_camera_input({'head_camera':direct()}, {'robot_frame':'map'})
    sdk = sdk_with_tf()
    resolve_geometry(sdk, data)
    assert sdk.state.get_transformer_martix.call_args.kwargs['target_frame'] == 'map'


def test_depth_in_another_frame_is_rejected_before_inference():
    sdk, handler = handler_fixture()
    sdk.sensors.capture_depth.return_value.frame_id = 'depth_optical'
    with pytest.raises(ValueError, match='深度图坐标系'):
        GeneratePointCloudTask(sdk, handler=handler).execute({})
    handler._client.assert_not_called()


def test_match_chooses_nearest_candidate_that_passes_all_axis_limits():
    result = YoloUtils.match_target([
        {'index':0, 'center_robot':[.11,0,0]},
        {'index':1, 'center_robot':[0,.2,0]},
    ], [0,0,0], [.1,.25,.1])
    assert result['success'] and result['best_match']['index'] == 1


def test_rotation_search_respects_range_and_keeps_better_coarse_score(monkeypatch):
    from galaxea_r1pro_0.utils.utils_yolo import PointCloudRotationMatcher
    matcher = PointCloudRotationMatcher(yaw_range=(0,1), search_step=10)
    matcher.load_clouds(np.eye(3), np.eye(3))
    calls = []
    def score(angle, axis):
        calls.append(angle)
        return 1.0 if angle == 0 else 0.0
    monkeypatch.setattr(matcher, '_score', score)
    result = matcher.estimate_pose()
    assert all(0 <= angle <= 1 for angle in calls)
    assert result['best_angle'] == 0 and result['best_iou'] == 1


def test_ability_connect_passes_json_cameras_to_handler(tmp_path, monkeypatch):
    import json
    from galaxea_r1pro_0.service import entrypoint
    from galaxea_r1pro_0.service.roles import AbilityRole
    path = tmp_path/'models.json'
    path.write_text(json.dumps({'cameras':{'head_camera':direct()}}))
    config = entrypoint.RuntimeConfig('robot.json',str(path))
    monkeypatch.setattr(entrypoint.RuntimeConfig, 'load', lambda _: config)
    monkeypatch.setattr(entrypoint.RobotProfile, 'load', lambda _: object())
    sdk, _ = handler_fixture()
    monkeypatch.setattr(entrypoint.R1ProSDK, 'open', lambda _: sdk)
    impl = entrypoint.ImplAbility(AbilityRole.yolo_detect, 'YoloDetect.Demo')
    impl.ability_port = 12345  # 不启动网络服务。
    impl.on_connect()
    assert impl.service.handler.cameras == {'head_camera':direct()}
    sdk.state.get_transformer_martix.assert_not_called()
