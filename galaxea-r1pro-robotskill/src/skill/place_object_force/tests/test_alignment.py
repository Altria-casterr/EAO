"""对照 morefine 最新单双手源码的纯离线接口检查。"""
import math

import pytest
from pydantic import ValidationError

from grasp_object_bilateral.scripts import controller as both_controller
from grasp_object_bilateral.scripts import models as both_models
from grasp_object_bilateral.scripts import solver as both_solver
from grasp_object_bilateral.tests.support import inputs as both_inputs
from grasp_object_unilateral.scripts import controller as single_controller
from grasp_object_unilateral.scripts import models as single_models
from grasp_object_unilateral.tests.support import inputs as single_inputs
from place_object_force.scripts import controller, models, solver
from .support import inputs


def poses():
    return {
        'left': dict(x=0.52, y=0.36, z=0.19, qx=0.0, qy=0.0, qz=0.0, qw=1.0),
        'right': dict(x=0.48, y=-0.28, z=0.17, qx=0.0, qy=0.0, qz=0.0, qw=1.0),
    }


def payload(action):
    return action.model_dump(exclude={'label'})


def test_approach_defaults_match_bilateral_with_place_timeout_override():
    reference = both_models.ApproachMotionSettings().model_dump()
    assert models.ApproachMotionSettings().model_dump() == {**reference, 'timeout': 10.0}
    params = inputs()
    assert params.retract_motion.pos_tol == 0.005
    assert params.motion.fine_pos_tol == 0.0005
    assert params.y_force.step_size_m == 0.01


def test_torso_payload_matches_latest_reference():
    settings = {'enabled': True, 'positions': [0.1, 0.2, -0.2, 0.3],
                'timeout': 12.0, 'tol': 0.01}
    assert payload(controller.torso(inputs(torso=settings))) == payload(
        both_controller.torso(both_inputs(torso=settings)))


@pytest.mark.parametrize('num_points', [None, 9, 20])
@pytest.mark.parametrize('trajectory', [False, True])
def test_both_payload_matches_latest_reference(num_points, trajectory):
    settings = models.TrajectorySettings(num_points=num_points, max_num_points=40,
                                         axis_order=' ZXY ', timeout=2.0)
    params = inputs()
    action = controller.both(params, models.BothPoses(**poses()), trajectory=trajectory, motion=settings)
    reference = both_controller.both(
        both_inputs(motion=settings.model_dump()), both_models.BothPoses(**poses()),
        trajectory=trajectory, label='reference')
    assert payload(action) == payload(reference)
    if trajectory and num_points is None:
        assert 'num_points' not in action.parameters


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('num_points', [None, 13])
@pytest.mark.parametrize('trajectory', [False, True])
def test_single_payload_matches_latest_reference(side, num_points, trajectory):
    settings = models.RetractionMotionSettings(num_points=num_points, max_num_points=40,
                                               sequential=False, axis_order='xyz', timeout=2.0)
    action = controller.single(inputs(), side, models.ToolPose(**poses()[side]),
                               trajectory=trajectory, motion=settings)
    reference = single_controller.single(
        side, single_models.ToolPose(**poses()[side]), trajectory=trajectory,
        label='reference', params=single_inputs(motion=settings.model_dump()))
    assert payload(action) == payload(reference)
    if trajectory and num_points is None:
        assert action.parameters['num_points'] == 40


@pytest.mark.parametrize('patch', [
    {'axis_order': 'xxz'}, {'axis_order': 'xy'}, {'max_num_points': 4},
    {'step_distance': 0.0}, {'num_points': 4}, {'num_points': True},
])
def test_invalid_trajectory_configuration_rejected(patch):
    with pytest.raises(ValidationError):
        inputs(approach_motion=patch)
    with pytest.raises(ValidationError):
        inputs(retract_motion=patch)


@pytest.mark.parametrize('frame', ['torso_link4', '/base_link/'])
def test_position_and_full_quaternion_transform_match_latest_reference(frame):
    data = {'position': [0.1, -0.2, -0.8], 'orientation': [0.0, 0.0, 1.0, 1.0]}
    floating = solver.transform(data)
    reference = both_solver.floating_base_transform(data, 'torso_link4')
    assert floating.model_dump() == reference.model_dump()
    angle = math.pi / 6
    source = models.ToolPose(x=0.5, y=0.2, z=1.0, qx=math.sin(angle),
                             qy=0.0, qz=0.0, qw=math.cos(angle))
    actual = solver.pose(source.sensor_data(frame), floating)
    expected = both_solver.parse_pose(source.sensor_data(frame), 'torso_link4', reference)
    assert actual.model_dump() == pytest.approx(expected.model_dump())
    world = {'position': [0.1, 0.2, 0.0], 'orientation': [0.0, 0.0, 0.0, 1.0]}
    assert solver.relative_target(inputs(), world, floating) == pytest.approx(
        both_solver.relative_target(both_inputs(target=inputs().target.model_dump()), world, data))


def test_mismatched_transform_frames_rejected_before_motion():
    data = {'position': [0.0, 0.0, -0.8], 'orientation': [0.0, 0.0, 0.0, 1.0]}
    with pytest.raises(ValueError):
        solver.transform(data, 'map')
    floating = solver.transform(data).model_copy(update={'source_frame': 'camera'})
    with pytest.raises(ValueError):
        solver.pose(models.ToolPose(**poses()['left']).sensor_data('base_link'), floating)
    with pytest.raises(ValueError):
        solver.relative_target(inputs(), data, floating)
