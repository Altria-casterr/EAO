"""只校验输入与 Action 参数，不执行机器人动作。"""
import json
from pathlib import Path

import pytest

from place_object_force.scripts import controller
from place_object_force.scripts.models import PlaceInput


@pytest.fixture
def example():
    path = Path(__file__).resolve().parents[4] / 'inputs/place_object_force.example.json'
    return json.loads(path.read_text())


@pytest.mark.parametrize('empty_group', [False, True])
def test_omitted_force_settings_match_example(example, empty_group):
    expected = PlaceInput.model_validate(example).y_force
    example.pop('y_force')
    if empty_group:
        example['y_force'] = {}
    actual = PlaceInput.model_validate(example).y_force
    assert actual == expected
    assert 'y_force' not in PlaceInput.model_json_schema()['required']


@pytest.mark.parametrize('field,value', [
    ('force_threshold_n', 5.0),
    ('force_coordinate', ['y', 'z']),
    ('max_distance_m', 0.02),
    ('timeout_s', 15.0),
    ('step_size_m', 0.002),
    ('loop_freq_s', 0.2),
])
def test_partial_force_settings_override_only_requested_field(example, field, value):
    expected = {**example['y_force'], field: value}
    example['y_force'] = {field: value}
    actual = PlaceInput.model_validate(example).y_force.model_dump(mode='json')
    assert actual == expected


@pytest.mark.parametrize('side', ['left', 'right'])
def test_omitting_force_settings_preserves_ability_payload(example, side):
    explicit = PlaceInput.model_validate(example)
    example.pop('y_force')
    defaulted = PlaceInput.model_validate(example)
    expected = controller.force(explicit, side, explicit.y_force, 'offline')
    actual = controller.force(defaulted, side, defaulted.y_force, 'offline')
    assert actual == expected


def test_example_can_override_retraction_height_while_omission_uses_three_cm(example):
    explicit = PlaceInput.model_validate(example)
    assert explicit.retract.safe_offset_z == example['retract']['safe_offset_z']
    example['retract'].pop('safe_offset_z')
    defaulted = PlaceInput.model_validate(example)
    assert defaulted.retract.safe_offset_z == 0.03
    assert defaulted.retract == explicit.retract.model_copy(update={'safe_offset_z': 0.03})


@pytest.mark.parametrize('side', ['left', 'right'])
def test_lower_force_threshold_default_and_example_reach_ability(example, side):
    assert example['y_force']['force_threshold_n'] == 1.5
    example.pop('y_force')
    params = PlaceInput.model_validate(example)
    assert params.y_force.force_threshold_n == 1.5
    assert params.y_force.force_coordinate == ('x',)
    action = controller.force(params, side, params.y_force, 'offline')
    assert action.parameters['force_threshold_n'] == 1.5
    assert action.parameters['move_coordinate'] == 'y'
    assert action.parameters['force_coordinate'] == ['x']
