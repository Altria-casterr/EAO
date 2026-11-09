"""在 Skill 仓库验证实际 Ability Task 接收参数；SDK 完全替身，不连接真机。"""
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from robot_prepare.scripts.controller import build_action
from robot_prepare.scripts.models import PrepareInput, LEFT, RIGHT


@pytest.mark.parametrize('site_input', [False, True])
def test_arm_action_runs_current_joint_ability_with_offline_sdk(monkeypatch, site_input):
    workspace = Path(__file__).resolve().parents[6]
    monkeypatch.syspath_prepend(str(workspace / 'semantic-ability/galaxea-r1pro-ability/src'))
    monkeypatch.syspath_prepend(str(workspace / 'semantic-robotsdk/galaxea-r1pro-sdk/src'))
    from galaxea_r1pro_0.tasks.joint_control import ControlBothArmsJointTask

    path = Path(__file__).resolve().parents[4] / 'inputs/robot_prepare.json'
    params = PrepareInput.model_validate_json(path.read_text()) if site_input else PrepareInput()
    action = build_action('arms', params)
    left_handle, right_handle = object(), object()
    publish = Mock(return_value=[left_handle, right_handle])
    feedback = Mock(side_effect=lambda side: SimpleNamespace(
        output={'data': list(LEFT if side == 'left' else RIGHT)}))
    sdk = SimpleNamespace(joint=SimpleNamespace(control_both_arms_joint=publish),
                          sensors=SimpleNamespace(get_joint_state=feedback))
    result = ControlBothArmsJointTask(sdk).execute(action.parameters)
    assert result is left_handle
    publish.assert_called_once()
    for side, positions in (('left', LEFT), ('right', RIGHT)):
        assert publish.call_args.kwargs[f'{side}_positions'] == list(positions)
        assert publish.call_args.kwargs[f'{side}_joint_names'] == [f'{side}_arm_joint{i}' for i in range(1, 8)]
    assert [call.args[0] for call in feedback.call_args_list] == ['left', 'right']
