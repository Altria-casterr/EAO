import asyncio
from pathlib import Path
import pytest
import yaml
from pydantic import ValidationError
from semantic_robot_skill_sdk import ActionResult, MockSkillContext, StopRequest, run_skill, SkillCancelled
from robot_prepare.scripts.models import PrepareInput, PrepareState, LEFT, RIGHT
from robot_prepare.scripts.controller import build_action, TYPES
from robot_prepare.scripts.skill import run, on_stop

class Context(MockSkillContext):
    def __init__(self, failure=None):
        super().__init__(PrepareInput())
        self.calls = []
        self.failure_stage, self.failure_status = failure or (None, None)
    async def execute(self, key, action):
        if key not in self._records:
            self.calls.append((key, action))
            status = self.failure_status if key == f'robot-prepare:{self.failure_stage}:1' else 'succeeded'
            self.queue_action(action.type, ActionResult(status=status, error_message='injected' if status != 'succeeded' else None))
        return await super().execute(key, action)

def test_order_and_exact_targets():
    ctx=Context()
    asyncio.run(run_skill(run, ctx))
    assert ctx.status == 'completed'
    assert [a.type for _, a in ctx.calls] == list(TYPES.values())
    assert ctx.calls[0][1].parameters['positions'] == [0]*4
    assert ctx.calls[1][1].parameters['left_positions'] == list(LEFT)
    assert ctx.calls[1][1].parameters['right_positions'] == list(RIGHT)
    assert ctx.result.ready
    asyncio.run(run(ctx))
    assert len(ctx.calls) == 2

@pytest.mark.parametrize('stage', list(TYPES))
@pytest.mark.parametrize('status', ['failed','interrupted','stopped'])
def test_failure_stops_following_actions_and_resume(stage, status):
    ctx=Context((stage,status))
    asyncio.run(run(ctx))
    assert ctx.status == 'failed'
    assert ctx.calls[-1][0] == f'robot-prepare:{stage}:1'
    count=len(ctx.calls)
    asyncio.run(run(ctx))
    assert len(ctx.calls) == count

@pytest.mark.parametrize('stage', list(TYPES))
def test_crash_after_action_result_reuses_key(stage):
    ctx=Context()
    original=ctx.execute
    async def crash(key, action):
        result=await original(key, action)
        if key == f'robot-prepare:{stage}:1': raise RuntimeError('crash')
        return result
    ctx.execute=crash
    with pytest.raises(RuntimeError): asyncio.run(run(ctx))
    ctx.execute=original
    asyncio.run(run(ctx))
    assert ctx.status == 'completed'
    assert len(ctx.calls) == 2

@pytest.mark.parametrize('patch', [
    {'torso':{'positions':[0,0,1,0]}},
    {'arms':{'left_positions':[0]*6}}, {'arms':{'right_positions':[float('nan')]*7}},
    {'arms':{'left_joint_names':['right_arm']}}, {'arms':{'timeout':0}},
    {'arms':{'left_joint_names':['left_arm']}},
    {'arms':{'right_joint_names':['right_arm']}},
    {'arms':{'left_joint_names':['joint1']*7}},
    {'arms':{'right_joint_names':['']*7}},
])
def test_invalid_inputs(patch):
    with pytest.raises(ValidationError): PrepareInput.model_validate(patch)

@pytest.mark.parametrize('started',[False,True])
def test_stop_does_not_issue_motion_or_claim_confirmed_stop(started):
    ctx=Context()
    ctx.checkpoint(PrepareState(physical_action_started=started))
    ctx.request_stop()
    with pytest.raises(SkillCancelled): asyncio.run(run(ctx))
    result=asyncio.run(on_stop(ctx,StopRequest(source='user',reason='stop')))
    assert result.safe == (not started)
    assert result.requires_intervention == started
    assert not ctx.calls

def test_actions_match_ability_contracts():
    root=Path(__file__).resolve().parents[6]
    base=root/'semantic-ability/galaxea-r1pro-ability/src/galaxea_r1pro_0/abilities'
    contracts={}
    for name in ('torso_control','joint_control'):
        for task in yaml.safe_load((base/name/'ability.manifest.yaml').read_text())['tasks']:
            if task.get('actionType'): contracts[task['actionType']]=task
    for stage in TYPES:
        action=build_action(stage, PrepareInput())
        task=contracts[action.type]
        allowed={p['name'] for p in task['params']}
        required={p['name'] for p in task['params'] if not p.get('optional',False)}
        assert required <= action.parameters.keys() <= allowed
        assert action.timeout_seconds > action.parameters['timeout']


def test_default_and_site_input_use_seven_ordered_joint_names():
    site = Path(__file__).resolve().parents[4] / 'inputs/robot_prepare.json'
    for params in (PrepareInput(), PrepareInput.model_validate_json(site.read_text())):
        action = build_action('arms', params)
        for side in ('left', 'right'):
            names = action.parameters[f'{side}_joint_names']
            assert names == [f'{side}_arm_joint{i}' for i in range(1, 8)]
            assert len(names) == len(action.parameters[f'{side}_positions']) == 7
        assert 'semantic_flag' not in action.parameters


def test_explicit_joint_names_preserve_position_order():
    names = [f'calibrated_joint{i}' for i in range(7)]
    params = PrepareInput.model_validate({'arms': {'left_joint_names': names}})
    action = build_action('arms', params)
    assert action.parameters['left_joint_names'] == names
    assert action.parameters['left_positions'] == list(LEFT)
