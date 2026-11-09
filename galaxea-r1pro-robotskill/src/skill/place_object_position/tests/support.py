"""离线 Action 响应器；禁止任何力控或单手运动请求。"""
import asyncio
from copy import deepcopy
from datetime import datetime, timezone

from semantic_robot_skill_sdk import ActionResult, MockSkillContext, run_skill

from place_object_position.scripts.models import PlaceInput
from place_object_position.scripts.skill import run


def inputs(**updates):
    data = {
        'target': {'object_ref': 'box-offline', 'position_m': [0.5, 0.0, 1.2],
                   'size': {'long_m': 0.53, 'width_m': 0.41, 'height_m': 0.34}},
        'torso': {'positions': [0.0, 0.0, 0.0, 0.0]}, 'stage_pause_s': 0.0,
    }
    data.update(updates)
    return PlaceInput.model_validate(data)


def pose(y):
    return {'header': {'frame_id': 'base_link'}, 'pose': {
        'position': {'x': 0.45, 'y': y, 'z': 1.3},
        'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}}}


class Responder:
    def __init__(self, context):
        self.ctx = context
        self.poses = {'left': pose(0.317), 'right': pose(-0.243)}
        self.floating = {'position': [0.0, 0.0, -0.8], 'orientation': [0.0, 0.0, 0.0, 1.0]}
        self.map_transform = {'position': [0.0, 0.0, 0.0], 'orientation': [0.0, 0.0, 0.0, 1.0]}
        self.actions = []
        self.physical_before = []
        self.torso_positions = None
        self.mutate = lambda key, action, result: result

    def respond(self, key, action):
        self.actions.append((key, action))
        values, output = action.parameters, {}
        if action.type in {'robot.get_state', 'robot.get_sensors_state'}:
            name = values['observation']
            if name == 'floating_base':
                data = self.floating
            elif name == 'transformer_martix':
                data = self.map_transform
            elif name in ('ee_pose_left', 'ee_pose_right'):
                data = self.poses[name.removeprefix('ee_pose_')]
            else:
                raise AssertionError(f'位置放置禁止读取 {name}')
            output = {'result': {'status': 'completed',
                                'observations': [{'name': name, 'data': deepcopy(data)}]}}
        elif action.type == 'torso.move_to_posture_all':
            self.torso_positions = list(values['positions'])
        elif action.type in {'motion.move_end_effector_both', 'motion.move_end_effector_both_trajectory'}:
            self.physical_before.append((key, deepcopy(self.poses)))
            for side in ('left', 'right'):
                target = values[f'{side}_target_pose']
                self.poses[side] = {'header': {'frame_id': values[f'{side}_frame_id']}, 'pose': {
                    'position': {axis: target[f'target_{axis}'] for axis in 'xyz'},
                    'orientation': {axis: target[axis] for axis in 'xyzw'}}}
        else:
            raise AssertionError(f'不允许的 Action：{action.type}')
        return self.mutate(key, action, ActionResult(
            status='succeeded', output=output,
            physical_effect='none' if action.type.startswith('robot.') else 'confirmed',
            evidence_refs=['evidence://offline/' + key]))


class Context(MockSkillContext):
    def __init__(self, params=None):
        self.params = params or inputs()
        self.clock = datetime(2026, 10, 9, tzinfo=timezone.utc)
        super().__init__(self.params)
        self.responder = Responder(self)

    def now(self):
        return self.clock

    async def execute(self, key, action):
        if key not in self._records:
            self.queue_action(action.type, self.responder.respond(key, action))
        return await super().execute(key, action)


def run_all(ctx):
    asyncio.run(run_skill(run, ctx))
    return ctx


def movements(ctx):
    return [(k, a) for k, a in ctx.responder.actions if a.type.startswith('motion.')]


def resumed_context(ctx, params=None):
    resumed = Context(params or ctx.params)
    resumed.clock = ctx.clock
    resumed.checkpoint(ctx.state.model_copy(deep=True))
    resumed._records = deepcopy(ctx._records)
    resumed.responder.poses = deepcopy(ctx.responder.poses)
    resumed.responder.floating = deepcopy(ctx.responder.floating)
    resumed.responder.map_transform = deepcopy(ctx.responder.map_transform)
    return resumed
