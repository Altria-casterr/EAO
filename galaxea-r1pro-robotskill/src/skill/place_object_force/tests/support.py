from copy import deepcopy
from datetime import datetime, timedelta, timezone

from semantic_robot_skill_sdk import ActionResult, MockSkillContext

from place_object_force.scripts.models import PlaceInput


def inputs(**updates):
    force = {'force_threshold_n': 1.5, 'force_coordinate': ['z'],
             'max_distance_m': 0.004, 'timeout_s': 10.0, 'loop_freq_s': 0.00001}
    data = {'force_phase_pause_s': 0.0,
            'target': {'object_ref': 'box-test', 'position_m': [0.5, 0.0, 1.2],
                       'size': {'long_m': 0.6, 'width_m': 0.4, 'height_m': 0.3}},
            'torso': {'positions': [0.0, 0.0, 0.0, 0.0]},
            'y_force': {**force, 'force_coordinate': ['y']}}
    data.update(updates)
    return PlaceInput.model_validate(data)


def pose(y):
    return {'header': {'frame_id': 'base_link'}, 'pose': {
        'position': {'x': 0.45, 'y': y, 'z': 1.3},
        'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}}}


class Responder:
    def __init__(self, context):
        self.ctx = context
        self.poses = {'left': pose(0.35), 'right': pose(-0.35)}
        self.actions = []
        self.action_stages = []
        self.physical_before = []
        self.hit_after = {'push_y': 2}
        self.hit_side = {}
        self.force_align_done = set()
        self.floating = {'position': [0.0, 0.0, -0.8], 'orientation': [0.0, 0.0, 0.0, 1.0]}
        self.torso_positions = None
        self.mutate = lambda key, action, result: result
        self.advance_on_step_s = 0.0

    def respond(self, key, action):
        self.actions.append((key, action))
        values, output = action.parameters, {}
        stage = self.ctx.state.stage
        self.action_stages.append(stage)
        if action.type in {'robot.get_state', 'robot.get_sensors_state'}:
            name = values['observation']
            if name == 'floating_base':
                data = deepcopy(self.floating)
            elif name == 'transformer_martix':
                data = {'position': [0.0, 0.0, 0.0], 'orientation': [0.0, 0.0, 0.0, 1.0]}
            elif name == 'force_data':
                base = 30.0 if stage == 'push_y' else 0.0
                data = {f'{side}_arm': {f'force_{axis}': base + i for i, axis in enumerate('xyz')}
                        for side in ('left', 'right')}
                # force_align 执行后再读 force_data 即视为已触阈。
                if stage in self.force_align_done:
                    side = self.hit_side.get(stage, self.ctx.params.put_mode)
                    setting = self.ctx.params.y_force
                    data[f'{side}_arm'][f'force_{setting.force_coordinate[0]}'] += setting.force_threshold_n
            else:
                data = deepcopy(self.poses[name.removeprefix('ee_pose_')])
            output = {'result': {'status': 'completed', 'observations': [{'name': name, 'data': data}]}}
        elif action.type == 'torso.move_to_posture_all':
            self.torso_positions = list(values['positions'])
        elif action.type.startswith('motion.move_end_effector_'):
            self.physical_before.append((key, deepcopy(self.poses)))
            sides = ('left', 'right') if 'both' in action.type else (values['axis'],)
            for side in sides:
                target = values[f'{side}_target_pose'] if 'both' in action.type else values['_target_pose']
                frame = values[f'{side}_frame_id'] if 'both' in action.type else values['frame_id']
                self.poses[side] = {'header': {'frame_id': frame}, 'pose': {
                    'position': {axis: target[f'target_{axis}'] for axis in 'xyz'},
                    'orientation': {axis: target[axis] for axis in 'xyzw'}}}
            if ':step:' in key:
                self.ctx.clock += timedelta(seconds=self.advance_on_step_s)
        elif action.type == 'manipulation.force_align':
            # 模拟主手沿 Y 推靠到位并触阈；后续 force_data 读取返回增力。
            self.physical_before.append((key, deepcopy(self.poses)))
            side = values['axis']
            direction = 1 if values['direction'] == 'positive' else -1
            cur = self.poses[side]['pose']['position']
            cur['y'] += direction * 0.002
            self.force_align_done.add(stage)
        else:
            raise AssertionError(action.type)
        result = ActionResult(status='succeeded', output=output,
                              physical_effect='confirmed' if action.type.startswith(('motion.', 'torso.', 'manipulation.')) else 'none',
                              evidence_refs=['evidence://offline/' + key])
        return self.mutate(key, action, result)


class Context(MockSkillContext):
    def __init__(self, params=None):
        self.params = params or inputs()
        self.clock = datetime(2026, 9, 30, tzinfo=timezone.utc)
        super().__init__(self.params)
        self.responder = Responder(self)

    def now(self):
        return self.clock

    async def execute(self, key, action):
        if key not in self._records:
            self.queue_action(action.type, self.responder.respond(key, action))
        return await super().execute(key, action)
