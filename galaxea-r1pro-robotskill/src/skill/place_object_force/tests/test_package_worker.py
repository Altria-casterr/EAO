"""独立 ZIP 中的真实 JSON-RPC Worker；物理 Action 完全使用离线响应。"""
import json
import os
from pathlib import Path
import queue
import subprocess
import sys
import threading
from zipfile import ZipFile

import semantic_robot_skill_sdk
from semantic_robot_skill_sdk import Action
import yaml

from place_object_force.package_skill import build_package
from place_object_force.scripts.models import PlaceState
from .support import Context, inputs


def test_zip_worker_validates_input_and_completes_all_stages(tmp_path):
    package = build_package(tmp_path / 'skill.zip')
    extracted = tmp_path / 'isolated'
    with ZipFile(package) as archive:
        names = archive.namelist()
        assert 'solver/target_point_solver/target_point_solver.py' in names
        assert 'SOURCES.md' in names and 'requirements.lock' in names
        assert not any('tests/' in name or '__pycache__' in name for name in names)
        archive.extractall(extracted)
    manifest = yaml.safe_load((extracted / 'SKILL.md').read_text().split('---', 2)[1])
    environment = dict(os.environ)
    # 子进程不能访问本 Skill 源码路径；只保留宿主 SDK 路径。
    environment['PYTHONPATH'] = str(Path(semantic_robot_skill_sdk.__file__).resolve().parent.parent)
    process = subprocess.Popen(
        [sys.executable, '-m', 'semantic_robot_skill_sdk.worker', '--skill-dir', str(extracted)],
        cwd=tmp_path, env=environment, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
        stderr=subprocess.PIPE, text=True, bufsize=1,
    )
    messages = queue.Queue()

    def reader():
        try:
            for line in process.stdout:
                messages.put(json.loads(line))
        except BaseException as error:
            messages.put(error)

    threading.Thread(target=reader, daemon=True).start()

    def receive():
        try:
            item = messages.get(timeout=10)
        except queue.Empty:
            if process.poll() is not None:
                raise AssertionError(process.stderr.read())
            raise
        if isinstance(item, BaseException):
            raise item
        return item

    def send(item):
        process.stdin.write(json.dumps({'jsonrpc': '2.0', **item}) + '\n')
        process.stdin.flush()

    try:
        assert receive()['method'] == 'ready'
        send({'id': 'init', 'method': 'worker.initialize', 'params': {
            'name': manifest['name'], 'version': manifest['version'], 'runtime': manifest['runtime'],
        }})
        initialized = receive()
        assert initialized['result']['status'] == 'initialized', initialized
        assert set(initialized['result']['input_schema']['required']) == {'target'}
        assert manifest == yaml.safe_load((Path(__file__).parents[1] / 'SKILL.md').read_text().split('---', 2)[1])
        params = inputs().model_dump(mode='json')
        invalid = {**params, 'motion': {**params['motion'], 'fine_pos_tol': 0.01}}
        send({'id': 'invalid', 'method': 'skill.validate_input', 'params': {'input': invalid}})
        assert receive()['result']['valid'] is False
        send({'id': 'validate', 'method': 'skill.validate_input', 'params': {'input': params}})
        assert receive()['result']['valid'] is True
        send({'id': 'run', 'method': 'skill.run', 'params': {
            'execution_id': 'offline-place', 'robot_ref': 'robot://offline',
            'input': params, 'checkpoint': None, 'log_fields': {},
        }})
        context = Context()
        responses, completed = {}, []
        terminal = None
        while terminal is None:
            message = receive()
            if message.get('id') == 'run':
                terminal = message
                continue
            method, values = message.get('method'), message.get('params', {})
            if method == 'action.start':
                action_id = f'action-{len(responses)}'
                responses[action_id] = context.responder.respond(values['key'], Action.model_validate(values['action']))
                send({'id': message['id'], 'result': {'action_id': action_id}})
            elif method == 'action.result':
                send({'id': message['id'], 'result': responses[values['action_id']].model_dump(mode='json')})
            elif method == 'runtime.now':
                send({'id': message['id'], 'result': context.clock.isoformat()})
            elif method == 'checkpoint':
                context.checkpoint(PlaceState.model_validate(values['state']))
            elif method == 'event.report':
                if values['event'] == 'stage.completed':
                    completed.append(values['stage'])
            else:
                assert method in {'complete', 'log'}, message
        assert terminal['result']['status'] == 'completed', terminal
        result = terminal['result']['result']
        assert result['placed'] is True and result['holding_object'] is None
        assert result['retracted_sides'] == ['right', 'left']
        assert set(result['contacts']) == {'push_y'}
        assert context.state.positioned_stages == ['align_x', 'lower_z']
        assert context.state.placement_revision == 6
        assert result['preposition_points']['left']['y'] == context.state.initial_poses.left.y == 0.35
        assert result['preposition_points']['right']['y'] == context.state.initial_poses.right.y == -0.35
        assert context.responder.torso_positions == params['torso']['positions']
        assert all(item['completed_steps'] == 1 for item in result['contacts'].values())
        assert completed == ['prepare', 'preposition', 'align_x', 'lower_z',
                             'retract_assist', 'push_y', 'retract_push', 'verify']
        send({'id': 'shutdown', 'method': 'worker.shutdown', 'params': {}})
        assert receive()['result']['status'] == 'shutting_down'
        process.wait(timeout=5)
        assert process.returncode == 0, process.stderr.read()
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(timeout=5)
        for stream in (process.stdin, process.stdout, process.stderr):
            stream.close()
