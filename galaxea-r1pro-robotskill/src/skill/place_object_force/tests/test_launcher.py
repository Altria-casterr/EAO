"""前置检查对照真实 heartbeat 结构；HTTP 和 Pilot 都使用本地替身。"""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import threading

import pytest
import semantic_robot_skill_sdk
import yaml

from .support import inputs


@pytest.mark.parametrize('case', ['valid', 'valid_run', 'valid_disabled_torso', 'valid_long_torso',
                                 'stale_binding', 'wrong_ability', 'not_running', 'invalid_input',
                                 'missing_torso', 'stale_torso', 'wrong_torso', 'stopped_torso'])
def test_launcher_precheck_and_budget_with_fake_pilot(tmp_path, case):
    root = Path(__file__).resolve().parents[4]
    heartbeat = [
        {'id': 'robot-1', 'abilityName': 'RobotObservation.Demo', 'state': 'Running'},
        {'id': 'motion-1', 'abilityName': 'JointEEFPoseControl.Demo', 'state': 'Running'},
        {'id': 'torso-1', 'abilityName': 'TorsoControl.Demo', 'state': 'Running'},
    ]
    if case == 'wrong_ability':
        heartbeat[1]['abilityName'] = 'TorsoControl.Demo'
    if case == 'not_running':
        heartbeat[1]['state'] = 'Stopped'
    if case == 'wrong_torso':
        heartbeat[2]['abilityName'] = 'JointEEFPoseControl.Demo'
    if case == 'stopped_torso':
        heartbeat[2]['state'] = 'Stopped'
    requests = []

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            requests.append(self.path)
            self.send_response(200)
            self.end_headers()
            self.wfile.write(json.dumps(heartbeat).encode())

        def log_message(self, *args):
            pass

    server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    deployment = {
        'ability_framework': {'endpoint': f'http://127.0.0.1:{server.server_port}'},
        'abilities': {'robot': {'instance_id': 'robot-1'},
                      'torso': {'instance_id': 'old-torso' if case == 'stale_torso' else 'torso-1'},
                      'manipulator_motion': {'instance_id': 'old-id' if case == 'stale_binding' else 'motion-1'},
                      'perception': {'instance_id': 'irrelevant'}},
    }
    if case == 'missing_torso':
        del deployment['abilities']['torso']
    profile, parameters = tmp_path / 'profile.yaml', tmp_path / 'input.json'
    profile.write_text(yaml.safe_dump(deployment))
    torso = ({'enabled': False} if case == 'valid_disabled_torso' else
             {'positions': [0.0, 0.0, 0.0, 0.0], 'timeout': 30000.0 if case == 'valid_long_torso' else 10.0})
    params = inputs(torso=torso)
    data = params.model_dump(mode='json')
    if case == 'invalid_input':
        data['motion']['fine_pos_tol'] = 0.01
    parameters.write_text(json.dumps(data))
    marker, pilot, arguments = tmp_path / 'pilot-started', tmp_path / 'pilot', tmp_path / 'pilot-arguments'
    pilot.write_text('#!/usr/bin/env bash\ntouch "$PLACE_TEST_MARKER"\n'
                     'printf "%s\\n" "$@" > "$PLACE_TEST_ARGS"\n')
    pilot.chmod(0o700)
    environment = dict(os.environ, PLACE_PYTHON=sys.executable, PLACE_PILOT=str(pilot),
                       PLACE_SKILL_SDK=str(Path(semantic_robot_skill_sdk.__file__).resolve().parent.parent),
                       PLACE_CHECK_ONLY='0' if case == 'valid_run' else '1',
                       PLACE_TEST_MARKER=str(marker), PLACE_TEST_ARGS=str(arguments))
    output = None
    try:
        result = subprocess.run(['bash', str(root / 'start-shell/run_place_force_skill.sh'),
                                 str(profile), str(parameters)], env=environment,
                                capture_output=True, text=True, timeout=10)
        assert marker.exists() is (case == 'valid_run')
        assert yaml.safe_load(profile.read_text()) == deployment
        if case in {'valid', 'valid_run', 'valid_disabled_torso', 'valid_long_torso'}:
            assert result.returncode == 0, result.stderr
            assert requests == ['/api/ability-heartbeat']
            output = Path(next(line.split('：', 1)[1] for line in result.stdout.splitlines()
                               if line.startswith('本次部署与执行日志：')))
            used = yaml.safe_load((output / 'deployment.yaml').read_text())
            assert set(used['abilities']) == {'robot', 'torso', 'manipulator_motion'}
            budget = int((output / 'timeout.txt').read_text())
            assert used['pilot']['worker_timeout_seconds'] >= budget > 1800
            assert budget > (params.approach_motion.timeout * params.approach_motion.max_num_points
                             + 8 * params.retract_motion.timeout * params.retract_motion.max_num_points
                             + (params.torso.timeout if params.torso.enabled else 0)
                             + 2 * params.motion.timeout + params.y_force.timeout_s)
            if case == 'valid_run':
                args = arguments.read_text().splitlines()
                manifest = yaml.safe_load((root / 'src/skill/place_object_force/SKILL.md')
                                          .read_text().split('---', 2)[1])
                assert args[args.index('--skill') + 1] == f"{manifest['name']}@{manifest['version']}"
                assert args[args.index('--timeout') + 1] == f'{budget}s'
        else:
            assert result.returncode != 0
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)
        if output is not None:
            shutil.rmtree(output)
