"""只启动本机假 HTTP 服务；不连接 Pilot、Ability 或机器人。"""
import ast
import copy
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import subprocess
import tempfile
import threading
import unittest
from unittest.mock import patch


SCRIPT = Path(__file__).resolve().parents[2] / 'start-shell' / 'stop_current_skill.sh'


class PilotDiscoveryTests(unittest.TestCase):
    def test_pilot_flags_are_discovered_without_exposing_credentials(self):
        source = SCRIPT.read_text().split("<<'PY'\n", 1)[1].rsplit('\nPY', 1)[0]
        module = ast.parse(source)
        self.assertIsInstance(module.body[-1], ast.Try)
        module.body.pop()  # 不运行 main，只测试发现函数。
        namespace = {'__name__': 'offline_discovery_test'}
        exec(compile(module, str(SCRIPT), 'exec'), namespace)
        with tempfile.TemporaryDirectory() as root:
            proc = Path(root)
            (proc / '123').mkdir()
            (proc / '123' / 'cmdline').write_bytes(b'/bin/semantic-pilot\0--server-http=http://localhost:9090\0'
                b'--pilot-id\0pilot1\0--data-dir\0/tmp/pilot\0--access-token\0secret\0')
            (proc / '124').mkdir()
            (proc / '124' / 'cmdline').write_bytes(b'/bin/semantic-pilot\0skill\0run\0')

            def test_path(value):
                return proc if value == '/proc' else Path(value)

            with patch.dict(namespace, {'Path': test_path}):
                config = namespace['pilot_config']()
                self.assertEqual(config, {'server-http': 'http://localhost:9090',
                                          'pilot-id': 'pilot1', 'data-dir': '/tmp/pilot'})
                self.assertNotIn('secret', repr(config))
                (proc / '125').mkdir()
                (proc / '125' / 'cmdline').write_bytes(b'/bin/semantic-pilot\0--pilot-id\0pilot2\0')
                with self.assertRaisesRegex(namespace['StopError'], '多个 Pilot'):
                    namespace['pilot_config']()
                self.assertEqual(namespace['pilot_config'](123), config)


class StopScriptTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.log = Path(self.temp.name) / 'pilot.log'
        self.log.write_text('')
        self.state = {
            'device': {'robot_id': 'r1', 'status': 'busy', 'pilot': {'status': 'online'},
                       'current_execution_id': 'exec1'},
            'execution': {'id': 'exec1', 'robot_id': 'r1', 'status': 'running',
                          'skill_name': 'any-skill', 'skill_version': '1.2.3', 'project_id': 'p1'},
            'outcome': 'stopped', 'ack': True, 'release': True, 'requests': [],
            'workflow': {'workflow': {'revision': 3}, 'tasks': []},
        }
        state, log = self.state, self.log

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def do_GET(self):
                self.handle_api('GET')

            def do_POST(self):
                self.handle_api('POST')

            def handle_api(self, method):
                path = self.path.removeprefix('/api/v1')
                payload = json.loads(self.rfile.read(int(self.headers.get('Content-Length', '0'))) or '{}')
                state['requests'].append((method, path, payload))
                if state.get('unauthorized'):
                    return self.reply({'error': {'code': 'AUTH_TOKEN_INVALID'}}, 401)
                if path == '/devices/r1':
                    if state.get('change_before_stop') and len(state['requests']) > 2:
                        state['device']['current_execution_id'] = 'exec2'
                    return self.reply({'device': state['device']})
                if path == '/robot-executions/exec1' and method == 'GET':
                    return self.reply({'execution': state['execution']})
                if path.endswith('/view'):
                    return self.reply({'workflow_view': state['workflow']})
                if path.endswith('/stop'):
                    if state.get('stop_http_error'):
                        return self.reply({'error': {'code': 'STOP_ERROR'}}, state['stop_http_error'])
                    state['execution']['status'] = state['outcome']
                    if state['outcome'] == 'stopped':
                        state['execution']['result'] = {'safe': state.get('safe', True)}
                        if state['release']:
                            state['device'].update(current_execution_id='', status='idle')
                        for task in state['workflow']['tasks']:
                            if not state.get('keep_task'):
                                task['status'] = 'stopped'
                    if state.get('new_after_stop'):
                        state['device'].update(current_execution_id='exec2', status='busy')
                    return self.reply({'execution': state['execution'], 'workflow_view': state['workflow']}, 202)
                if path.endswith('/confirm-stop'):
                    state['execution'].update(status='stopped', result={'safe': True})
                    if state['release']:
                        state['device'].update(current_execution_id='', status='idle')
                    if state['ack']:
                        with log.open('a') as stream:
                            stream.write(json.dumps({'execution_id': state.get('ack_id', 'exec1'),
                                'msg': '已按操作员确认释放物理动作占用'}) + '\n')
                    for task in state['workflow']['tasks']:
                        task['status'] = 'stopped'
                    return self.reply({'execution': state['execution']}, 202)
                return self.reply({'error': {'code': 'NOT_FOUND'}}, 404)

            def reply(self, value, status=200):
                self.send_response(status)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps(copy.deepcopy(value)).encode())

        self.server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()
        self.temp.cleanup()

    def run_script(self, *args, token=True):
        env = dict(os.environ, SEMANTIC_TOKEN='offline-test-secret' if token else '')
        result = subprocess.run(['bash', str(SCRIPT), '--server',
            f'http://127.0.0.1:{self.server.server_port}', '--robot', 'r1', '--timeout', '0.6',
            '--pilot-log', str(self.log), *args], env=env, text=True, capture_output=True, timeout=5)
        self.assertNotIn('offline-test-secret', result.stdout + result.stderr)
        return result

    def posts(self):
        return [r for r in self.state['requests'] if r[0] == 'POST']

    def test_dry_run_never_posts_even_with_confirmation(self):
        result = self.run_script('--dry-run', '--confirm-safe')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.posts(), [])

    def test_normal_stop_releases(self):
        result = self.run_script()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual([r[1] for r in self.posts()], ['/robot-executions/exec1/stop'])
        self.assertIn('占用已释放', result.stdout)

    def test_unknown_physical_state_is_not_automatically_confirmed(self):
        self.state['outcome'] = 'interrupted'
        result = self.run_script()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('--confirm-safe', result.stderr)
        self.assertEqual(len(self.posts()), 1)

    def test_interrupted_stop_can_be_retried_and_confirmed(self):
        self.state['execution']['status'] = 'interrupted'
        self.state['outcome'] = 'interrupted'
        result = self.run_script('--confirm-safe')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual([r[1] for r in self.posts()],
                         ['/robot-executions/exec1/stop', '/robot-executions/exec1/confirm-stop'])
        self.assertIs(self.posts()[1][2]['physical_state_confirmed'], True)

    def test_acceptance_without_pilot_receipt_is_not_success(self):
        self.state.update(outcome='interrupted', ack=False)
        result = self.run_script('--confirm-safe')
        self.assertNotEqual(result.returncode, 0)
        self.assertNotIn('占用已释放', result.stdout)

    def test_old_or_different_execution_receipt_cannot_confirm_this_stop(self):
        self.log.write_text(json.dumps({'execution_id': 'exec1',
            'msg': '已按操作员确认释放物理动作占用'}) + '\n')
        self.state.update(outcome='interrupted', ack_id='old-exec')
        self.assertNotEqual(self.run_script('--confirm-safe').returncode, 0)

    def test_timeout_does_not_confirm_still_running_stop(self):
        self.state['outcome'] = 'stopping'
        result = self.run_script('--confirm-safe')
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(len(self.posts()), 1)

    def test_unsafe_stop_result_is_not_success(self):
        self.state['safe'] = False
        self.assertNotEqual(self.run_script().returncode, 0)

    def test_server_pointer_must_be_released(self):
        self.state['release'] = False
        self.assertNotEqual(self.run_script().returncode, 0)

    def test_new_execution_before_stop_is_not_touched(self):
        self.state['change_before_stop'] = True
        self.assertNotEqual(self.run_script().returncode, 0)
        self.assertEqual(self.posts(), [])

    def test_new_execution_after_stop_is_not_touched(self):
        self.state['new_after_stop'] = True
        self.assertNotEqual(self.run_script().returncode, 0)
        self.assertEqual(len(self.posts()), 1)

    def test_idle_is_idempotent(self):
        self.state['device'].update(current_execution_id='', status='idle')
        self.assertEqual(self.run_script().returncode, 0)
        self.assertEqual(self.posts(), [])

    def test_run_reservation_is_not_mistaken_for_idle(self):
        self.state['device'].update(current_execution_id='', status='idle', current_run={'id': 'run1'})
        self.assertNotEqual(self.run_script().returncode, 0)
        self.assertEqual(self.posts(), [])

    def test_offline_pilot_is_not_touched(self):
        self.state['device']['pilot']['status'] = 'offline'
        self.assertNotEqual(self.run_script().returncode, 0)
        self.assertEqual(self.posts(), [])

    def test_missing_auth_is_reported_before_http(self):
        result = self.run_script(token=False)
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(self.state['requests'], [])

    def test_token_file(self):
        token_file = Path(self.temp.name) / 'token'
        token_file.write_text('offline-test-secret\n')
        self.assertEqual(self.run_script('--token-file', str(token_file), token=False).returncode, 0)

    def test_invalid_auth_never_posts(self):
        self.state['unauthorized'] = True
        self.assertNotEqual(self.run_script().returncode, 0)
        self.assertEqual(self.posts(), [])

    def test_stop_error_does_not_skip_to_confirmation(self):
        self.state['stop_http_error'] = 500
        self.assertNotEqual(self.run_script('--confirm-safe').returncode, 0)
        self.assertEqual(len(self.posts()), 1)

    def test_workflow_needs_explicit_scope(self):
        self.state['execution'].update(task_id='task1', workflow_id='wf1')
        self.assertNotEqual(self.run_script().returncode, 0)
        self.assertEqual(self.posts(), [])

    def test_workflow_stop_releases_task_reservation(self):
        self.state['execution'].update(task_id='task1', workflow_id='wf1')
        self.state['workflow']['tasks'] = [{'assigned_robot_id': 'r1', 'status': 'running'}]
        result = self.run_script('--stop-workflow')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.posts()[0][1], '/projects/p1/workflows/wf1/stop')
        self.assertEqual(self.posts()[0][2]['revision'], 3)

    def test_workflow_task_lock_must_also_be_released(self):
        self.state['execution'].update(task_id='task1', workflow_id='wf1')
        self.state['workflow']['tasks'] = [{'assigned_robot_id': 'r1', 'status': 'stopping'}]
        self.state['keep_task'] = True
        self.assertNotEqual(self.run_script('--stop-workflow').returncode, 0)

    def test_workflow_confirmation_uses_workflow_endpoint(self):
        self.state['execution'].update(task_id='task1', workflow_id='wf1')
        self.state['outcome'] = 'interrupted'
        result = self.run_script('--stop-workflow', '--confirm-safe')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue(self.posts()[1][1].endswith('/workflows/wf1/confirm-stop'))


if __name__ == '__main__':
    unittest.main()
