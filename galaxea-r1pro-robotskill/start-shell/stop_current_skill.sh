#!/usr/bin/env bash
# 停止 Server/Pilot 当前执行，并核实机器人占用是否释放。单文件，依赖 python3。
# 不按进程名 pkill：由 Pilot 的 execution.stop 精确结束对应 Worker。
set -euo pipefail
exec python3 - "$@" <<'PY'
import argparse
import json
import math
import os
from pathlib import Path
import sys
import time
import urllib.error
import urllib.parse
import urllib.request


class StopError(Exception):
    pass


class APIError(StopError):
    def __init__(self, status, code):
        self.status = status
        super().__init__(f"HTTP {status}: {code}")


def fail(message):
    raise StopError(message)


def pilot_config(pid=None):
    candidates = []
    paths = [Path('/proc') / str(pid)] if pid else Path('/proc').iterdir()
    for path in paths:
        if not path.name.isdigit():
            continue
        try:
            args = (path / 'cmdline').read_bytes().decode().rstrip('\0').split('\0')
        except (OSError, UnicodeError):
            continue
        if not args or Path(args[0]).name != 'semantic-pilot' or 'skill' in args[1:2]:
            continue
        flags = {}
        for index, arg in enumerate(args[1:], 1):
            # 只取配置字段；不读取或打印 Pilot 接入凭据。
            for key in ('server-http', 'pilot-id', 'data-dir'):
                if arg == '--' + key and index + 1 < len(args):
                    flags[key] = args[index + 1]
                elif arg.startswith('--' + key + '='):
                    flags[key] = arg.split('=', 1)[1]
        candidates.append(flags)
    if pid and not candidates:
        fail('指定 PID 不是运行中的 Server/Pilot 服务。')
    if len(candidates) > 1:
        fail('发现多个 Pilot，请用 --pilot-pid 指定目标。')
    return candidates[0] if candidates else {}


def main():
    parser = argparse.ArgumentParser(prog='stop_current_skill.sh',
        description='自动停止当前占用机器人的 Skill，由 Pilot 清理 Worker 和资源锁。',
        epilog='例：SEMANTIC_TOKEN=网页登录token bash stop_current_skill.sh --dry-run\n'
               '确认现场已停止后：bash stop_current_skill.sh --confirm-safe\n'
               '仅适用于 Server 管理的 Pilot；不处理独立的 pilot skill run。',
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--server', default=os.getenv('SEMANTIC_SERVER'), help='Server 地址；默认从本机 Pilot 获取')
    parser.add_argument('--robot', default=os.getenv('SEMANTIC_ROBOT_ID'), help='robot_id；默认匹配本机 Pilot')
    parser.add_argument('--pilot-pid', type=int, help='本机有多个 Pilot 时指定 PID')
    parser.add_argument('--pilot-log', type=Path, help='Pilot JSON 日志；默认 data-dir/logs/pilot.log')
    parser.add_argument('--token-file', type=Path, help='网页登录 token 文件；也可设置 SEMANTIC_TOKEN')
    parser.add_argument('--timeout', type=float, default=30, help='每个等待阶段的最长秒数，默认 30')
    parser.add_argument('--reason', default='stop_current_skill.sh: operator requested stop', help='停止/确认原因')
    parser.add_argument('--dry-run', action='store_true', help='只查询目标，不发送停止或确认请求')
    parser.add_argument('--confirm-safe', action='store_true', help='调用者已确认现场物理状态安全；允许释放 interrupted 占用')
    parser.add_argument('--stop-workflow', action='store_true', help='允许连带停止所属整个 Workflow，防止任务继续派发 Skill')
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0 or not args.reason.strip():
        fail('--timeout 必须为正数，--reason 不能为空。')
    config = pilot_config(args.pilot_pid) if args.pilot_pid or not (args.server and args.robot) else {}
    server = (args.server or config.get('server-http') or 'http://127.0.0.1:9090').rstrip('/')
    parsed = urllib.parse.urlsplit(server)
    if parsed.scheme not in ('http', 'https') or not parsed.netloc or parsed.query or parsed.fragment or parsed.username:
        fail('--server 必须为有效的 http(s) Server 地址。')
    token = args.token_file.read_text().strip() if args.token_file else os.getenv('SEMANTIC_TOKEN', '').strip()
    if not token or '\n' in token or '\r' in token:
        fail('请设置 SEMANTIC_TOKEN 或 --token-file（框架网页登录 token，非 Pilot 接入 token）。')

    # 禁止重定向，避免向另一地址转发认证头；不输出响应正文中的业务数据。
    class NoRedirect(urllib.request.HTTPRedirectHandler):
        def redirect_request(self, *unused):
            return None
    opener = urllib.request.build_opener(NoRedirect)

    def api(method, path, body=None):
        request = urllib.request.Request(server + '/api/v1' + path,
            data=json.dumps(body).encode() if body is not None else None,
            headers={'Authorization': 'Bearer ' + token, 'Content-Type': 'application/json'}, method=method)
        try:
            with opener.open(request, timeout=min(5, args.timeout)) as response:
                value = json.load(response)
                if not isinstance(value, dict):
                    fail('Server 响应格式异常。')
                return value
        except urllib.error.HTTPError as error:
            try:
                code = json.load(error).get('error', {}).get('code', 'API_ERROR')
            except (ValueError, AttributeError):
                code = 'API_ERROR'
            raise APIError(error.code, code) from None
        except (urllib.error.URLError, TimeoutError):
            fail('Server 连接失败或超时；未确认资源释放，请用 --dry-run 重新查看状态。')

    def esc(value):
        return urllib.parse.quote(value, safe='')

    if args.robot:
        robot_id = args.robot
    else:
        devices = api('GET', '/devices').get('devices') or []
        pilot_id = config.get('pilot-id')
        if not pilot_id:
            fail('无法确定本机 Pilot 的 robot_id，请指定 --robot。')
        matches = [d for d in devices if (d.get('pilot') or {}).get('instance_id') == pilot_id]
        if len(matches) != 1:
            fail('无法唯一匹配本机 Pilot 的设备，请指定 --robot。')
        robot_id = matches[0]['robot_id']

    device_path = '/devices/' + esc(robot_id)

    def device():
        result = api('GET', device_path).get('device')
        if not isinstance(result, dict) or result.get('robot_id') != robot_id:
            fail('设备响应缺少正确的 robot_id。')
        return result

    initial = device()
    execution_id = initial.get('current_execution_id')
    if not execution_id:
        if initial.get('current_run') or initial.get('task_id') or initial.get('status') != 'idle':
            fail('当前没有可定位的 Skill，但设备仍有任务/调试占用或已离线；未执行清理。')
        print(f'{robot_id}: 没有 Skill 占用，无需清理。')
        return
    execution_path = '/robot-executions/' + esc(execution_id)

    def execution():
        result = api('GET', execution_path).get('execution')
        if not isinstance(result, dict) or result.get('id') != execution_id or result.get('robot_id') != robot_id:
            fail('执行记录与目标机器人不一致，停止清理。')
        return result

    record = execution()
    print(f"目标: robot={robot_id}, skill={record.get('skill_name')}@{record.get('skill_version')}, "
          f"execution={execution_id}, status={record.get('status')}", flush=True)
    workflow_id = record.get('workflow_id')
    if args.dry_run:
        print('dry-run: 只读取状态，未发送停止/确认请求。')
        if workflow_id:
            print('该 Skill 属于 Workflow，清理时需要 --stop-workflow。')
        return
    if (initial.get('pilot') or {}).get('status') != 'online':
        fail('Pilot 离线，无法确认停止或释放其内存锁。')
    if record.get('task_id') and not workflow_id:
        fail('该 Skill 有 Task 占用但缺少 Workflow 引用，请从任务控制入口停止。')
    if workflow_id and not args.stop_workflow:
        fail('该 Skill 属于 Workflow；需加 --stop-workflow 停止整个 Workflow，避免任务继续占用/派发。')

    log_path = args.pilot_log
    if not log_path and config.get('data-dir'):
        log_path = Path(config['data-dir']) / 'logs' / 'pilot.log'
    if args.confirm_safe and (not log_path or not log_path.is_file()):
        fail('--confirm-safe 需要本机 Pilot 日志以核实确认已送达；请指定 --pilot-log。')
    log_offset = log_path.stat().st_size if args.confirm_safe else 0
    log_inode = log_path.stat().st_ino if args.confirm_safe else None

    def still_ours(allow_empty=False):
        current = device().get('current_execution_id')
        if current != execution_id and not (allow_empty and not current):
            fail('当前执行已变化；只处理最初记录的执行，未清理新任务。')

    workflow_path = None
    if workflow_id:
        workflow_path = '/projects/' + esc(record['project_id']) + '/workflows/' + esc(workflow_id)

    def workflow_view():
        return api('GET', workflow_path + '/view')['workflow_view']

    active_statuses = {'queued', 'starting', 'running', 'waiting_agent', 'stopping'}
    still_ours()
    # 先停止调度者。普通对话 Run 由 Service.Stop 在同一链路中取消。
    if workflow_path:
        view = workflow_view()
        api('POST', workflow_path + '/stop', {'revision': view['workflow']['revision']})
    elif record.get('status') in active_statuses | {'interrupted'}:
        try:
            api('POST', execution_path + '/stop', {'reason': args.reason})
        except APIError as error:
            if error.status != 409:
                raise
            # 处理停止前恰好已进入终态的竞争；其他冲突不能强制跳过。
            record = execution()
            if record.get('status') in active_statuses:
                raise

    print('已请求 Pilot 停止；等待执行终态。', flush=True)
    deadline = time.monotonic() + args.timeout
    while True:
        record = execution()
        if record.get('status') not in active_statuses:
            break
        still_ours(allow_empty=True)
        if time.monotonic() >= deadline:
            fail('等待停止超时；未强行确认仍在 stopping 的执行。请稍后重试。')
        time.sleep(0.25)

    confirmed = False
    if record.get('status') == 'interrupted' or (
            record.get('status') in {'failed', 'stopped', 'cancelled'} and device().get('current_execution_id') == execution_id):
        if not args.confirm_safe:
            fail('物理停止尚未确认，资源占用保留。确认现场已停止后加 --confirm-safe 重试。')
        still_ours(allow_empty=True)
        body = {'physical_state_confirmed': True, 'reason': args.reason}
        if workflow_path:
            body['revision'] = workflow_view()['workflow']['revision']
            api('POST', workflow_path + '/confirm-stop', body)
        else:
            api('POST', execution_path + '/confirm-stop', body)
        confirmed = True

    # confirm-stop 的 HTTP 202 只表示接收；必须看到本次 Pilot 确认日志。
    # 普通 stopped 则要求 safe 证据，再核实 Server 的当前执行指针已清除。
    deadline = time.monotonic() + args.timeout
    pilot_confirmed = not confirmed
    while True:
        if confirmed and not pilot_confirmed:
            stat = log_path.stat()
            if stat.st_ino != log_inode or stat.st_size < log_offset:
                fail('Pilot 日志发生轮转，无法确认释放回执，请重新检查占用。')
            with log_path.open() as stream:
                stream.seek(log_offset)
                for line in stream:
                    try:
                        entry = json.loads(line)
                    except ValueError:
                        continue
                    if entry.get('execution_id') == execution_id and entry.get('msg') == '已按操作员确认释放物理动作占用':
                        pilot_confirmed = True
        current = device()
        if current.get('current_execution_id') and current['current_execution_id'] != execution_id:
            fail('原执行已结束，但机器人已被新执行占用；未触碰新任务。')
        record = execution()
        safe_terminal = record.get('status') in {'completed', 'failed', 'cancelled'} or (
            record.get('status') == 'stopped' and (record.get('result') or {}).get('safe') is True)
        tasks_free = True
        if workflow_path:
            tasks_free = not any(t.get('assigned_robot_id') == robot_id and
                t.get('status') in {'pending', 'running', 'paused', 'stopping'} for t in workflow_view().get('tasks', []))
        if safe_terminal and pilot_confirmed and not current.get('current_execution_id') and not current.get('current_run') and current.get('status') == 'idle' and tasks_free:
            print(f'{robot_id}: 当前 Skill 已停止，机器人占用已释放。')
            return
        if time.monotonic() >= deadline:
            fail('尚未确认所有占用已释放（Pilot 回执/执行指针/任务占用）；请用 --dry-run 检查。')
        time.sleep(0.25)


try:
    main()
except (StopError, OSError, ValueError, KeyError) as error:
    print(f'停止失败: {error}', file=sys.stderr)
    sys.exit(1)
except KeyboardInterrupt:
    print('已中断等待；不代表 Skill 已停止，请重新查询状态。', file=sys.stderr)
    sys.exit(130)
PY
