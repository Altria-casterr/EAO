#!/usr/bin/env bash
# 显式执行真机 Skill；PLACE_CHECK_ONLY=1 只校验输入和 Ability 绑定。
set -euo pipefail
if [[ $# -ne 2 ]]; then
  echo "用法：$0 <robot-deployment.yaml> <现场标定输入.json>" >&2
  exit 2
fi
PLACE_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PLACE_WORKSPACE="$(cd "$PLACE_ROOT/../.." && pwd)"
PLACE_PYTHON="${PLACE_PYTHON:-$PLACE_WORKSPACE/.venvs/galaxea-skill/bin/python}"
PLACE_PILOT="${PLACE_PILOT:-$PLACE_WORKSPACE/semantic-framework/.output/bin/semantic-pilot}"
PLACE_SKILL_SDK="${PLACE_SKILL_SDK:-$PLACE_ROOT/../robot-skill}"
[[ -f "$1" && -f "$2" ]] || { echo "部署或输入文件不存在" >&2; exit 2; }
[[ -x "$PLACE_PYTHON" ]] || { echo "Python 不存在：$PLACE_PYTHON" >&2; exit 2; }
PLACE_OUTPUT="$(mktemp -d /tmp/place-position.XXXXXX)"
export PYTHONPATH="$PLACE_SKILL_SDK:$PLACE_ROOT/src:$PLACE_ROOT/src/skill${PYTHONPATH:+:$PYTHONPATH}"
"$PLACE_PYTHON" - "$1" "$2" "$PLACE_OUTPUT/deployment.yaml" <<'PY'
import json
import math
import sys
import urllib.request
from pathlib import Path

import yaml
import place_object_position
from place_object_position.scripts.models import PlaceInput

deployment = yaml.safe_load(Path(sys.argv[1]).read_text())
params = PlaceInput.model_validate_json(Path(sys.argv[2]).read_text())
endpoint = deployment['ability_framework']['endpoint'].rstrip('/')
heartbeat = json.load(urllib.request.urlopen(endpoint + '/api/ability-heartbeat', timeout=5))
bindings = {}
for role, name in {'robot': 'RobotObservation.Demo',
                   'torso': 'TorsoControl.Demo',
                   'manipulator_motion': 'JointEEFPoseControl.Demo'}.items():
    instance = (deployment.get('abilities', {}).get(role) or {}).get('instance_id')
    if not instance:
        raise SystemExit(f'部署缺少 {role}.instance_id')
    matches = [item for item in heartbeat if item.get('id') == instance
               and item.get('abilityName') == name and item.get('state') == 'Running']
    if len(matches) != 1:
        raise SystemExit(f'前置检查失败：{role} 绑定 {instance} 不是当前 Running 的 {name}')
    bindings[role] = {'instance_id': instance}
deployment['abilities'] = bindings
# 三段双手轨迹、三段同步撤手普通点位，以及躯干/读取/段间等待。
approach = params.approach_motion
motion = params.motion
budget = math.ceil(
    approach.timeout * (approach.num_points or approach.max_num_points) + 15
    + 2 * (motion.timeout * (motion.num_points or motion.max_num_points) + 15)
    + 3 * (params.retract_motion.timeout + 15)
    + (params.torso.timeout + 10 if params.torso.enabled else 0)
    + 2 * params.stage_pause_s + 300)
budget = max(1800, budget)
pilot = deployment.setdefault('pilot', {})
pilot['worker_timeout_seconds'] = max(budget, pilot.get('worker_timeout_seconds') or 0)
Path(sys.argv[3]).write_text(yaml.safe_dump(deployment, allow_unicode=True, sort_keys=False))
Path(sys.argv[3]).with_name('timeout.txt').write_text(str(budget) + '\n')
manifest = yaml.safe_load((Path(place_object_position.__file__).parent / 'SKILL.md').read_text().split('---', 2)[1])
Path(sys.argv[3]).with_name('skill-ref.txt').write_text(f"{manifest['name']}@{manifest['version']}\n")
print('输入及三个 Ability 绑定校验通过。')
PY
echo "本次部署与执行日志：$PLACE_OUTPUT"
if [[ "${PLACE_CHECK_ONLY:-0}" == "1" ]]; then
  echo "仅检查，未启动 Skill。"
  exit 0
fi
[[ -x "$PLACE_PILOT" ]] || { echo "Pilot 不存在：$PLACE_PILOT" >&2; exit 2; }
read -r PLACE_RUN_TIMEOUT < "$PLACE_OUTPUT/timeout.txt"
read -r PLACE_SKILL_REF < "$PLACE_OUTPUT/skill-ref.txt"
"$PLACE_PILOT" skill run \
  --profile "$PLACE_OUTPUT/deployment.yaml" --skill "$PLACE_SKILL_REF" --input "$2" \
  --skill-catalog "$PLACE_ROOT/src/skill" --python "$PLACE_PYTHON" \
  --python-path "$PLACE_ROOT/src" --python-path "$PLACE_SKILL_SDK" \
  --events "$PLACE_OUTPUT/events.jsonl" --result "$PLACE_OUTPUT/result.json" \
  --timeout "${PLACE_TIMEOUT:-${PLACE_RUN_TIMEOUT}s}"
