#!/usr/bin/env bash
# 默认仅执行本 Skill；PREPARE_CHECK_ONLY=1 时只校验输入和能力绑定。
set -euo pipefail
PREPARE_ROOT="$(cd "$(dirname "$0")" && pwd)"
PREPARE_REPO="$(cd "$PREPARE_ROOT/.." && pwd)"
PREPARE_WORKSPACE="$(cd "$PREPARE_REPO/../.." && pwd)"
PREPARE_PYTHON="${PREPARE_PYTHON:-$PREPARE_WORKSPACE/.venvs/galaxea-skill/bin/python}"
PREPARE_PILOT="${PREPARE_PILOT:-$PREPARE_WORKSPACE/semantic-framework/.output/bin/semantic-pilot}"
if [[ $# -ne 0 && $# -ne 2 ]]; then
  echo "用法：$0 [Deployment.yaml input.json]" >&2
  exit 2
fi
PREPARE_PROFILE="${1:-$PREPARE_REPO/src/config/prepare-deployment.yaml}"
PREPARE_INPUT="${2:-$PREPARE_REPO/inputs/robot_prepare.json}"
PREPARE_OUTPUT="$(mktemp -d /tmp/robot-prepare.XXXXXX)"
echo "执行日志：$PREPARE_OUTPUT"
PYTHONPATH="$PREPARE_REPO/src/skill:$PREPARE_WORKSPACE/semantic-skill/robot-skill${PYTHONPATH:+:$PYTHONPATH}" \
"$PREPARE_PYTHON" - "$PREPARE_PROFILE" "$PREPARE_INPUT" "$PREPARE_OUTPUT" <<'PY'
import json, sys, urllib.request
from pathlib import Path
import yaml
from robot_prepare.scripts.models import PrepareInput

try:
    profile = yaml.safe_load(Path(sys.argv[1]).read_text())
    params = PrepareInput.model_validate_json(Path(sys.argv[2]).read_text())
    output = Path(sys.argv[3])
    endpoint = profile['ability_framework']['endpoint'].rstrip('/')
    with urllib.request.urlopen(endpoint + '/api/ability-heartbeat', timeout=5) as response:
        heartbeat = json.load(response)
    expected = {'torso': 'TorsoControl.Demo',
                'joint_motion': 'JointControl.Demo'}
    bindings = {}
    for role, name in expected.items():
        pinned = (profile.get('abilities', {}).get(role) or {}).get('instance_id')
        matches = [r for r in heartbeat if r.get('abilityName') == name and r.get('state') == 'Running'
                   and (not pinned or r.get('id') == pinned)]
        if len(matches) != 1:
            raise ValueError(f'{role} 需要唯一 Running 的 {name}，找到 {len(matches)} 个；'
                             '请激活对应能力，多个实例时在 Deployment 填写 instance_id')
        bindings[role] = {'instance_id': matches[0]['id']}
    profile['abilities'] = bindings
    (output / 'deployment.yaml').write_text(yaml.safe_dump(profile, allow_unicode=True, sort_keys=False))
    (output / 'input.json').write_text(params.model_dump_json(indent=2))
    print('预备 Skill 输入和两项 Ability 绑定检查通过。')
    for role, binding in bindings.items():
        print(f"  {role}: {binding['instance_id']}")
except Exception as error:
    raise SystemExit(f'预备 Skill 前置检查失败：{error}') from error
PY
if [[ "${PREPARE_CHECK_ONLY:-0}" == "1" ]]; then
  echo "仅校验输入和能力绑定，未启动 Skill；未验证真机通信或动作。"
  exit 0
fi
set +e
"$PREPARE_PILOT" skill run \
  --profile "$PREPARE_OUTPUT/deployment.yaml" --skill robot-prepare@0.1.0 \
  --input "$PREPARE_OUTPUT/input.json" --skill-catalog "$PREPARE_REPO/src/skill" \
  --python "$PREPARE_PYTHON" --python-path "$PREPARE_REPO/src/skill" \
  --python-path "$PREPARE_WORKSPACE/semantic-skill/robot-skill" \
  --events "$PREPARE_OUTPUT/events.jsonl" --result "$PREPARE_OUTPUT/result.json" --timeout 5m
PREPARE_EXIT=$?
set -e
if [[ -f "$PREPARE_OUTPUT/result.json" ]]; then
  "$PREPARE_PYTHON" -m json.tool "$PREPARE_OUTPUT/result.json"
fi
exit "$PREPARE_EXIT"
