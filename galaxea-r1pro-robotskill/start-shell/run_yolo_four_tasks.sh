#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_ROOT="$(cd "$ROOT_DIR/../../.." && pwd)"
RUN_PYTHON="${RUN_PYTHON:-${ABILITY_PYTHON:-/home/morefine/anaconda3/envs/Ability/bin/python}}"
TASK="${1:-all}"
if [[ "$#" -gt 0 ]]; then shift; fi
case "$TASK" in
  all|detect|point_cloud|match|pose) ;;
  *) echo '用法: ./run_yolo_four_tasks.sh [all|detect|point_cloud|match|pose]（all 可追加 suite 参数；单项可追加输入 JSON 路径）' >&2; exit 2 ;;
esac
if [[ "$TASK" != all && "$#" -gt 1 ]]; then echo '单项只接受一个可选输入 JSON 路径' >&2; exit 2; fi
# This entrypoint always routes to the saved RGB-D replay framework.
export PROFILE="$ROOT_DIR/../src/config/yolo-replay-deployment.yaml"
"$RUN_PYTHON" "$ROOT_DIR/../test/start_yolo_replay_test.py"
"$RUN_PYTHON" - <<'PY'
import json,urllib.request
with urllib.request.urlopen('http://127.0.0.1:8082/api/ability-heartbeat',timeout=5) as response:
    items=json.load(response)
active=[h for h in items if h.get('abilityName','').startswith('YoloDetect.') and h.get('state')=='Running' and h.get('abilityPort',0)>0]
if len(active)!=1 or active[0]['abilityName']!='YoloDetect.Replay':
    raise SystemExit('测试要求唯一活动的 YoloDetect.Replay，避免错误路由')
PY
if [[ "$TASK" == all ]]; then
  exec "$ROOT_DIR/run_yolo_skill_suite.sh" --replay --gray-box-only --top-k 2 \
    --reference "$ROOT_DIR/../test/single-skill/yolo_runtime/suite-reference-20260928-113928/reference.json" "$@"
fi
INPUT="${1:-$ROOT_DIR/../inputs/yolo_tasks_replay/$TASK.json}"
exec "$ROOT_DIR/run_yolo_detect_skill.sh" "$INPUT"
