#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_ROOT="$(cd "$ROOT_DIR/../../.." && pwd)"
PILOT="${PILOT:-$WORKSPACE_ROOT/semantic-framework/.output/bin/semantic-pilot}"
RUN_PYTHON="${RUN_PYTHON:-${ABILITY_PYTHON:-/home/morefine/anaconda3/envs/Ability/bin/python}}"
SKILL_SDK_ROOT="${SKILL_SDK_ROOT:-$WORKSPACE_ROOT/semantic-skill/robot-skill}"
GALAXEA_SDK_SRC="${GALAXEA_SDK_SRC:-$WORKSPACE_ROOT/semantic-robotsdk/galaxea-r1pro-sdk/src}"
PROFILE="${PROFILE:-$ROOT_DIR/../src/config/yolo-robot-deployment.yaml}"
INPUT="${1:-$ROOT_DIR/../inputs/yolo_detect_native.json}"
CHECK_ONLY=0
if [[ "$INPUT" == '--check' ]]; then
  CHECK_ONLY=1
  INPUT="${2:-$ROOT_DIR/../inputs/yolo_detect_native.json}"
fi
[[ -x "$RUN_PYTHON" ]] || { echo "Python 不存在或不可执行: $RUN_PYTHON" >&2; exit 2; }
PYTHONPATH="$ROOT_DIR/../test/single-skill:$SKILL_SDK_ROOT:$GALAXEA_SDK_SRC${PYTHONPATH:+:$PYTHONPATH}" "$RUN_PYTHON" - "$INPUT" "$PILOT" "$PROFILE" <<'PY'
import importlib.util, os, sys, urllib.request, urllib.error
from pathlib import Path
import yaml
from yolo_detect_native.scripts.models import YoloInput
params = YoloInput.model_validate_json(Path(sys.argv[1]).read_text())
errors=[]
if not os.access(sys.argv[2],os.X_OK): errors.append('semantic-pilot 不存在或不可执行')
for module in ['semantic_robot_skill_sdk','ultralytics'] + (['sklearn'] if params.mode!='detect' else []):
    if importlib.util.find_spec(module) is None: errors.append(f'当前 Python 缺少 {module}')
profile=yaml.safe_load(Path(sys.argv[3]).read_text())
endpoint=profile['ability_framework']['endpoint']
try:
    urllib.request.urlopen(endpoint,timeout=3).close()
except urllib.error.HTTPError:
    pass  # 根路由 404 不代表服务没有启动。
except (urllib.error.URLError, OSError) as exc:
    errors.append(f'AbilityFramework 不可连接: {endpoint}: {exc}')
if errors:
    print('YOLO 测试前置检查失败：',file=sys.stderr)
    for error in errors: print(' - '+error,file=sys.stderr)
    print('请准备依赖、启动框架并加载声明 perception 动作的 YoloDetect.Demo。',file=sys.stderr)
    raise SystemExit(2)
print('基础环境检查通过；模型、话题与标定是否有效由真实 Ability 调用验证。')
PY
if [[ "$CHECK_ONLY" == 1 ]]; then exit 0; fi
OUTPUT_ROOT="${OUTPUT_ROOT:-$ROOT_DIR/outputs}"
mkdir -p "$OUTPUT_ROOT"
RUN_DIR="$(mktemp -d "$OUTPUT_ROOT/yolo-detect.XXXXXXXX")"
RESULT="$RUN_DIR/result.json"
EVENTS="$RUN_DIR/events.jsonl"
echo "测试结果目录: $RUN_DIR"
pilot_status=0
"$PILOT" skill run \
  --profile "$PROFILE" \
  --skill yolo-detect-native@0.1.0 \
  --input "$INPUT" \
  --skill-catalog "$ROOT_DIR/../test/single-skill" \
  --python "$RUN_PYTHON" \
  --python-path "$SKILL_SDK_ROOT" \
  --python-path "$GALAXEA_SDK_SRC" \
  --events "$EVENTS" \
  --result "$RESULT" \
  --timeout 10m || pilot_status=$?
if [[ -f "$RESULT" ]]; then
  "$RUN_PYTHON" -m json.tool "$RESULT"
else
  echo 'Pilot 未生成结果文件，请检查上面的错误。' >&2
  if [[ "$pilot_status" == 0 ]]; then exit 3; fi
fi
if [[ "$pilot_status" != 0 ]]; then exit "$pilot_status"; fi
"$RUN_PYTHON" - "$RESULT" <<'PY'
import json,sys
from pathlib import Path
result=json.loads(Path(sys.argv[1]).read_text()).get('result') or {}
if result.get('overall_status') != 'succeeded':
    print('YOLO 测试未全部成功，请查看 readings 中的 error。',file=sys.stderr)
    raise SystemExit(3)
PY
