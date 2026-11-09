#!/usr/bin/env bash
# 真机 Robot Skill 本地验证：semantic-pilot skill run（不经 Server）。
# 用法：./run_skill.sh [skill名@版本]（默认 gripper-native@0.1.0）
# 输入取 inputs/<skill名下划线化>.json，无则用 {}。
set -euo pipefail

SKILL="${1:-gripper-native@0.1.0}"
ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_ROOT="$(cd "$ROOT_DIR/../../.." && pwd)"
SKILL_NAME="${SKILL%%@*}"
# 原生 skill 在 test/single-skill，业务 skill 在 src/skill；按技能名选择 catalog。
CATALOG="$ROOT_DIR/../test/single-skill"
[ -d "$CATALOG/${SKILL_NAME//-/_}" ] || CATALOG="$ROOT_DIR/../src/skill"

PILOT="${PILOT:-$WORKSPACE_ROOT/semantic-framework/.output/bin/semantic-pilot}"
RUN_PYTHON="${RUN_PYTHON:-$WORKSPACE_ROOT/.venvs/galaxea-skill/bin/python}"
SKILL_SDK_ROOT="${SKILL_SDK_ROOT:-$WORKSPACE_ROOT/semantic-skill/robot-skill}"
GALAXEA_SDK_SRC="${GALAXEA_SDK_SRC:-$WORKSPACE_ROOT/semantic-robotsdk/galaxea-r1pro-sdk/src}"

INPUT="$ROOT_DIR/../inputs/${SKILL_NAME//-/_}.json"
[ -f "$INPUT" ] || INPUT="$ROOT_DIR/../inputs/${SKILL_NAME}.json"
[ -f "$INPUT" ] || { echo '{}' > /tmp/skill-default-input.json; INPUT=/tmp/skill-default-input.json; }

"$PILOT" skill run \
  --profile "$ROOT_DIR/../src/config/robot-deployment.yaml" \
  --skill "$SKILL" \
  --input "$INPUT" \
  --skill-catalog "$CATALOG" \
  --python "$RUN_PYTHON" \
  --python-path "$SKILL_SDK_ROOT" \
  --python-path "$GALAXEA_SDK_SRC" \
  --python-path "$CATALOG" \
  --python-path "$ROOT_DIR/../src" \
  --events "/tmp/${SKILL_NAME}-events.jsonl" \
  --result "/tmp/${SKILL_NAME}-result.json" \
  --timeout 5m

echo "=== 结果 ==="
python3 -m json.tool "/tmp/${SKILL_NAME}-result.json"
