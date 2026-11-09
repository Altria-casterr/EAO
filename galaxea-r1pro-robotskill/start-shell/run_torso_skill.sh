#!/usr/bin/env bash
# 真机 Robot Skill 本地验证：semantic-pilot skill run（不经 Server）。
# 用法：./run_torso_skill.sh [skill名@版本]（默认 torso-native@0.1.0）。
# 运行前需准备 inputs/torso_native.json；示例文件不能自动用于真机。
set -euo pipefail

SKILL="${1:-torso-native@0.1.0}"
ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_ROOT="$(cd "$ROOT_DIR/../../.." && pwd)"
SKILL_NAME="${SKILL%%@*}"

PILOT="${PILOT:-$WORKSPACE_ROOT/semantic-framework/.output/bin/semantic-pilot}"
RUN_PYTHON="${RUN_PYTHON:-$WORKSPACE_ROOT/.venvs/galaxea-skill/bin/python}"
SKILL_SDK_ROOT="${SKILL_SDK_ROOT:-$WORKSPACE_ROOT/semantic-skill/robot-skill}"

INPUT="$ROOT_DIR/../inputs/${SKILL_NAME//-/_}.json"
[ -f "$INPUT" ] || { echo "缺少输入文件：$INPUT" >&2; exit 2; }

"$PILOT" skill run \
  --profile "$ROOT_DIR/../src/config/robot-deployment.yaml" \
  --skill "$SKILL" \
  --input "$INPUT" \
  --skill-catalog "$ROOT_DIR/../test/single-skill" \
  --python "$RUN_PYTHON" \
  --python-path "$SKILL_SDK_ROOT" \
  --events "/tmp/${SKILL_NAME}-events.jsonl" \
  --result "/tmp/${SKILL_NAME}-result.json" \
  --timeout 5m

echo "=== 结果 ==="
python3 -m json.tool "/tmp/${SKILL_NAME}-result.json"
