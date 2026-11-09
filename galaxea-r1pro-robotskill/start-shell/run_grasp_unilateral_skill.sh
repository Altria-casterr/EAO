#!/usr/bin/env bash
# 手工真机入口：默认右手主抓。GRASP_CHECK_ONLY=1 时只检查，绝不启动 Skill。
# 用法：bash run_grasp_unilateral_skill.sh [部署.yaml 现场输入.json]
set -euo pipefail
if [[ $# -ne 0 && $# -ne 2 ]]; then
  echo "用法：$0 [单手部署.yaml 单手现场输入.json]" >&2
  exit 2
fi
GRASP_CATALOG="$(cd "$(dirname "$0")" && pwd)"
GRASP_REPO="$(cd "$GRASP_CATALOG/.." && pwd)"
GRASP_WORKSPACE="$(cd "$GRASP_REPO/../.." && pwd)"
GRASP_PROFILE="${1:-$GRASP_REPO/src/config/grasp-unilateral-deployment.yaml}"
GRASP_INPUT="${2:-$GRASP_REPO/inputs/grasp_object_unilateral.json}"
GRASP_PILOT="${GRASP_PILOT:-$GRASP_WORKSPACE/semantic-framework/.output/bin/semantic-pilot}"
GRASP_PYTHON="${GRASP_PYTHON:-/home/morefine/anaconda3/envs/Ability/bin/python}"
export GRASP_PYTHON
export PYTHONPATH="$GRASP_REPO/src:$GRASP_REPO/src/skill:$GRASP_WORKSPACE/semantic-skill/robot-skill:$GRASP_WORKSPACE/semantic-robotsdk/galaxea-r1pro-sdk/src${PYTHONPATH:+:$PYTHONPATH}"
[[ -f "$GRASP_PROFILE" && -f "$GRASP_INPUT" ]] || { echo "部署或输入文件不存在" >&2; exit 2; }

# 只加载输入模型，不调用 Skill.run、Ability 或 Robot SDK。
GRASP_OUTPUT="$(mktemp -d /tmp/grasp-unilateral.XXXXXX)"
"$GRASP_PYTHON" - "$GRASP_INPUT" "$GRASP_OUTPUT" <<'PY'
import json, sys
from pathlib import Path

import yaml
from grasp_object_unilateral.scripts.models import GraspInput
params = GraspInput.model_validate(json.loads(Path(sys.argv[1]).read_text()))
print(f"单手输入有效：主抓手={params.grasp_axis}，验证侧={params.verify_side}，"
      f"世界目标={params.target.position_m}，X偏移={params.capture_x_offset_m}m")
print("提示：motion.speed 不是实际限速；执行前请确认现场目标、工具接合和安全空间。")
# Skill 版本从 SKILL.md frontmatter 现取：Pilot 的 SkillCatalog.Resolve 做精确匹配，
# 脚本里写死版本号会在升版本后静默失配（报 "version X is unavailable" 且不下发动作）。
import grasp_object_unilateral
manifest = yaml.safe_load(
    (Path(grasp_object_unilateral.__file__).parent / 'SKILL.md').read_text().split('---', 2)[1])
Path(sys.argv[2], 'skill-ref.txt').write_text(f"{manifest['name']}@{manifest['version']}\n")
PY
read -r GRASP_SKILL_REF < "$GRASP_OUTPUT/skill-ref.txt"
echo "Skill：$GRASP_SKILL_REF"

# 复用稳定双手入口的只读前置检查，但强制 CHECK_ONLY，绝不会从该子脚本运行双手 Skill。
# 同时强制开启预检，禁止继承 GRASP_SKIP_PREFLIGHT=1 意外跳过相机/TF/绑定检查。
GRASP_CHECK_ONLY=1 GRASP_SKIP_PREFLIGHT=0 bash "$GRASP_CATALOG/run_grasp_bilateral_skill.sh" "$GRASP_PROFILE" "$GRASP_INPUT"
if [[ "${GRASP_CHECK_ONLY:-0}" == "1" ]]; then
  echo "单手前置检查完成；未启动 Skill、未发送机器人动作。"
  exit 0
fi

echo "开始执行单手 Skill；执行日志：$GRASP_OUTPUT"
"$GRASP_PILOT" skill run \
  --profile "$GRASP_PROFILE" --skill "$GRASP_SKILL_REF" --input "$GRASP_INPUT" \
  --skill-catalog "$GRASP_REPO/src/skill" --python "$GRASP_PYTHON" \
  --python-path "$GRASP_REPO/src" \
  --python-path "$GRASP_WORKSPACE/semantic-skill/robot-skill" \
  --events "$GRASP_OUTPUT/events.jsonl" --result "$GRASP_OUTPUT/result.json" --timeout 15m
