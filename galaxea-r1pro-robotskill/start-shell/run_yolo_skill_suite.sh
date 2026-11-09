#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_ROOT="$(cd "$ROOT_DIR/../../.." && pwd)"
exec "${RUN_PYTHON:-${ABILITY_PYTHON:-/home/morefine/anaconda3/envs/Ability/bin/python}}" "$ROOT_DIR/../test/test_yolo_skill_suite.py" "$@"
