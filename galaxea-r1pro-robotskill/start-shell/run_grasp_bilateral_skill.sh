# 显式执行真机 Skill；构建和离线测试不调用本脚本。
# 用法：bash run_grasp_bilateral_skill.sh /path/robot-deployment.yaml /path/现场输入.json
set -euo pipefail
if [[ $# -ne 2 ]]; then
  echo "用法：$0 <robot-deployment.yaml> <现场输入.json>" >&2
  exit 2
fi
GRASP_CATALOG="$(cd "$(dirname "$0")" && pwd)"
GRASP_REPO="$(cd "$GRASP_CATALOG/.." && pwd)"
GRASP_WORKSPACE="$(cd "$GRASP_REPO/../.." && pwd)"
GRASP_PILOT="${GRASP_PILOT:-$GRASP_WORKSPACE/semantic-framework/.output/bin/semantic-pilot}"
GRASP_PYTHON="${GRASP_PYTHON:-/home/morefine/anaconda3/envs/Ability/bin/python}"
export PYTHONPATH="$GRASP_WORKSPACE/semantic-robotsdk/galaxea-r1pro-sdk/src${PYTHONPATH:+:$PYTHONPATH}"
[[ -f "$1" && -f "$2" ]] || { echo "部署或输入文件不存在" >&2; exit 2; }

# 真机动作前先验证绑定实例和 RGB-D 新帧；感知不健康时不得进入 Skill。
if [[ "${GRASP_SKIP_PREFLIGHT:-0}" != "1" ]]; then
  "$GRASP_PYTHON" - "$1" "$2" "$GRASP_WORKSPACE" <<'PY'
import json, os, socket, sys, time, urllib.request
from pathlib import Path
from urllib.parse import urlsplit

import roslibpy
import yaml
from galaxea_r1pro.galaxea_r1pro_0.utils.tf_buffer import TransformBuffer

deployment = yaml.safe_load(Path(sys.argv[1]).read_text())
inputs = json.loads(Path(sys.argv[2]).read_text())
endpoint = deployment["ability_framework"]["endpoint"].rstrip("/")
heartbeat = json.load(urllib.request.urlopen(endpoint + "/api/ability-heartbeat", timeout=3))
running = {item.get("id") for item in heartbeat if item.get("state") == "Running"}
missing = {role: spec.get("instance_id") for role, spec in deployment.get("abilities", {}).items()
           if spec.get("instance_id") not in running}
if missing:
    raise SystemExit(f"真机 Skill 前置检查失败：Ability 实例未运行或 UUID 已变化：{missing}")

yolo_profile = Path(os.environ.get(
    "GRASP_YOLO_PROFILE",
    str(Path(sys.argv[3]) / "semantic-ability/galaxea-r1pro-ability/src/galaxea_r1pro_0/config/r1pro-real-yolo.json"),
))
profile = json.loads(yolo_profile.read_text())
sensor = (inputs.get("perception") or {}).get("sensor_id", "head_camera")
topics = ((profile.get("options") or {}).get("camera_topics") or {}).get(sensor)
if not topics:
    raise SystemExit(f"真机 Skill 前置检查失败：Yolo profile 未配置相机 {sensor!r}")
parsed = urlsplit(profile["endpoint"])
# Twisted reactor 在同一 Python 进程中不可重启；先用 TCP 做有限重试，随后只
# 创建一次 roslibpy 客户端，避免首轮失败后出现 ReactorNotRestartable。
tcp_error = None
for attempt in range(1, 4):
    try:
        with socket.create_connection((parsed.hostname, parsed.port), timeout=3):
            tcp_error = None
        break
    except Exception as exc:
        tcp_error = exc
        if attempt < 3:
            time.sleep(0.5)
if tcp_error is not None:
    raise SystemExit(
        f"真机 Skill 前置检查失败：ROS Bridge {parsed.hostname}:{parsed.port} "
        f"连续 3 次 TCP 连接失败：{type(tcp_error).__name__}: {tcp_error}"
    )
ros = roslibpy.Ros(host=parsed.hostname, port=parsed.port)
try:
    ros.run(timeout=5)
except Exception as exc:
    raise SystemExit(
        f"真机 Skill 前置检查失败：ROS Bridge WebSocket 握手失败："
        f"{type(exc).__name__}: {exc}"
    ) from exc
subscribers, seen = [], set()
try:
    topic_types = (profile.get("options") or {}).get("camera_topic_types") or {}
    for topic in topics.values():
        sub = roslibpy.Topic(ros, topic, topic_types.get(topic, "sensor_msgs/msg/CompressedImage"))
        sub.subscribe(lambda _message, current=topic: seen.add(current))
        subscribers.append(sub)
    deadline = time.monotonic() + 6
    expected = set(topics.values())
    while seen != expected and time.monotonic() < deadline:
        time.sleep(0.05)
    absent = sorted(expected - seen)
    if absent:
        raise SystemExit(f"真机 Skill 前置检查失败：6 秒内没有 RGB-D 新帧：{absent}")

    tf_buffer = TransformBuffer()
    for topic in ("/tf_static", "/tf"):
        sub = roslibpy.Topic(ros, topic, "tf2_msgs/msg/TFMessage")
        sub.subscribe(lambda message, static=(topic == "/tf_static"):
                      tf_buffer.update(message, static=static))
        subscribers.append(sub)
    deadline = time.monotonic() + 10
    transform_error = None
    while time.monotonic() < deadline:
        try:
            tf_buffer.lookup("map", "body", max_age_s=1.0)
            transform_error = None
            break
        except LookupError as exc:
            transform_error = exc
            time.sleep(0.05)
    if transform_error is not None:
        raise SystemExit(
            "真机 Skill 前置检查失败：10 秒内没有有效的 map → body TF。"
            "请先完成机器人地图定位，确认 TF 中存在 map → odom → body。"
        )
finally:
    for sub in subscribers:
        sub.unsubscribe()
    if ros.is_connected:
        ros.close()
print("真机 Skill 前置检查通过：所需 Ability Running，RGB-D 正在发布。")
PY
fi

if [[ "${GRASP_CHECK_ONLY:-0}" == "1" ]]; then
  echo "仅执行前置检查，未启动 Skill。"
  exit 0
fi

GRASP_OUTPUT="$(mktemp -d /tmp/grasp-bilateral.XXXXXX)"
echo "执行日志：$GRASP_OUTPUT"
# --skill 的版本号必须与 src/skill/grasp_object_bilateral/SKILL.md frontmatter 的
# version 完全一致：Pilot 的 SkillCatalog.Resolve 做精确匹配，写错会直接报
# "skill grasp-object-bilateral version X is unavailable" 且不下发任何动作。
"$GRASP_PILOT" skill run \
  --profile "$1" --skill grasp-object-bilateral@0.2.3 --input "$2" \
  --skill-catalog "$GRASP_REPO/src/skill" --python "$GRASP_PYTHON" \
  --python-path "$GRASP_REPO/src" \
  --python-path "$GRASP_WORKSPACE/semantic-skill/robot-skill" \
  --events "$GRASP_OUTPUT/events.jsonl" --result "$GRASP_OUTPUT/result.json" --timeout 15m
