---
name: yolo-detect-native
description: 通过 Pilot 测试 YOLO 检测、点云生成、目标匹配和姿态估计四个 task
category: robot_skill
version: 0.1.0
runtime:
  api_version: 1
  python: '>=3.11'
  entrypoint: scripts.skill:run
  stop_entrypoint: scripts.skill:on_stop
  input_model: scripts.models:YoloInput
  state_model: scripts.models:YoloState
  result_model: scripts.models:YoloResult
  controllers: {}
required_actions:
- type: perception.yolo_detect
  schema_version: 2
- type: perception.generate_point_cloud
  schema_version: 2
- type: perception.match_target
  schema_version: 2
- type: perception.estimate_object_pose
  schema_version: 2
stop_actions:
- type: perception.yolo_detect
  schema_version: 2
---

默认 detect，仅测试 RGB-D 获取和 YOLO 检测；point_cloud 测试点云及完整外参；both 依次执行两项。
match 测试目标匹配；pose 使用独立标定的参考 RGB-D 或同坐标系参考点云测试姿态估计。
内外参来自 Ability 的 models.example.json，不在 skill 中伪造或重复写入。
stop_actions 是 Pilot 必须提供的非空允许列表，on_stop 不调用检测或运动任务。
检测没有物体时默认记录 object_count=0；require_objects=true 时判失败。
停止不会启动后续测试；已经提交的同步推理是否及时结束取决于 Ability/Pilot 的取消机制。

测试入口：`./start-shell/run_yolo_four_tasks.sh all`，或用 `detect`、`point_cloud`、`match`、`pose` 单独执行。
默认使用保存 RGB-D 的 YoloDetect.Replay，输入在 `inputs/yolo_tasks_replay/`。
四个输入均设置 `gray_box_only=true, top_k=2`；Top-K 依据相机坐标系三维距离中位数。
具体命令与结果说明见 `YOLO_FOUR_TASKS.md`。
