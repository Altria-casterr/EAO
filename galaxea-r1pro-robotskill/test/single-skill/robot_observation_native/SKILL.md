---
name: robot-observation-native
description: 读取机器人状态、传感器数据与 RGB-D 元数据
category: robot_skill
version: 0.1.0
runtime:
  api_version: 1
  python: '>=3.11'
  entrypoint: scripts.skill:run
  stop_entrypoint: scripts.skill:on_stop
  input_model: scripts.models:ObservationInput
  state_model: scripts.models:ObservationState
  result_model: scripts.models:ObservationResult
  controllers: {}
required_actions:
- type: robot.get_state
  schema_version: 2
- type: robot.get_sensors_state
  schema_version: 2
- type: robot.capture_rgbd
  schema_version: 2
stop_actions:
- type: robot.get_state
  schema_version: 2
---

只读观测。stop_actions 为 Pilot 允许列表；停止入口不调用任何 Ability。RGB-D 返回现有 Ability 提供的元数据，不包含图像文件。
