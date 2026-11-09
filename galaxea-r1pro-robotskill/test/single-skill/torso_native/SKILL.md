---
name: torso-native
description: 躯干原生参数链路验证：单关节控制后执行四关节控制
category: robot_skill
version: 0.1.0
runtime:
  api_version: 1
  python: ">=3.11"
  entrypoint: scripts.skill:run
  stop_entrypoint: scripts.skill:on_stop
  input_model: scripts.models:TorsoNativeInput
  state_model: scripts.models:TorsoNativeState
  result_model: scripts.models:TorsoNativeResult
  controllers: {}
required_actions:
  - { type: torso.move_to_target_pos_single, schema_version: 2 }
  - { type: torso.move_to_posture_all, schema_version: 2 }
# Pilot 要求停止阶段至少声明一个可绑定 Action；on_stop 当前不发运动指令。
stop_actions:
  - { type: torso.move_to_posture_all, schema_version: 2 }
---
