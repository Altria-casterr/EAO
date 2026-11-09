---
name: woosh-move-native
description: 验证新栈底盘相对移动链路的最小 Skill（navigation.relative_move 单轴 x/y/z，停止用 navigation.stop）
category: robot_skill
version: 0.1.0
runtime:
  api_version: 1
  python: ">=3.11"
  entrypoint: scripts.skill:run
  stop_entrypoint: scripts.skill:on_stop
  input_model: scripts.models:WooshMoveInput
  state_model: scripts.models:WooshMoveState
  result_model: scripts.models:WooshMoveResult
  controllers: {}
required_actions:
  - { type: navigation.relative_move, schema_version: 2 }
stop_actions:
  - { type: navigation.stop, schema_version: 2 }
debug_input:
  axis: x
  value: 0.1
  speed: 0.1
  timeout: 10.0
---
