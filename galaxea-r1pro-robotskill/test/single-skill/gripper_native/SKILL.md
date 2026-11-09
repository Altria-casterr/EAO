---
name: gripper-native
description: 验证新栈 gripper 原生参数链路的两段 Skill（单爪 set_opening_single → 双爪 set_opening_both，仅 end_effector 角色）
category: robot_skill
version: 0.1.0
runtime:
  api_version: 1
  python: ">=3.11"
  entrypoint: scripts.skill:run
  stop_entrypoint: scripts.skill:on_stop
  input_model: scripts.models:GripperNativeInput
  state_model: scripts.models:GripperNativeState
  result_model: scripts.models:GripperNativeResult
  controllers: {}
required_actions:
  - { type: gripper.set_opening_single, schema_version: 2 }
  - { type: gripper.set_opening_both, schema_version: 2 }
stop_actions:
  - { type: gripper.set_opening_both, schema_version: 2 }
debug_input:
  single:
    hand: left
    position: 50.0
    speed: 0.5
    effort: 0.3
  both:
    left_position: 50.0
    right_position: 50.0
    left_speed: 0.5
    right_speed: 0.5
    left_effort: 0.3
    right_effort: 0.3
---
