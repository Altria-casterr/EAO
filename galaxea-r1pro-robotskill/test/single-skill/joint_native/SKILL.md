---
name: joint-native
description: 手臂关节原生参数链路验证：单臂到位后执行双臂控制
category: robot_skill
version: 0.1.0
runtime:
  api_version: 1
  python: ">=3.11"
  entrypoint: scripts.skill:run
  stop_entrypoint: scripts.skill:on_stop
  input_model: scripts.models:JointNativeInput
  state_model: scripts.models:JointNativeState
  result_model: scripts.models:JointNativeResult
  controllers: {}
required_actions:
  - { type: motion.move_to_posture_single, schema_version: 2 }
  - { type: motion.move_to_posture_both, schema_version: 2 }
# Pilot 要求停止阶段声明可绑定 Action；on_stop 不发运动指令。
stop_actions:
  - { type: motion.move_to_posture_both, schema_version: 2 }
---

输入文件为 inputs/joint_native.json。single 指定一侧七个关节角度；其成功到位后执行 both，分别指定左右臂七个关节角度。数组顺序必须与机器人反馈的关节顺序一致，角度与容差单位为弧度。

实机右臂反馈的 name 为 ["right_arm"]、position 为七个值；输入中的 joint_names 等名称数组使用单个臂组名称 ["left_arm"] 或 ["right_arm"]。也可输入七个实际关节名称。可选 velocities、effort 及对应左右臂字段均为七个数值；未提供时使用 SDK 默认行为。

停止入口仅报告手臂物理状态待确认，不向机器人发运动指令。双臂 Ability 当前只登记左臂命令句柄，停止双臂时需要现场确认两侧状态。
