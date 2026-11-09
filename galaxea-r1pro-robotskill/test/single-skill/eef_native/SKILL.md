---
name: eef-native
description: 通过 Pilot 调用单臂或双臂末端位姿、轨迹及力反馈精调 Ability
category: robot_skill
version: 0.1.0
runtime:
  api_version: 1
  python: ">=3.11"
  entrypoint: scripts.skill:run
  stop_entrypoint: scripts.skill:on_stop
  input_model: scripts.models:EEFInput
  state_model: scripts.models:EEFState
  result_model: scripts.models:EEFResult
  controllers: {}
required_actions:
  - { type: motion.move_end_effector_single, schema_version: 2 }
  - { type: motion.move_end_effector_both, schema_version: 2 }
  - { type: motion.move_end_effector_single_trajectory, schema_version: 2 }
  - { type: motion.move_end_effector_both_trajectory, schema_version: 2 }
  - { type: manipulation.force_align, schema_version: 2 }
stop_actions:
  - { type: motion.move_end_effector_single, schema_version: 2 }
  - { type: motion.move_end_effector_both, schema_version: 2 }
  - { type: motion.move_end_effector_single_trajectory, schema_version: 2 }
  - { type: motion.move_end_effector_both_trajectory, schema_version: 2 }
  - { type: manipulation.force_align, schema_version: 2 }
---

每次输入选择一个任务。所有目标坐标和力控现场参数必须由操作者填写。位置单位为米，四元数顺序为 xyzw，姿态容差为弧度。轨迹的 speed 是 Ability 日志参数，并不限制物理速度。停止后物理状态须根据机器人反馈确认。
