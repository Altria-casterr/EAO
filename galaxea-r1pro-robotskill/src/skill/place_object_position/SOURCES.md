# 实现依据

- morefine (`172.16.70.143`) 的 galaxea-r1pro-robotskill，快照提交
  `2ecd38220c9e9e8f07b229ceab4c63d1ff9e75dd`：以 place_object_force 的
  prepare、起始 Y 保存、X/Z 定位、高度确认、Action/检查点约定为基础独立派生。
- 同版本共享 TargetPointSolver 及 CapturePointSolver 的放置姿态约定。
  新 Skill 的双手撤手目标在自身 solver 中生成：固定外撤距离，末段相对开始高度抬升。
- 原行为 Put_Down/Agibot_G2：托举工具的预定位及下降、外撤、抬升后退撤手顺序。
- 现场 Ability 的 robot.get_state、robot.get_sensors_state、torso.move_to_posture_all、
  motion.move_end_effector_both_trajectory 和 motion.move_end_effector_both schema-v2 契约。

该 Skill 不修改原 place-object-force，不新增 Ability 或 Robot SDK 接口。
离线测试使用 MockSkillContext、模拟 Action 响应及独立 SDK Worker，不执行真机动作。
