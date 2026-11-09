# 接口来源

- ../torso_native、../joint_native：现有真机 Action 参数约定。
- ../../../../../semantic-ability/galaxea-r1pro-ability/src/galaxea_r1pro_0/abilities/{torso_control,joint_control}/ability.manifest.yaml：schemaVersion 2。
- 双臂预备角来自用户，单位弧度；躯干四角固定为零。
- JointControl 要求每侧 joint_names 与 positions 非空且等长；本 Skill 各传七个名称和七个角度。
- 默认逐关节命名沿用 ../../../../../semantic-ability/galaxea-r1pro-ability/src/galaxea_r1pro_0/tasks/semantic_adapter.py 中的 LEFT_ARM_JOINT_NAMES / RIGHT_ARM_JOINT_NAMES；本 Skill 不调用旧 semantic 适配器。
