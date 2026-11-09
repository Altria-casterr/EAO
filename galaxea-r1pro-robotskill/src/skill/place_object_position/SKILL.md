---
name: place-object-position
description: R1 Pro 双臂位置放置：保持双手起始 Y，先 X 后 Z 到达放置目标，随后双手同步下降、外撤、抬高后退，以位姿确认完成。
category: robot_skill
version: 0.1.1
runtime:
  api_version: 1
  python: ">=3.11"
  entrypoint: scripts.skill:run
  stop_entrypoint: scripts.skill:on_stop
  input_model: scripts.models:PlaceInput
  state_model: scripts.models:PlaceState
  result_model: scripts.models:PlaceResult
  controllers: {}
required_actions:
  - { type: torso.move_to_posture_all, schema_version: 2 }
  - { type: robot.get_state, schema_version: 2 }
  - { type: robot.get_sensors_state, schema_version: 2 }
  - { type: motion.move_end_effector_both_trajectory, schema_version: 2 }
  - { type: motion.move_end_effector_both, schema_version: 2 }
stop_actions:
  - { type: torso.move_to_posture_all, schema_version: 2 }
  - { type: motion.move_end_effector_both_trajectory, schema_version: 2 }
  - { type: motion.move_end_effector_both, schema_version: 2 }
---

# 双臂位置放置

本 Skill 基于 morefine 的 `place-object-force` 独立派生，处理上游已双手托持的箱体。
一次执行完成一个箱体的放置；底盘站位及多箱目标调度由上游负责。
执行顺序为：躯干准备 → 保存双手起始 Y → X/Z 预定位 → 对齐 X → 下放 Z →
三段双手同步撤出 → 校验完成。全程不调用力控，不读取力数据。

使用 `place-object-position@0.1.0`，绑定现有的 `robot`（RobotObservation.Demo）、
`torso`（TorsoControl.Demo）、`manipulator_motion`（JointEEFPoseControl.Demo）三个角色。
关闭躯干调整时仍保留三个角色绑定。所有动作通过现有 schema-v2 Action 执行。

## 输入

位置、尺寸和偏移单位为米；时间为秒；躯干关节角为弧度，四元数顺序为 xyzw。
完整示例：`inputs/place_object_position.example.json`。

**以下参数必须从 realmap（语义地图 `real_map` 槽位）的目标放置实体读取，不得照抄示例或硬编码：**

| 字段 | 取值来源（realmap 实体） |
|---|---|
| `target.object_ref` | 实体 `name`（放置目标实体，如 `dest-l5-1`） |
| `target.position_m` | 实体 `pose.position` 的 `x/y/z`（米，`world_frame` 固定 `map`） |
| `target.size` | 实体 `bounds.size`：`long_m ← size.y`、`width_m ← size.x`、`height_m ← size.z` |
| `torso.positions` | 实体 `properties.torso_joint_positions`（四个躯干关节角，弧度） |
| `torso.speeds` | 实体 `properties.torso_speeds`（四个躯干关节速度，弧度/秒） |

取值方式：先 `map.query`（或读 `real_map` 槽位）按实体名查到该放置实体，再填上面各项；
realmap 中不存在该放置实体时不要执行本 Skill。其余字段（偏移、运动、撤手等）不在
realmap 中，使用本文件默认值/现场标定值。

| 参数 | 含义与默认值 |
| --- | --- |
| `target` | 必填：箱体引用 object_ref、map 下目标箱心 position_m=[x,y,z]、尺寸 size={long_m,width_m,height_m} |
| `relative_frame` | 固定 torso_link4 |
| `torso` | enabled 默认 true；启用时必须传四关节 positions；timeout=10，tol=0.02，speeds=[0.5,0.8,0.7,0.7] |
| `preposition` | x_offset_m=-0.03、z_offset_m=0.07、grasp_center_offset_m=[0,0,0]；工具偏移仅 X/Z 生效 |
| `motion` | X/Z 双手轨迹：pos_tol=0.005、orient_tol=0.01、timeout=10、loop_freq=0.1、num_points=null、max_num_points=100、sequential=true、axis_order=yzx、step_distance=0.05、speed=0.05 |
| `approach_motion` | 预定位轨迹，沿用 motion 的轨迹字段；pos_tol=0.03、axis_order=zyx，其余默认同 motion |
| `retract_motion` | 双手普通点位撤手：pos_tol=0.005、orient_tol=0.01、timeout=10、loop_freq=0.1；继承的轨迹字段不下发，speed 默认 0.03 |
| `retract` | lower_m=0.02、minimum_outward_m=0.03、backward_m=0.1、safe_offset_z=0.03 |
| `stage_pause_s` | X/Z 完成后各等待 0.5 秒；设为 0 可关闭 |

最小输入为 target 加 `torso.enabled=false`，或 target 加已标定的 `torso.positions`。
现场示例沿用既有标定值，包括工具 X 偏移 0.07、下降 0.05、外撤 0.1、后退 0.1、
撤手末段抬高 0.15；这些是显式覆盖值，省略 safe_offset_z 时默认抬高 0.03。
示例坐标应按实际箱体放置点填写。

新输入不接收 y_force、put_mode、force_phase_pause_s、motion.fine_pos_tol、
motion.step_timeout_s，以及原先未参与运动的 preposition.y_offset_m、retract.lift_m。

## 定位和撤手

prepare 先完成可选躯干动作，再读取变换及两手位姿，保存 torso_link4 下的 initial_poses。
目标转换沿用共享 TargetPointSolver：XY 使用 transformer_martix 的 R p + t，
Z 使用 map 目标 Z 减 abs(floating_base.z)，mm_scale=1.0。
base_link 末端反馈经 floating_base 做完整位置及四元数转换；torso_link4 反馈直接使用。
手臂阶段保持底盘和躯干固定。

双手持箱目标 = 转换后的箱心减 grasp_center_offset_m 的 X/Z 分量。
预定位使用目标 X+x_offset_m、目标 Z+z_offset_m，左右 Y 分别保持起始值，
姿态沿用固定放置四元数：左右 RPY 分别为 (-90,-30,0)、(-90,30,0) 度，yxz 序。
X 阶段按实测双手中点与最终 X 的差值同步平移两手，保持实测 Z；
Z 阶段按实测双手中点与最终 Z 的差值同步平移，保持实测 X。
两段 Y 始终使用保存的起始值，姿态保持当前值，不按目标 Y 或箱长重算 Y。

预定位、X、Z 完成后校验各手 Z 目标误差及两手高度差，均不得超过 motion.pos_tol；
X/Z 另校验完整目标位姿。失败时终止后续运动。

X/Z 均完成后读取当前双手位姿，一次保存三组撤手目标。每组使用一个
motion.move_end_effector_both 普通点位 Action，同步下发左右目标：

1. 两手分别降 Z：开始 Z-lower_m，X/Y 与姿态不变。
2. 左手 Y 增加 minimum_outward_m，右手 Y 减少 minimum_outward_m，保持降低后的 Z。
3. 两手 X 分别减少 backward_m，Z 到各自撤手开始 Z+safe_offset_z，保持外撤后的 Y。

minimum_outward_m 在本版本直接作为外撤距离，与 morefine 当前实际计算一致。
每段完成后读取双手位姿并分别确认，再保存完成进度；任一手不符合目标即失败。
同步表示同一个双手 Action 提交目标，并等待两手均到位。

轨迹 timeout 为每个轨迹点的等待上限，Action 上限为 timeout×点数预算+15 秒。
撤手 timeout 为每段等待上限，Action 上限为 timeout+15 秒。每次反馈读取上限 10 秒。

## 完成、恢复与停止

阶段为 prepare、preposition、align_x、lower_z、retract_both、verify。
完成需 X/Z 标志齐全、三段双手撤出成功且最终两手仍符合保存的撤出目标。
结果包含 placed=true、holding_object=null、verification_method=position_and_retraction、
relative_target_m、preposition_points、final_poses、retracted_sides 和 evidence_refs。
这是基于手臂位姿与流程完成的确认，不测量箱体最终位置，也不保证箱心 Y 到达输入目标 Y。

检查点版本为 1，Skill 标识及 Action key 前缀均为 place-object-position。
起始位姿、绝对运动目标、三组撤手目标及动作结果在执行时保存，恢复复用这些值。
等待截止时间也保存，恢复只等待剩余时间。输入变化或不兼容检查点会终止执行；
原 place-object-force 检查点不能用于本 Skill。
停止由 Pilot 取消当前 Action；on_stop 不下发运动。发生过物理动作时报告状态未知，
没有开始物理动作时报告 no_motion_started。

## 离线检查与打包

```bash
PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=../robot-skill:src:src/skill python -m pytest -p no:cacheprovider -q src/skill/place_object_position/tests
python src/skill/place_object_position/package_skill.py --output /tmp/place-object-position-0.1.0.zip
```

ZIP 包含脚本、manifest、依赖清单及共享数值求解器，不依赖原 place-object-force 包。
契约测试默认读取当前 Ability 清单，可用 PLACE_ABILITY_MANIFESTS 指定离线目录。

独立启动入口为 start-shell/run_place_position_skill.sh，读取本 Skill manifest 的名称及版本。
传入 src/config/place-position-deployment.yaml 和现场输入；PLACE_CHECK_ONLY=1 仅检查输入和绑定。
脚本不启动 Ability，绑定需与当前 Running 实例匹配。环境变量 PLACE_PYTHON、PLACE_PILOT、
PLACE_SKILL_SDK 可覆盖工具路径，PLACE_TIMEOUT 可覆盖总超时。
总预算覆盖三段双手轨迹、三段普通撤手、可选躯干、反馈读取及段间等待。
