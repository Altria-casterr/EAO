---
name: place-object-force
description: R1 Pro 双臂码垛：prepare 调整躯干后等高预定位，先 X 后 Z 到达最终放置目标并检查双手高度，撤辅助手后主手沿 Y 力反馈推靠，最后撤主手；撤手末段默认到开始高度上方 3 cm。
category: robot_skill
version: 0.4.15
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
  # 保留轨迹与非轨迹两种形态的动作声明。当前双手预定位及 X/Z 调整使用轨迹；
  # 当前撤手传 trajectory=False，使用单手单点动作，timeout 固定为 10 秒。
  - { type: torso.move_to_posture_all, schema_version: 2 }
  - { type: robot.get_state, schema_version: 2 }
  - { type: robot.get_sensors_state, schema_version: 2 }
  - { type: motion.move_end_effector_both_trajectory, schema_version: 2 }
  - { type: motion.move_end_effector_both, schema_version: 2 }
  - { type: motion.move_end_effector_single_trajectory, schema_version: 2 }
  - { type: motion.move_end_effector_single, schema_version: 2 }
  - { type: manipulation.force_align, schema_version: 2 }
stop_actions:
  - { type: torso.move_to_posture_all, schema_version: 2 }
  - { type: motion.move_end_effector_both_trajectory, schema_version: 2 }
  - { type: motion.move_end_effector_single_trajectory, schema_version: 2 }
  - { type: manipulation.force_align, schema_version: 2 }
---

# 双臂放置：双手等高，X/Z 目标到位，Y 力反馈

## 使用条件与 Ability

上游已经双手托持箱体，并完成底盘站位。沿用 Put_Down 的托举工具，无需夹爪开闭。
本 Skill 完成单个箱体的一次放置，多箱、多层的目标计算和调度由上游负责。

当前源码版本为 `place-object-force@0.4.15`。绑定现有 `robot`（RobotObservation.Demo）、
`torso`（TorsoControl.Demo）、`manipulator_motion`（JointEEFPoseControl.Demo）三个角色。
所有动作通过 schema-v2 Action 执行，沿用抓取 Skill 的 SkillContext、检查点和求解器架构。

躯干默认在 prepare 中调整：等待 `torso.move_to_posture_all` 到位，再读取变换和末端位姿。
预定位与 Z/X 均使用双手轨迹动作，Y 使用主手 manipulation.force_align 力控推靠，
撤手使用单手单点动作。手臂阶段保持底盘与躯干固定；预定位使用 solver 固定姿态，
后续调整与撤手沿用读取到的实际姿态。

## 输入与坐标

### 参数构造规则

本文件是模型构造本 Skill 输入的统一说明入口。先调用
`robot.get(skill_name="place-object-force")`，确认当前唯一安装启用的准确版本并读取
`skill_contract.documentation`；如存在多个启用版本，先核对执行版本再显式提供
`skill_version`，不得选 Registry latest。如工具仅返回头尾预览，必须按返回的
artifact 引用读取完整正文。非地图参数使用下方完整示例中明确列出的现场配置，
不再从 `/workspace/inputs/place_object_force.example.json`、历史调用或其他版本文档补值。
用户明确指定的参数优先，但必须符合当前输入契约；冲突时报告具体冲突，
不得自行改值。没有明确覆盖要求时，不自行调整力阈值、步长、偏移、容差或超时。

输入模型为 `scripts.models:PlaceInput`，必填 `target`；默认启用躯干调整，
因此当前现场配置还必须填写 `torso.positions`。虽然其他字段多有程序默认值，
仍应显式使用本文现场配置，不通过省略字段隐式选择不同参数。
位置、尺寸与偏移为米，力为牛顿，关节角和姿态容差为弧度，
关节速度为弧度/秒，时间和轮询间隔为秒。

### 完整输入示例

以下输入取自 2026-10-26 第 1 箱第 5 步
`place-object-force@0.4.14` 的成功执行，目标为 `dest-l2-1`。
它完整保留该次执行输入，包括程序补齐的字段及兼容字段；
字段被接受并不意味着一定参与当前运动，实际作用见后文。

**每次执行必须重新查询当前目的放置实体，替换地图字段；不能将本例的目标名、
坐标、尺寸、放置侧或躯干姿态用于任意目的位。**
非地图字段是本文收录的当前现场配置，不是任意格式占位值。
将构造后的完整对象作为 `robot.run.input` 提交。

```json
{
  "approach_motion": {
    "axis_order": "zyx",
    "loop_freq": 0.1,
    "max_num_points": 100,
    "num_points": null,
    "orient_tol": 0.1,
    "pos_tol": 0.03,
    "sequential": true,
    "speed": 0.05,
    "step_distance": 0.05,
    "timeout": 10
  },
  "force_phase_pause_s": 0.5,
  "motion": {
    "axis_order": "yzx",
    "fine_pos_tol": 0.0005,
    "loop_freq": 0.1,
    "max_num_points": 100,
    "num_points": 20,
    "orient_tol": 0.1,
    "pos_tol": 0.05,
    "sequential": true,
    "speed": 0.03,
    "step_distance": 0.05,
    "step_timeout_s": 2,
    "timeout": 10
  },
  "preposition": {
    "grasp_center_offset_m": [
      0.07,
      0,
      0
    ],
    "x_offset_m": -0.03,
    "y_offset_m": 0.1,
    "z_offset_m": 0.07
  },
  "put_mode": "right",
  "relative_frame": "torso_link4",
  "retract": {
    "backward_m": 0.1,
    "lift_m": 0.15,
    "lower_m": 0,
    "minimum_outward_m": 0.1,
    "safe_offset_z": 0.15
  },
  "retract_motion": {
    "axis_order": "yzx",
    "loop_freq": 0.1,
    "max_num_points": 100,
    "num_points": null,
    "orient_tol": 0.1,
    "pos_tol": 0.03,
    "sequential": true,
    "speed": 0.03,
    "step_distance": 0.05,
    "timeout": 30
  },
  "target": {
    "object_ref": "dest-l2-1",
    "position_m": [
      1.11,
      -0.842,
      0.55
    ],
    "size": {
      "height_m": 0.23,
      "long_m": 0.53,
      "width_m": 0.41
    },
    "world_frame": "map"
  },
  "torso": {
    "enabled": true,
    "positions": [
      1.7,
      -2.7,
      -1.05,
      0
    ],
    "speeds": [
      0.5,
      0.8,
      0.7,
      0.7
    ],
    "timeout": 10,
    "tol": 0.02
  },
  "y_force": {
    "force_coordinate": [
      "x"
    ],
    "force_threshold_n": 1.5,
    "loop_freq_s": 0.1,
    "max_distance_m": 0.01,
    "step_size_m": 0.01,
    "timeout_s": 60
  }
}
```

### 地图参数及字段映射

先用 `map.query` 在项目 `real_map` 槽位按当前任务的目的放置实体名查询。
只选择当前 generation 中 `status=active` 的实体，忽略同名 removed 记录；
多个 active 候选须先消歧。以下字段必须来自**同一个当前目的放置实体**，
不能混用源箱、上一目的位或携物导航修正后的坐标。

| Skill 输入字段 | real_map 目的实体字段 | 映射要求 | 上例值 |
|---|---|---|---|
| `target.object_ref` | `name` | 使用目的实体业务名，不填实体 id 或源箱名称 | `dest-l2-1` |
| `target.world_frame` | `frame_id` | 当前契约只接受 `map`，不得仅修改标签冒充坐标转换 | `map` |
| `target.position_m[0]` | `pose.position.x` | 原始最终放置箱体中心 X | `1.11` |
| `target.position_m[1]` | `pose.position.y` | 原始中心 Y，不加携物导航的 ±0.10 m 修正 | `-0.842` |
| `target.position_m[2]` | `pose.position.z` | 最终箱体中心 Z，不是支撑面高度；不另加减半箱高 | `0.55` |
| `target.size.long_m` | `bounds.size.y` | Y 向跨度，正数，米 | `0.53` |
| `target.size.width_m` | `bounds.size.x` | X 向跨度，正数，米，不与 long_m 对调 | `0.41` |
| `target.size.height_m` | `bounds.size.z` | 箱高，正数，米，填写真实尺寸 | `0.23` |
| `put_mode` | `properties.place_mode` | 仅接受 `left/right`，决定最后推靠的主手 | `right` |
| `torso.positions` | `properties.torso_joint_positions` | 按原顺序复制四个关节角，弧度；不按层号或箱高猜测 | `[1.7,-2.7,-1.05,0]` |
| `torso.speeds` | `properties.torso_speeds` | 按原顺序复制四个正关节速度，弧度/秒 | `[0.5,0.8,0.7,0.7]` |

`put_mode` 与源箱的 `grasp_mode/carry_mode/verify_side` 没有推导关系。
`right` 表示右手为主手：先撤左手、右手沿 +Y 推靠、再撤右手；
`left` 时先撤右手、左手沿 -Y 推靠、再撤左手。
不要把放置模式写成导航的 `mode` 或抓取的 `grasp_axis`。

`torso.enabled=true` 是本文现场配置，启用后必须提供当前目的实体的躯干角和速度。
本 Skill 不会在放置结束后自动将躯干回零；下一箱的预备属于下一箱 Task。
`relative_frame=torso_link4` 是执行侧求解与末端反馈坐标系，与
`target.world_frame=map` 含义不同，二者均受当前输入模型约束。

目的实体不存在、必需字段缺失、模式不支持或类型不合法时报告具体问题，
不能用示例补齐。地图保持只读。`yaw/orientation` 不映射为额外输入字段；
本 Skill 不接受 `target.yaw_rad`，不提供任意旋转放置求解。
旧字段 `z_force`、`x_force`、`adjustment` 已移除，不得提交。

### 现场配置与程序默认值

| 参数 | 本文现场配置 | 程序默认值或实际约束 |
|---|---|---|
| `relative_frame` | `torso_link4` | 默认同值，当前只接受该值 |
| `put_mode` | 来自目的实体，本例 `right` | 程序默认 `left`，不能因此省略地图映射 |
| `torso` | 启用，地图角度与速度，`timeout=10`、`tol=0.02` | enabled 默认 true、positions 默认 null；启用时必须提供角度 |
| `force_phase_pause_s` | `0.5` | 默认同值；X 到位、Z 到位和 Y 接触后分别等待 |
| `preposition.x_offset_m/z_offset_m` | `-0.03/+0.07` | 默认同值，只偏移预定位 X/Z，不改变最终放置目标 |
| `preposition.grasp_center_offset_m` | `[0.07,0,0]` | 默认 `[0,0,0]`；从转换后的箱心减去 X/Z 工具偏移，Y 不参与；现场 Y 必须保持 0 |
| `preposition.y_offset_m` | `0.1` | 默认同值；兼容字段，当前不参与预定位或放置求点 |
| `y_force` | X 轴、`1.5 N`、步长 `0.01`、间隔 `0.1`、超时 `60`、max_distance_m `0.01` | 超时默认 `30`；其余默认同值；max_distance_m 不限制真实行程 |
| `retract.lower_m` | `0` | 默认 `0.02`；本现场第一段不降低 Z，但仍生成该段目标 |
| `retract.minimum_outward_m` | `0.1` | 默认 `0.03`；撤手 Y 向外移的最小距离 |
| `retract.backward_m` | `0.1` | 默认同值；末段沿负 X 回撤 |
| `retract.safe_offset_z` | `0.15` | 默认 `0.03`；末段目标 Z = 该手撤手开始时 Z + 此值 |
| `retract.lift_m` | `0.15` | 默认同值；保留的兼容字段，当前不参与撤手求点，不能替代 safe_offset_z |
| `approach_motion` | `pos_tol=0.03`、`orient_tol=0.1`、单点超时 `10`、自动点数、顺序 `zyx` | orient_tol 默认 `0.01`；其余项与完整示例一致 |
| `motion` | `pos_tol=0.05`、`orient_tol=0.1`、单点超时 `10`、`num_points=20`、speed `0.03`、顺序 `yzx` | 对应默认值为 `0.005`、`0.01`、`10`、null、`0.05`、`yzx`；其余项与示例一致 |
| `retract_motion` | `pos_tol=0.03`、`orient_tol=0.1`、输入 timeout `30`，其余见完整示例 | 对应默认值为 `0.005`、`0.01`、`10`；当前撤手实际调用覆盖 timeout 为 `10`，见下文 |

`preposition.y_offset_m`、`retract.lift_m`、`motion.axis_order` 和
`motion.max_num_points` 在该次成功调用中由程序补齐；本例已显式列出，
避免模型再从外部示例或历史调用补值。
`motion.pos_tol=0.05` 还用于预定位及 X/Z 阶段的实际高度检查：
每手目标 Z 误差和左右手高度差均须不超过 0.05 m，不是程序默认的 0.005 m。

### 三组运动参数的实际作用

- `approach_motion`：用于双手预定位轨迹，Action 为
  `motion.move_end_effector_both_trajectory`。本例 `num_points=null`，
  Ability 按 `step_distance=0.05` 自动计算点数，`max_num_points=100`。
  `sequential=true`、`axis_order=zyx` 表示逐轴 Z→Y→X 插值；
  Y 目标仍是各手原始 Y，并非额外侧向偏移。
- `motion`：用于先 X、后 Z 两段双手轨迹，Action 同上。
  本例显式使用 20 点；`step_distance/max_num_points` 不再决定点数。
  `fine_pos_tol=0.0005` 仅保留输入校验约束，当前 X/Z 到位检查使用
  `pos_tol=0.05`，不能宣称具有 0.5 mm 实际到位精度。
  `step_timeout_s=2` 参与本地启动脚本的预算，不控制当前 X/Z 轨迹单点超时。
- `retract_motion`：当前每只手按 Z、Y、X 三个目标依次下发
  `motion.move_end_effector_single`，代码明确 `trajectory=false`。
  实际使用其中的位置容差、姿态容差和 loop_freq；
  输入 `timeout=30` 被 `RETRACT_POINT_TIMEOUT=10` 覆盖。
  `num_points/max_num_points/speed/sequential/axis_order/step_distance`
  在当前撤手动作中不下发、不决定路径或点数；`RETRACT_POINT_CAP=100`
  仅对轨迹分支有效，当前单点分支不使用。不能把字段名或旧文档当作撤手轨迹证据。

双手轨迹的 timeout 为单点超时，Action 等待预算为
`timeout × (num_points 或 max_num_points) + 15` 秒。
本例预定位为 1015 秒，X/Z 两段各 215 秒；
撤手每个单点为 `10+15=25` 秒，共六个单点；
Y 力控为 `60+15=75` 秒，躯干为 `10+10=20` 秒。
这些是等待上限，不是预计动作时长，Pilot 总超时仍可能更早终止。
`loop_freq` 为轮询间隔（秒）；轨迹 speed 当前只用于日志，不是实际限速。
双手轨迹 `sequential=false` 时使用同步插值，axis_order 不再控制轴顺序。

### 目标点与撤手点如何计算

躯干到位后读取变换、双手位姿，保存 `initial_poses`。
先由 TargetPointSolver 将地图箱体中心转换到执行坐标系：
XY 使用 `transformer_martix` 的 R p + t，
Z 使用地图目标 Z 减 abs(floating_base.z)，显式使用米制 `mm_scale=1.0`。
再减去工具偏移的 X/Z 分量得到 `relative_target_m`：

```text
relative_target_m.x = 转换后的箱心 X - grasp_center_offset_m[0]
relative_target_m.y = 转换后的箱心 Y
relative_target_m.z = 转换后的箱心 Z - grasp_center_offset_m[2]

预定位 X = relative_target_m.x + preposition.x_offset_m
预定位 Z = relative_target_m.z + preposition.z_offset_m
预定位及后续 X/Z 目标的 left.y = initial_poses.left.y
预定位及后续 X/Z 目标的 right.y = initial_poses.right.y

X 阶段：两手各自 X += relative_target_m.x - 当前双手中点 X
Z 阶段：两手各自 Z += relative_target_m.z - 当前双手中点 Z
```

预定位姿态由 solver 固定给出：左臂 RPY(-90,-30,0)、右臂
RPY(-90,30,0)，yxz 欧拉序；不是沿用搬运结束时的四元数。
后续 X/Z 目标及撤手目标沿用读取到的实际姿态。
X 阶段保持当前 Z，Z 阶段保持当前 X；两段均使用保存的初始 Y，
保留双手在 X/Z 上的相对位置。最终目标不保留预定位的 -0.03/+0.07 偏移，
也不按箱体尺寸额外加减目标高度。

本例工具 X 偏移为 0.07 m，不能再使用旧文档的 0.10 m；
偏移在执行坐标系中由 solver 处理，模型不要预先从地图输入坐标中扣除。
目标 Y 用于撤手箱缘几何，不用于重排放下前的双手 Y；
要求 `left.y > right.y`。

撤手对每只手从该手撤手开始时的实际位姿规划三个目标：
降 Z → 向外撤 Y → 回撤 X 并改变 Z。每侧共三个单点动作。
本例 lower_m=0，第一段保持原高度；第二段左手沿 +Y、右手沿 -Y 外撤，
距离为 `max(|abs(箱缘Y)-abs(当前Y)|/2, minimum_outward_m)`，
本现场最小外撤距离为 0.1 m。第三段 X 减去 backward_m=0.1，
目标 Z 为该手撤手开始高度 + safe_offset_z=0.15，Y 保持外撤后的位置；
这一段由单点 Action 到达完整目标，不保证按 retract_motion.axis_order 逐轴执行。

末端反馈支持 base_link 和 torso_link4，前者经 floating_base 转换完整位置和姿态。
手臂阶段保持底盘、躯干及该坐标关系固定。放置完成判据是
X/Z 到位、Y 力接触及撤手流程完成，不提供箱体位置的视觉复核。

## Y 力反馈参数

仅 `push_y` 建立力基线，并调用 `manipulation.force_align` 完成主手沿 Y 的力控推靠。
准备阶段校验完整双侧 XYZ 力数据，Z/X 阶段不读取力或以力量值决定完成。
Y 在辅助手撤出后重新取基线，步进与触阈由 Ability 执行，动作成功后读取末次力数据。

| 字段 | 规则 |
| --- | --- |
| `force_threshold_n` | 默认 1.5 N，正数；传给 Ability 的力触发阈值 |
| `force_coordinate` | 默认 `["x"]`，原始传感器监测轴；可与移动轴 Y 不同，轴不重复 |
| `max_distance_m` | 默认 0.01 米，正数；当前仅写入结果记录，未下发给 Ability 限制行程 |
| `timeout_s` | 默认 30.0 秒，正数；传给 Ability 的动作超时 |
| `step_size_m` | 默认 0.01 米，正数；传给 Ability 的步进距离 |
| `loop_freq_s` | 默认 0.1 秒，正数；传给 Ability 的力控循环间隔 |

`motion.fine_pos_tol` 不得超过 Y 力控步长的一半，该约束保留在输入校验中。
Skill 以 `force_align` 动作成功作为接触完成依据，并记录动作前后的力数据。
完成结果保留 `holding_object=null`。

结果中的 `commanded_travel_m` 当前直接填写输入 `max_distance_m`，
`completed_steps` 固定记录为 1（一次 force_align 调用），不是 Ability 内部步数或
实测行程。`triggered_sides` 按主手写入，Skill 不再独立用读回的 delta_n
重复判阈；完成依据是 force_align 的成功契约。不能把这些记录解释为真实行程
已被限制为 0.01 m 或只移动了一小步。

## 流程与完成判据

1. `prepare`：默认调整躯干到位，再读取变换、双手起始位姿和完整力数据，保存预定位计划。
2. `preposition`：保持左右手各自起始 Y，移动到 X/Z 偏移位置，读取并校验实际高度；
   沿用 approach_motion 轴顺序，Y 目标无预定位偏移。
3. `align_x`：先读实测双手位姿（ee_pose 经 floating_base 由 base_link 转到 torso_link4），
   由当前双手中点 X 与最终目标的差值得到 X 偏移，Y 使用起始值，保持当前 Z 与姿态，复核到位和高度，默认前移 3 cm。
4. `lower_z`：要求 X 已完成；重新读实测双手位姿，由当前双手中点 Z 与最终目标的差值
   得到 Z 偏移，Y 使用起始值，保持当前 X 与姿态，复核到位和高度，默认下降 7 cm。
5. `retract_assist`：要求 X/Z 均完成，以当前实测位姿规划辅助手撤出，并确认到位。
6. `push_y`：重新取力基线，仅主手沿 Y 小步推靠，以主手触阈结束。
7. `retract_push`：以 Y 接触后的实测位姿规划主手撤出，并确认到位。
8. `verify`：确认 Z/X 已到位、Y 已接触及双手已撤出，复核最终末端位姿并读取最终力。

Z/X 到位和 Y 触阈后等待 force_phase_pause_s，再进入下一阶段。
等待截止时间保存在检查点，恢复只等待剩余时间，每隔最多 0.1 秒检查停止请求。
预定位、X、Z 完成后各读取实际双手位姿，要求每只手的目标 Z 误差以及
abs(left.z - right.z) 都不超过 motion.pos_tol，程序默认 5 mm，本现场显式为 50 mm。两手分别在到位容差内
但高度差超限时仍失败。高度超限报告 PLACE_HEIGHT_UNCONFIRMED，记录目标高度、
左右实测高度、高度差和容限；不自动补正，不继续下降、前移或撤手。
撤手会主动改变单手高度，因此撤手与 Y 阶段不要求双手等高。
未到位、超时、非法反馈或 Action 失败也终止执行，后续阶段不会继续。

结果保留 `placed=true`、`verification_method=force_contact_and_retraction`、
两手最终位姿、最终力和证据引用。`contacts` 仅含 push_y，记录 Y 的基线、最终力、
主手、声明行程和实测接触位姿；声明行程不等于实测移动距离，详见上节。
Z/X 的目标与到位状态保存在检查点。

## 恢复与停止

检查点使用 `placement_revision=6`。旧版（包含 revision=5）的检查点报告
`PLACEMENT_CHECKPOINT_INCOMPATIBLE`，确认现场物理状态后新建执行；不复用旧目标或躯干变换。
新执行在首个动作前写入版本及输入指纹，恢复时输入改变会终止执行。

起始双手位姿只保存一次，恢复复用保存的 Y，不以恢复时的新读数替换。
躯干、Z/X、预定位和撤手动作保存目标与稳定 Action key，恢复复用同一动作。
Z/X 恢复使用保存目标，避免再次叠加位移。Y 使用稳定 Action key 调用 force_align，
保存完成后的接触记录；恢复时复用已有动作结果与接触记录，不由模型重放力控。

停止交由 Pilot 取消当前 Action，on_stop 不发送额外运动。只要开始过躯干或手臂运动，
返回 `safe=false`、`physical_state=unknown`、`requires_intervention=true`；
没有开始物理动作时返回 `safe=true`、`physical_state=no_motion_started`。

## 离线检查、打包与启动

```bash
PYTHONPATH=../robot-skill:src:src/skill PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -p no:cacheprovider -q src/skill/place_object_force/tests
python src/skill/place_object_force/package_skill.py --output /tmp/place-object-force-0.4.15.zip
```

ZIP 包含共享 solver、控制器和输入模型。运行需要 requirements.lock 所列依赖及
semantic_robot_skill_sdk。测试使用模拟 Ability 响应，隔离 Worker 测试验证独立 ZIP。
契约测试默认读取当前工作区 Ability 清单，本地可用 PLACE_ABILITY_MANIFESTS 指定缓存目录。

模型调用 robot.run 时直接提交按本文构造的 input，无需读取外部示例 JSON。
本地脚本入口仍要求部署文件与输入文件；输入文件仅保存本次完整参数，是调用载体，
不是额外参数来源。运行器检查三个角色的当前 Running 绑定，不启动 Ability。
沿用抓取的 required_actions 约定，关闭躯干动作时仍须保留 torso 角色绑定。

```bash
PLACE_CHECK_ONLY=1 bash start-shell/run_place_force_skill.sh src/config/place-force-deployment.yaml /path/calibrated-input.json
bash start-shell/run_place_force_skill.sh src/config/place-force-deployment.yaml /path/calibrated-input.json
```

PLACE_CHECK_ONLY=1 仅检查输入及绑定，不执行 Skill。部署 UUID 重新激活后须更新。
PLACE_PYTHON、PLACE_PILOT、PLACE_SKILL_SDK 可覆盖路径。PLACE_TIMEOUT 可覆盖总超时。
当前启动脚本仍按旧的八段撤手轨迹模型估算总预算，与实际六个单点撤手及 X/Z
轨迹预算不一致；不能把脚本估算值当作当前动作数量或时长证据。实际各 Action
预算见本文参数说明，发布文档不会自动修改启动脚本。
