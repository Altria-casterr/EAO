---
name: grasp-object-bilateral
description: R1 Pro 真机双手搬箱（approach 轨迹版）。输入世界系箱体中心与尺寸，转换并匹配目标后双手按可配置的轴顺序笛卡尔轨迹接近（现场 yzx，代码默认 zyx）、逐侧力反馈对齐、按 lift_m 抬升（代码默认 0.1 米，现场输入 0.15 米）和携箱收回、躯干四关节回零站起；拍照记录并用一次单侧 force_align 触阈判定持物。
category: robot_skill
version: 0.2.7
runtime:
  api_version: 1
  python: ">=3.11"
  entrypoint: scripts.skill:run
  stop_entrypoint: scripts.skill:on_stop
  input_model: scripts.models:GraspInput
  state_model: scripts.models:GraspState
  result_model: scripts.models:GraspResult
  controllers: {}
required_actions:
  - { type: torso.move_to_posture_all, schema_version: 2 }
  - { type: robot.get_state, schema_version: 2 }
  - { type: robot.get_sensors_state, schema_version: 2 }
  - { type: perception.match_target, schema_version: 2 }
  - { type: manipulation.force_align, schema_version: 2 }
  - { type: motion.move_end_effector_both_trajectory, schema_version: 2 }
  - { type: robot.capture_rgbd, schema_version: 2 }
# 声明可由 Pilot 取消的物理 Action；on_stop 不把这些动作重新下发为停止命令。
stop_actions:
  - { type: torso.move_to_posture_all, schema_version: 2 }
  - { type: manipulation.force_align, schema_version: 2 }
  - { type: motion.move_end_effector_both_trajectory, schema_version: 2 }
---

# 真机双手搬箱

## 模型调用条件

需要双手接近指定箱子、按 `lift_m` 抬升（代码默认 0.1 米，现场输入 0.15 米）并向身体
收回时使用本 Skill。
上游只需完成底盘站位；双手预备点的姿态由本 Skill 调用的 CapturePointSolver **固定给出**
（左臂 RPY(-90°, -30°, 0)、右臂 RPY(-90°, 30°, 0)，欧拉序 yxz），当前末端四元数会被
覆盖，工具无需额外夹爪开闭动作。
只支持已按现场坐标约定标定的正向箱体；本 Skill 不做单侧外拉、箱体旋转求解、
导航或抓取策略切换。不让模型生成轨迹点或猜测力阈值。

## 输入

### 参数构造规则

本文件是模型构造本 Skill 输入的统一说明入口。先调用
`robot.get(skill_name="grasp-object-bilateral")`，确认当前唯一安装启用的准确版本并读取
`skill_contract.documentation`；如存在多个启用版本，先核对执行版本再显式提供
`skill_version`，不得选 Registry latest。如工具仅返回头尾预览，必须按返回的
artifact 引用读取完整正文。非地图参数使用下方完整示例中明确列出的现场配置，
不再从 `/workspace/inputs/grasp_object_bilateral.example.json`、历史调用或其他版本文档补值。
用户明确指定的参数优先，但必须符合当前输入契约；冲突时报告具体冲突，
不得自行改值。没有明确覆盖要求时，不自行调整本文的力阈值、步长、偏移、
运动顺序、回零速度或超时。

输入模型为 `scripts.models:GraspInput`，必填 `target`、`approach_force`、
`verify_force`。下例显式列出全部有效输入字段；`center_x_offset_m` /
`center_z_offset_m` 为当前 solver 未使用的兼容字段，不列入调用示例。
距离为米，关节角和姿态容差为弧度，关节速度为弧度/秒，力为牛顿，
超时及轮询间隔为秒。`speed` 当前仅用于日志，不代表实际限速。

### 完整输入示例

地图字段取自 2026-10-26 第 2 箱第 3 步成功执行所用的 `gray_box-l5-2` 快照。
**每次执行都必须按后文映射表重新查询当前源箱，不得将本例的实体名、坐标、
尺寸或躯干属性当作任意箱子的固定配置。**
非地图字段沿用该次 `grasp-object-bilateral@0.2.6` 成功执行的配置，并将当时由
程序补齐的有效字段显式写出；不是任意格式示例。地图字段替换并满足契约后，
将整体作为 `robot.run.input` 提交，不提交字符串占位符。

```json
{
  "target": {
    "object_ref": "gray_box-l5-2",
    "world_frame": "map",
    "position_m": [1.01, 1.529, 1.24],
    "size": {"long_m": 0.53, "width_m": 0.41, "height_m": 0.23}
  },
  "relative_frame": "torso_link4",
  "safe_offset_m": 0.1,
  "capture_x_offset_m": -0.1,
  "capture_z_offset_m": 0.0,
  "lift_m": 0.15,
  "torso": {
    "enabled": true,
    "positions": [0, 0, 0, 0],
    "speeds": [0.5, 0.8, 0.7, 0.7],
    "timeout": 10.0,
    "tol": 0.02
  },
  "stand": {
    "speeds": [0.5, 0.5, 0.5, 0.5],
    "timeout": 10.0,
    "tol": 0.02
  },
  "perception": {
    "gray_box_only": true,
    "sensor_id": "head_camera",
    "conf_threshold": 0.5,
    "iou_threshold": 0.5,
    "minimum_confidence": 0.25,
    "top_k": 2,
    "delta_threshold_x": 0.2,
    "delta_threshold_y": 0.25,
    "delta_threshold_z": 0.2,
    "timeout_seconds": 120.0
  },
  "approach_force": {
    "force_threshold_n": 0.5,
    "force_coordinate": ["x"],
    "step_size_m": 0.01,
    "loop_freq_s": 0.1,
    "timeout_s": 12.0
  },
  "verify_force": {
    "force_threshold_n": 1.0,
    "force_coordinate": ["x"],
    "step_size_m": 0.01,
    "loop_freq_s": 0.1,
    "timeout_s": 12.0
  },
  "verify_side": "left",
  "approach_motion": {
    "pos_tol": 0.05,
    "orient_tol": 0.05,
    "timeout": 5.0,
    "loop_freq": 0.1,
    "speed": 0.001,
    "num_points": null,
    "max_num_points": 100,
    "step_distance": 0.02,
    "sequential": true,
    "axis_order": "yzx"
  },
  "motion": {
    "pos_tol": 0.05,
    "orient_tol": 0.05,
    "timeout": 10.0,
    "loop_freq": 0.1,
    "speed": 0.05,
    "num_points": null,
    "max_num_points": 100,
    "step_distance": 0.01,
    "sequential": true,
    "axis_order": "zyx"
  }
}
```

### 地图参数及字段映射

先用 `map.query` 在项目的 `real_map` 槽位按当前任务的源箱实体名查询。
只选当前 generation 中 `status=active` 的源箱，忽略同名 removed 记录；
若仍有多个 active 候选，必须先消歧。以下字段必须来自**同一个当前源箱实体**，
不得使用目的放置实体、上一箱数据或导航修正后的坐标替代。

| Skill 输入或选择条件 | real_map 源箱字段 | 映射要求 | 上例值 |
|---|---|---|---|
| `target.object_ref` | `name` | 使用业务实体名，不填实体 `id` | `gray_box-l5-2` |
| `target.world_frame` | `frame_id` | 当前输入模型只接受 `map`；不能只改标签冒充坐标转换 | `map` |
| `target.position_m[0]` | `pose.position.x` | 地图原始箱体中心 X，米 | `1.01` |
| `target.position_m[1]` | `pose.position.y` | 地图原始中心 Y，米，不加导航侧向修正 | `1.529` |
| `target.position_m[2]` | `pose.position.z` | 地图原始中心 Z，米，不按层号重算 | `1.24` |
| `target.size.long_m` | `bounds.size.y` | 箱体 Y 向跨度，正数，米 | `0.53` |
| `target.size.width_m` | `bounds.size.x` | 箱体 X 向跨度，正数，米，不与 long_m 对调 | `0.41` |
| `target.size.height_m` | `bounds.size.z` | 箱体高度，正数，米；仍须填写真实尺寸 | `0.23` |
| 选择双手 Skill 的条件 | `properties.grasp_mode` | 应为 `both`；这是选技能的条件，不是可传入的 `mode/grasp_axis` 字段 | `both` |
| `torso.positions` | `properties.torso_joint_positions` | 按原顺序复制四个关节角，弧度；不能按箱高或层号猜测 | `[0,0,0,0]` |
| `torso.speeds` | `properties.torso_speeds` | 按原顺序复制四个正关节速度，弧度/秒 | `[0.5,0.8,0.7,0.7]` |

`verify_side` 另有可选地图映射：源箱提供 `properties.verify_side` 时，
取其 `left/right` 值；缺失时显式使用本文现场配置 `left`。
提供非法值时报告错误，不静默回退；不根据 `grasp_mode`、`carry_mode`、
`place_mode` 推导验证侧。本例源箱为 `verify_side=left`。

`torso.enabled=true` 是本文现场配置，不是地图字段；启用后按上表提供源箱的
躯干角与速度。`stand.speeds=[0.5,0.5,0.5,0.5]` 是收箱后回零的独立现场配置，
不得用源箱的 `torso.speeds=[0.5,0.8,0.7,0.7]` 覆盖。
`stand` 必定执行，与 `torso.enabled` 无关；回零目标由代码固定为四个零，
不开放 `stand.positions/enabled`。

实体不存在、必需属性缺失或类型不合法时报告具体缺项，不用示例补齐。
`grasp_mode=left/right` 时应选择适用的单手技能，不能伪造 `both`。
`yaw/orientation` 不映射为额外输入：本 Skill 不接受 `target.yaw_rad`，
也不求任意旋转箱体的抓取点，仅适用于部署已标定的正向箱体。
`relative_frame=torso_link4` 是求解和末端反馈坐标系，与地图的
`target.world_frame=map` 含义不同；必须与感知、变换和末端坐标约定一致，
不得互相替代。地图本身保持只读。

### 非地图参数与程序默认值

完整示例是当前现场配置。下表列出它与输入模型默认值的区别，
**不能因为字段有默认值就省略现场配置或改用默认值**。

| 参数 | 本文现场配置 | 程序默认值或约束 |
|---|---|---|
| `relative_frame` | `torso_link4` | 默认同值，不是自动 TF |
| `safe_offset_m` | `0.1` | 默认 `0.07`，双手初始预备点距箱侧的 Y 向安全间距 |
| `capture_x_offset_m` | `-0.1` | 默认 `0`，预备点 X = 匹配中心 X + 该值 |
| `capture_z_offset_m` | `0` | 默认 `0.05`，预备点 Z = 用于求点的中心 Z + 该值 |
| `lift_m` | `0.15` | 默认 `0.1`，双手接触后各自从实际末端 Z 抬升的指令位移，不按箱高推算 |
| `torso.enabled` | `true` | 默认 `false`；`timeout=10`、`tol=0.02` 与程序默认一致 |
| `stand` | 速度 `[0.5,0.5,0.5,0.5]`、容差 `0.02`、超时 `10` | 默认速度 `[0.5,0.8,0.7,0.7]`，其余同值 |
| `perception` | 完整示例的十个字段 | `gray_box_only` 默认 `false`，现场必须显式设 `true`；其余九项与默认一致 |
| `approach_force` | x 轴、`0.5 N`、步长 `0.01`、间隔 `0.1`、超时 `12` | 阈值和监测轴必填；默认步长 `0.01`、间隔 `0.1`、超时 `3` |
| `verify_force` | x 轴、`1 N`、步长 `0.01`、间隔 `0.1`、超时 `12` | 阈值和监测轴必填；默认步长 `0.01`、间隔 `0.1`、超时 `3` |
| `verify_side`（地图未提供时） | `left` | 默认 `left`；仅一次单侧验证，不双侧复核 |
| `approach_motion` | 顺序 `yzx`、单点超时 `5`、步距 `0.02`、speed `0.001`、位置/姿态容差均 `0.05` | 默认依次为 `zyx`、`60`、`0.05`、`0.05`、位置 `0.03` / 姿态 `0.01`；其余项与示例一致 |
| `motion` | 顺序 `zyx`、单点超时 `10`、步距 `0.01`、位置/姿态容差均 `0.05` | 默认依次为 `yzx`、`30`、`0.05`、位置 `0.03` / 姿态 `0.01`；其余项与示例一致 |
| `center_x_offset_m/center_z_offset_m` | 不传 | 默认均为 `0.2`；兼容字段，当前 solver 不使用，改变它们不会改变收箱点 |

本次成功执行记录中两个 `center_*_offset_m` 由程序自动补齐为 `0.2`；
上例省略它们仍得到相同的有效输入行为，不应把它们当作可调收箱距离。
`height_m` 当前未参与抓取/收箱点公式，但仍是必填正尺寸，必须如实映射地图，
不能因暂不参与求点而随意填写。

### 感知、力控和轨迹参数的实际作用

`perception.conf_threshold/iou_threshold` 是此双手 Skill 下发给
`perception.match_target` 的检测阈值；`minimum_confidence` 是 Skill 对返回匹配结果
额外检查的门槛，不下发给 Ability。`delta_threshold_x/y/z` 是目标匹配容差，
`top_k` 控制候选数；`sensor_id` 也用于最终采图。
`timeout_seconds` 控制匹配 Action 的等待预算，不作为感知参数透传。
不要把双手 Skill 的检测阈值字段套用到不支持它们的其他 Skill。

`approach_force` 用于左、右两次接触，`verify_force` 用于最后的单侧验证。
两者都是主动沿 Y 步进的 `force_align`：左手沿负 Y，右手沿正 Y，运动方向由
Skill 固定；`force_coordinate=["x"]` 指监测的传感器轴，不是运动轴。
本现场验证步长为 `0.01 m`，不能沿用旧文档的 `0.001 m`。
力控 Action 等待预算为对应 `timeout_s + 10` 秒，本例均为 22 秒。

`approach_motion` 仅用于双手接近预备点的轨迹，本现场采用 Y→Z→X；
`motion` 仅用于 lift 的抬升、收箱两段轨迹，本现场采用 Z→Y→X。
`sequential=true` 时按 `axis_order` 逐轴插值；设为 `false` 时六维同步插值，
Ability 忽略轴顺序。`axis_order` 必须是 x/y/z 的一个排列，入口归一化为小写。

两组运动参数中 `num_points=null` 均表示由 Ability 按 `step_distance` 自动计算
左右点数并取最大值同步执行，点数下限为 5，上限由 `max_num_points=100` 控制。
显式指定 `num_points`（5～1000）时按指定点数执行，`step_distance` 和
`max_num_points` 不再决定点数。`loop_freq=0.1` 是逐点轮询间隔（秒），不是 Hz；
`speed` 当前仅用于日志，不能用示例中的 `0.001` 宣称已限制真实接近速度。

轨迹 `timeout` 为**单点**超时；指定点数时 Action 总预算为
`timeout × num_points + 15` 秒，自动点数时为
`timeout × max_num_points + 15` 秒。本例接近预算为 `5×100+15=515` 秒，
抬升/收箱各为 `10×100+15=1015` 秒；它们是等待上限，不是预计运动时长，
Pilot 的执行总超时仍可能提前终止执行。
prepare 和 stand 躯干 Action 各使用对应 `timeout + 10` 秒，本例均为 20 秒。

偏移只作用于求得的预备点，不修改地图或匹配中心。收箱点由抬升后的实际末端
反馈再求解，不能假设修改某个预备点偏移会精确等量改变最终收箱位置。
具体公式见下文；模型不直接构造末端轨迹点，也不能传入单手技能的
`grasp_axis` 或 `pull_y_distance_m`。

## 五阶段

1. **prepare**：可选 torso.move_to_posture_all；按需调用 robot.get_state 读取
   floating_base 与 transformer_martix（恢复执行且检查点已有对应结果时跳过）；
   调用 TargetPointSolver.solve 得到相对点；perception.match_target 匹配箱体中心
   （匹配点的 Z 会被 solver 相对点的 Z 覆盖，感知返回的 Z 不参与计算）；读取左右
   末端位姿，再调用 CapturePointSolver.get_capture_point(axis="both") 保存双手
   预备点（两侧 Y 取最靠近箱缘的安全点、非随机，姿态为固定 RPY）。
2. **approach**：motion.move_end_effector_both_trajectory 双手按 `approach_motion`
   指定的插值模式与轴顺序到预备点（现场 yzx，代码默认 zyx 顺序插值）；随后左手 force_align
   沿 Y 负方向、右手沿 Y 正方向依次接近。轨迹的起点由 Ability 在执行时读取实际
   末端位姿，与目标同坐标系，因此不会出现整条路径偏移。
3. **lift**：重新读取双手实际位姿，各自 Z 加 `lift_m`（代码默认 0.1 米，现场输入
   0.15 米），通过双手 trajectory 抬升；再读实时双手位姿，调用
   CapturePointSolver.move_box_to_center，双手 trajectory 携箱收回。两段 lift 轨迹
   使用 `motion` 指定的插值模式，默认 yzx 逐轴顺序插值（现场输入为 zyx），
   与 approach 解耦。`lift_m` 是末端指令位移，不是物体独立验证高度，也不随箱高变化。
4. **stand**：只调用一次 torso.move_to_posture_all，将躯干四关节目标设为零；
   使用独立的 `stand` 速度、容差与超时配置。成功后进入 verify，失败即终止。
5. **verify**：robot.capture_rgbd(axis="rgb") 采一帧日志；对 verify_side 调用
   一次 force_align。Action 成功即按用户指定判据设置 holding_object=true。

成功结果明确标记 `verification_method=single_force_align`，记录验证侧、力阈值、
匹配中心、预备/抬升/收箱目标、`lift_distance_m`（等于本次实际使用的 `lift_m`）
和证据引用。没有添加稳定时长、双侧承载或视觉抬升验收条件，也不声称已达到
仿真版独立验证的 HeldObjectState 契约。

## 上游接口的实际语义

- TargetPointSolver 默认输出毫米；本 Skill 显式传 `mm_scale=1.0` 全程使用米。
  保留上游 `R p + t` 和 `z = 输入世界Z - abs(floating_base.z)` 的公式。
  RobotObservation 当前读取 map→body 变换，因此这里只接受 world_frame=map。
  这套专用公式不是任意 TF 求解器，需与现场机器人坐标约定配套。
- CapturePointSolver 以 `randomize_capture_y=False` 调用：两侧 Y 取**最靠近箱缘的
  安全点，是确定性的，不是随机采样**；姿态也不再沿用当前末端四元数，而是固定
  RPY（左 -90/-30/0、右 -90/30/0，yxz）转出的四元数，因此预备点姿态与本 Skill
  执行前的末端朝向无关。每次执行只解算一次并在下发运动前 checkpoint。
  Skill 校验完整姿态和 frame，不用缺失四元数默认值。
- move_box_to_center 的实际公式：`x = 当前左末端 x - width/4`、
  `z = 当前左末端 z`（即抬升后的高度）、`left_y=(当前left_y-当前right_y)/2`、
  `right_y=-left_y`，沿用实际姿态。旧文档写的 `x=width/2+x_offset`、
  `z=height/2+z_offset` 已被上游注释掉；对应输入的
  `center_x_offset_m/center_z_offset_m`（`end_pose_*_offset`）**不再被函数体引用**，
  改它们不会改变点位。因为 z 取自抬升后的实际高度，收箱高度会随 `lift_m`、
  `capture_z_offset_m`、站位与躯干高度一起漂移，不再是由箱体尺寸决定的固定量；
  `height_m` 也因此不参与任何计算。
- force_align 是主动步进、监测相对启动基线的力变化；verify 也会移动指定手臂，
  不是静态绝对力检测。当前上游触阈后返回最后一个命令，未提供独立触阈证据或
  物理停止回执，Skill 按该 Action 的成功契约判断。
- 当前 trajectory 的 speed 只用于日志，不是底层限速；位置与姿态容差由
  `motion` / `approach_motion` 的 `pos_tol`（默认 0.03 米）与 `orient_tol`
  （默认 0.01 弧度）显式给出；物理速度由实际控制栈决定。

## 图像日志

采集使用现有 robot.capture_rgbd。当前 RobotObservation 实现只返回帧元数据时，
结果会标记 `image_status=metadata_only` 并报告 stage.evidence_unavailable，不能
伪称有可展示图片。若执行侧提供 artifact:// 或 pilot-artifact:// 引用，则保留在
image_refs 并标记 available。采图失败标记 failed，仍按指定单次力反馈判据验证。
图像正文不写入检查点。实际图片持久化需要 Ability/Pilot 提供图像 Artifact。

## 恢复与停止

每个子步骤有固定 Action key，计划参数和结果保存在检查点。左右接触分别记录，
恢复时不重复首侧接近；已完成抬升不重复抬升，已完成 stand 不重复躯干回零。
失败即终止，不自动刷新现场并重抓。
数据缺失、frame 不匹配、未匹配到目标或 solver 越界均不会继续发运动。
每个物理动作成功返回后，Skill 会固定等待 2 秒再进入下一步（给机构收敛与振动
衰减留时间），因此整体耗时比纯动作时间略长；等待结束后仍会检查取消请求。

停止交由 Pilot 取消当前 Action。on_stop 不重新发布运动；只要本次已经开始过
物理动作，就返回 safe=false、physical_state=unknown、requires_intervention=true。
当前上游同步循环执行完成才登记 Execution，运行中取消及双侧保持的能力边界仍
需在 Ability 层解决；本 Skill 不把收到停止请求当作真机已经停止。

## 依赖与使用

需要四种 Ability role：`torso`、`perception`、`manipulator_motion`、`robot`。
Pilot 根据 required_actions 预检绑定；prepare 的躯干动作在输入 `torso.enabled=true`（本示例）
时执行、省略时跳过，stand 必定使用
躯干回零 Action，部署需提供声明的 Action。
SDK、RobotDeployment 和相机标定由实际部署提供，Skill 不直接连接 ROS/Robot SDK。

离线测试（不运行机器人，在仓库根执行）：

```bash
PYTHONPATH=../robot-skill:src:src/skill PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -p no:cacheprovider -q src/skill/grasp_object_bilateral/tests
```

打包（包含两个 solver，无需运行时访问源码仓库）：

```bash
python src/skill/grasp_object_bilateral/package_skill.py --output /tmp/grasp-object-bilateral-0.2.7.zip
```

模型通过 robot.run 选择已核对安装启用的准确版本（当前源码为
`grasp-object-bilateral@0.2.7`），直接提供按本文构造的 input 对象。
Action key 前缀为 `grasp-object-bilateral:`，与本目录的 Skill 名一致。

> **版本号必须与本文件 frontmatter 的 `version` 完全一致。** Pilot 的
> `SkillCatalog.Resolve`（`semantic-framework/internal/pilot/skillcatalog.go`）做精确
> 匹配，写错版本会直接报 `skill grasp-object-bilateral version X is unavailable`
> 并且不会下发任何动作。源码文档更新不会自动更新已安装包；`robot.get` 返回的仍是安装包文档，
> 需在另行打包和部署后才会改变。

本地 Pilot 调用入口沿用仓库脚本 `start-shell/run_grasp_bilateral_skill.sh`（当前内置
`--skill grasp-object-bilateral@0.2.3`）。该脚本自己把 `--skill-catalog` 指向
`<repo>/src/skill`，因此解析的是源码目录里的 frontmatter 版本，而不是真机部署目录
里的包；目前脚本内置版本与源码 0.2.7 不一致，不能直接用该脚本执行当前版本。
运行前须先核对脚本版本配置，或使用下方直接调用形式显式指定源码版本：

```bash
"$GRASP_PILOT" skill run --profile <deployment.yaml> \
  --skill grasp-object-bilateral@0.2.7 --input <现场输入.json> \
  --skill-catalog <repo>/src/skill --python "$GRASP_PYTHON" \
  --python-path <repo>/src --python-path <workspace>/semantic-skill/robot-skill \
  --events <out>/events.jsonl --result <out>/result.json --timeout 25m
```

本地 CLI 调用须显式传部署文件和现场输入文件；输入文件仅保存按本文构造的对象，
是调用载体，不是模型需要读取的额外参数来源。本 Skill 的构建与测试不会调用真机脚本。
