---
name: grasp-object-unilateral
description: R1 Pro 真机单手抓箱。输入世界系箱体中心与尺寸，指定抓取侧，先由该侧手接近、力控对齐、上抬 0.10 米并沿 y 拉箱远离辅助手方向；辅助手依次沿 Y 预定位、沿 Z 到接近高度、沿 X 到 target，再力控对齐、上抬 0.10 米；随后双手按 lift_m（默认 0.1 米）抬升并携箱收回、躯干四关节回零站起；拍照记录并用一次单侧 force_align 触阈判定持物。
category: robot_skill
version: 0.1.12
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
  - { type: motion.move_end_effector_single, schema_version: 2 }
  - { type: manipulation.force_align, schema_version: 2 }
  - { type: motion.move_end_effector_single_trajectory, schema_version: 2 }
  - { type: motion.move_end_effector_both, schema_version: 2 }
  - { type: motion.move_end_effector_both_trajectory, schema_version: 2 }
  - { type: robot.capture_rgbd, schema_version: 2 }
# 声明可由 Pilot 取消的物理 Action；on_stop 不把这些动作重新下发为停止命令。
stop_actions:
  - { type: torso.move_to_posture_all, schema_version: 2 }
  - { type: motion.move_end_effector_single, schema_version: 2 }
  - { type: motion.move_end_effector_single_trajectory, schema_version: 2 }
  - { type: motion.move_end_effector_both, schema_version: 2 }
  - { type: manipulation.force_align, schema_version: 2 }
  - { type: motion.move_end_effector_both_trajectory, schema_version: 2 }
---

# 真机单手抓箱

## 模型调用条件

需要单手先抓箱再由辅助手对侧接合时使用本 Skill。上游应先完成底盘站位，
末端已处于适合接合箱子的朝向，工具无需额外夹爪开闭动作。只支持已按现场
坐标约定标定的正向箱体；本 Skill 不做箱体旋转求解、导航或抓取策略切换。
不让模型生成轨迹点或猜测力阈值。

## 输入

### 参数构造规则

本文件是模型构造本 Skill 输入的统一说明入口。先调用
`robot.get(skill_name="grasp-object-unilateral")`，确认当前唯一安装启用的准确版本并读取
`skill_contract.documentation`；如存在多个启用版本，先核对执行版本再显式提供
`skill_version`，不得选 Registry latest。如工具仅返回头尾预览，必须按返回的
artifact 引用读取完整正文。非地图参数使用下方完整示例中明确列出的现场配置，
不再从 `/workspace/inputs/grasp_object_unilateral.example.json`、历史调用或其他版本文档补值。
用户明确指定的参数优先，但必须符合当前输入契约；与契约冲突时报告具体冲突，
不得自行改值。没有明确覆盖要求时，不自行调整本文的力阈值、步长、偏移、运动顺序或超时。

输入模型为 `scripts.models:GraspInput`，必填 `target`、`grasp_axis`、
`approach_force`、`verify_force`。下方示例显式列出全部有效输入字段，避免依赖程序
默认值隐式选择现场参数；`center_x_offset_m` / `center_z_offset_m` 是保留的兼容字段，
当前 solver 未使用，不列入调用示例。距离单位为米，关节角为弧度、关节速度为
弧度/秒；力阈值为牛顿，超时和循环间隔为秒。

### 完整输入示例

下例的地图字段取自 2026-10-26 执行所用的 `gray_box-l5-1` 地图快照，
用于展示合法的数据类型和完整结构；**以后每次执行都必须按后文映射表从当前
`real_map` 源箱实体重新取值，不得将这份快照当作任意箱子的目标**。
非地图字段是本文件收录的当前现场配置，沿用该次成功执行的取值，
不是“仅展示格式”的任意数值。地图字段替换完成并满足当前契约后，整体作为
`robot.run.input` 提交；不得提交占位符或将字符串填入数值字段。

```json
{
  "target": {
    "object_ref": "gray_box-l5-1",
    "world_frame": "map",
    "position_m": [1.11, 0.978, 1.24],
    "size": {"long_m": 0.53, "width_m": 0.41, "height_m": 0.23}
  },
  "grasp_axis": "right",
  "relative_frame": "torso_link4",
  "pull_y_distance_m": 0.2,
  "pull_force_ratio_threshold": 1.5,
  "max_grasp_retries": 1,
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
    "speeds": [0.5, 0.8, 0.7, 0.7],
    "tol": 0.02,
    "timeout": 10.0
  },
  "perception": {
    "gray_box_only": true,
    "sensor_id": "head_camera",
    "minimum_confidence": 0.25,
    "top_k": 2,
    "delta_threshold_x": 0.2,
    "delta_threshold_y": 0.25,
    "delta_threshold_z": 0.2,
    "timeout_seconds": 120.0
  },
  "approach_force": {
    "force_threshold_n": 1.0,
    "force_coordinate": ["x"],
    "step_size_m": 0.01,
    "loop_freq_s": 0.1,
    "timeout_s": 12.0
  },
  "verify_force": {
    "force_threshold_n": 1.0,
    "force_coordinate": ["x"],
    "step_size_m": 0.001,
    "loop_freq_s": 0.1,
    "timeout_s": 12.0
  },
  "verify_side": "left",
  "motion": {
    "pos_tol": 0.05,
    "orient_tol": 0.05,
    "timeout": 10.0,
    "loop_freq": 0.1,
    "num_points": null,
    "max_num_points": 100,
    "speed": 0.03,
    "step_distance": 0.01,
    "sequential": true,
    "axis_order": "zyx"
  }
}
```

### 地图参数及字段映射

先用 `map.query` 在项目的 `real_map` 槽位按当前 Task 的源箱实体名查询，
选择 `status=active` 的源箱；忽略同名 `removed` 记录。若仍有多个 active 候选，
必须先消歧，不能任取一条。以下字段必须来自**同一个当前源箱实体**，
不得使用目的放置实体、上一箱数据或导航修正后的目标替代。

| Skill 输入字段 | real_map 源箱字段 | 映射要求 | 上例值 |
|---|---|---|---|
| `target.object_ref` | `name` | 使用业务实体名，不填实体 `id` | `gray_box-l5-1` |
| `target.world_frame` | `frame_id` | 当前输入模型只接受 `map`；其他坐标系须先解决变换，不能只改标签 | `map` |
| `target.position_m[0]` | `pose.position.x` | 地图原始箱体中心 X，米 | `1.11` |
| `target.position_m[1]` | `pose.position.y` | 地图原始箱体中心 Y，米；不加导航侧向偏移 | `0.978` |
| `target.position_m[2]` | `pose.position.z` | 地图原始箱体中心 Z，米 | `1.24` |
| `target.size.long_m` | `bounds.size.y` | 箱体 Y 向跨度，正数，米；不要与 X 对调 | `0.53` |
| `target.size.width_m` | `bounds.size.x` | 箱体 X 向跨度，正数，米 | `0.41` |
| `target.size.height_m` | `bounds.size.z` | 箱体高度，正数，米 | `0.23` |
| `grasp_axis` | `properties.grasp_mode` | 仅接受 `left` / `right`；`both` 应选择双手 Skill | `right` |
| `torso.positions` | `properties.torso_joint_positions` | 按原顺序复制四个关节角，弧度；不根据层号或箱体高度猜测 | `[0,0,0,0]` |
| `torso.speeds` | `properties.torso_speeds` | 按原顺序复制四个正关节速度，弧度/秒 | `[0.5,0.8,0.7,0.7]` |

`verify_side` 另有可选地图映射：源箱提供 `properties.verify_side` 时，
取其 `left` / `right` 值；字段缺失时显式使用本文现场配置 `left`。
提供了非法值时报告错误，不静默回退；不根据 `grasp_axis`、`carry_mode` 或
`place_mode` 推导验证侧。上例源箱属性为 `verify_side=left`。

`torso.enabled=true` 是本文现场配置，不是地图字段；启用后必须提供上表两项
源箱躯干属性。`stand.speeds` 是回零阶段的独立现场配置，不映射为源箱的
`torso.speeds`，即使本例两者数值相同。`relative_frame=torso_link4` 是执行侧
求解坐标系，与 `target.world_frame=map` 含义不同，不互相替代。

源箱不存在、必需属性缺失或类型不合法时，报告具体缺项，不能用本例数值补齐。
`yaw` / `orientation` 不映射为额外的抓取输入字段：本 Skill 不接受
`target.yaw_rad`，也不求任意旋转箱体的抓取点；仅适用于部署已标定的正向箱体。
地图本身保持只读。

### 非地图参数与程序默认值

上述完整示例是当前现场调用配置。下表说明它与输入模型默认值的区别，
**不要因为模型存在默认值就省略现场配置或改用默认值**。

| 参数 | 本文现场配置 | 程序默认值或约束 |
|---|---|---|
| `relative_frame` | `torso_link4` | 默认同值；须与感知、变换和末端反馈的坐标约定一致 |
| `pull_y_distance_m` | `0.2` | 默认 `0.1`；left 沿 +Y、right 沿 -Y 外拉；辅助手安全余量为该值的一半 |
| `safe_offset_m` | `0.1` | 默认 `0.07`；初始抓取预备点的箱侧 Y 向安全间距 |
| `capture_x_offset_m` / `capture_z_offset_m` | `-0.1` / `0` | 默认均为 `0`；只偏移主抓手初始预备点，不改写感知中心 |
| `lift_m` | `0.15` | 默认 `0.1`；仅作用于双手 lift 段，不控制两次单臂接触后上抬 |
| `torso.enabled` | `true` | 默认 `false`；`timeout=10`、`tol=0.02` 与程序默认值一致 |
| `stand` | 速度 `[0.5,0.8,0.7,0.7]`、容差 `0.02`、超时 `10` | 默认相同；四关节回零目标由代码固定，不开放 `positions` / `enabled` |
| `perception` | 完整示例的八个字段 | `gray_box_only` 默认 `false`，调用必须显式设为 `true`；其余七项与程序默认值一致 |
| `approach_force` | x 轴、`1N`、步长 `0.01`、间隔 `0.1`、超时 `12` | 阈值和监测轴必填；程序默认步长 `0.01`、间隔 `0.1`、超时 `3` |
| `verify_force` | x 轴、`1N`、步长 `0.001`、间隔 `0.1`、超时 `12` | 阈值和监测轴必填；程序默认步长 `0.01`、间隔 `0.1`、超时 `3` |
| `motion` | 顺序 `zyx`、步距 `0.01`、姿态容差 `0.05`、单点超时 `10`，其余见完整示例 | 对应程序默认值依次为 `yzx`、`0.05`、`0.01`、`30`；其余项与示例一致 |
| `center_x_offset_m` / `center_z_offset_m` | 不传 | 兼容字段，默认均为 `0.2`，当前 solver 不使用，调整它们不会改变收箱点 |

`perception.minimum_confidence` 是匹配结果的最低置信度，`delta_threshold_*`
是目标匹配容差；这些不是 YOLO 检测模型的 `conf_threshold` / `iou_threshold`。
不向本 Skill 添加这两个未声明字段，检测模型自身阈值由 Ability 配置提供。

轨迹共用一组 `motion`，影响主抓手接近、单臂上抬、拉箱、辅助手接合及双手携箱。
本现场配置显式采用 `sequential=true`、`axis_order=zyx`；这是调用值，
不要与程序默认的 YZX 顺序混淆。`num_points=null` 时，双手 Ability 按
`step_distance` 自动算点，单臂因 Ability 的 `int(num_points)` 契约使用
`max_num_points=100`；显式点数模式下 `step_distance` 不决定单臂点数。
Action 等待预算为 `motion.timeout × (num_points 或 max_num_points) + 15` 秒；
`motion.timeout` 表示单点超时。`speed` 当前仅用于日志，不代表底层限速。

主抓手和辅助手接触后的单臂上抬距离由代码常量 `GRASP_LIFT_M=0.10` 固定，
辅助手预备点的 Z 相对主抓手当前 Z 下降量由另一常量 `OTHER_ARM_Z_LIFT_M=0.10`
固定；两者均不是模型可传的输入字段，不能用 `lift_m` 或 `capture_z_offset_m` 替代。
这些运动配置不是避障保证；成功判据以本 Skill 的实际终态与验证输出为准。

## 外拉持箱检测与自动重抓

每轮主抓手到达 capture_point 后、接触力控前，调用一次
robot.get_sensors_state(observation="force_data")，读取 grasp_axis 对应主手的 Fx/Fy/Fz。
主抓手完成上抬与外拉后、辅助手接近前，再读取一次相同主手的力反馈。
两次均使用 **F = |Fx| + |Fy| + |Fz|**（单位 N），不是欧氏范数，不扣除基线。
仅当 F_after > F_before × pull_force_ratio_threshold 时通过，等于阈值不通过。
计算直接使用乘法比较，基准为零时仍按此公式处理，不做除法。

- pull_force_ratio_threshold：有限正数，默认 1.5，可调，不映射地图字段。
- max_grasp_retries：非负整数，默认 1，表示额外尝试次数（最多两轮）；0 表示不重试。
- 每轮各采样一次，沿用现有物理 Action 结束后的等待；不增加平均采样或主动 force_align 检测动作。
- 仅倍率检测未通过触发重抓。依据已确认现场条件：未卡进凹槽时箱子留在原位，
  保持手臂当前位置、不增加退让动作，返回目标匹配（原第 4 步），重新 YOLO 识别，
  读取双手位姿并重新求抓取点。躯干调整和未变化的地图变换不重复执行。
- 每轮重新采集基准，不能复用前轮合力。达到重试上限返回 GRASP_RETENTION_FAILED，
  不继续辅助手动作。数据缺失、非有限值、运动失败、状态未知或用户停止均不重试。
- 此恢复条件仅适用于箱子仍留在原位的场景；沿用现有匹配范围和地图高度约定，
  不用于箱子跌落、移位后的自主搜索或换箱。
- prepare:match、prepare:pose 和全部 approach Action 使用 attempt:<轮次>: 前缀，
  首轮为 0；检查点保存轮次和历次动作、结果、pull_force_checks，同一轮恢复不重复成功动作。
- 成功结果增加 grasp_attempts 和 pull_force_checks。检查项记录两次 XYZ、两次 L1 合力、
  倍率、主手侧及判定。失败时同样保留检查点与 grasp.pull_force_checked 事件。
- approach_revision=4：拒绝恢复已进入旧版 approach 的检查点，避免把外拉后的读数当基准。

## 五阶段

1. **prepare**：可选 torso.move_to_posture_all；通过 robot.get_state 分别读取
   transformer_martix 和 floating_base；调用 TargetPointSolver.solve 得到相对点；
   perception.match_target 匹配箱体中心；按双手版契约读取
   `best_match.center_robot=[x,y,z]` 和 `best_match.confidence`，复制坐标并用
   输入目标的转换高度覆盖 Z，不修改原始感知结果；读取左右末端位姿，再调用
   CapturePointSolver.get_capture_point(axis=grasp_axis) 保存抓取侧预备点，
   辅助手保留当前 ee_pose。
2. **approach**（原 10 步，新增预备点后及外拉后的两次只读力检测）：
   1. `motion.move_end_effector_single` 抓取侧移到 capture_point；
      到位后读取本轮主手基准力，再继续。
   2. `manipulation.force_align` 抓取侧沿 Y 方向力控对齐（left→negative，
      right→positive）；
   3. 读取抓取侧接触后的实际位姿，保存只加 Z=GRASP_LIFT_M（0.10 米）的目标，再通过
      `motion.move_end_effector_single_trajectory` 上抬；保持接触后的实际 X/Y/姿态；
   4. 读取上抬后的实际位姿，保存拉箱目标，通过单臂 trajectory 沿 y 拉箱
      （left→+y、right→-y），位移 = `pull_y_distance_m`；
      外拉后读取主手力并执行上述倍率检测；通过后才进入后续步骤。
   5. 拉箱完成后读取双手实际位姿，保存辅助手三个目标：`y_move_pose` 只改 Y，
      `x_move_pose` 在 Y 到位后只改 Z，`target_pose` 在 Y/Z 到位后改 X 到主抓手当前 X；
      最终 Z=主抓手当前 Z-0.10（由 OTHER_ARM_Z_LIFT_M 显式传入），侧向安全余量=`pull_y_distance_m / 2`；
   6. `motion.move_end_effector_single` 辅助手仅沿 Y 到安全位置；
   7. `motion.move_end_effector_single` 辅助手仅沿 Z 到接近高度；
   8. `motion.move_end_effector_single` 辅助手仅沿 X 到 `target_pose`；
   9. `manipulation.force_align` 辅助手力控对齐（left→negative，right→positive）；
   10. 读取辅助手接触后的实际位姿，保存只加 Z=0.1 的目标，再通过单臂 trajectory
      上抬；不返回接触前的 target。接触、上抬、拉箱及辅助手接合的全部动作成功
      才进入 lift。
3. **lift**：重新读取双手实际位姿，各自 Z 加 `lift_m`（默认 0.1 米），通过双手 trajectory 抬升；
   再读实时双手位姿，调用 CapturePointSolver.move_box_to_center，双手 trajectory
   携箱收回。与稳定双手 Skill 使用相同求点公式、轨迹参数和等待预算；左右使用
   相同点数，默认 YZX 逐轴插值（可配置），并非固定六维同步插值。
4. **stand**：只调用一次 torso.move_to_posture_all，将躯干四关节目标设为零；
   使用独立的 `stand` 速度、容差与超时配置。成功后进入 verify，失败即终止。
5. **verify**：robot.capture_rgbd(axis="rgb") 采一帧日志；对 verify_side 调用
   一次 force_align。Action 成功即按用户指定判据设置 holding_object=true。

成功结果明确标记 `verification_method=single_force_align`，记录验证侧、力阈值、
匹配中心、抓取/抬升/收箱目标和证据引用，以及拉箱位移与辅助手上抬量。

## 上游接口的实际语义

- 本 Skill 显式传 `mm_scale=1.0` 全程使用米。
  保留上游 `R p + t` 和 `z = 输入世界Z - abs(floating_base.z)` 的公式。
- CapturePointSolver.get_capture_point(axis=grasp_axis) 单手返回抓取侧
  xyz + 求解器固定抓取姿态四元数；单手 Skill 适配层返回
  `{"y_move_pose", "x_move_pose", "target_pose", "other_axis"}`；三个姿态按
  Y、Z、X 顺序单轴变化，最终目标与主抓手当前 x 相同、z 为当前 z 减去 z_lift
  （本 Skill 适配层显式传 OTHER_ARM_Z_LIFT_M=0.10 米；共享 solver 自身默认 0.1，勿依赖）。
- pull_point 沿 y 方向拉箱：left→+y、right→-y（远离辅助手），位移 = pull_y_distance_m。
- 辅助手目标 Y：右手主抓时为 `主抓手实际Y + long_m + pull_y_distance_m/2`，
  左手主抓时为 `主抓手实际Y - long_m - pull_y_distance_m/2`。
- move_box_to_center 当前设置 `x=当前left_x-width_m/4`、`z=当前left_z`，
  双手使用相同 X/Z；`left_y=(当前left_y-当前right_y)/2`、`right_y=-left_y`，
  各自沿用实际姿态。旧的 `width/2+x_offset`、`height/2+z_offset` 公式已被注释，
  所以 `center_x_offset_m/center_z_offset_m` 不参与当前收箱计算。
- force_align 是主动步进、监测相对启动基线的力变化；verify 也会移动指定手臂。
- 当前 trajectory 的 speed 只用于日志，不是底层限速；默认位置容差在 Skill 中
  显式设置为 0.05 米，可通过 motion 配置调整；默认 speed=0.03，仍只用于日志。

## 图像日志

采集使用现有 robot.capture_rgbd。当前 RobotObservation 实现只返回帧元数据时，
结果会标记 `image_status=metadata_only` 并报告 stage.evidence_unavailable，不能
伪称有可展示图片。若执行侧提供 artifact:// 或 pilot-artifact:// 引用，则保留在
image_refs 并标记 available。采图失败标记 failed，仍按指定单次力反馈判据验证。
图像正文不写入检查点。

## 恢复与停止

每个子步骤有固定 Action key，计划参数和结果保存在检查点。抓取侧接触、拉箱、
辅助手接触、上抬分别记录，恢复时不重复已完成的子步骤；已完成抬升不重复抬升。
已完成 stand 不重复躯干回零；verify 必须在 stand 成功之后执行。
仅外拉持箱倍率检测未通过按上述上限重新识别并重抓；其他失败即终止。数据缺失、frame 不匹配、未匹配到目标或 solver
越界均不会继续发运动。

停止交由 Pilot 取消当前 Action。on_stop 不重新发布运动；只要本次已经开始过
物理动作，就返回 safe=false、physical_state=unknown、requires_intervention=true。
当前上游同步循环执行完成才登记 Execution，运行中取消及单手保持的能力边界仍
需在 Ability 层解决；本 Skill 不把收到停止请求当作真机已经停止。

## 依赖与使用

需要四种 Ability role：`torso`、`perception`、`manipulator_motion`、`robot`。
Pilot 根据 required_actions 预检绑定；prepare 的躯干动作默认跳过，stand 必定使用
躯干回零 Action，部署需提供声明的 Action。
SDK、RobotDeployment 和相机标定由实际部署提供，Skill 不直接连接 ROS/Robot SDK。
本 Skill 通过绝对导入访问 `src/solver/` 包，部署时须把仓库的 `src` 加入 PYTHONPATH。
公共阶段回归测试会对比同仓库的 `grasp_object_bilateral`；生产单手包不依赖双手 Skill。
stand 使用与双手 Skill 相同的参数和回零 Action，不改变共享 CapturePointSolver。
新 approach 使用检查点 `approach_revision=4`，接触后的上抬点、拉箱点、辅助手
Y/Z/X 三个目标均在运动前持久保存。同版本恢复复用已保存目标与 Action，不重复读取旧阶段
的现场后重新求点，也不重复执行成功动作。
**升级边界**：若旧版检查点停在 approach 且已有该阶段动作、结果或接触状态，
返回 `APPROACH_CHECKPOINT_INCOMPATIBLE`，不自动覆盖旧动作、不混用新旧路径。
必须人工确认物理状态后新建执行；仅完成 prepare 的旧检查点可进入新版 approach。
已经进入 lift 的旧检查点继续复用已保存 Action 参数，完成收箱后进入 stand。
旧版 verify 检查点若缺少 stand 完成标记，会返回 `GRASP_DATA_INVALID`，不自动补发
躯干动作；须确认现场物理状态后新建执行。

离线测试（不运行机器人，在仓库根执行）：

```bash
PYTHONPATH=../robot-skill:src:src/skill PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -p no:cacheprovider -q src/skill/grasp_object_unilateral/tests
```

打包（不包含 solver 副本，运行时访问根包）：

```bash
python src/skill/grasp_object_unilateral/package_skill.py --output /tmp/grasp-object-unilateral-0.1.12.zip
```

模型通过 robot.run 选择 `grasp-object-unilateral@0.1.12` 并提供上述输入。
