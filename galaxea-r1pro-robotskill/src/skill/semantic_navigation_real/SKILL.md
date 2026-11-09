---
name: semantic-navigation-real
description: R1 Pro 真机箱子作业导航。Skill 按 mode/yaw 将 move_y_m 加到原始目标地图 Y，再由本地 solver 计算底盘站位；携物放置到位后检测对应侧腕力。
category: robot_skill
version: 0.2.0
runtime:
  api_version: 1
  python: ">=3.11"
  entrypoint: scripts.skill:run
  stop_entrypoint: scripts.skill:on_stop
  input_model: scripts.models:SemanticNavigationInput
  state_model: scripts.models:SemanticNavigationState
  result_model: scripts.models:SemanticNavigationResult
  controllers: {}
required_actions:
  - { type: navigation.follow_route, schema_version: 2 }
  - { type: navigation.verify_arrival, schema_version: 2 }
stop_actions:
  - { type: navigation.follow_route, schema_version: 2 }
---

# 真机箱子作业导航

## 模型调用条件与流程

用于前往箱子抓取站位，或携物前往放置站位。Robot Agent 传入原始目标及用途，
Skill 在内部修正地图 Y 后交给本地 solver，WooshNavigation 完成路径规划与到位检测。

- approach_grasp + left/right/both：navigation → 完成。
- carry_to_place + left/right：navigation → verify → 完成；不接受 both。
- 导航到位后没有相对移动 Action。本 Skill 不抓取、放置或调整肩高。
- verify_arrival 是到位后的指定侧腕力检测，不是位置/姿态验收，也不是途中持物监测。

## 输入构造规则

当前准确版本的完整 current_skill_contract.documentation 已注入执行提示时直接使用。
缺少契约时调用 robot.get(skill_name="semantic-navigation-real")；使用当前 Robot 已安装且
启用的准确版本，不选 Registry latest；返回正文截断时按 artifact 引用读取完整文档。
非地图参数使用本文明确列出的现场配置，不从 /workspace/inputs、历史调用或其他版本补值。
用户明确覆盖优先，但必须符合当前契约；冲突时报告，不自行调参。

输入模型为 scripts.models:SemanticNavigationInput。距离单位米，yaw 为弧度，力为牛顿，
超时为秒。必填 navigation_purpose、mode、target；left/right 还必须显式传 move_y_m。

**target.position_m 必须是未经任何侧向修正的地图原始箱心或槽位中心。**
模型不计算 Y 偏移、不传 solver 已求出的底盘站位、不传手中箱子的当前位置作为放置目标。
Skill 仅修改目标副本的 Y，保持 X/Z 不变，地图只读。抓取、放置 Skill 仍接收原始地图中心。

### 完整输入示例

示例中的实体与坐标仅说明格式，执行时必须查询当前任务目标。
现场配置：单手接近 move_y_m=0.15，携物放置 move_y_m=0.10；Skill 不按用途隐式补值。

右手接近：模型传入原始中心 [1.11, 0.978, 1.24]；
Skill 内部得到 [1.11, 0.828, 1.24] 再交给 solver：

```json
{
  "navigation_purpose": "approach_grasp",
  "mode": "right",
  "move_y_m": 0.15,
  "target": {
    "target_ref": "gray_box-l5-1",
    "position_m": [1.11, 0.978, 1.24],
    "yaw_rad": 0.0
  },
  "frame_id": "map",
  "target_point_type": 0,
  "timeout": 120.0
}
```

右手携物放置：传入原始槽位中心；内部得到 Y=-0.942：

```json
{
  "navigation_purpose": "carry_to_place",
  "mode": "right",
  "move_y_m": 0.10,
  "target": {
    "target_ref": "dest-l2-1",
    "position_m": [1.11, -0.842, 0.55],
    "yaw_rad": 0.0
  },
  "frame_id": "map",
  "target_point_type": 0,
  "timeout": 120.0,
  "verify": {
    "carrying_force_axis": "x",
    "carrying_min_load_n": 1.0
  }
}
```

双手接近不偏移：省略 move_y_m（也可传 0），不传 verify：

```json
{
  "navigation_purpose": "approach_grasp",
  "mode": "both",
  "target": {
    "target_ref": "gray_box-l5-2",
    "position_m": [1.01, 1.529, 1.24],
    "yaw_rad": 0.0
  },
  "frame_id": "map",
  "target_point_type": 0,
  "timeout": 120.0
}
```

### 地图字段映射

用 map.query 查询当前任务目标：接近抓取选源箱，携物放置选目的槽位。
只使用当前 generation 中唯一 active 实体，忽略 removed 记录；多个候选先消歧。
以下字段取自同一个选定实体：

| 输入 | 来源/规则 |
|---|---|
| navigation_purpose | 当前业务阶段：approach_grasp 或 carry_to_place |
| target.target_ref | 实体 name，不填实体 id |
| target.position_m | pose.position 的原始 x/y/z；模型不偏移、不按层号推算高度 |
| target.yaw_rad | properties.yaw_rad；缺失时由同实体 pose.orientation 求 yaw |
| frame_id | 实体 frame_id，本现场为 map |
| mode，接近抓取 | 源箱 properties.grasp_mode：left/right/both |
| mode，携物放置 | 目的槽位 properties.place_mode：left/right；不从抓取侧、carry_mode 或 verify_side 推导 |

四元数按 [x,y,z,w] 读取，确认非零并归一化后，
yaw = atan2(2*(w*z+x*y), 1-2*(y*y+z*z))。
若同时存在 properties.yaw_deg，使用 yaw_deg*π/180 核对；角度按模 2π 比较。
缺失、冲突或类型不合法时报告具体缺项，不默认猜 yaw=0。
frame_id 同时约束输入目标和输出底盘站位，Skill 不进行坐标系转换。
尺寸、躯干角/速度及 verify_side 不属于本 Skill 输入。

### Skill 内部 Y 偏移

move_y_m 是非负有限幅值，方向由 mode/yaw 确定：

| mode | yaw=0 时 ΔY | yaw=±π 时 ΔY |
|---|---|---|
| left | +move_y_m | -move_y_m |
| right | -move_y_m | +move_y_m |
| both | 0 | 0 |

yaw 按模 2π 归一化，0/±π 比较容差为 1e-6 rad。
非零偏移遇到其他朝向时，在任何 Action 下发前以 NAV_TARGET_INVALID 失败。
move_y_m=0 或 both 不偏移，允许共享 solver 支持的任意 yaw。
left/right 必须显式传 move_y_m，0 关闭偏移；both 仅允许省略或 0。

计算顺序：原始中心 → Skill 修正 Y → 共享 solver 求底盘站位 → 导航一次到位。
共享 solver 不改动，也不接收 mode 或 move_y_m。

### 非地图参数

| 参数 | 现场配置与约束 |
|---|---|
| move_y_m | 单手接近 0.15、携物 0.10；可显式覆盖，非负；both 省略或 0 |
| target_point_type | 0；严格整数 |
| timeout | 现场 120 秒，程序默认 60；仅控制导航 Ability |
| verify | 携物必填；接近抓取禁止提供 |
| verify.carrying_force_axis | 现场 x，程序默认 z；支持 x/y/z/norm |
| verify.carrying_min_load_n | 现场 1 N，无程序默认值，必须为正数 |

检测侧由 mode 自动生成 tool_refs=[mode]，模型不能另传 tool_refs。
不接受 speed、tolerance、move 对象或已删除的 move_distance_m。

## Action 与完成契约

绑定 WooshNavigation.Demo，不依赖 WooshMove：

| Stage | taskName | ActionType@schemaVersion | 参数 |
|---|---|---|---|
| navigation | navigate_to_map_pose | navigation.follow_route@2 | target_x, target_y, target_orientation, target_point_type, frame_id, timeout |
| verify | verify_arrival | navigation.verify_arrival@2 | tool_refs, carrying_force_axis, carrying_min_load_n |

solver.Nav.navigation.solve_box_navigation_pose 和 NavigationPose.to_action_parameters
生成底盘参数，四元数顺序 [qx,qy,qz,qw]。导航 Action 超时为 timeout+10 秒，
力检测 Action 超时为 15 秒。invocation_id 由 Pilot 提供。
不发送 target.pose、route_ref、maximum_speed_mps 或 arrival_radius_m。

navigation 首次求解后，在动作下发前 checkpoint；恢复时复用已保存目标与稳定
Action key，不再次计算/叠加偏移。只有 Action succeeded 才算导航及到位检测成功。
failed/stopped/interrupted 均不继续，不自动重放。

接近抓取导航成功后完成：holding_object=null、load_verification=not_required；
未检测不代表已经确认空手。携物导航成功后进入 verify：
除 Action 成功外，还要求合法的 verdict/holding_object/carrying_check、
checked=true、无传感错误、检测侧与 mode 一致、方向/阈值与请求一致，
并且该侧 loaded=true 且力绝对值达到阈值。力检测不识别箱子身份。

结果保留 target_ref、navigation_target、navigation_succeeded、
holding_object、load_verification、carrying_check、evidence_refs。
没有 move_required、move_succeeded 或 move_target。

## 错误、恢复与停止

| 错误码 | 含义 |
|---|---|
| NAV_TARGET_UNREACHABLE / NAV_TARGET_INVALID | 无可用站位、目标非法或偏移朝向不支持；未下发导航 |
| NAVIGATION_FAILED | 导航未成功 |
| ACTION_STATE_UNKNOWN | 导航物理终态未知，禁止自动重放 |
| INVALID_VERIFY_STATE | verify 不在单手携物导航成功之后 |
| VERIFY_ACTION_FAILED | 力检测调用失败 |
| VERIFY_OUTPUT_INVALID / VERIFY_CONTRACT_MISMATCH | 返回缺失或与请求契约不符 |
| TOOL_LOAD_UNVERIFIED | 未完整读取指定侧腕力 |
| CARRYING_LOAD_NOT_CONFIRMED | 未确认指定侧仍携物 |

无内部自动重试或 Agent 决策等待。Pilot 通过 Ability StopExecution 取消当前 Action。
on_stop 不下发新的运动；导航尚未确认完成时返回 safe=false、physical_state=unknown，
要求确认。导航已成功、只剩只读力检测时可以报告 base_motion_completed。
共享 solver 只做几何站位与名义双臂可达性计算；肩高是假设值，实际升降由后续 Skill
负责。地图、底盘/箱体包络、速度与避障由导航配置负责；导航成功不代表抓放完成。

## 从 0.1.x 迁移

0.2.0 的输入、检查点及结果不兼容旧版：
- 删除 move_distance_m；不能将旧值 0.01 原样改名，应按业务设置 move_y_m。
- 删除调用方/规划提示中的“先修正地图 Y”，传原始中心，避免双重偏移。
- 旧执行及检查点保留在旧版本，不跨版本恢复；新 state 没有 move 阶段/状态。
- 同步更新依赖旧 move_* 结果字段的调用方。
Skill 无法从坐标数值识别目标是否已预偏移，调用方必须遵守原始中心契约。

## 开发与打包

本地输入载体为 inputs/semantic_navigation_real.json；模型直接提交 robot.run.input，
无需读外部示例 JSON。准确版本为 semantic-navigation-real@0.2.0。

```bash
./start-shell/run_skill.sh semantic-navigation-real@0.2.0
python src/skill/semantic_navigation_real/package_skill.py --output /tmp/semantic-navigation-real-0.2.0.zip
PYTHONPATH="../robot-skill:src:src/skill" PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -p no:cacheprovider -q src/skill/semantic_navigation_real/tests
```

ZIP 包含 SKILL.md、requirements.lock、scripts/ 和共享 solver/。打包不会发布或启动机器人。
离线测试覆盖偏移方向、原始输入不变、分支、携物判据、检查点恢复、动作幂等及独立 Worker。
