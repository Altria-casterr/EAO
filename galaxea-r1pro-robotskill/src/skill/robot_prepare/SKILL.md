---
name: robot-prepare
description: 将 R1 Pro 真机设置为搬箱前预备状态：躯干四关节回零、双臂到达指定七关节预备角。严格按两个阶段串行执行。
category: robot_skill
version: 0.1.2
runtime:
  api_version: 1
  python: ">=3.11"
  entrypoint: scripts.skill:run
  stop_entrypoint: scripts.skill:on_stop
  input_model: scripts.models:PrepareInput
  state_model: scripts.models:PrepareState
  result_model: scripts.models:PrepareResult
  controllers: {}
required_actions:
  - { type: torso.move_to_posture_all, schema_version: 2 }
  - { type: motion.move_to_posture_both, schema_version: 2 }
stop_actions:
  - { type: torso.move_to_posture_all, schema_version: 2 }
  - { type: motion.move_to_posture_both, schema_version: 2 }
---

# 机器人预备姿态

## 模型调用条件

在用户要求机器人回到搬箱前空手预备状态时使用本 Skill，当前源码版本为
`robot-prepare@0.1.2`。本 Skill 不应作为携箱时的恢复动作，没有碰撞规划或自动回退。
严格串行执行：先躯干四关节回零，再将双臂移动到输入指定的七关节预备角。

各阶段等待 Ability 闭环到位成功后才开始下一阶段。失败或状态未知即结束；
不重新下发不确定动作，不自行复位或进入抓取 Skill。
检查点保存动作和结果，恢复时以同一 Action key 获取既有结果。
停止入口不声称机器人已停，停止确认依赖 Pilot/Ability；双臂 Ability
目前主执行记录仅关联左臂句柄，停止时需确认两侧状态。

## 输入

### 参数构造规则

本文件是模型构造本 Skill 输入的统一说明入口。先调用
`robot.get(skill_name="robot-prepare")`，确认当前唯一安装启用的准确版本并读取
`skill_contract.documentation`；如存在多个启用版本，先核对执行版本再显式提供
`skill_version`，不得选 Registry latest。如工具仅返回头尾预览，必须按返回的
artifact 引用读取完整正文。使用本文完整示例中的现场配置，不再从
`/workspace/inputs/robot_prepare.example.json`、其他输入 JSON、历史调用或其他版本文档补值。
用户明确指定的参数优先，但必须符合当前输入契约；冲突时报告具体冲突，
不得自行改值。没有明确覆盖要求时，不自行修改预备角、速度、容差或超时。

输入模型为 `scripts.models:PrepareInput`，顶层仅接受 `torso`、`arms`。
程序允许输入 `{}`，但它会选择程序默认臂姿，**与下方当前现场预备角不同**；
当前现场调用应显式填写完整示例，不能以 `{}` 代替。
关节角及容差单位为弧度，躯干关节速度为弧度/秒，超时和轮询间隔为秒。

### 完整输入示例

以下现场配置沿用 2026-10-26 成功执行的预备输入，补齐实际采用的默认关节名称。
所有输入字段均显式列出；双臂七个角度与各自七个关节名称按索引一一对应。
将整体作为 `robot.run.input` 提交，无须拼接目标物体或地图字段。

```json
{
  "torso": {
    "positions": [0, 0, 0, 0],
    "speeds": [0.5, 0.8, 0.7, 0.7],
    "timeout": 30.0,
    "tol": 0.02,
    "loop_period": 0.1
  },
  "arms": {
    "left_positions": [0.727, 0.039, 0.085, -1.954, 0.097, -0.354, -0.472],
    "right_positions": [0.727, -0.039, -0.085, -1.954, -0.097, -0.354, 0.472],
    "left_joint_names": [
      "left_arm_joint1", "left_arm_joint2", "left_arm_joint3", "left_arm_joint4",
      "left_arm_joint5", "left_arm_joint6", "left_arm_joint7"
    ],
    "right_joint_names": [
      "right_arm_joint1", "right_arm_joint2", "right_arm_joint3", "right_arm_joint4",
      "right_arm_joint5", "right_arm_joint6", "right_arm_joint7"
    ],
    "timeout": 30.0,
    "tol": 0.01,
    "loop_freq": 0.2
  }
}
```

### 地图参数与现场参数的边界

本 Skill 没有目标物体，也没有必需的地图输入。按上述现场配置执行时不需要查询
`real_map`，不能为了“与地图一致”而自动替换预备姿态。

| Skill 输入字段 | 默认构造来源 | 与 real_map 的关系 |
|---|---|---|
| `torso.positions` | 固定 `[0,0,0,0]` | 代码要求四个零，禁止映射源箱或目的位的 `properties.torso_joint_positions` |
| `torso.speeds` | 本文现场配置 `[0.5,0.8,0.7,0.7]` | 默认不读地图；只有用户明确要求从指定实体取速度时，才映射该实体的 `properties.torso_speeds` |
| `arms.left_positions/right_positions` | 本文现场预备角 | 不在地图中；不从箱体中心、高度、层号或抓取侧推导 |
| `arms.left_joint_names/right_joint_names` | 本文 R1 Pro 逐关节名称 | 不在地图中；与输入角度顺序一致 |
| 两侧 `timeout/tol`、`torso.loop_period`、`arms.loop_freq` | 本文现场配置 | 不映射地图属性 |

如果用户明确要求从地图取 `torso.speeds`，先在当前 `real_map` generation 中
查到指定的唯一 active 实体，按原顺序读取四个有限正速度值；缺失、非法或目标
不唯一时报告具体问题，不猜测、不静默回退。即使该实体有非零躯干角，
也不能将它们传给本 Skill 的 `torso.positions`。
源箱和目的位目前可能具有相同速度值，这不意味着预备技能需要绑定某个箱子。
地图保持只读。

`target`、`frame_id`、`grasp_axis`、`mode`、`torso.enabled`、
`semantic_flag` 均不属于当前输入契约，不得添加。双臂每侧始终需要七个非空且
不重复的实际关节名称；不接受单个 `left_arm` / `right_arm` 臂组名称。
只有明确的现场配置变更才覆盖关节名称或角度，不能靠模型推算新预备姿态。

### 输入参数含义与程序默认值

| 字段 | 含义及本文现场配置 | 程序默认值或约束 |
|---|---|---|
| `torso.positions` | 躯干四关节回零目标 `[0,0,0,0]` | 默认同值，四项均受 `Literal[0]` 约束 |
| `torso.speeds` | 按四关节顺序给定速度 `[0.5,0.8,0.7,0.7]` | 默认同值；模型要求四个有限数，现场使用上述正速度 |
| `torso.timeout` | 躯干闭环到位超时 `30.0` 秒 | 默认同值，正数 |
| `torso.tol` | 躯干到位角度容差 `0.02` rad | 默认同值，正数 |
| `torso.loop_period` | 躯干闭环轮询间隔 `0.1` 秒 | 默认同值，正数 |
| `arms.left_positions` | 左臂七关节预备角，见完整示例 | 默认 `[0.857,0.159,0.010,-2.074,0.127,-0.360,-0.453]`，与现场配置不同 |
| `arms.right_positions` | 右臂七关节预备角，见完整示例 | 默认 `[0.855,-0.129,-0.035,-2.073,-0.127,-0.354,0.492]`，与现场配置不同 |
| `arms.left_joint_names` | 左臂七个逐关节名称，见完整示例 | 默认 `left_arm_joint1` 至 `left_arm_joint7` |
| `arms.right_joint_names` | 右臂七个逐关节名称，见完整示例 | 默认 `right_arm_joint1` 至 `right_arm_joint7` |
| `arms.timeout` | 双臂闭环到位超时 `30.0` 秒 | 默认同值，正数 |
| `arms.tol` | 双臂到位角度容差 `0.01` rad | 默认同值，正数 |
| `arms.loop_freq` | 双臂闭环轮询间隔 `0.2` 秒 | 默认同值，正数；虽名为 freq，当前 Ability 按 sleep 秒数使用，不是 Hz |

超时不是运动速度设置。双臂没有可传入的 `speed` 或力矩字段，不因更改超时而
改变底层速度或力矩。`torso` 和 `arms` 均没有跳过阶段的输入开关。

## Action 参数映射

| 阶段 | ActionType@schemaVersion | Action.parameters |
|---|---|---|
| `torso` | `torso.move_to_posture_all@2` | 完整 `torso` 对象：positions、speeds、timeout、tol、loop_period |
| `arms` | `motion.move_to_posture_both@2` | 完整 `arms` 对象：两侧 positions、两侧 joint_names、timeout、tol、loop_freq |

控制器将对应子对象序列化后原样作为 Action.parameters，不计算新的关节角。
每阶段 Action 等待预算为该阶段输入 `timeout + 15` 秒；示例中均为 45 秒。
只有两个阶段均成功，Skill 才返回预备完成。

## 部署和运行

需要 TorsoControl.Demo、JointControl.Demo 两个 Running 实例。
JointEEFPoseControl.Demo 不提供本 Skill 所需的关节控制动作。
若尚未激活 JointControl，可在框架已安装 joint.control.demo 包时创建实例：

```bash
curl --fail -X POST http://127.0.0.1:8080/api/instance \
  -H 'Content-Type: application/json' \
  -d '{"template":"joint-control-demo","start":true,"connect":true}'
```

创建返回的是异步任务，不代表已经 Running；随后用仅检查模式确认。
Skill 遵循现有 JointControl 的名称与角度等长要求，无需修改 Ability。
若已经有 Running 实例，不要重复创建。

本地运行器使用仓库的 `src/config/prepare-deployment.yaml` 与 `inputs/robot_prepare.json`。
输入文件应保存按本文构造的完整对象；它只是本地调用载体，不是模型的额外参数来源。
通过 `robot.run` 执行时直接提交 `input`，无需读取该 JSON 文件。
运行器读取心跳按确切 Ability 名称选择唯一 Running 实例，写入本次日志目录
中的 Deployment 快照；不激活能力或自动执行后续抓取。

```bash
cd /home/morefine/ws/SEMF/workspace/semantic-skill/galaxea-r1pro-robotskill
PREPARE_CHECK_ONLY=1 bash start-shell/run_robot_prepare_skill.sh
bash start-shell/run_robot_prepare_skill.sh
```

也可传入自定义 Deployment 与输入文件两个位置参数。仅检查模式不启动 Skill。
日志保存在 `/tmp/robot-prepare.XXXXXX`。此流程不依赖相机、地图或 YOLO。
