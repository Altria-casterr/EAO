# Galaxea R1 Pro Skill 测试路径

以下路径以 `/home/morefine/ws/SEMF/workspace` 为工作区根。业务测试使用
`semantic-skill/galaxea-r1pro-robotskill/src/skill`；单 Action Skill 位于该仓库
`test/single-skill`，用于错误审查。

| 用途 | 工作区内路径 |
| --- | --- |
| Ability 源码、manifest、CR | `semantic-ability/galaxea-r1pro-ability/src/galaxea_r1pro_0` |
| 普通真机配置 | `semantic-ability/galaxea-r1pro-ability/src/galaxea_r1pro_0/config/r1pro-real.json` |
| YOLO 真机配置 | `semantic-ability/galaxea-r1pro-ability/src/galaxea_r1pro_0/config/r1pro-real-yolo.json` |
| 真机模型与相机标定 | `semantic-ability/galaxea-r1pro-ability/src/galaxea_r1pro_0/config/models.local.json` |
| 独立 Robot SDK | `semantic-robotsdk/galaxea-r1pro-sdk/src` |
| Skill Worker SDK | `semantic-skill/robot-skill` |
| Pilot | `semantic-framework/.output/bin/semantic-pilot` |
| 独立 AbilityFramework | `semantic-ability/ability-runtime/AbilityFramework` |
| Skill 部署配置 | `semantic-skill/galaxea-r1pro-robotskill/src/config/*deployment.yaml` |
| Skill 现场输入 | `semantic-skill/galaxea-r1pro-robotskill/inputs/*.json` |

启动脚本从自身位置解析工作区路径，保留已有环境覆盖变量。prepare 与放置默认
Python 为工作区 `.venvs/galaxea-skill/bin/python`。YOLO 默认 Python 为
`/home/morefine/anaconda3/envs/Ability/bin/python`，可用 `RUN_PYTHON` 或
`ABILITY_PYTHON` 覆盖；双臂抓取使用 `GRASP_PYTHON`。

备用真机 Ability 的 `bin/ability` 默认加载 `galaxea-r1pro-ability/src`，
也可用 `GALAXEA_ABILITY_ROOT` 指定能力仓库。普通 CR 使用普通真机配置，YOLO CR
及包级环境配置使用新的 YOLO profile 和 `models.local.json`。

部署 YAML 的 `abilities.*.instance_id` 是框架真实实例绑定，不由文件夹名称生成。
本次只迁移文件引用；实际实例须再次检查 Running、已安装 manifest、CR、进程
`PYTHONPATH` 及 `ROBOT_PROFILE_PATH`，确认使用当前能力仓库。

`PREPARE_CHECK_ONLY=1` 只验证输入与实例绑定；该检查通过仍不代表已验证机器人
连接、运动范围或逐 Action 确认。prepare 真机测试按每个 Action 打印输入和输出、
等待用户确认的流程执行。临时审核入口的预检代码已更新能力路径，原报告是历史记录，
须重新预检后才能使用。

YOLO 保存帧回放继续使用 `test/single-skill/yolo_runtime/r1pro-yolo-replay.json`，
保持 `test_mode=true` 与本地禁用端点；回放框架二进制可用
`ABILITY_FRAMEWORK_BINARY` 覆盖。回放需要另行提供已打包的
`test/single-skill/yolo_runtime/yolo.detect.replay-1.0.0.zip` 和保存帧数据，
本次不构建或启动回放实例。

## 本次离线验证（2026-10-03）

- prepare、放置、双臂抓取、导航：200 项通过。
- 路径、CR、源码导入路径、启动参数、离线回放隔离：65 项通过。
- prepare 逐 Action 确认和日志记录的离线审核测试：4 项通过。
- YOLO Worker 显式加载独立 Robot SDK，TF 模块导入来源已核验。
- 全量业务测试：219 项通过、32 项失败，失败集中于单臂抓取；其代表性失败在
  修改前的 HEAD 中也可复现。本次没有修改其业务算法。
- 8080 框架只读查询返回 0 个 Ability 心跳实例，目前无法完成真机绑定检查。
- 未启动 Pilot 或 Ability 实例，未发送真机动作。
