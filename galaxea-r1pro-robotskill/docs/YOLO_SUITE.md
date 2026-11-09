# YOLO Skill 批量测试

当前使用 **保存的 RGB-D 回放**。四个业务任务均通过：

`run_yolo_skill_suite.sh → semantic-pilot → yolo_detect_native → YoloDetect.Replay`

模型是真实的用户 YOLO11x-seg 权重；检测、点云、匹配和姿态估计算法均为 Ability 原实现。只替换相机采集入口，返回同一次拍摄保存的 RGB 和深度；不连接机器人，不读取实时 TF。

## sam 执行

```bash
cd /home/sam/workspace/semantic/semantic-skill/real_galaxea_robotskill
/home/sam/miniconda3/envs/robot/bin/python test/start_yolo_replay_test.py
./start-shell/run_yolo_skill_suite.sh --replay \
  --reference test/single-skill/yolo_runtime/suite-reference-20260928-113928/reference.json
```

回放框架独立监听 `127.0.0.1:8082`。原来的实时采集测试实例已经停止。

源图：`outputs/yolo-overlay-20260928-104315/rgb.png`。
配对深度：同目录 `depth.npy`。

## 测试项

| 测试 | 调用方式 | 检查内容 |
|---|---|---|
| yolo_detect | skill / Pilot | 实际模型输出，必须有目标 |
| generate_point_cloud | skill / Pilot | 点云目标、body 坐标、实际外参 |
| match_target | skill / Pilot | 用本轮点云选出的中心作为目标，检查匹配成功 |
| estimate_object_pose | skill / Pilot | 参考 RGB-D 独立标定、坐标系、姿态结果、IoU、success |
| GetExecution | 框架代理控制接口 | 查询本轮 skill 创建的已完成 YOLO invocation |
| StopExecution | 框架代理控制接口 | 对同一个已完成、无运动 command_id 的 invocation 验证幂等停止 |

后两项是生命周期控制接口测试，不伪装成业务 skill 调用；不验证进行中的同步推理能否被中断。

默认回放没有 map 测试：body 目标来自回放点云，避免使用实时 TF。实时模式另有原始 map 目标测试；目前用户要求机器人供其他人使用，**不要运行实时模式**。

参考图和目标图是同一组已保存数据，因此姿态测试属于一致性/流程检查，不能证明外参准确或真实姿态估计精度。

## 结果与计时

每轮创建 `outputs/yolo-suite-时间-随机串/`：

- `summary.json`：每项状态、每次尝试耗时、重试记录、失败原因。
- `01_yolo_detect/` 至 `04_estimate_object_pose/`：实际输入、Pilot result.json、events.jsonl、控制台日志。
- `lifecycle-responses.json`：GetExecution/StopExecution 的请求和响应。
- `reference.json`：本轮参考标定快照。

`wall_seconds` 包含启动脚本、环境检查、Pilot/skill worker、Ability 执行和结果写出；回放模式不包含真实图像传输。`skill_elapsed_seconds` 是 skill 内坐标转换及 Action 调用耗时。

只对明确的相机超时/不同步有限重试，默认 3 次。每次尝试都保留结果，不能用最终成功掩盖之前失败。零目标默认判失败。没有参考图或目标点时明确标记 blocked，不填造坐标。任一项未通过返回非零退出码。

## 文件

- `start-shell/run_yolo_skill_suite.sh` / `test/test_yolo_skill_suite.py`：批量任务测试入口。
- `test/yolo_suite_lifecycle.py`：只针对本轮已完成的 YOLO 执行记录测试控制任务。
- `test/yolo_replay_main.py`：仅测试使用的采集适配；强制检查 test_mode=true。
- `test/start_yolo_replay_test.py`：启动已准备好的独立回放框架/包。
- `src/config/yolo-replay-deployment.yaml`：回放 Pilot 配置。
- `src/config/prepare_yolo_reference.py`：从历史采样的 RGB、uint16 深度和标定快照生成参考资料。
- `test/single-skill/yolo_detect_native/scripts/models.py`：补充 pose 输入和参考数据校验。
- `test/single-skill/yolo_detect_native/scripts/controller.py` / `scripts/skill.py`：pose Action、结果校验和逐项计时。
- Ability manifest：补充 `perception.estimate_object_pose`，四个业务任务都声明 physical=false。

测试部署资料位于 sam `test/single-skill/yolo_runtime/replay-package/`、`models.replay.json`、`r1pro-yolo-replay.json`。所有修改前的 skill 文件备份在 `test/single-skill/yolo_runtime/before-suite-*`。没有创建 Git 分支，没有提交或推送。

## 本地测试

```bash
cd /home/sam/workspace/semantic/semantic-skill/real_galaxea_robotskill
PYTHONPATH= /tmp/yolo-test-env/bin/python -m pytest -q test/single-skill/yolo_runtime/src/galaxea_r1pro_0/tests
```

42 项离线测试通过，覆盖输入、结果、坐标转换、pose 失败传播、停止调用限定范围和解码。

## 2026-09-28 最终回放结果

四个业务任务都经 skill/Pilot 成功执行，且均为第一次尝试通过：

| 任务 | 脚本全程耗时 | 结果 |
|---|---:|---|
| yolo_detect | 1.402 秒 | 19 个检测目标 |
| generate_point_cloud | 2.365 秒 | 19 个三维候选 |
| match_target | 2.170 秒 | 匹配成功 |
| estimate_object_pose | 4.842 秒 | yaw=-0.4663°，IoU=0.984496 |

GetExecution 与对已完成记录的 StopExecution 也通过。模型已预热，表格是单轮结果，不是多轮平均；无真实图像传输和实时 TF 耗时。

sam 结果：`outputs/yolo-suite-20260928-115015-306667/summary.json`。
本地结果：`suite_results_20260928/yolo-suite-20260928-115015-306667/summary.json`。

先前调试失败记录仍保留在 sam：首次实时实例未就绪、采集不同步/权重配置问题，以及首轮回放生命周期返回 payload 未解析。最终修复的是测试端的 payload 解包；未修改 Ability 控制接口实现。

## 标签筛选与 Top-K（2026-09-28）

Skill 和 Ability 四个业务 task 接受两个新参数：

```json
{"gray_box_only": true, "top_k": 2}
```

- `gray_box_only`：严格布尔值，默认 `false`。开启时仅 `gray_box` 参与点云生成，排除 `gray_box_p` 及其他类别。
- `top_k`：正整数，默认 `2`。先筛标签，再精炼 mask，排除无有效深度的 mask，利用内参计算有效像素对应三维点到相机原点的距离，按距离中位数升序选前 K 个，最后反投影、变换和过滤点云。相同深度按原始检测序号排序；不足 K 个则全部处理。距离指标是 `median(sqrt(X²+Y²+Z²))`（相机坐标系、米），不是光轴深度 Z，也不是到机器人原点的距离。输出 `camera_distance_m` 可核对排序，`median_depth_m` 仅作深度参考。
- 纯检测 task 不生成点云，保留全部 YOLO 标签；其余三个 task 应用筛选。姿态估计的参考 RGB-D 也采用同样筛选，参考点应位于保留的物体上，并通过既有 XYZ 匹配阈值；超出阈值会明确报错。直接传入的参考点云不涉及 mask 筛选。
- `top_k` 限制物体数；既有 `max_points` 限制每个物体返回的点数，两者不同。

在 sam 使用已保存 RGB-D 跑四个 task：

```bash
cd /home/sam/workspace/semantic/semantic-skill/real_galaxea_robotskill
./start-shell/run_yolo_skill_suite.sh --replay --gray-box-only --top-k 2 \
  --reference test/single-skill/yolo_runtime/suite-reference-20260928-113928/reference.json
```

回放只使用隔离的 8082 框架及本地保存的 RGB-D，不访问机器人相机或实时 TF。
