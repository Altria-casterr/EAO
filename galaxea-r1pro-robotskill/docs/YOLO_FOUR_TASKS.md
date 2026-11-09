# YOLO 四个 task 测试 skill

复用 `yolo-detect-native@0.1.0` 的四种 mode，通过 semantic-pilot 调用真正的 Ability。
测试在 sam 上运行，使用已保存的同一组 RGB-D，不读取机器人实时相机或 TF。

```bash
cd /home/sam/workspace/semantic/semantic-skill/real_galaxea_robotskill
./start-shell/run_yolo_four_tasks.sh all
```

脚本自动启动或复用隔离的 8082 回放框架，确认唯一活动 YOLO 实例为 `YoloDetect.Replay`。

单独测试：

```bash
./start-shell/run_yolo_four_tasks.sh detect
./start-shell/run_yolo_four_tasks.sh point_cloud
./start-shell/run_yolo_four_tasks.sh match
./start-shell/run_yolo_four_tasks.sh pose
```

| 命令 | Ability task | 验证内容 |
|---|---|---|
| detect | yolo_detect | 返回检测标签，当前保存帧 19 个 |
| point_cloud | generate_point_cloud | 仅完整灰箱，取距离相机最近的两个，输出 body 坐标和距离 |
| match | match_target | 在保留的点云候选中匹配中间箱子 |
| pose | estimate_object_pose | 保存的参考 RGB-D 与目标做姿态估计，检查 success 和 IoU |

四份可编辑输入在 `inputs/yolo_tasks_replay/`。默认 `gray_box_only=true`、`top_k=2`。
距离是有效点到相机原点三维距离的中位数，不是 Z 深度。
match/pose 的目标点已经在 body 下，关闭 TF 转换。它们是保存帧的测试坐标，不能直接用于其他现场图像。
pose 的 reference_camera 是参考图拍摄时的标定快照，参考目录已在 sam 上准备。

也可以提供自己的单项输入（仍走回放）：

```bash
./start-shell/run_yolo_four_tasks.sh point_cloud /绝对路径/input.json
```

整套测试会从当次点云结果选择匹配目标；单项使用 JSON 中固定的保存帧目标。
全部测试还检查本次已完成调用的 GetExecution/StopExecution。
单项结果保存在 `outputs/yolo-detect.*/`；整套结果保存在 `outputs/yolo-suite-*/`，包括 summary.json、输入、result.json、events.jsonl 和日志。失败返回非零退出码。

耗时为单次流程耗时，含 Pilot/框架开销，不能作为平均推理耗时或严格的加速对照。
