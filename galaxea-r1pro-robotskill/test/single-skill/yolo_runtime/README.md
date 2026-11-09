# YOLO Pilot 测试 Skill

本机：`/home/ww/workspace/skill/test/yolo_skill_review/real_galaxea_robotskill`
sam：`/home/sam/workspace/semantic/semantic-skill/real_galaxea_robotskill`

## 本次实测（2026-09-27）

- robot Python 环境已安装 torch 2.5.1+cpu、torchvision 0.20.1+cpu、ultralytics 8.4.163、scikit-learn 1.9.1；依赖检查通过。
- 权重：`/home/sam/下载/yolo11x-seg_for_WAIC_0715.pt`，CPU，imgsz=640。
- 真实检测通过完整 skill → Pilot → Ability → SDK → 172.16.70.147 相机链路，返回 19 个检测结果。对应 sam `outputs/yolo-detect.NwOrS3hM/result.json`。
- 地图目标 `[2.671, 0.512, 1.21]` 米的匹配失败：没有可用的 map → base_link TF。另有相机到机器人外参尚未补齐，不能据此声称三维匹配通过。
- 测试未发送机器人运动命令。
- 结果已复制到本地 `real_test_20260927/`。其中 `rgb-detection.jpg` 来自另外一次原始 RGB 诊断推理，不是 skill 运行的同一帧。
- 本机 26 项离线测试通过，覆盖 skill 输入、失败传播、坐标转换、RGB/深度解码等。

## sam 执行

```bash
cd /home/sam/workspace/semantic/semantic-skill/real_galaxea_robotskill
./start-shell/run_yolo_detect_skill.sh --check
./start-shell/run_yolo_detect_skill.sh
# 补齐 map → base_link 和相机外参之后：
./start-shell/run_yolo_detect_skill.sh inputs/yolo_detect_match.json
```

其他输入：`inputs/yolo_detect_point_cloud.json`、`inputs/yolo_detect_both.json`。
默认 detect 获取 RGB-D 并做分割，不返回完整 mask。point_cloud 返回物体中心和过滤统计。both 分别采集两次。检测到零个物体是否算失败由 require_objects 决定。
结果和事件写入 `outputs/yolo-detect.XXXXXXXX/`；任意步骤失败时脚本返回非零退出码。

## 坐标转换

`yolo_detect_native/scripts/coordinates.py` 订阅 `/tf` 和 `/tf_static`，查询从输入 frame 到检测 frame 的变换，按 `point @ R.T + t` 计算。输入和 Ability 都使用米，不乘 1000。

`inputs/yolo_detect_match.json` 设置 `target_input_frame=map`、`target_frame=base_link`、`trans_flag=true`。TF 缺失或过期时，skill 返回失败，不提交 match_target。禁止将 body 自动当成 base_link。

点云和匹配读取运行实例的 `models.example.json` 顶层 cameras。已填现场 head_camera 内参，外参仍为空；需要真实标定。当前 pinhole 投影也未处理非零畸变，不能将本次检测通过等同于定位精度验证。

## 运行目录与配置

sam 主 Ability 仓库当前在 `fix/initial-import`，没有切换用户分支。测试使用 `test/single-skill/yolo_runtime` 下的源代码副本（不是新 Git 分支）：

- `test/single-skill/yolo_runtime/src/galaxea_r1pro_0/config/models.example.json`：真实权重、相机内参和待补外参。
- `test/single-skill/yolo_runtime/src/galaxea_r1pro_0/config/r1pro-yolo-test.json`：真实 rosbridge 和图像话题配置。
- `test/single-skill/yolo_runtime/framework`：独立测试框架，127.0.0.1:8080。
- `test/single-skill/yolo_runtime/src/galaxea_r1pro_0/abilities/yolo_detect`：包含三项 perception 绑定和完整 spec.tasks 的测试包。

现场压缩 RGB 消息 data 为空，改用 `/camera/camera/color/image_raw`；深度用 `/camera/camera/aligned_depth_to_color/image_raw`。SDK 增加按配置选用 sensor_msgs/Image 和原始 RGB/深度解码，保留默认压缩话题行为。原始数据的通道顺序、字节序和行跨度有测试。

SDK 修改文件（本机与 sam 同步）：
- `backends/real.py`：每实例相机话题配置副本，以及可配置消息类型。
- `utils/utils_ImageCapture.py`：原始 RGB/BGR、16UC1/32FC1 解码。

SDK 文件原始备份位于 sam `test/single-skill/yolo_runtime/*.before-raw-depth`。未提交或推送 Git。

## Skill 文件

- `SKILL.md`：允许的 perception 动作，physical=false。
- `scripts/models.py`：参数和结果校验。
- `scripts/controller.py`：构造检测、点云、目标匹配请求。
- `scripts/coordinates.py`：TF 查询和目标点转换。
- `scripts/skill.py`：执行、检查点、失败记录；停止入口不发新任务。
- `start-shell/run_yolo_detect_skill.sh`：环境检查和 Pilot 执行。

```bash
cd /home/sam/workspace/semantic/semantic-skill/real_galaxea_robotskill
PYTHONPATH= /tmp/yolo-test-env/bin/python -m pytest -q test/single-skill/yolo_runtime/src/galaxea_r1pro_0/tests
```
