# YOLO 点云与相机标定

四个任务名和 taskType 保持不变。`generate_point_cloud`、`match_target`、`estimate_object_pose` 统一采用 `main_agi_fast.py` 的点云流程：掩码深度清理、反投影、变换到输出坐标系、KNN 去离群点、体素加权 DBSCAN、保留最大簇对应的原始点。

**行为变化：`center_robot` 是过滤后完整点云的包围盒中心，不再是采样点平均值。** 它表示可见点云的包围盒中心，并不保证等于物体真实几何中心。中心和边界先计算，再应用 `max_points`（默认 5000，0 表示不限制）。结果新增 `center_method`、`bounds_robot`、`filter_stats`，顶层包含实际变换及其来源。

## 配置相机

内外参统一填写到 `src/galaxea_r1pro_0/config/models.example.json` 的顶层 `cameras`，按实际 `sensor_id` 区分。YOLO 启动时默认读取这个文件，路径按代码位置定位，不依赖当前工作目录。文件中的 `defaults`、`profiles` 继续配置模型，`cameras` 配置相机。

也可通过环境变量 `MODEL_REGISTRY_PATH` 或 CR 的 `spec.config.modelRegistryPath` 指向另一份同结构 JSON，优先级为环境变量 > CR > 默认文件。CR 中原来的 `spec.config.cameras` 不再读取；应迁移到 JSON。任务参数仍可覆盖所选相机配置。JSON 在能力连接初始化时读取，修改后需重新加载能力实例。

所有 null 都必须替换；示例坐标系名称也必须换成设备实际名称。`cameras.example.yaml` 仅用于展示结构，不自动加载。旧代码中写死的内外参已删除。

每路相机必须配置：

- `camera_intrinsics`: `fx/fy/cx/cy`，对应实际 RGB-D 分辨率及对齐后的光学坐标系。
- `depth_scale`: 深度到米的比例，毫米深度为 0.001，米深度为 1。
- `camera_frame`: 相机光学坐标系。SDK 保留 ROS 图像头中的 frame_id；若图像头非空且与标定不一致则报错。
- `base_frame`、`robot_frame`: 默认均为 `base_link`。它们分别表示标定中的基座和实际输出坐标系；二者不同必须查询并执行坐标变换，不能只改标签。
- `camera_extrinsics`: 下面两种链路之一。

固定相机相对 base 不变时：

```yaml
camera_extrinsics:
  camera_to_base: [x_mm, y_mm, z_mm, rx_rad, ry_rad, rz_rad]
```

安装在手腕或活动头部上：

```yaml
camera_extrinsics:
  camera_to_tcp: [x_mm, y_mm, z_mm, rx_rad, ry_rad, rz_rad]
  tcp_frame: 实际安装座坐标系
```

这里 `tcp_frame` 可指手臂 TCP、法兰或头部安装链接，但必须与标定终点完全一致。组合顺序为 `T_base_tcp @ T_tcp_camera`。安装座到基座通过 SDK 每次重新读取；头部相机无需经过手臂 EEF。

兼容原来的 `camera_to_tcp + tcp_to_base`，后者必须是调用方提供的当次快照。也可直接传 `transform_camera_to_robot: {position: [x_m,y_m,z_m], orientation: [qx,qy,qz,qw]}`；这个完整变换直接到输出坐标系，不再乘 TCP 或 base 变换。

三种输入互斥。`camera_to_base` 下不要保留 AGIBOT 示例中的占位 `tcp_to_base: [0,0,0,0,0,0]`。任务参数覆盖相机配置时，外参整组替换，避免旧标定残留。单独覆盖 fx/fy/cx/cy 时必须提供全部四个值。

默认 TF 等待上限为 `transform_timeout_s: 2.0`，动态边最近更新允许间隔为 `transform_max_age_s: 1.0` 秒，静态边不失效。SDK 合并 `/tf` 和 `/tf_static`，按动态边本机接收时间判断更新间隔，重复同一 ROS 时间戳不算新更新。它不提供历史 TF 插值，也不保证 ROS 时钟与图像时间同步。**首版要求停稳后采集。** 断链、过期、标定缺失都明确报错，不退回假外参。

## 多路图像话题

SDK RobotProfile 的 `options.camera_topics` 配置各相机的压缩 RGB 和对齐深度话题；Ability 的 `sensor_id` 与这里的键对应：

```json
{"options":{"camera_topics":{
  "left_wrist_camera":{
    "rgb8":"/实际腕部相机/color/image_raw/compressed",
    "32FC1":"/实际腕部相机/aligned_depth_to_color/image_raw/compressedDepth"
  }
}}}
```

SDK 保留默认 head_camera 映射；新增相机不会改写全局映射。当前接口沿用 JPEG RGB 和 compressedDepth 解码格式，两个图像必须已经对齐，不能仅凭尺寸相同认定完成对齐。图像本机接收时间默认不得超过 1 秒（RobotProfile `options.image_max_age_s`）；两帧有 ROS 时间戳时检查相差不超过 0.1 秒，否则用接收时间检查。没有时间元数据的自定义后端须由调用方保证同步。

## 点云参数与参考图

可在相机配置或任务输入中传 `point_cloud_filter`，默认：

```yaml
point_cloud_filter:
  mode: fast_voxel
  voxel_size_m: 0.01
  dbscan_eps_m: 0.15
  min_samples: 60
  neighbors: 10
  std_ratio: 5.0
  min_cluster_ratio: 0.10
  fast_fallback: true
```

快速聚类失败时回退普通 DBSCAN，并在 `filter_stats` 写出原因；`mode: legacy_dbscan` 用于对照。普通 DBSCAN 全部判作噪点时沿用参考脚本行为，保留 KNN 后的点，不能把这类结果理解为成功找到物体簇。`refine_mask` 默认 true，深度带宽按 0.2 米换算到输入深度单位。

姿态估计支持两种参考来源：

1. `reference_points`：米制 N×3 点云；必须声明 `reference_frame` 与目标 `robot_frame` 相同。
2. 参考 RGB-D：使用独立的 `reference_camera` 对象（与单路相机配置格式相同），必须提供拍摄参考图时的完整静态外参或 TCP 快照，禁止读取当前手腕 TF。`reference_dir` 默认指向随包的 reference 图片目录。

参考图的深度比例与内参也来自 `reference_camera`。删除了旧的固定参考目标坐标；图中多个候选物体时，需提供 `reference_target_point` 或能唯一选中物体的 `reference_class_id/reference_class_name`。指定类别不存在时不会改选其他类别。直接输入参考点云和图像生成的参考点云都经过同一过滤流程，姿态匹配仍沿用现有的单轴旋转搜索。

这是外参接口迁移：旧调用依赖默认标定、默认 `body` 标签或旧参考图默认标定时，必须先补齐配置。`yolo_detect` 纯检测任务不要求外参。安装 Ability 的 Galaxea requirements（新增 scikit-learn），并同时更新本次 SDK 改动；保持模型路径与机器人连接配置由现场提供。新增同步关系检查不会启动、使能或移动执行器。
