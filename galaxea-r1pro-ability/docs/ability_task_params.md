# Ability 清单总览

本文档按 **ability → taskType → 输入参数** 结构整理所有能力。

## Ability 总览

### taskType 与 actionType 对照

#### 1. Gripper.Demo

| taskType | taskName | actionType |
|---|---|---|
| 0 | HandleSingleGripper | `gripper.set_opening_single` |
| 1 | HandleBothGripper | `gripper.set_opening_both` |
| 2 | GetExecution | — |
| 3 | StopExecution | — |

#### 2. JointControl.Demo

| taskType | taskName | actionType |
|---|---|---|
| 0 | control_single_arm_joint | `motion.move_to_posture_single` |
| 1 | control_both_arms_joint | `motion.move_to_posture_both` |
| 2 | GetExecution | — |
| 3 | StopExecution | — |

#### 3. JointEEFPoseControl.Demo

| taskType | taskName | actionType |
|---|---|---|
| 0 | control_single_arm | `motion.move_end_effector_single` |
| 1 | control_both_arm | `motion.move_end_effector_both` |
| 2 | force_align | `manipulation.force_align` |
| 3 | control_single_arm_trajectory | `motion.move_end_effector_single_trajectory` |
| 4 | control_both_arms_trajectory | `motion.move_end_effector_both_trajectory` |
| 5 | GetExecution | — |
| 6 | StopExecution | — |

#### 4. TorsoControl.Demo

| taskType | taskName | actionType |
|---|---|---|
| 0 | control_torso_joints | `torso.move_to_posture_all` |
| 1 | control_single_torso_joint | `torso.move_to_target_pos_single` |
| 2 | GetExecution | — |
| 3 | StopExecution | — |

#### 5. WooshMove.Demo

| taskType | taskName | actionType |
|---|---|---|
| 0 | move_robot | `navigation.relative_move` |
| 1 | stop | `navigation.stop` |
| 2 | GetExecution | — |
| 3 | StopExecution | — |

#### 6. WooshNavigation.Demo

| taskType | taskName | actionType |
|---|---|---|
| 0 | navigate_to_map_pose | `navigation.follow_route` |
| 1 | stop | `navigation.stop` |
| 2 | verify_arrival | `navigation.verify_arrival` |
| 3 | GetExecution | — |
| 4 | StopExecution | — |

#### 7. RobotObservation.Demo

| taskType | taskName | actionType |
|---|---|---|
| 0 | get_robot_state | `robot.get_state` |
| 1 | get_sensors_state | `robot.get_sensors_state` |
| 2 | capture_rgbd | `robot.capture_rgbd` |
| 3 | GetExecution | — |
| 4 | StopExecution | — |

#### 8. YoloDetect.Demo

| taskType | taskName | actionType |
|---|---|---|
| 0 | generate_point_cloud | — |
| 1 | yolo_detect | — |
| 2 | match_target | — |
| 3 | estimate_object_pose | — |
| 4 | GetExecution | — |
| 5 | StopExecution | — |

> 所有 ability 都含两个通用任务：`GetExecution`（查询执行记录）和 `StopExecution`（停止执行），本文档在末尾统一说明，各 ability 中不重复展开。
> YoloDetect.Demo 的任务未在 manifest 中声明 `actionType` 字段。

---

## 1. Gripper.Demo

### taskType 0 — HandleSingleGripper
**summary**：控制单侧夹爪移动到目标开度（含反馈闭环）

| 参数 | 类型 | 必填 | 默认值 | 说明 |
|---|---|---|---|---|
| hand | string | 否 | left | 夹爪侧，left / right |
| position | float | 否 | 50.0 | 目标开度（百分制 0-100） |
| speed | float | 否 | 0.5 | 夹爪运动速度（0.0-1.0） |
| effort | float | 否 | 0.3 | 夹爪力矩限制（0.0-1.0） |
| tol | float | 否 | 5.0 | 到位容差（开度百分制差值） |
| timeout | float | 否 | 10.0 | 运动超时（秒） |
| loop_period | float | 否 | 0.1 | 状态轮询间隔（秒） |

### taskType 1 — HandleBothGripper
**summary**：控制左右夹爪同时移动到目标开度（含双向反馈闭环）

| 参数 | 类型 | 必填 | 默认值 | 说明 |
|---|---|---|---|---|
| hand | string | 否 | both | 控制模式，仅支持 both |
| left_position | float | 否 | 50.0 | 左夹爪目标开度（0-100） |
| right_position | float | 否 | 50.0 | 右夹爪目标开度（0-100） |
| left_speed | float | 否 | 0.5 | 左夹爪速度（0.0-1.0） |
| right_speed | float | 否 | 0.5 | 右夹爪速度（0.0-1.0） |
| left_effort | float | 否 | 0.3 | 左夹爪力矩（0.0-1.0） |
| right_effort | float | 否 | 0.3 | 右夹爪力矩（0.0-1.0） |
| tol | float | 否 | 5.0 | 左右共用到位容差 |
| timeout | float | 否 | 10.0 | 左右共用超时（秒） |
| loop_period | float | 否 | 0.1 | 左右共用轮询间隔（秒） |

### taskType 2 — GetExecution（通用）
### taskType 3 — StopExecution（通用）

---

## 2. JointControl.Demo

### taskType 0 — control_single_arm_joint
**summary**：控制单臂关节移动到目标位置（发布目标后轮询实际关节位置直到到位或超时）

| 参数 | 类型 | 必填 | 默认值 | 说明 |
|---|---|---|---|---|
| arm_side | string | 否 | left | 手臂侧，left / right |
| positions | array[float] | 是 | — | 目标关节角度数组（弧度） |
| joint_names | array[string] | 是 | — | 关节名称数组，与 positions 等长 |
| velocities | array[float] | 否 | — | 关节运动速度数组 |
| effort | array[float] | 否 | — | 关节力矩限制数组 |
| tol | float | 否 | 0.01 | 到位容差（弧度） |
| timeout | float | 否 | 10.0 | 超时（秒） |
| loop_freq | float | 否 | 0.2 | 轮询间隔（秒） |

### taskType 1 — control_both_arms_joint
**summary**：控制双臂关节同时移动到目标位置（左右并行发布，两侧均到位才算成功）

| 参数 | 类型 | 必填 | 默认值 | 说明 |
|---|---|---|---|---|
| left_positions | array[float] | 是 | — | 左臂目标关节角度（弧度） |
| right_positions | array[float] | 是 | — | 右臂目标关节角度（弧度） |
| left_joint_names | array[string] | 是 | — | 左臂关节名称 |
| right_joint_names | array[string] | 是 | — | 右臂关节名称 |
| left_velocities | array[float] | 否 | — | 左臂速度数组 |
| right_velocities | array[float] | 否 | — | 右臂速度数组 |
| left_effort | array[float] | 否 | — | 左臂力矩数组 |
| right_effort | array[float] | 否 | — | 右臂力矩数组 |
| tol | float | 否 | 0.01 | 左右共用容差（弧度） |
| timeout | float | 否 | 10.0 | 左右共用超时（秒） |
| loop_freq | float | 否 | 5.0 | 左右共用轮询间隔（秒） |

### taskType 2 — GetExecution（通用）
### taskType 3 — StopExecution（通用）

---

## 3. JointEEFPoseControl.Demo

### taskType 0 — control_single_arm
**summary**：控制单臂末端移动到目标位姿（frame_id 不一致时先做坐标系转换）

| 参数 | 类型 | 必填 | 默认值 | 说明 |
|---|---|---|---|---|
| axis | string | 否 | left | 执行臂侧，left / right |
| _target_pose | object | 是 | — | 目标位姿：target_x/y/z（米）+ x/y/z/w（四元数） |
| frame_id | string | 否 | torso_link4 | 目标位姿所在坐标系 |
| pos_tol | float | 否 | 0.05 | 位置容差（米） |
| orient_tol | float | 否 | 0.01 | 姿态容差（弧度） |
| timeout | float | 否 | 10.0 | 超时（秒） |
| loop_freq | float | 否 | 1.0 | 轮询间隔（秒） |

### taskType 1 — control_both_arm
**summary**：控制双臂末端同时移动到目标位姿

| 参数 | 类型 | 必填 | 默认值 | 说明 |
|---|---|---|---|---|
| left_target_pose | object | 是 | — | 左臂目标位姿（结构同 _target_pose） |
| right_target_pose | object | 是 | — | 右臂目标位姿 |
| left_frame_id | string | 否 | "1" | 左臂目标坐标系 |
| right_frame_id | string | 否 | "1" | 右臂目标坐标系 |
| left_pos_tol / right_pos_tol | float | 否 | 0.05 | 左右位置容差（米） |
| left_orient_tol / right_orient_tol | float | 否 | 0.01 | 左右姿态容差（弧度） |
| timeout | float | 否 | 10.0 | 共用超时（秒） |
| loop_freq | float | 否 | 1.0 | 共用轮询间隔（秒） |

### taskType 2 — force_align
**summary**：力反馈精调：末端沿指定轴按步长步进逼近，监测轴力变化触阈即停

| 参数 | 类型 | 必填 | 默认值 | 说明 |
|---|---|---|---|---|
| axis | string | 否 | left | 执行臂侧 |
| move_coordinate | string | 否 | "y" | 步进移动轴 x / y / z |
| force_coordinate | array[string] | 否 | [x,y,z] | 力反馈监测轴集合 |
| force_threshold_n | float | 是 | — | 力变化阈值（牛） |
| direction | string | 否 | negative | 步进方向 negative / positive |
| step_size_m | float | 否 | 0.01 | 单步步长（米） |
| loop_freq_s | float | 否 | 0.1 | 控制周期（秒） |
| timeout_s | float | 否 | 10.0 | 超时（秒） |

### taskType 3 — control_single_arm_trajectory
**summary**：按笛卡尔五次多项式轨迹逐点控制单臂末端

| 参数 | 类型 | 必填 | 默认值 | 说明 |
|---|---|---|---|---|
| axis | string | 否 | left | 执行臂侧 |
| _target_pose | object | 是 | — | 轨迹终点位姿 |
| frame_id | string | 否 | torso_link4 | 目标坐标系 |
| num_points | int | 否 | 100 | 轨迹插值点数 |
| speed | float | 否 | 0.05 | 末端线速度（米/秒，仅日志） |
| axis_order | string | 否 | zyx | 逐轴执行顺序 |
| sequential | bool | 否 | true | 是否逐轴顺序插值 |
| step_distance | float | 否 | 0.01 | 自动计算点数的步进距离（米） |
| max_num_points | int | 否 | 100 | 插值点数上限 |
| pos_tol | float | 否 | 0.05 | 单点位置容差（米） |
| orient_tol | float | 否 | 0.01 | 单点姿态容差（弧度） |
| timeout | float | 否 | 10.0 | 单点超时（秒） |
| loop_freq | float | 否 | 1.0 | 单点轮询间隔（秒） |

### taskType 4 — control_both_arms_trajectory
**summary**：按笛卡尔五次多项式轨迹同步控制双臂末端

| 参数 | 类型 | 必填 | 默认值 | 说明 |
|---|---|---|---|---|
| left_target_pose | object | 是 | — | 左臂轨迹终点位姿 |
| right_target_pose | object | 是 | — | 右臂轨迹终点位姿 |
| left_frame_id / right_frame_id | string | 否 | "1" | 左右目标坐标系 |
| left_axis_order / right_axis_order | string | 否 | zyx | 左右逐轴顺序 |
| left_sequential / right_sequential | bool | 否 | true | 左右是否顺序插值 |
| num_points | int | 否 | — | 轨迹点数（不传取左右建议值最大值） |
| speed | float | 否 | 0.05 | 末端线速度 |
| step_distance | float | 否 | 0.01 | 步进距离（米） |
| max_num_points | int | 否 | 100 | 点数上限 |
| left_pos_tol / right_pos_tol | float | 否 | 0.05 | 左右位置容差 |
| left_orient_tol / right_orient_tol | float | 否 | 0.01 | 左右姿态容差 |
| timeout | float | 否 | 10.0 | 单点超时（秒） |
| loop_freq | float | 否 | 1.0 | 单点轮询间隔（秒） |

### taskType 5 — GetExecution（通用）
### taskType 6 — StopExecution（通用）

---

## 4. TorsoControl.Demo

### taskType 0 — control_torso_joints
**summary**：控制躯干四关节同时移动到目标位置

| 参数 | 类型 | 必填 | 默认值 | 说明 |
|---|---|---|---|---|
| positions | array[float] | 是 | — | 躯干四关节目标位置（弧度） |
| speeds | array[float] | 否 | — | 躯干四关节速度（弧度/秒） |
| effort | array[float] | 否 | — | 力矩限制数组 |
| names | array[string] | 否 | — | 关节名称（透传底层 name 字段） |
| tol | float | 否 | 0.02 | 到位容差（弧度） |
| timeout | float | 否 | 10.0 | 超时（秒） |
| loop_period | float | 否 | 0.1 | 轮询间隔（秒） |

### taskType 1 — control_single_torso_joint
**summary**：控制躯干单个关节移动到目标位置（其余关节保持当前位置）

| 参数 | 类型 | 必填 | 默认值 | 说明 |
|---|---|---|---|---|
| joint_id | int | 否 | 0 | 关节编号（0-3） |
| target_pos | float | 是 | — | 目标关节位置（弧度） |
| speed | float | 否 | 1.5 | 目标关节速度（弧度/秒） |
| tol | float | 否 | 0.02 | 到位容差（弧度） |
| timeout | float | 否 | 8.0 | 超时（秒） |
| loop_period | float | 否 | 0.1 | 轮询间隔（秒） |
| effort | array[float] | 否 | — | 力矩限制数组 |
| names | array[string] | 否 | — | 关节名称 |

### taskType 2 — GetExecution（通用）
### taskType 3 — StopExecution（通用）

---

## 5. WooshMove.Demo

### taskType 0 — move_robot
**summary**：控制底盘沿指定轴相对运动（x/y/z，含到位闭环：定位优先 + 积分兜底）

| 参数 | 类型 | 必填 | 默认值 | 说明 |
|---|---|---|---|---|
| target_x | float | 否 | 0.0 | 前后轴相对位移（米） |
| target_y | float | 否 | 0.0 | 左右轴相对位移（米） |
| target_orientation | array[float] | 否 | [0,0,0,1] | 目标姿态四元数，仅取 yaw 分量 |
| speed | float | 否 | 0.5 | 运动速度 |
| tolerance | float | 否 | x/y=0.02, z=0.5 | 到位容差 |
| timeout | float | 否 | 10.0 | 超时（秒） |
| refresh_interval | float | 否 | 0.1 | 速度命令刷新间隔（秒） |
| settle_time | float | 否 | 0.2 | 停止后稳定等待时间（秒） |

> 约束：target_x / target_y / yaw(由 target_orientation 换算) 三者中恰好一个非零。

### taskType 1 — stop
**summary**：停止底盘运动（紧急停止 + 零速发布）

无业务参数（仅 invocation_id）。

### taskType 2 — GetExecution（通用）
### taskType 3 — StopExecution（通用）

---

## 6. WooshNavigation.Demo

### taskType 0 — navigate_to_map_pose
**summary**：导航到地图坐标系下的目标位置

| 参数 | 类型 | 必填 | 默认值 | 说明 |
|---|---|---|---|---|
| target_x | float | 否 | 0.0 | 目标位置 x（米） |
| target_y | float | 否 | 0.0 | 目标位置 y（米） |
| target_orientation | array[float] | 否 | [0,0,0,1] | 目标姿态四元数，仅取 yaw |
| target_point_type | int | 否 | 0 | 目标点类型（透传 SDK） |
| frame_id | string | 否 | map | 目标位姿所在坐标系 |
| timeout | float | 否 | 60.0 | 导航超时（秒） |

### taskType 1 — stop
**summary**：停止所有运动与导航（紧急停止置位 + 零速发布）

无业务参数。

### taskType 2 — verify_arrival
**summary**：按腕部力传感检查手中是否仍有搬运物体

| 参数 | 类型 | 必填 | 默认值 | 说明 |
|---|---|---|---|---|
| tool_refs | array[string] | 否 | — | 检查侧别列表 [left]/[right]/[left,right] |
| carrying_force_axis | string | 否 | z | 负载判定轴 x/y/z/norm |
| carrying_min_load_n | float | 否 | 3.0 | 负载阈值（牛） |

### taskType 3 — GetExecution（通用）
### taskType 4 — StopExecution（通用）

---

## 7. RobotObservation.Demo

### taskType 0 — get_robot_state
**summary**：读取机器人状态单项观测

| 参数 | 类型 | 必填 | 默认值 | 说明 |
|---|---|---|---|---|
| observation | string | 是 | — | 观测项：snapshot / torso_status / floating_base / transformer_martix |

### taskType 1 — get_sensors_state
**summary**：读取传感器数据单项观测

| 参数 | 类型 | 必填 | 默认值 | 说明 |
|---|---|---|---|---|
| observation | string | 是 | — | 观测项：ee_pose_left / ee_pose_right / gripper_state_left / gripper_state_right / joint_state_left / joint_state_right / force_data / torso_positions / chassis_feedback / localization |

### taskType 2 — capture_rgbd
**summary**：采集指定传感器的 RGB 与/或深度图像

| 参数 | 类型 | 必填 | 默认值 | 说明 |
|---|---|---|---|---|
| sensor_id | string | 是 | — | 传感器 ID，如 camera_head |
| axis | string | 否 | rgb | 采集通道 rgb / depth / both |

### taskType 3 — GetExecution（通用）
### taskType 4 — StopExecution（通用）

---

## 8. YoloDetect.Demo

### taskType 0 — generate_point_cloud
**summary**：根据 YOLO 掩码和深度图生成目标点云并转换到机器人本体坐标系

| 参数 | 类型 | 必填 | 默认值 | 说明 |
|---|---|---|---|---|
| sensor_id | string | 否 | — | 相机 ID |
| camera_frame | string | 否 | — | 相机光学坐标系 |
| robot_frame | string | 否 | base_link | 输出坐标系 |
| camera_extrinsics | object | 否 | — | 相机外参（六维，平移毫米/旋转弧度） |
| transform_camera_to_robot | object | 否 | — | 完整相机→机器人变换（position 米 + orientation xyzw），与 camera_extrinsics 互斥 |
| camera_intrinsics | object | 否 | — | 完整内参 fx/fy/cx/cy |
| fx / fy / cx / cy | float | 否 | — | 相机内参（与 camera_intrinsics 二选一） |
| depth_scale | float | 否 | — | 深度值到米的比例 |
| max_points | int | 否 | — | 每个目标最多保留点数 |
| conf_threshold | float | 否 | — | YOLO 置信度阈值 |
| iou_threshold | float | 否 | — | YOLO IoU 阈值 |
| base_frame | string | 否 | base_link | 外参中的 base 坐标系 |
| point_cloud_filter | object | 否 | — | 点云过滤配置（fast_voxel / legacy_dbscan） |
| refine_mask | bool | 否 | true | 是否执行深度掩码清理 |
| transform_timeout_s | float | 否 | 2.0 | 动态 TF 等待时限 |
| transform_max_age_s | float | 否 | 1.0 | 动态 TF 最大时间 |

### taskType 1 — yolo_detect
**summary**：执行 YOLO 目标检测与掩码分割

| 参数 | 类型 | 必填 | 默认值 | 说明 |
|---|---|---|---|---|
| sensor_id | string | 否 | — | 相机 ID |
| conf_threshold | float | 否 | — | YOLO 置信度阈值 |
| iou_threshold | float | 否 | — | YOLO IoU 阈值 |
| include_masks | bool | 否 | — | 是否返回完整二值掩码 |
| imgsz | array | 否 | — | YOLO 输入图像尺寸 |
| device | string | 否 | — | YOLO 推理设备 |
| class_names | object | 否 | — | 类别名称映射 |

### taskType 2 — match_target
**summary**：根据目标点匹配距离最近的 YOLO 分割目标

| 参数 | 类型 | 必填 | 默认值 | 说明 |
|---|---|---|---|---|
| target_point | array | 是 | — | 目标点（机器人本体坐标系，米） |
| delta_threshold_x / y / z | float | 否 | — | X/Y/Z 方向最大偏差（米） |
| sensor_id | string | 否 | — | 相机 ID |
| depth_scale | float | 否 | — | 深度值到米的比例 |
| max_points | int | 否 | — | 每个目标最多点数 |
| camera_intrinsics | object | 否 | — | 内参 |
| camera_extrinsics | object | 否 | — | 外参 |
| transform_camera_to_robot | object | 否 | — | 完整变换 |
| base_frame | string | 否 | base_link | base 坐标系 |
| camera_frame | string | 否 | — | 相机光学坐标系 |
| robot_frame | string | 否 | base_link | 输出坐标系 |
| point_cloud_filter | object | 否 | — | 点云过滤配置 |
| refine_mask | bool | 否 | true | 深度掩码清理 |
| transform_timeout_s | float | 否 | 2.0 | TF 等待时限 |
| transform_max_age_s | float | 否 | 1.0 | TF 最大时间 |

### taskType 3 — estimate_object_pose
**summary**：使用参考点云和目标点云估计物体旋转姿态

| 参数 | 类型 | 必填 | 默认值 | 说明 |
|---|---|---|---|---|
| reference_points | array | 否 | — | 参考点云 N×3（米），需 reference_frame |
| target_point | array | 是 | — | 目标点（机器人本体坐标系，米） |
| delta_threshold_x / y / z | float | 否 | — | X/Y/Z 最大偏差 |
| sensor_id | string | 否 | — | 相机 ID |
| depth_scale | float | 否 | — | 深度值到米比例 |
| max_points | int | 否 | — | 每个目标最多点数 |
| camera_intrinsics | object | 否 | — | 内参 |
| camera_extrinsics | object | 否 | — | 外参 |
| transform_camera_to_robot | object | 否 | — | 完整变换 |
| conf_threshold | float | 否 | — | YOLO 置信度阈值 |
| iou_threshold | float | 否 | — | YOLO IoU 阈值 |
| yaw_range | array | 否 | — | Yaw 搜索范围（度） |
| search_step | float | 否 | — | 粗搜索步长（度） |
| focus_step | float | 否 | — | 精搜索容差（度） |
| enable_roll | bool | 否 | — | 是否搜索 Roll 轴 |
| enable_pitch | bool | 否 | — | 是否搜索 Pitch 轴 |
| enable_yaw | bool | 否 | — | 是否搜索 Yaw 轴 |
| base_frame | string | 否 | base_link | base 坐标系 |
| camera_frame | string | 否 | — | 相机光学坐标系 |
| robot_frame | string | 否 | base_link | 输出坐标系 |
| point_cloud_filter | object | 否 | — | 点云过滤配置 |
| refine_mask | bool | 否 | true | 深度掩码清理 |
| transform_timeout_s | float | 否 | 2.0 | TF 等待时限 |
| transform_max_age_s | float | 否 | 1.0 | TF 最大时间 |
| reference_camera | object | 否 | — | 参考图拍摄时的独立标定 |
| reference_frame | string | 否 | — | 参考点云坐标系 |
| reference_target_point | array | 否 | — | 参考图多目标时的选择点 |
| reference_dir | string | 否 | — | 参考 RGB-D 图片目录 |
| reference_class_id | int | 否 | — | 参考类别 ID |
| reference_class_name | string | 否 | — | 参考类别名称 |

### taskType 4 — GetExecution（通用）
### taskType 5 — StopExecution（通用）

---

## 通用任务说明

所有 ability 都包含以下两个通用任务（taskType 编号因 ability 而异，通常为最大的两个编号）：

### GetExecution
**summary**：查询本 Ability 启动的语义执行

| 参数 | 类型 | 必填 | 默认值 | 说明 |
|---|---|---|---|---|
| invocation_id | string | 是 | — | 调用 ID |

### StopExecution
**summary**：停止底层命令并让相关资源进入 hold

| 参数 | 类型 | 必填 | 默认值 | 说明 |
|---|---|---|---|---|
| invocation_id | string | 是 | — | 调用 ID |
| reason | string | 否 | pilot stop | 停止原因 |

> 所有任务均支持可选参数 `invocation_id`（幂等调用 ID），用于查询/停止本次执行。本文档业务参数表中已省略该通用参数。

---

## 数据来源

- `abilities/*/ability.manifest.yaml`（任务契约定义）
