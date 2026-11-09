# 调试用指令集

> 本文档基于 `skill-Ability/ability/Mock-*` 目录下的 `ability.manifest.yaml` 定义重写。请根据能力包实际启动时的 Instance ID 将 `<INSTANCE_ID>` 替换为真实值。

## 通用调试命令

查询任务状态：
```bash
curl -s -X POST "http://localhost:8080/api/ability/f1c1828f-f930-49be-af46-fb8bb6d0f51e/api/task/status" \
  -H 'Content-Type: application/json' \
  -d '{"task_id":"task-aa5badb3"}' | python3 -m json.tool
```

启动任务通用格式：
```bash
curl -s -X POST "http://localhost:8080/api/ability/<INSTANCE_ID>/api/task/start" \
  -H 'Content-Type: application/json' \
  -d '{"task_type": <TASK_TYPE>, "input": <INPUT_OBJECT>}' | python3 -m json.tool
```

---

## 手臂末端控制测试（Mock-ArmEEFPose）

### 1. 单臂闭环控制

```bash
curl -s -X POST "http://localhost:8080/api/ability/<INSTANCE_ID>/api/task/start" \
  -H 'Content-Type: application/json' \
  -d '{
    "task_type": 0,
    "input": {
      "hand": "left",
      "target_pose": {
        "x": 0.50,
        "y": 0.30,
        "z": 0.10,
        "roll": 90.0,
        "pitch": -90.0,
        "yaw": 0.0
      },
      "frame_id": "1",
      "pos_tol": 0.005,
      "orient_tol": 0.01,
      "timeout": 10.0,
      "loop_freq": 1.0
    }
  }' | python3 -m json.tool
```

### 2. 双臂闭环控制
```bash
curl -s -X POST "http://localhost:8080/api/ability/<INSTANCE_ID>/api/task/start" \
  -H 'Content-Type: application/json' \
  -d '{
    "task_type": 1,
    "input": {
      "hand": "both",
      "left_target_pose": {
        "x": 0.50,
        "y": 0.30,
        "z": 0.10,
        "roll": 90.0,
        "pitch": -90.0,
        "yaw": 0.0
      },
      "right_target_pose": {
        "x": 0.50,
        "y": -0.30,
        "z": 0.10,
        "roll": 90.0,
        "pitch": 90.0,
        "yaw": 0.0
      },
      "frame_id": "1",
      "pos_tol": 0.005,
      "orient_tol": 0.01,
      "timeout": 10.0,
      "loop_freq": 1.0
    }
  }' | python3 -m json.tool
```

### 3. 单臂轨迹控制
```bash
curl -s -X POST "http://localhost:8080/api/ability/<INSTANCE_ID>/api/task/start" \
  -H 'Content-Type: application/json' \
  -d '{
    "task_type": 2,
    "input": {
      "axis": "left",
      "start_pose": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0
      },
      "target_pose": {
        "x": 0.50,
        "y": 0.30,
        "z": 0.10,
        "roll": 90.0,
        "pitch": -90.0,
        "yaw": 0.0
      },
      "frame_id": "1",
      "pos_tol": 0.005,
      "orient_tol": 0.01,
      "timeout": 10.0,
      "loop_freq": 1.0,
      "num_points": 5
    }
  }' | python3 -m json.tool
```

### 4. 双臂同步轨迹控制

```bash
curl -s -X POST "http://localhost:8080/api/ability/<INSTANCE_ID>/api/task/start" \
  -H 'Content-Type: application/json' \
  -d '{
    "task_type": 3,
    "input": {
      "left_start_pose": {
        "x": 0.40,
        "y": 0.25,
        "z": 0.10,
        "roll": 0.0,
        "pitch": -90.0,
        "yaw": 0.0
      },
      "right_start_pose": {
        "x": 0.40,
        "y": -0.25,
        "z": 0.10,
        "roll": 0.0,
        "pitch": 90.0,
        "yaw": 0.0
      },
      "left_target_pose": {
        "x": 0.50,
        "y": 0.30,
        "z": 0.10,
        "roll": 90.0,
        "pitch": -90.0,
        "yaw": 0.0
      },
      "right_target_pose": {
        "x": 0.50,
        "y": -0.30,
        "z": 0.10,
        "roll": 90.0,
        "pitch": 90.0,
        "yaw": 0.0
      },
      "pos_tol": 0.005,
      "orient_tol": 0.01,
      "timeout": 10.0,
      "loop_freq": 1.0,
      "num_points": 5
    }
  }' | python3 -m json.tool
```

---

## 手臂末端规划调试（Mock-ArmEEFPoseSolver）

```bash
curl -s -X POST "http://localhost:8080/api/ability/<INSTANCE_ID>/api/task/start" \
  -H 'Content-Type: application/json' \
  -d '{
    "task_type": 0,
    "input": {
      "target_pose": {
        "x": 0.414,
        "y": 0.348,
        "z": 0.066,
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0
      },
      "grasp_mode": "left"
    }
  }' | python3 -m json.tool
```

---

## 手臂关节控制调试（Mock-ArmJointPose）

### 1. 单手臂关节移动

```bash
curl -s -X POST "http://localhost:8080/api/ability/<INSTANCE_ID>/api/task/start" \
  -H 'Content-Type: application/json' \
  -d '{
    "task_type": 0,
    "input": {
      "hand": "left",
      "positions": {
        "joint1": 0.5,
        "joint2": 1.0
      },
      "joint_names": {
        "joint1": "shoulder",
        "joint2": "elbow"
      },
      "velocities": {
        "joint1": 0.2,
        "joint2": 0.2
      },
      "effort": {
        "joint1": 0.1,
        "joint2": 0.1
      },
      "timeout": 10.0,
      "tol": 0.5,
      "loop_freq": 0.2
    }
  }' | python3 -m json.tool
```

### 2. 双手臂关节移动

```bash
curl -s -X POST "http://localhost:8080/api/ability/<INSTANCE_ID>/api/task/start" \
  -H 'Content-Type: application/json' \
  -d '{
    "task_type": 1,
    "input": {
      "left_positions": {
        "joint1": 0.5,
        "joint2": 1.0
      },
      "right_positions": {
        "joint1": 0.5,
        "joint2": 1.0
      },
      "left_joint_names": {
        "joint1": "left_shoulder",
        "joint2": "left_elbow"
      },
      "right_joint_names": {
        "joint1": "right_shoulder",
        "joint2": "right_elbow"
      },
      "left_velocities": {
        "joint1": 0.2,
        "joint2": 0.2
      },
      "right_velocities": {
        "joint1": 0.2,
        "joint2": 0.2
      },
      "left_effort": {
        "joint1": 0.1,
        "joint2": 0.1
      },
      "right_effort": {
        "joint1": 0.1,
        "joint2": 0.1
      },
      "timeout": 10.0,
      "tol": 0.5,
      "loop_freq": 0.2
    }
  }' | python3 -m json.tool
```

### 3. 单手臂关节轨迹执行

```bash
curl -s -X POST "http://localhost:8080/api/ability/<INSTANCE_ID>/api/task/start" \
  -H 'Content-Type: application/json' \
  -d '{
    "task_type": 2,
    "input": {
      "arm_side": "left",
      "start_positions": {
        "joint1": 0.0,
        "joint2": 0.0
      },
      "target_positions": {
        "joint1": 0.5,
        "joint2": 1.0
      },
      "joint_names": {
        "joint1": "shoulder",
        "joint2": "elbow"
      },
      "velocities": {
        "joint1": 0.2,
        "joint2": 0.2
      },
      "effort": {
        "joint1": 0.1,
        "joint2": 0.1
      },
      "num_points": 10,
      "timeout": 10.0,
      "tol": 0.5,
      "loop_freq": 0.2
    }
  }' | python3 -m json.tool
```

---

## 相对移动调试（Mock-ChassisMove）

### 移动到指定位置

axis = x为前后移动，y为左右移动，z为原地转圈
```bash
curl -s -X POST "http://localhost:8080/api/ability/c853aea5-a8db-4956-bfb3-c0837a642ae5/api/task/start" \
  -H 'Content-Type: application/json' \
  -d '{
    "task_type": 0,
    "input": {
      "target_x": -0.2
      "speed": 0.5,
      "tolerance": 0.01,
      "timeout": 15.0
    }
  }' | python3 -m json.tool
```

---

## 导航调试（Mock-ChassisNavigation）

### 导航到指定位置

```bash
curl -s -X POST "http://localhost:8080/api/ability/<INSTANCE_ID>/api/task/start" \
  -H 'Content-Type: application/json' \
  -d '{
    "task_type": 0,
    "input": {
      "x": 0.27,
      "y": -0.30,
      "yaw": 0.0,
      "target_point_type": 1,
      "frame_id": "map",
      "timeout": 30.0
    }
  }' | python3 -m json.tool
```

---

## 语义导航调试（Mock-SemNav）

### SemanNav 语义导航
```bash
curl -s -X POST "http://localhost:8080/api/ability/<INSTANCE_ID>/api/task/start" \
  -H 'Content-Type: application/json' \
  -d '{
    "task_type": 0,
    "input": {
      "color_img_path": "/home/m9pro/wbq/output/dataset/color.png",
      "depth_img_path": "/home/m9pro/wbq/output/dataset/depth.png",
      "reference_image_path": "/home/m9pro/wbq/output/dataset/reference_color.png",
      "reference_depth_path": "/home/m9pro/wbq/output/dataset/reference_depth.png",
      "target_dict": {
        "x": 0.5,
        "y": -0.2,
        "z": 0.0
      },
      "conf_threshold": 0.3,
      "tol_x": 0.02,
      "tol_y": 0.02,
      "tol_yaw": 5.0,
      "iteration": 10,
      "speed": 0.2,
      "class_names": {"red": "box"}
    }
  }' | python3 -m json.tool
```

### 查询目标状态

```bash
curl -s -X POST "http://localhost:8080/api/ability/<INSTANCE_ID>/api/task/start" \
  -H 'Content-Type: application/json' \
  -d '{
    "task_type": 1,
    "input": {
      "color_img_path": "/home/m9pro/wbq/output/dataset/color.png",
      "depth_img_path": "/home/m9pro/wbq/output/dataset/depth.png",
      "target_dict": {
        "x": 0.5,
        "y": -0.2,
        "z": 0.0
      },
      "conf_threshold": 0.3,
      "class_names": {"red": "box"}
    }
  }' | python3 -m json.tool
```

---

## YOLO检测识别调试（Mock-DetectYolo）

### 1. 目标匹配（YoloSegMatchTarget）

```bash
curl -s -X POST "http://localhost:8080/api/ability/<INSTANCE_ID>/api/task/start" \
  -H 'Content-Type: application/json' \
  -d '{
    "task_type": 0,
    "input": {
      "color_img_path": "/home/m9pro/wbq/output/dataset/color.png",
      "depth_img_path": "/home/m9pro/wbq/output/dataset/depth.png",
      "target_point": {
        "x": 0.5,
        "y": 0.0,
        "z": 0.0
      },
      "class_name": {"red": "box"},
      "delta_threshold_x": 0.02,
      "delta_threshold_y": 0.02,
      "delta_threshold_z": 0.02,
      "apply_tf_transform": true,
      "yolo_confidence": 0.3,
      "image_scale": 1.0
    }
  }' | python3 -m json.tool
```

### 2. 目标 RPY 偏差（YoloSegTargetRpy）

```bash
curl -s -X POST "http://localhost:8080/api/ability/<INSTANCE_ID>/api/task/start" \
  -H 'Content-Type: application/json' \
  -d '{
    "task_type": 1,
    "input": {
      "sample_color_path": "/home/m9pro/wbq/output/dataset/sample_color.png",
      "sample_depth_path": "/home/m9pro/wbq/output/dataset/sample_depth.png",
      "color_img_path": "/home/m9pro/wbq/output/dataset/color.png",
      "depth_img_path": "/home/m9pro/wbq/output/dataset/depth.png",
      "target_point": {
        "x": 0.5,
        "y": 0.0,
        "z": 0.0
      },
      "class_name": {"red": "box"},
      "sample_target_point": {
        "x": 0.45,
        "y": 0.05,
        "z": 0.0
      },
      "yolo_confidence": 0.3,
      "image_scale": 1.0
    }
  }' | python3 -m json.tool
```

### 3. 点云检测（YoloSegWithPcd）

```bash
curl -s -X POST "http://localhost:8080/api/ability/<INSTANCE_ID>/api/task/start" \
  -H 'Content-Type: application/json' \
  -d '{
    "task_type": 2,
    "input": {
      "color_img_path": "/home/m9pro/wbq/output/dataset/color.png",
      "depth_img_path": "/home/m9pro/wbq/output/dataset/depth.png",
      "class_name": {"red": "box"},
      "yolo_confidence": 0.3,
      "image_scale": 1.0,
      "target_coordinate_system": "base"
    }
  }' | python3 -m json.tool
```

---

## 拆垛流程调试（Mock-GrabUp）

### 单左手

```bash
curl -s -X POST "http://localhost:8080/api/ability/<INSTANCE_ID>/api/task/start" \
  -H 'Content-Type: application/json' \
  -d '{
    "task_type": 0,
    "input": {
      "box_middle_point_x": 0.5,
      "box_middle_point_y": 0.0,
      "box_middle_point_z": 0.0,
      "grasp_mode": "left",
      "sepecial_mode": false,
      "left_roll":-90.0,
      "left_pitch":-30.0,
      "left_yaw":0.0,
      "right_roll":-90.0,
      "right_pitch":30.0,
      "right_yaw":0.0,
      "box_long" : 0.60,
      "box_width" : 0.40,
      "box_height" : 0.23,
      "torso": 90.0
    }
  }' | python3 -m json.tool
```

## 夹爪调试（Mock-Gripper）

### 左手
```bash
curl -s -X POST "http://localhost:8080/api/ability/555d8347-97ab-41a0-8ff3-890e3562c0ff/api/task/start" \
  -H 'Content-Type: application/json' \
  -d '{
    "task_type": 0,
    "input": {
      "hand": "left",
      "speed": 0.5,
      "position": 10.0
    }
  }' | python3 -m json.tool
```

### 双手
```bash
curl -s -X POST "http://localhost:8080/api/ability/<INSTANCE_ID>/api/task/start" \
  -H 'Content-Type: application/json' \
  -d '{
    "task_type": 1,
    "input": {
      "hand": "both",
      "left_speed": 0.5,
      "left_position": 50.0,
      "right_speed": 0.5,
      "right_position": 50.0
    }
  }' | python3 -m json.tool
```

## 导航序列调试（Mock-NavSolver）
{'id': 18, 'position':  [0.5, -0.20, 1.8, 0.0, 0.0, 0.0]},
```bash
curl -s -X POST "http://localhost:8080/api/ability/<INSTANCE_ID>/api/task/start" \
  -H 'Content-Type: application/json' \
  -d '{
    "task_type": 0,
    "input": {
      "object_pose": {
        "x": 0.5,
        "y": -0.2,
        "z": 1.8,
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0
      },
      "split": 5,
      "axis": "rad"
    }
  }' | python3 -m json.tool
```

---

## 躯干控制调试（Mock-Torso）

### 单个关节移动

```bash
curl -s -X POST "http://localhost:8080/api/ability/<INSTANCE_ID>/api/task/start" \
  -H 'Content-Type: application/json' \
  -d '{
    "task_type": 0,
    "input": {
      "joint_id": 1,
      "target_pos": 0.0
    }
  }' | python3 -m json.tool
```

### 四个关节全部移动

```bash
curl -s -X POST "http://localhost:8080/api/ability/<INSTANCE_ID>/api/task/start" \
  -H 'Content-Type: application/json' \
  -d '{
    "task_type": 1,
    "input": {
      "target_positions": {
        "joint1": 0.2,
        "joint2": 0.0,
        "joint3": 0.0,
        "joint4": 0.0
      },
      "speeds": {
        "joint1": 0.1,
        "joint2": 0.2,
        "joint3": 0.3,
        "joint4": 0.1
      },
      "tolerances": {
        "joint1": 0.02,
        "joint2": 0.02,
        "joint3": 0.02,
        "joint4": 0.02
      },
      "timeout": 8.0
    }
  }' | python3 -m json.tool
```
