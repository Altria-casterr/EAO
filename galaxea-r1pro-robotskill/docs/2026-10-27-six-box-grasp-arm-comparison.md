# 六箱单双手抓取：输入参数、手臂点位与执行状态对比

日期：2026-10-27。Workflow：`wf-ca4729aa-5e8a-46ba-8ad1-59f7e47f8056`。

结论：c1/c3/c5 使用 grasp-object-unilateral@0.1.11，c2/c4/c6 使用 grasp-object-bilateral@0.2.7。同一种 Skill 三次执行的输入只有 target.object_ref、target.position_m、torso.positions 发生变化；全部手臂规划、运动和力控参数一致。六次均 completed，holding_object=true、retracted=true。

## 统计口径和点位来源

- 输入取 robot_executions.body.input，代表本次执行实际收到的参数。
- 模型提供地图目标、尺寸、偏移量与运动配置；预备、上抬、外拉、辅助手接近、收箱点由 solver 在执行中计算。
- 地图目标用 map 坐标系；下文运动点位为 torso_link4 下的 (x, y, z)，单位米，显示四位小数。
- 点位取 Skill result 和 action.terminal.result.target，是规划/下发目标，不等同于独立测得的最终实际末端坐标。
- 匹配中心 X/Y 取感知匹配，Z 被替换为地图目标转换到 torso_link4 后的高度。
- center_x_offset_m=0.2、center_z_offset_m=0.2 虽出现在六次输入，但当前共享 solver 函数体不使用这两个兼容字段。

## 三次输入中变化的字段

| Task | 类型 | object_ref | target.position_m（map，m） | torso.positions（rad） |
|---|---|---|---|---|
| c1 | 单手 | gray_box-l5-1 | [1.01,0.978,1.24] | [0,0,0,0] |
| c2 | 双手 | gray_box-l5-2 | [1.01,1.529,1.24] | [0,0,0,0] |
| c3 | 单手 | gray_box-l4-1 | [1.01,0.978,0.99] | [0.75,-1.5,-0.75,0] |
| c4 | 双手 | gray_box-l4-2 | [1.01,1.529,0.99] | [0.75,-1.5,-0.75,0] |
| c5 | 单手 | gray_box-l3-1 | [1.01,0.978,0.76] | [1.3,-2.1,-0.8,0] |
| c6 | 双手 | gray_box-l3-2 | [1.01,1.529,0.76] | [1.3,-2.1,-0.8,0] |

## grasp-object-unilateral：三次相同的手臂相关输入

```json
{
  "relative_frame": "torso_link4",
  "grasp_axis": "right",
  "safe_offset_m": 0.1,
  "capture_x_offset_m": -0.1,
  "capture_z_offset_m": 0,
  "pull_y_distance_m": 0.2,
  "lift_m": 0.15,
  "center_x_offset_m": 0.2,
  "center_z_offset_m": 0.2,
  "motion": {
    "axis_order": "zyx",
    "loop_freq": 0.1,
    "max_num_points": 100,
    "num_points": null,
    "orient_tol": 0.05,
    "pos_tol": 0.05,
    "sequential": true,
    "speed": 0.03,
    "step_distance": 0.01,
    "timeout": 10
  },
  "approach_force": {
    "force_coordinate": [
      "x"
    ],
    "force_threshold_n": 1,
    "loop_freq_s": 0.1,
    "step_size_m": 0.01,
    "timeout_s": 12
  },
  "verify_side": "left",
  "verify_force": {
    "force_coordinate": [
      "x"
    ],
    "force_threshold_n": 1,
    "loop_freq_s": 0.1,
    "step_size_m": 0.001,
    "timeout_s": 12
  },
  "target_size": {
    "height_m": 0.23,
    "long_m": 0.53,
    "width_m": 0.41
  }
}
```

### 三次实际规划点位

| 点位 | c1 / gray_box-l5-1 | c3 / gray_box-l4-1 | c5 / gray_box-l3-1 |
|---|---|---|---|
| 匹配中心 | (0.6737, 0.1570, 0.0917) | (0.6735, 0.1460, 0.0297) | (0.5948, 0.1403, -0.0257) |
| 左手预备点 | (0.1841, 0.2177, -0.0126) | (0.1841, 0.2177, -0.0126) | (0.1841, 0.2177, -0.0127) |
| 右手预备点 | (0.5737, -0.2100, 0.0917) | (0.5735, -0.2200, 0.0297) | (0.4948, -0.2200, -0.0257) |
| 右手接触后上抬目标 | (0.5745, -0.1171, 0.1884) | (0.5743, -0.1268, 0.1269) | (0.4947, -0.1362, 0.0726) |
| 右手外拉目标 | (0.5745, -0.3171, 0.1884) | (0.5743, -0.3268, 0.1269) | (0.4947, -0.3362, 0.0726) |
| 左手 Y 预定位 | (0.1858, 0.3128, -0.0133) | (0.1841, 0.3041, -0.0128) | (0.1839, 0.2944, -0.0126) |
| 左手 Z 接近 | (0.1858, 0.3128, 0.0838) | (0.1841, 0.3041, 0.0236) | (0.1839, 0.2944, -0.0303) |
| 左手 X 接近 | (0.5770, 0.3128, 0.0838) | (0.5749, 0.3041, 0.0236) | (0.4948, 0.2944, -0.0303) |
| 左手接触后上抬目标 | (0.5838, 0.2560, 0.1762) | (0.5787, 0.2386, 0.1170) | (0.4964, 0.2381, 0.0632) |
| 双手抬升：左 | (0.5864, 0.2573, 0.3211) | (0.5796, 0.2413, 0.2626) | (0.4967, 0.2402, 0.2100) |
| 双手抬升：右 | (0.5772, -0.3184, 0.3337) | (0.5755, -0.3279, 0.2731) | (0.4951, -0.3370, 0.2196) |
| 收箱：左 | (0.4858, 0.2888, 0.3167) | (0.4790, 0.2863, 0.2588) | (0.3956, 0.2899, 0.2069) |
| 收箱：右 | (0.4858, -0.2888, 0.3167) | (0.4790, -0.2863, 0.2588) | (0.3956, -0.2899, 0.2069) |

单手 capture_points.left 是 prepare 时保留的左手当前位置，不是左手接箱目标。主手接触后上抬 0.10 m、辅助手接触后上抬 0.10 m 均为 Skill 常量；双手 lift 段才使用输入 lift_m=0.15。右手外拉保持主手上抬目标的 X/Z，Y 减少 0.20 m。左手依次按 Y→Z→X 接近，Y 取拉箱后右手实际 Y+0.53+0.10，Z 取右手实际 Z−0.10。

## grasp-object-bilateral：三次相同的手臂相关输入

```json
{
  "relative_frame": "torso_link4",
  "safe_offset_m": 0.1,
  "capture_x_offset_m": -0.1,
  "capture_z_offset_m": 0,
  "lift_m": 0.15,
  "center_x_offset_m": 0.2,
  "center_z_offset_m": 0.2,
  "motion": {
    "axis_order": "zyx",
    "loop_freq": 0.1,
    "max_num_points": 100,
    "num_points": null,
    "orient_tol": 0.05,
    "pos_tol": 0.05,
    "sequential": true,
    "speed": 0.05,
    "step_distance": 0.01,
    "timeout": 10
  },
  "approach_motion": {
    "axis_order": "yzx",
    "loop_freq": 0.1,
    "max_num_points": 100,
    "num_points": null,
    "orient_tol": 0.05,
    "pos_tol": 0.05,
    "sequential": true,
    "speed": 0.001,
    "step_distance": 0.02,
    "timeout": 5
  },
  "approach_force": {
    "force_coordinate": [
      "x"
    ],
    "force_threshold_n": 0.5,
    "loop_freq_s": 0.1,
    "step_size_m": 0.01,
    "timeout_s": 12
  },
  "verify_side": "left",
  "verify_force": {
    "force_coordinate": [
      "x"
    ],
    "force_threshold_n": 1,
    "loop_freq_s": 0.1,
    "step_size_m": 0.01,
    "timeout_s": 12
  },
  "target_size": {
    "height_m": 0.23,
    "long_m": 0.53,
    "width_m": 0.41
  }
}
```

### 三次实际规划点位

| 点位 | c2 / gray_box-l5-2 | c4 / gray_box-l4-2 | c6 / gray_box-l3-2 |
|---|---|---|---|
| 匹配中心 | (0.6537, -0.0164, 0.0918) | (0.6549, -0.0043, 0.0297) | (0.5667, -0.0058, -0.0257) |
| 左手预备点 | (0.5537, 0.3500, 0.0918) | (0.5549, 0.3600, 0.0297) | (0.4667, 0.3600, -0.0257) |
| 右手预备点 | (0.5537, -0.3800, 0.0918) | (0.5549, -0.3700, 0.0297) | (0.4667, -0.3700, -0.0257) |
| 双手抬升：左 | (0.5597, 0.2725, 0.2358) | (0.5453, 0.2709, 0.1786) | (0.4668, 0.2728, 0.1204) |
| 双手抬升：右 | (0.5575, -0.2934, 0.2373) | (0.5530, -0.2893, 0.1784) | (0.4667, -0.2834, 0.1200) |
| 收箱：左 | (0.4595, 0.2852, 0.2312) | (0.4443, 0.2822, 0.1747) | (0.3652, 0.2803, 0.1174) |
| 收箱：右 | (0.4595, -0.2852, 0.2312) | (0.4443, -0.2822, 0.1747) | (0.3652, -0.2803, 0.1174) |

双手先按 approach_motion.axis_order=yzx 到预备点，逐侧 force_align；从实际接触位姿分别抬升 0.15 m，再收箱。没有单手外拉及辅助手 Y→Z→X 接合流程。

## 点位计算规则

六次 size 均为 long_m=0.53、width_m=0.41、height_m=0.23。

- 初始预备点：X=匹配中心 X−0.10；Z=地图转换后的目标 Z；Y 在可达范围内选取最近箱侧安全点，使用 long_m/2 和 safe_offset_m=0.10，并取到厘米精度。
- 预备姿态是 solver 固定姿态：左手四元数 xyzw≈(-0.183013,-0.683013,-0.183013,0.683013)，右手≈(0.183013,-0.683013,0.183013,0.683013)，不是模型给定。
- 双手上抬目标：两手各自当前实际 Z+0.15，其余位置和姿态保持。
- 收箱：读取抬升后的实际末端位置，双手目标 X=左手实际 X−width_m/4=左手实际 X−0.1025，Z=左手实际 Z，Y 分别为 ±(左手实际 Y−右手实际 Y)/2，各自保留当前四元数。
- 因收箱前会重新读取实际位置，不能直接把 lift_points 的目标值代入当作精确收箱公式的实测输入。

## 三次执行状态与阶段耗时

单位秒；Skill 总耗时含准备、躯干回零与检查，阶段之间的少量开销不在分阶段列中。

| Task | 类型 | prepare | approach | lift（上抬+收箱） | stand | verify | 总计 | 手臂动作成功数 |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| c1 | 单手 | 4.30 | 29.70 | 6.59 | 2.14 | 3.72 | 46.50 | 12/12 |
| c2 | 双手 | 4.36 | 12.68 | 6.52 | 2.15 | 2.90 | 28.65 | 6/6 |
| c3 | 单手 | 6.77 | 29.22 | 6.75 | 4.71 | 4.40 | 51.89 | 12/12 |
| c4 | 双手 | 6.95 | 11.88 | 6.82 | 5.67 | 2.78 | 34.14 | 6/6 |
| c5 | 单手 | 7.62 | 28.40 | 6.97 | 5.41 | 4.09 | 52.52 | 12/12 |
| c6 | 双手 | 7.64 | 11.42 | 6.70 | 6.89 | 2.69 | 35.40 | 6/6 |

单手三次 approach 为 29.70→29.22→28.40 秒，lift 为 6.59→6.75→6.97 秒；双手三次 approach 为 12.68→11.88→11.42 秒，lift 为 6.52→6.82→6.70 秒。总耗时随层位降低而增加主要对应 prepare 与 stand 阶段变长，手臂 approach 并未变慢。

全部 action.terminal 均 succeeded；单手每次包含 12 个手臂运动/力控动作，双手每次 6 个（均含末尾力控验证）。持物成功依据为左侧 single_force_align、阈值 1 N。六次 RGB 均 metadata_only、image_refs 为空，不能将本次结果表述为已经图像复核通过。

## 执行证据

| Task | Skill 版本 | execution_id |
|---|---|---|
| c1 | grasp-object-unilateral@0.1.11 | rex-c2ea6bd7-f6d7-43d7-b1d1-c8fc5d3e2e2b |
| c2 | grasp-object-bilateral@0.2.7 | rex-5c1d21bc-02f0-4996-9053-2a41265982f7 |
| c3 | grasp-object-unilateral@0.1.11 | rex-5dc8847c-474f-4e5b-a2e7-457e55fb433c |
| c4 | grasp-object-bilateral@0.2.7 | rex-6115a5b0-8c68-4c2b-b1df-e88a9f829096 |
| c5 | grasp-object-unilateral@0.1.11 | rex-067c733e-8a7d-4505-b7be-6cda85502dcd |
| c6 | grasp-object-bilateral@0.2.7 | rex-38a94976-82fb-470f-bdff-b17d6f2075c7 |

数据来源：semantic-framework/.output/data/semantic.db 的 robot_executions 和 robot_execution_events，只读提取。

点位规则参考仓库：

- [单手 skill.py](../src/skill/grasp_object_unilateral/scripts/skill.py)
- [单手 solver.py](../src/skill/grasp_object_unilateral/scripts/solver.py)
- [双手 skill.py](../src/skill/grasp_object_bilateral/scripts/skill.py)
- [共享 capture_point_solver.py](../src/solver/capture_point_solver/capture_point_solver.py)
