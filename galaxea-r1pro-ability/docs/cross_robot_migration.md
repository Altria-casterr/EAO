# 跨机器人能力迁移指南

本文档梳理 skill_ability 与 galaxea-r1pro-sdk 在迁移到其他机器人时需要修改的内容。

迁移原则：
- 具体硬件细节（关节数、数据结构字段、单位换算、话题名、坐标系名）只在 SDK 内处理
- skill_ability 层只处理抽象能力逻辑，不感知机器人差异
- skill_ability 不直接访问 SDK 内部结构，统一通过 SDK 公开接口

---
## 以下是Ability相关的迁移内容
## Step1：SDK 打开方式

**位置**：`src/galaxea_r1pro_0/service/entrypoint.py` `ImplAbility.on_connect` (L114)

**现有代码**：
```python
self.sdk = R1ProSDK.open(RobotProfile.load(config.robot_profile_path))
```

**迁移要点**：
- 不同机器人的 SDK 类不同，需要根据机器人类型选择对应的 SDK
- 星海图：`R1ProSDK.open(RobotProfile.load(...))`

**关联 TODO**：entrypoint.py L109-112

---

## Step2：反馈闭环判断逻辑（以夹爪为例）

**位置**：`src/galaxea_r1pro_0/tasks/gripper.py` `HandleSingleGripperTask.execute` (L125-172)

**现有代码**：
```python
while time.time() - start < timeout:
    current = _gripper_state(sdk, hand)
    if check_reached([current], [position], [tol]):
        return handle
    time.sleep(loop_period)
```

**迁移要点**：
- 反馈闭环的轮询逻辑需要重写
- `_gripper_state` 函数需要适配新机器人的状态读取
- 到位判断逻辑（`check_reached` 的容差和比较方式）可能需要调整

**关联 TODO**：gripper.py L123-124

---

## Step3：状态读取函数与结果解析（以夹爪为例）

**位置**：`src/galaxea_r1pro_0/tasks/gripper.py` `_gripper_state` (L47-63)

**现有代码**：
```python
def _gripper_state(sdk, hand: str) -> float | None:
    result = sdk.sensors.get_gripper_state(hand)
    data = (result.output or {}).get("data")
    ...
    return data
```

**迁移要点**：
- 状态读取函数名称可能不同（新机器人可能是 `get_gripper_position` 等）
- `result.output.data` 的数据结构可能不同（可能不是直接的数值）
- 单位可能不同（0-100 vs 0-1 vs 米），需要在 SDK 层统一换算

**关联 TODO**：gripper.py L45-46, L52

---

## 以下是SDK 相关的迁移内容
## Step4：控制下发逻辑（start_command）

**位置**：`src/galaxea_r1pro/galaxea_r1pro_0/backends/real.py` `RealBackend.start_command` (L162-313)

**现有代码**：
```python
spec = GALAXEA_R1PRO_PUBLISH_COMMANDS.get((resource, operation))
if spec is None:
    raise BackendUnavailableError(...)
...
publisher.publish(message)
```

**迁移要点**：
- `GALAXEA_R1PRO_PUBLISH_COMMANDS` 命令映射表需要按新机器人修改
- `BackendUnavailableError` 的错误处理逻辑可能不同
- 控制逻辑（publisher/action/service 的调度方式）需要适配
- 消息构造函数（`build_*`）需要适配新机器人的 ROS 消息格式

**关联 TODO**：real.py L156-160

---

## Step5：状态查询逻辑（get_command）

**位置**：`src/galaxea_r1pro/galaxea_r1pro_0/backends/real.py` `RealBackend.get_command` (L321-...)

**现有代码**：
```python
spec = GALAXEA_R1PRO_GET_COMMANDS.get((resource, operation))
if spec is None:
    raise BackendUnavailableError(...)
...
record = self._commands.get(command_id)
```

**迁移要点**：
- `GALAXEA_R1PRO_GET_COMMANDS` 命令映射表需要按新机器人修改
- `CommandNotFoundError` 的错误处理逻辑可能不同
- 各 `get_*` 处理函数的数据解析逻辑需要适配新机器人的反馈消息结构

**关联 TODO**：real.py L316-320

---

## Step6：skill_ability 结果结构访问抽象

**位置**：skill_ability 多处（共 9 处）

**现有代码模式**：
```python
data = (result.output or {}).get("data")
```

**涉及文件**：
- `tasks/gripper.py` L54
- `tasks/joint_control.py` L31
- `tasks/torso_control.py` L29
- `utils/utils_JointEEFPoseControl.py` L131, L138, L222
- `utils/utils_RobotObservation.py` L12
- `utils/utils_WooshMove.py` L72
- `abilities/yolo_detect/camera_geometry.py` L136

**迁移要点**：
- `result.output.data` 是 SDK 内部约定，skill_ability 不应直接感知
- 迁移时如果新 SDK 把数据放在 `output.value` 或平铺到 `output`，skill_ability 全部要改

---

## Step7：skill_ability 数据类型与单位假设

**位置**：skill_ability 多处

**现有代码模式**：
```python
# gripper.py L62
float(data)                          # 假设夹爪开度直接是数值
# gripper.py L77
position = float(input_data.get("position", 50.0))  # 假设开度范围 0-100
# joint_control.py L80
current[:len(positions)]             # 假设关节位置是列表且可切片
```

**涉及文件**：
- `tasks/gripper.py` L62, L77
- `tasks/joint_control.py` L80

**迁移要点**：
- 单位（开度是 0-100 还是 0-1）应由 SDK 统一，skill_ability 不硬编码
- 数据类型（float 还是 dict）应由 SDK 保证，skill_ability 不做假设
- 建议：SDK 在 `get_data()` 返回前完成单位换算和类型归一化

---

## Step8：CommandHandle 内部字段访问

**位置**：skill_ability 多处

**现有代码模式**：
```python
getattr(handle, "command_id", None)
```

**涉及文件**：
- `tasks/gripper.py` L116, L251, L252
- `tasks/woosh_move.py` L172

**迁移要点**：
- `command_id` 是 `CommandHandle` 的内部字段，新 SDK 可能叫 `id` 或 `task_id`
- 建议：SDK 提供 `handle.get_id()` 统一接口

**当前 TODO 状态**：❌ 无 TODO，迁移遗漏点

---

## 迁移执行顺序建议

1. **Step1** — 先打通 SDK 打开，确认新机器人 SDK 能正常连接
2. **Step4 + Step5** — 替换命令映射表和消息构造，让控制下发和状态查询能跑通
3. **Step2 + Step3** — 适配夹爪反馈闭环
4. **Step6-10** — 修复 skill_ability 层对 SDK 内部细节的依赖，改为统一接口调用
5. 最后逐一验证各能力模块

---
