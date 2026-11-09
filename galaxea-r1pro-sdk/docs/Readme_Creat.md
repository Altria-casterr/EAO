# 新增控制函数（xxx_control.py）开发流程

> 以 torso_control.py、gripper_control.py、eef_control.py 等现有实现为参考，总结新增一个控制模块的固定流程。

## 1. 架构定位

一次命令的完整流转（详见外层 README「详细代码流转逻辑」）：

```text
Ability 调用
   ▼
xxx_control.py（客户端层：语义参数 → target 字典，不感知 ROS）
   ▼
backends/real.py start_command / get_command（真机后端）
   ▼
transport/process_pub.py（执行型：resource+operation → topic + 消息构造）
transport/process_sub.py（读取型：resource+operation → handler 解析反馈）
   ▼
utils/Connect_Ros.py（话题注册表 + rosbridge 连接）
```

各层职责边界：

| 层 | 文件 | 职责 | 禁止事项 |
|---|---|---|---|
| 客户端层 | `xxx_control.py` | 参数校验、默认值、target 组装 | 不 import roslibpy，不拼消息 |
| 消息转换层 | `transport/process_pub.py` | builder 构造 ROS 消息、topic 映射 | 不做业务逻辑 |
| 读取处理层 | `transport/process_sub.py` | 等待话题、解析反馈数据 | 不做参数业务校验 |
| 连接层 | `utils/Connect_Ros.py` | 建连、注册全部话题 | — |

## 2. 命名规范

| 项 | 规范 | 示例 |
|---|---|---|
| 文件名 | `xxx_control.py`（底盘/导航沿用 `woosh_xxx.py`） | `head_control.py` |
| 类名 | `XxxControl`，构造只收 `backend` | `HeadControl` |
| resource 字符串 | 与文件名一致的 `"xxx_control"` | `"head_control"` |
| 执行型操作名 | `control_*` | `control_head_pan` |
| 读取型操作名 | `get_*` | `get_head_state` |

## 3. 创建步骤

### Step 1 新建客户端文件 `xxx_control.py`

```python
"""Xxx_Control 能力：一句话说明用途。

输入参数仅保留与 publisher 消息构造相关的字段。
"""

from __future__ import annotations

from .backends.base import RobotBackend
from .utils.types import CommandHandle


class XxxControl:
    """R1 Pro xxx 控制。"""

    def __init__(self, backend: RobotBackend) -> None:
        self._backend = backend

    def control_xxx(
        self,
        *,
        param_a: float,
        param_b: str = "left",
    ) -> CommandHandle:
        return self._backend.start_command(
            "xxx_control", "control_xxx",
            {"param_a": param_a, "param_b": param_b},
        )
```

约定：

- 参数一律 keyword-only（`*` 之后），只保留与 publisher 消息构造相关的字段；
- 默认值写在函数签名或类常量（参考 woosh_move.py 的 `DEFAULT_LINEAR_SPEED_X`）；
- 执行型返回 `CommandHandle`，读取型返回 `CommandResult`；
- 单位约定：位置/速度用米与米每秒，角度入参用度，弧度换算放在 transport 层；欧拉角转四元数用 `utils_common.euler_to_quaternionx`（eef 使用 `seq="yxz"`）；
- 需要读当前状态来组装 target 时，把其他客户端通过构造函数注入（参考 torso_control.py 的 `control_single_torso_joint`）；
- 双侧并行控制（左右臂/左右夹爪）用 daemon 线程 + results 字典模板（参考 gripper_control.py 的 `control_both_gripper`）。

### Step 2 门面挂载 `sdk.py`

```python
from .xxx_control import XxxControl
```

在 `R1ProSDK.__init__` 中追加：

```python
self.xxx = XxxControl(backend)
```

### Step 3 执行型命令映射 `transport/process_pub.py`

builder 与映射条目放在一起：

```python
# ==================== xxx_control ====================
def build_xxx_message(target: dict) -> dict:
    """构造 xxx 消息。"""
    return {
        "header": create_timestamp_message(),
        # 按 ROS 消息定义从 target 取值，float()/str() 显式转换
    }

GALAXEA_R1PRO_PUBLISH_COMMANDS[("xxx_control", "control_xxx")] = {
    "kind": "publisher",
    "topic": "/motion_target/xxx",  # topic 依赖参数时用 lambda：lambda target: f"...{target['param_b']}"
    "message_type": "sensor_msgs/msg/JointState",  # 按实际消息类型填写
    "builder": build_xxx_message,
}
```

topic 依赖参数时必须用 lambda（参考 `control_single_arm_joint` 按 `arm_side` 选左右话题）。

### Step 3b Action 型命令（可选）

任务型长时动作（如导航）用 `kind: "action"`，后端走 goal 发送、回调收结果、`stop_command` 取消的完整状态机（参考 woosh_navigation.py）：

```python
GALAXEA_R1PRO_PUBLISH_COMMANDS[("xxx_control", "control_xxx")] = {
    "kind": "action",
    "topic": "/xxx/xxx/action",
    "message_type": "xxx_msg/action/Xxx",
    "builder": build_xxx_goal,
}
```

### Step 4 注册话题 `utils/Connect_Ros.py`

新话题必须先加入 `GALAXEA_R1PRO_TOPICS`，否则 connect 后 publisher 为 None，`start_command` 会抛 `ENDPOINT_UNREACHABLE`：

```python
GALAXEA_R1PRO_TOPICS["publishers"]["/motion_target/xxx"] = "sensor_msgs/msg/JointState"
```

订阅（供 get 函数读取）加入 `subscribers`，Action 加入 `actions`。

### Step 5 读取型函数（可选）

处理函数写入 `transport/process_sub.py`，用 `_wait_for_topic` 等待消息后解析：

```python
def get_xxx_state(ros: ConnectRos) -> Optional[dict]:
    message = _wait_for_topic(ros, "/hdas/feedback_xxx", 5.0)
    # 解析并返回纯数据（dict/list/float），无数据返回 None
    return parsed

GALAXEA_R1PRO_GET_COMMANDS[("xxx_control", "get_xxx_state")] = {"handler": get_xxx_state}
```

- handler 需要 axis 等固定参数时用 `partial(get_xxx_state, axis="left")`；
- 需要调用方传参数时注册 `"accepts_target": True`（参考 `get_transformer_martix`）。

对外入口按语义挂载：本体感知类放 sensors.py（`SensorsClient`），全局状态类放 state.py（`StateClient`），也可作为 `XxxControl.get_*` 方法。

### Step 6 测试 `example/`

参照 test_real_commands.py：

1. 把 `("xxx_control", "control_xxx")` 加进 `test_ability_functions_are_mapped` 的 key 列表；
2. 加 builder 单测，断言构造出的消息字段；
3. 用 `_FakePublisher` + `_real_backend(publishers={...})` 验证 `start_command` 确实发布到映射话题；
4. 读取型函数用 `_FakeRosConnection.latest_messages` 预置消息验证 handler 解析。

运行：

```bash
python -m unittest discover -s example -v
```

### Step 7 文档同步

- 外层 README 的映射/流转逻辑有变化时同步更新；
- commit 按规范：`feat(Add): 新增 xxx_control 控制 xxx`。

## 4. test_mode 说明

`test_mode=true` 时 `RealBackend` 全部走 `InMemoryBackend` 内存模拟，任意 `(resource, operation)` 直接登记并返回 `CommandHandle`，因此新增控制函数**不需要改 memory.py** 即可被 Ability 在无真机环境调用。真机模式下未注册的 `(resource, operation)` 明确抛 `BackendUnavailableError(NOT_IMPLEMENTED)`，不伪装成功。

## 5. 检查清单

| # | 检查项 | 位置 |
|---|---|---|
| 1 | 客户端类已创建，参数 kw-only，返回 `CommandHandle` | `xxx_control.py` |
| 2 | 门面已挂载 `self.xxx = XxxControl(backend)` | `sdk.py` |
| 3 | builder + `GALAXEA_R1PRO_PUBLISH_COMMANDS` 映射已注册 | `transport/process_pub.py` |
| 4 | 新话题已加入 `GALAXEA_R1PRO_TOPICS` | `utils/Connect_Ros.py` |
| 5 | get 函数 handler 已注册（如有） | `transport/process_sub.py` |
| 6 | 读取入口已挂到 Sensors/State/控制类（如有） | `sensors.py` / `state.py` |
| 7 | 映射、builder、发布路径测试已补充并通过 | `example/` |
| 8 | 文档同步、commit 符合规范 | README / git |
