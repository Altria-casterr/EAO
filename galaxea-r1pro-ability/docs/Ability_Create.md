# 新建 Ability（registry 版）开发流程

> 本文档基于 `src/galaxea_r1pro_0` 当前代码梳理，以 abilities/gripper、tasks/gripper.py 等现有实现为参考，总结新建 `abilities/xxx` 能力的固定流程。环境安装见外层 README Step1/Step2。

## 1. 架构与调用链

```text
AbilityFramework（ability_py：AbilityService / task_manager / task_server）
        ▼ 按 taskType 调度
abilities/xxx/main.py（入口：run_ability(AbilityRole.xxx, "Xxx.Demo")）
        ▼
service/entrypoint.py（ImplAbility 生命周期：读 CR → 连 SDK → 组装 Service → 注册 Task）
        ▼
service/ability_service.py（invoke 分派：GetExecution / StopExecution / 业务 Task）
        ▼
tasks/xxx.py（AbilityTask.execute：解析参数 → sdk.xxx.control_* 发布命令 → 反馈闭环）
        ▼
galaxea_r1pro（Robot SDK，来自 skill_library）
```

各文件职责：

| 文件 | 职责 |
|---|---|
| `abilities/xxx/main.py` | 入口，仅两行：`run_ability(AbilityRole.xxx, "Xxx.Demo")` |
| `abilities/xxx/package.yaml` | 包名/版本/架构 |
| `abilities/xxx/ability.manifest.yaml` | 能力清单：abilityName、config schema、全部 task 的 params/returns/taskType |
| `abilities/xxx/crs/xxx.cr.yaml` | CR 实例模板：abilityName、package、robotProfilePath、task 列表 |
| `tasks/xxx.py` | 业务 Task：参数解析、SDK 调用、反馈闭环 |
| `tasks/__init__.py` | `ROLE_TASK_SPECS`：角色 → Task 类映射（顺序即 taskType） |
| `service/roles.py` | `AbilityRole` 枚举（一个 Ability 一个角色） |
| `service/entrypoint.py` | 生命周期装配：`RuntimeConfig.load` 读 CR、`on_connect` 开 SDK、`DispatchTask` 转发 |
| `service/ability_service.py` | `invoke` 分派 + invocation_id 幂等 + CommandHandle 登记执行记录 |
| `service/execution.py` | 执行记录：running/completed/failed/stopped 状态推进与真实停止 |
| `tests/test_xxx_flow.py` | test_mode SDK 全流程测试 |

启动时自动发生的事（不需要写代码）：

1. `RuntimeConfig.load(ability_name)` → `find_cr` 按 `spec.abilityName == abilityName` 找 `crs/*.yaml` → 读取 `spec.config.robotProfilePath`（可用环境变量 `ABILITY_CR_PATH`、`ROBOT_PROFILE_PATH` 覆盖）；
2. `R1ProSDK.open(RobotProfile.load(...))` 单例连接（断连自愈）；
3. `role_task_types(role)` 取 Task 类声明的 `(task_type, name)` 注册 DispatchTask，并在末尾自动追加框架内置的 `GetExecution` / `StopExecution`。

## 2. 命名规范

| 项 | 规范 | 示例 |
|---|---|---|
| 能力目录 | 小写下划线 | `abilities/head_control/` |
| 角色 role | 枚举小写下划线，与目录一致 | `head_control` |
| abilityName | 大驼峰 `.Demo` | `HeadControl.Demo` |
| package.yaml name | 小写点分 | `headcontrol.demo` 或 `head_control.demo` |
| Task 类名 | 大驼峰 + Task 后缀 | `ControlHeadPanTask` |
| Task.name | 与 SDK 语义操作一致的小写下划线 | `control_head_pan` |
| CR 文件 | `xxx-xxx.demo.cr.yaml`（小写连字符） | `crs/head-control.demo.cr.yaml` |

## 3. 创建步骤

### Step 1 编写业务 Task：`tasks/xxx.py`

继承 `AbilityTask`，声明 `name / summary / task_type`，实现 `execute()`：

```python
"""Xxx_Control 业务 Task：一句话说明（借鉴 Atom Xxx_Control）。"""

from __future__ import annotations

import logging
import time
from typing import Any, Mapping

from galaxea_r1pro import CommandHandle

from ..utils.utils_common import check_reached
from .base import AbilityTask

logger = logging.getLogger(__name__)


class ControlXxxTask(AbilityTask):
    """xxx 控制（taskType 0），含反馈闭环。"""

    name = "control_xxx"
    summary = "控制 xxx 移动到目标位置（含反馈闭环）"
    task_type = 0

    def execute(self, input_data: Mapping[str, Any]) -> CommandHandle:
        sdk = self.sdk
        position = float(input_data.get("position", 0.0))
        timeout = float(input_data.get("timeout", 10.0))
        loop_period = float(input_data.get("loop_period", 0.1))

        # ① 发布目标指令（调用 skill_library 的 galaxea_r1pro SDK）
        handle = sdk.xxx.control_xxx(position=position)
        logger.info("xxx 目标发布: position=%s", position)

        # ② 反馈闭环：轮询实际状态直到 check_reached 通过或超时
        start = time.time()
        while time.time() - start < timeout:
            result = sdk.sensors.get_xxx_state()
            current = (result.output or {}).get("data")
            if current is not None and check_reached(
                [current], [position], [5.0], description="xxx到位"
            ):
                return handle
            time.sleep(loop_period)

        raise RuntimeError(f"xxx 控制超时（{timeout}s）：目标 position={position}")
```

约定：

- `execute()` 返回 `CommandHandle`（物理命令，由 Service 登记为 running，可被 GetExecution/StopExecution 查询与停止）或普通 `dict`（同步任务，直接 register_completed）；
- 参数错误抛 `ValueError`，超时/执行失败抛 `RuntimeError`，不要返回 success/failure 字典；
- `GetExecution` / `StopExecution` 为框架内置 Task，无需编写；
- 纯读取型 Task（如 robot_observation 的 get_robot_state）返回 dict 即可，无需闭环；
- 依赖模型文件时使用注入的 `self.models`（参考 tasks/robot_observation.py + utils/utils_RobotObservation.py）。

### Step 2 注册 Task：`tasks/__init__.py`

```python
from .xxx import ControlXxxTask

ROLE_TASK_SPECS: dict[str, tuple[type[AbilityTask], ...]] = {
    "xxx": (ControlXxxTask,),
}
```

- 角色名为 Step 4 的枚举值；元组顺序即业务 Task 顺序，taskType 以 Task 类声明为准（第一个业务 Task 用 0）。

### Step 3 新增角色：`service/roles.py`

```python
class AbilityRole(StrEnum):
    xxx = "xxx"
```

### Step 4 创建能力目录：`abilities/xxx/`

```text
abilities/xxx/
├── main.py
├── package.yaml
├── ability.manifest.yaml
└── crs/
    └── xxx-xxx.demo.cr.yaml
```

**main.py**：

```python
from galaxea_r1pro_0.service.entrypoint import run_ability
from galaxea_r1pro_0.service.roles import AbilityRole


if __name__ == "__main__":
    run_ability(AbilityRole.xxx, "Xxx.Demo")
```

**package.yaml**：

```yaml
name: xxx.demo
version: 1.0.0
arch: x86_64
```

**ability.manifest.yaml**（完整示例见 abilities/gripper/ability.manifest.yaml）：

```yaml
abilityName: Xxx.Demo          # 与 main.py 第二个参数一致
kind: AtomAbility
schema:
  config:
    openAPIV3Schema:
      description: 能力配置结构
      properties:
        robotProfilePath:
          description: R1 Pro SDK profile 路径
          type: string
tasks:
- params:                       # 与 execute() 中 input_data.get(...) 的键一一对应
  - description: 幂等调用 ID（可选）
    name: invocation_id
    optional: true
    type: string
  - description: 目标位置
    name: position
    optional: false
    type: float
  - description: 运动超时时间,单位为秒,默认为10.0
    name: timeout
    optional: true
    type: float
  returns:                      # 固定结构：output{invocation_id/task_name/status/progress/phase/command_id/result/error} + message
  - description: 执行记录字典
    name: None
    type: object
    properties: ...
  summary: 控制 xxx 移动到目标位置
  taskName: control_xxx         # 与 Task.name 一致
  taskType: 0                   # 与 Task.task_type 一致
```

**crs/xxx-xxx.demo.cr.yaml**：

```yaml
kind: AtomAbility
metadata:
  name: xxx-demo
spec:
  abilityName: Xxx.Demo        # find_cr 按此字段查找 CR，必须与 main.py 一致
  package: xxx.demo
  version: 1.0.0
  position: localhost
  config:
    robotProfilePath: /home/liuxinjd/LiuXin/Jushi/SEMF/skill_ability/src/galaxea_r1pro_0/config/r1pro-real.json
  tasks:
    - taskType: 0
      taskName: control_xxx
  autoStart: false
  singleton: false
```

### Step 5 流程测试：`tests/test_xxx_flow.py`

参照 tests/test_torso_control_flow.py：test_mode SDK（不依赖真机）+ mock 反馈 + `R1ProAbilityService.invoke` 全流程。

```python
"""Xxx 控制 Task 全流程测试（test_mode SDK，mock 状态反馈）。"""

from __future__ import annotations

import unittest
from unittest import mock

from galaxea_r1pro import (
    BackendKind,
    CommandFeedback,
    CommandResult,
    CommandStatus,
    RobotProfile,
    R1ProSDK,
)

from galaxea_r1pro_0.service.ability_service import R1ProAbilityService
from galaxea_r1pro_0.tasks import role_task_types


def _state_result(data) -> CommandResult:
    return CommandResult(
        command_id="",
        status=CommandStatus.SUCCEEDED,
        feedback=CommandFeedback("", CommandStatus.SUCCEEDED, 1.0, "completed"),
        output={"data": data},
    )


class XxxControlFlowTest(unittest.TestCase):
    def _sdk(self) -> R1ProSDK:
        return R1ProSDK.open(
            RobotProfile(
                robot_id="r1pro-ability-test",
                backend=BackendKind.REAL,
                firmware_profile="test-v1",
                options={"test_mode": True, "defer_commands": True},
            )
        )

    def test_role_task_types(self) -> None:
        self.assertEqual(role_task_types("xxx"), ((0, "control_xxx"),))

    def test_control_xxx_full_flow(self) -> None:
        sdk = self._sdk()
        try:
            service = R1ProAbilityService("xxx", sdk)
            with mock.patch.object(
                sdk.sensors,
                "get_xxx_state",
                return_value=_state_result(0.0),
            ):
                result = service.invoke("control_xxx", {"position": 0.0})
            self.assertEqual(result["status"], "completed")
        finally:
            sdk.close()


if __name__ == "__main__":
    unittest.main()
```

运行：

```bash
cd src/galaxea_r1pro_0
python -m unittest discover -s tests -v
```

## 4. 一致性对照表（核心检查项）

一个 Ability 由 5 处配置互相咬合，任何一处不一致都会导致启动失败或 Task 找不到：

| # | 位置 | 字段 | 必须等于 |
|---|---|---|---|
| 1 | `tasks/xxx.py` | `Task.name` | manifest 的 `taskName`、CR 的 `taskName` |
| 2 | `tasks/xxx.py` | `Task.task_type` | manifest 的 `taskType`；业务 Task 从 0 连续编号 |
| 3 | `tasks/__init__.py` | `ROLE_TASK_SPECS` 键 | `AbilityRole.xxx` 的枚举值 |
| 4 | `abilities/xxx/main.py` | `AbilityRole.xxx` | roles.py 中已定义 |
| 5 | `abilities/xxx/main.py` | `"Xxx.Demo"` | manifest 的 `abilityName`、CR 的 `spec.abilityName` |
| 6 | manifest params | `name/optional/type` | `execute()` 中 `input_data.get()` 的键 |
| 7 | CR | `spec.config.robotProfilePath` | SDK RobotProfile JSON 路径（config/r1pro-real.json） |
| 8 | manifest/CR tasks | 列表内容 | 一致，且覆盖全部业务 Task |

## 5. 常见问题

| 现象 | 原因 | 处理 |
|---|---|---|
| 启动报 `没有找到 Xxx.Demo 的 CR` | CR 的 `spec.abilityName` 与 main.py 不一致，或 crs 目录不在查找路径 | 核对 abilityName；开发期设 `ABILITY_CR_PATH` 指向 CR 文件 |
| 启动报 `没有找到 ... 的 CR，也未设置 ROBOT_PROFILE_PATH` | 同上 | 同上，或直接设 `ROBOT_PROFILE_PATH` |
| `invoke` 报 `未注册 Task: xxx` | `Task.name` 与调用名不一致，或 `ROLE_TASK_SPECS` 漏注册 | 核对 Step 2 与对照表 #1 |
| 实例一直 standby 无法 running | rosbridge 未先启动 | 先开 rosbridge 再启动能力（详见 Galaxea_Ability_Test.md） |
| GetExecution 一直 running | SDK profile 用了 `defer_commands: true` 且无回调推进 | 仿真/真机联调时将 `options.defer_commands` 设为 false |

## 6. 部署与联调

- 框架部署、打包上传、实例启动与调试指令见 [Galaxea_Ability_Test.md](Galaxea_Ability_Test.md)；
- MCP 自然语言调用见 [Ability_Mcp.md](Ability_Mcp.md)；
- commit 按外层 README 规范：`feat(Add): 新增 xxx 能力（含 Task/manifest/CR/测试）`。
