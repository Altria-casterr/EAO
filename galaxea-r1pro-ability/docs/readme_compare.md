# Mock-ChassisMove 能力代码流转逻辑（旧实现 × 参考骨架对照）

> 以 `skill-Ability/ability/Mock-ChassisMove`（真实能力包）为主线，梳理从
> **框架激活实例 → 生命周期回调 → Task 注册 → MCP 调用 → 任务执行** 的完整代码流转；
> 并与 `example/Semantic/skill-Ability`（参考骨架 `r1pro_abilities`）的新形态对照，
> 标出关键点（task 注册、生命周期、CR 解析、SDK 创建、MCP 动态工具）。

## 1. 能力包结构

```
Mock-ChassisMove/
├── bin/ability                        # 启动脚本（exec python3 main.py）
├── ability.manifest.yaml              # 能力清单：abilityName / tasks（含参数与返回 Schema）
├── package.yaml                       # 包名 mock.chassismove.demo / 版本 / 架构
├── main.py                            # ★ 入口：创建共享实例 + task 注册 + 启动服务
├── task.py                            # MoveToPositionTask（业务执行）
├── service/
│   ├── __init__.py                    # 导出 ImplAbility 与 ability_py.task_manager
│   └── ability.py                     # ImplAbility（生命周期 + CR 解析 + 控制器构建 + task_server）
├── crs/mock-chassismove-demo.cr.yaml  # CR：spec.config.robotUri 等部署配置
└── skills/SKILL.md                    # MCP agent 使用说明（与 manifest task 一一对应）
```

## 2. 启动链路：`bin/ability → main.py`

框架启动实例时执行 `bin/ability`，其内部就是：

```bash
exec python3 "$SCRIPT_DIR/main.py" "$@"
```

`main.py` 是**整个能力的装配点**：

```python
from service import ImplAbility, task_manager
import ability_py
from task import MoveToPositionTask

# 共享能力实例（进程内只有一个，Task 都引用它）
ability_x = ImplAbility()

# ★ 关键：task 注册（taskType → Task 实例 的映射表）
tasks = {
    0: MoveToPositionTask(ability_x),
}

# ★ 关键：注册到 ability_py 的全局 task_manager（task_server 按 taskType 查表分发）
task_manager.register_tasks(tasks)

if __name__ == "__main__":
    # 阻塞运行：把生命周期交给 AbilityFramework（on_start/on_connect/...）
    service = ability_py.AbilityService()
    service.run(ability_x)
```

> **关键点 ①：task 注册**。`task_manager.register_tasks({taskType: TaskInstance})`
> 是「manifest 里声明的 taskType ↔ 进程内 Task 实现」的绑定点。
> 框架侧 `POST /api/ability/:id/api/task/start` 携带 taskType，
> task_server 据此查表，调用对应 Task 的 `execute()`。

## 3. 生命周期：`ImplAbility`

### 3.1 `__init__` — 实例化即完成「CR 解析 + 控制器构建」

```python
class ImplAbility(ability_py.AbilityInterface):
    def __init__(self, ability_name="MockChassisMove.Demo"):
        self.ability_port = 0
        self.st = None
        self.ability_name = ability_name

        # ① 按 abilityName 查找并读取 CR（crs/*.yaml）
        self.config_path = _find_config_for_ability(ability_name)
        self.config_text = self.config_path.read_text(encoding="utf-8")

        # ② 从 CR 提取部署参数 robotUri（真机 rosbridge 地址）
        robot_uri = self._get_config_value("robotUri", "ws://192.168.1.235:9091")
        self.robot_host, self.robot_port = _parse_uri_host_port(robot_uri, "192.168.1.235", 9091)

        # ③ 构建业务控制器（旧形态：直接引用 skill_library 的 Atom 类）
        self.simulation_controller = self._build_simulation_controller()

    def _build_simulation_controller(self) -> WooshMove:
        # 旧 skill_library 引用：Atom.WooshMove.Galaxea_R1Pro.scripts.main.WooshMove
        return WooshMove(ros_host=self.robot_host, ros_port=self.robot_port)
```

> **关键点 ②：CR 配置解析**。`_find_config_for_ability → _extract_field(robotUri)`
> 是「部署配置 ↔ 控制器实例」的绑定点。旧形态把 rosbridge 地址直接喂给旧 Atom 控制器。

### 3.2 `on_connect` — 连接检查 + 启动 task 代理服务

```python
def on_connect(self):
    # ★ 关键：rosbridge 连接检查（不存活则重建控制器）
    if not self.simulation_controller.ros_connection.client.is_connected:
        logging.error("SimulationController未初始化")
        self.simulation_controller = self._build_simulation_controller()

    # ★ 关键：启动 task_server（werkzeug 承载 ability_py.task_server.app）
    if self.ability_port:
        return
    self.ability_port = ability_py.get_free_port()
    self.st = ServerThread(host="0.0.0.0", port=self.ability_port)
    self.st.start()
```

> **关键点 ③：task 代理端口**。`get_ability_port()` 返回的端口通过心跳上报框架，
> 框架据此把 `/api/ability/:id/api/task/start` 代理到该端口的 task_server。
> **关键点 ④：连接检查**。旧形态手动检查 `client.is_connected` 并重建控制器——
> 这正是我们刚在 Robot_SDK 里做成 `is_connected()` + `connect()` 断连自愈的场景。

### 3.3 `on_disconnect / on_terminate` — 释放

```python
def on_disconnect(self):
    if self.simulation_controller.ros_connection.client.is_connected:
        self.simulation_controller.close()      # 关 rosbridge
    if self.ability_port != 0:
        self.st.shutdown()                       # 停 task_server
        self.ability_port = 0
```

## 4. 任务执行：`task.py`

```python
class MoveToPositionTask(TaskInterface):
    def execute(self, input_data: dict) -> dict:
        # 连接检查（与 on_connect 同样的防御）
        simulation_controller = self.ability.simulation_controller
        if not simulation_controller.ros_connection.client.is_connected:
            simulation_controller = self.ability._build_simulation_controller()

        # 参数解析（缺省值）
        axis = input_data.get("axis", "x")
        target_value = float(input_data.get("target_value", 0.0))
        speed = float(input_data.get("speed", 0.0))
        tolerance = float(input_data.get("tolerance", 0.01))
        timeout = float(input_data.get("timeout", 10.0))

        # 调用旧 Atom 控制器的闭环运动
        result = simulation_controller.move_robot(
            axis=axis, target_value=target_value, speed=speed,
            tolerance=tolerance, timeout=timeout,
        )
        while simulation_controller._is_moving:      # 轮询闭环结束
            time.sleep(0.1)

        # ★ 关键：标准化输出（{"output": {...}, "message": ...}）
        if not result["success"]:
            return _failure("移动执行失败", "moveToPosition returned False", ...)
        return _success({"axis": axis, ..., "success": True}, "移动到目标位置完成")
```

> **关键点 ⑤：MCP 动态工具 ↔ Task**。manifest 里的 `MoveToPosition`（taskType 0）
> 被 MCP server 生成动态工具 `mock_move_demo__move_to_position(...)`，
> 调用链为：MCP tool → `POST /api/ability/:id/api/task/start` → task_server
> → `task_manager` 查表 → `MoveToPositionTask.execute(input_data)`。

## 5. 完整调用时序

```
AbilityFramework                能力进程(main.py)                  Robot（rosbridge）
      │  activate 实例                │                                 │
      │ ── bin/ability ─────────────▶│                                 │
      │                              │ ImplAbility()                  │
      │                              │  · 读 CR → robotUri            │
      │                              │  · WooshMove(host, port) ─────▶│ 连接
      │                              │ task_manager.register_tasks()  │
      │                              │ AbilityService().run()         │
      │ ◀── 心跳(port) ──────────────│                                 │
      │ on_connect                   │  · 检查 is_connected            │
      │                              │  · task_server 启动             │
      │ MCP: start_ability ─────────▶│                                 │
      │ MCP: move_to_position ──────▶│ task/start → MoveToPositionTask │
      │                              │  · move_robot() ──────────────▶│ 发布 TwistStamped
      │                              │  · 轮询 _is_moving             │
      │ ◀── {output, message} ───────│                                 │
      │ MCP: stop_ability ──────────▶│ on_disconnect（关控制器/服务）   │
```

## 6. 参考骨架对照（新形态 `r1pro_abilities`）

### 6.1 入口对照：手写 main.py → `run_ability(role, ability_name)`

| 旧形态（Mock-ChassisMove） | 参考骨架（r1pro_abilities） |
|---|---|
| `main.py` 手写 `ImplAbility()` + `tasks={0: MoveToPositionTask(...)}` + `register_tasks` | `main.py` 一行：`run_ability(AbilityRole.BASE_MOTION, "R1ProBaseMotion.V1")` |
| 每个能力各自维护生命周期/控制器/端口 | `entrypoint.py` 统一 `Lifecycle` + `DispatchTask` |
| task 注册手写，taskType 由开发者编号 | `TASKS[role]` 枚举 + 自动追加 `GetExecution/StopExecution`，**统一注册** |

参考骨架的 task 注册：

```python
# entrypoint.py（★ 关键：统一 task 注册）
task_names = (*TASKS[role], "GetExecution", "StopExecution")
task_manager.register_tasks(
    {task_type: DispatchTask(lifecycle, task_name)
     for task_type, task_name in enumerate(task_names)}
)
```

### 6.2 CR 解析对照：robotUri → robotProfilePath

| 旧形态 | 参考骨架 |
|---|---|
| `_extract_field("robotUri")` → rosbridge 地址 → 旧 Atom 控制器 | `RuntimeConfig.load(ability_name)` → `spec.config.robotProfilePath` → `R1ProSDK.open(RobotProfile.load(...))` |
| 控制器构建写在能力里 | `on_connect` 里统一 `R1ProSDK.open` + `ModelProfileRegistry.load` |

```python
# entrypoint.py on_connect（★ 关键：进程内 SDK 创建）
config = RuntimeConfig.load(ability_name)
self.sdk = R1ProSDK.open(RobotProfile.load(config.robot_profile_path))
self.service = R1ProAbilityService(role, self.sdk, models)
```

### 6.3 任务执行对照：旧 Atom 调用 → SDK 能力 + 执行记录

| 旧形态（task.py） | 参考骨架（service.py） |
|---|---|
| 直接调 `simulation_controller.move_robot(...)` + 轮询 `_is_moving` | `service.invoke(task_name, input_data)` → 分派到 `sdk.woosh_move.move_robot(...)` 等 |
| 返回 `_success/_failure` | `ExecutionRegistry.register(handle)` + `refresh()`（invocation_id 幂等查询/停止） |
| 无执行记录 | `GetExecution / StopExecution` 两个内置 Task 统一暴露执行状态 |

### 6.4 连接检查对照：手动重建 → SDK 断连自愈

| 旧形态 | 参考骨架/新 SDK |
|---|---|
| `on_connect`/`execute` 里 `if not controller.ros_connection.client.is_connected: 重建` | `R1ProSDK.open` 按 endpoint 单例复用 + `is_connected()` + `connect()` 断连自愈（`afd539d`） |

## 7. 关键点速查表

| # | 关键点 | 位置 | 作用 |
|---|---|---|---|
| ① | task 注册 | `main.py` / `entrypoint.py` | manifest taskType ↔ 进程内 Task 绑定，task_server 查表分发 |
| ② | CR 配置解析 | `service/ability.py` / `entrypoint.RuntimeConfig` | 部署参数（robotUri / robotProfilePath）→ 控制器/SDK |
| ③ | task 代理端口 | `on_connect` + `get_ability_port` | 框架 `/api/ability/:id/api/task/*` 代理到能力进程 |
| ④ | 连接检查 | `on_connect` / `execute` | rosbridge 不存活时重建控制器（新 SDK 已自愈） |
| ⑤ | MCP 动态工具 | MCP server（manifest 生成） | `ability__task` → task/start → Task.execute |
| ⑥ | 执行记录 | `ExecutionRegistry`（参考骨架） | invocation_id 幂等、GetExecution/StopExecution 查询停止 |
| ⑦ | 标准化输出 | `_success/_failure` / `record.to_dict()` | 统一 `{output, message}` / `{invocation_id, status, ...}` |

## 8. 结论

改造的本质是把「旧能力包里的三块手动样板」替换为参考骨架的统一实现：

1. **生命周期样板**（CR 解析 + 控制器构建 + task_server）→ `r1pro_abilities.entrypoint.run_ability`
2. **task 注册样板**（手写 tasks 表）→ `TASKS[role]` + `GetExecution/StopExecution` 统一注册
3. **业务调用样板**（旧 Atom 控制器 + 手动轮询）→ `R1ProAbilityService` 分派到真实 Robot_SDK 能力（如 `woosh_move.move_robot`），
   并复用新 SDK 的单例复用与断连自愈。

## 9.新结构
 Config:存储当前能力的相关描述信息
 ability：主要函数目录--但是当前如何封装一个能力还没想好，是否把task放到当前目录中
   -- task.py 负责当前能力的业务逻辑代码--考虑invoke函数应该如何接入
   -- main.py主函数
      -- 所有的共用一个entrypoint函数建立连接
 Utils：功能函数层
  - utils_common.py 共同使用的功能函数、例如坐标转换等
  - utils_xxx.py 独立于特定控制函数接口使用
service：因为当前的是一个robot_sdk，所以可以使用一个service完成所有的连接建立
   -- entrypoint.py 只是负责建立连接，不负责注册task
   -- execution.py暂时保留跟现在一样
   -- models.py 暂时保留跟现在一样

### 9.1 设计定稿：Task 承载业务，invoke 只引用 Task

结论：**可行**。分层原则：

- `service` 层：连接（entrypoint 只建连）、执行记录（execution）、模型（models）、分发（invoke → TaskRegistry）
- `ability/tasks` 层：每个 Task 一个类，承载全部业务逻辑（SDK 调用、反馈控制、YOLO 检测等）
- `invoke(task_name, input_data)` **不引用底层 SDK**，只查 TaskRegistry 并调用 task.execute()
- Task 通过注入的 TaskContext 拿 SDK / 执行记录 / 模型，不直接依赖 service

```python
# service/task_context.py —— 注入给 Task 的上下文

class TaskContext:
    sdk: R1ProSDK                    # 单例复用的 SDK（entrypoint 建立）
    executions: ExecutionRegistry    # 物理命令执行记录
    models: ModelProfileRegistry | None = None
```

```python
# ability/task.py —— 业务 Task 基类
class AbilityTask(ABC):
    name: str          # taskName，与 manifest 一致
    summary: str       # 描述，可生成 manifest 与 MCP 动态工具
    task_type: int | None = None     # 显式 taskType（可选）

    def __init__(self, ctx: TaskContext) -> None:
        self.ctx = ctx

    
    def execute(self, input_data: Mapping[str, Any]) -> Mapping[str, Any]:
        """执行任务，返回标准化结果。"""
```

```python
# service/registry.py —— task 映射表（task_name → Task 实例）
class TaskRegistry:
    def __init__(self) -> None:
        self._tasks: dict[str, AbilityTask] = {}
        self._types: dict[int, str] = {}

    def add(self, task: AbilityTask) -> "TaskRegistry":
        self._tasks[task.name] = task
        if task.task_type is not None:
            self._types[task.task_type] = task.name
        return self

    def get(self, task_name: str) -> AbilityTask:
        task = self._tasks.get(task_name)
        if task is None:
            raise ValueError(f"未注册 Task: {task_name}")
        return task

    def names(self) -> tuple[str, ...]:
        return tuple(self._tasks)
```

```python
# service/service.py —— invoke 改造：不引用 SDK，只引用 Task
class R1ProAbilityService:
    def __init__(self, role, sdk, models=None, tasks: TaskRegistry | None = None):
        self.executions = ExecutionRegistry()
        ctx = TaskContext(sdk, self.executions, models)
        self.tasks = tasks or build_role_tasks(role, ctx)

    def invoke(self, task_name, input_data):
        if task_name == "GetExecution":
            return self.executions.refresh(self.sdk, str(input_data["invocation_id"])).to_dict()
        if task_name == "StopExecution":
            return self.executions.stop(self.sdk, str(input_data["invocation_id"]), str(input_data.get("reason", "pilot stop"))).to_dict()
        task = self.tasks.get(task_name)      # ★ 只引用 Task，不引用底层 SDK
        return task.execute(input_data)
```

Task 注册方式（二选一，建议 A）：
- A（推荐）：每个 Task 声明 name / task_type，`ability/tasks/__init__.py` 提供 `build_role_tasks(role, ctx)` 按角色组装
- B：各 task 文件模块级自注册到全局 registry


### 9.2 第一个 Task 骨架：BaseMotion.NavigateToPose

```python
# ability/tasks/base_motion.py —— 第一个业务 Task（底盘导航）
from ability.task import AbilityTask, TaskContext

class NavigateToPoseTask(AbilityTask):
    name = "NavigateToPose"
    summary = "在指定坐标系中导航到底盘目标位姿"
    task_type = 0

    def execute(self, input_data: Mapping[str, Any]) -> Mapping[str, Any]:
        sdk = self.ctx.sdk   # 通过上下文拿 SDK，invoke 不感知

        # 业务逻辑：导航（SDK 内部已含解锁刹车 / 等待完成 / 超时取消 / 断连自愈）
        result = sdk.woosh_navigation.navigate_to_map_pose(
            x=float(input_data["x"]),
            y=float(input_data["y"]),
            yaw=float(input_data.get("yaw", 0.0)),
            frame_id=str(input_data.get("frame_id", "map")),
            timeout=float(input_data.get("timeout", 30.0)),
        )

        # 结果判定（业务层做语义转换）
        if result.status is CommandStatus.SUCCEEDED:
            return {"ok": True, "success": True, "status": result.status.value}
        return {"ok": False, "success": False, "status": result.status.value,
                "error": result.error_message or "导航未成功"}
```

注册（ability/tasks/__init__.py，建议方案 A）：

```python
def build_role_tasks(role: AbilityRole, ctx: TaskContext) -> TaskRegistry:
    registry = TaskRegistry()
    if role is AbilityRole.BASE_MOTION:
        registry.add(NavigateToPoseTask(ctx)).add(MoveRelativeTask(ctx))
    if role is AbilityRole.UPPER_BODY_MOTION:
        registry.add(MoveEndEffectorTask(ctx)).add(MoveJointsTask(ctx)).add(FollowTrajectoryTask(ctx))
    if role is AbilityRole.gripper_controller:
        registry.add(SetGripperTask(ctx)).add(GraspUntilForceTask(ctx)).add(ReleaseTask(ctx))
    if role is AbilityRole.ROBOT_OBSERVATION:
        registry.add(GetRobotStateTask(ctx)).add(CaptureRGBDTask(ctx)).add(VerifyHoldingTask(ctx))
    if role is AbilityRole.OBJECT_PERCEPTION:
        registry.add(DetectObjectsTask(ctx)).add(SegmentObjectTask(ctx)).add(EstimateObjectPoseTask(ctx))
    if role is AbilityRole.GRASP_PLANNING:
        registry.add(PlanGraspCandidatesTask(ctx))
    if role is AbilityRole.POLICY_CONTROL:
        registry.add(RunPolicyTask(ctx))
    return registry
```

任务分派结论：`invoke` 只做「查表 + execute」，业务全部下沉到 `ability/tasks/*.py`；
后续新增 YOLO 检测、反馈控制等复杂逻辑只改对应 Task 文件，不动 service。

开工前待确认的 2 个接口细节：
1. `ExecutionRegistry`（参考骨架用 sdk.command.get / sdk.command.stop / sdk.safety.hold）需要对齐真实 Robot_SDK 的 CommandClient / SafetyClient 方法名
2. Task 输出统一格式：`{ok, success, status, error, ...}` vs 参考骨架的 `record.to_dict()`

## 10. run_ability 代码流转逻辑（v3.0.0 registry 版）

### 10.1 入口：能力包 main.py

```python
# abilities/gripper/main.py
from galaxea_r1pro_0.service.entrypoint import run_ability
from galaxea_r1pro_0.service.roles import AbilityRole

if __name__ == "__main__":
    run_ability(AbilityRole.gripper_controller, "Gripper.Demo")
```

框架启动实例时执行 `bin/ability → main.py`，进入 `run_ability`。

### 10.2 run_ability 装配（service/entrypoint.py）

```python
def run_ability(role: AbilityRole, ability_name: str) -> None:
    lifecycle = ImplAbility(role, ability_name)          # ① 生命周期实例

    registered = list(role_task_types(role.value))       # ② 业务 task：(taskType, taskName)
    next_type = max((t for t, _ in registered), default=-1) + 1
    registered.append((next_type, "GetExecution"))       # ③ 框架内置 task 追加
    registered.append((next_type + 1, "StopExecution"))
    task_manager.register_tasks(                         # ④ ★ task 注册
        {task_type: DispatchTask(lifecycle, task_name)
         for task_type, task_name in registered}
    )

    AbilityService().run(lifecycle)                      # ⑤ 阻塞交还框架
```

分步说明：

| 步骤 | 动作 | 结果 |
|---|---|---|
| ① | `ImplAbility(role, ability_name)` | 生命周期实例，记住 role（如 grippercontroller）与 ability_name（如 Gripper.Demo），供 CR 查找/日志 |
| ② | `role_task_types("grippercontroller")` | 来自 `tasks/__init__.py` 的 `ROLE_TASK_SPECS`：`((0,"HandleSingleGripper"), (1,"HandleBothGripper"))`，taskType 由 Task 类声明 |
| ③ | 追加内置 task | `(2,"GetExecution"), (3,"StopExecution")`，与 manifest 一致 |
| ④ | `task_manager.register_tasks(...)` | ★ 框架 task_server 按 taskType 查表的绑定点：每个 taskType → `DispatchTask(lifecycle, task_name)` |
| ⑤ | `AbilityService().run(lifecycle)` | 阻塞运行，框架接管生命周期（on_start/on_connect/...） |

### 10.3 生命周期（框架回调 ImplAbility）

```python
on_start()      # 记录 _started_at + _preload_resources 钩子（预加载模型等）
on_connect()    # ★ 连接阶段
  ├─ RuntimeConfig.load(ability_name)
  │    ├─ 环境变量 ROBOT_PROFILE_PATH / MODEL_REGISTRY_PATH 优先
  │    └─ 否则 find_cr(ability_name) → CR 的 spec.config.robotProfilePath
  ├─ sdk = R1ProSDK.open(RobotProfile.load(robot_profile_path))   # ★ 单例复用 + 断连自愈
  ├─ models = ModelProfileRegistry.load(...) 或 None
  ├─ service = R1ProAbilityService(role.value, sdk, ExecutionRegistry(), models)
  │    └─ 内部 build_role_tasks → TaskRegistry（实例化业务 Task）
  ├─ ability_port = get_free_port()
  └─ ServerThread(port).start()     # task 代理服务（werkzeug + ability_py.task_server.app）
get_ability_port()  # 端口经心跳上报框架，供 /api/ability/:id/api/task/* 代理
on_disconnect()     # 停 task_server、sdk.close()（引用计数）、置空
on_terminate()      # = on_disconnect()
```

### 10.4 任务调用：MCP → task_server → DispatchTask → invoke

```text
MCP 动态工具 mock_gripper__handle_single_gripper(...)
  → POST /api/ability/:id/api/task/start
  → 能力进程 task_server（ability_port）
  → task_manager 按 taskType 查表
  → DispatchTask.execute(input_data)
      ├─ service is None → RuntimeError("Ability 尚未连接 Robot SDK")
      └─ service.invoke(task_name, input_data)
```

### 10.5 invoke 分派（service/ability_service.py）

```python
def invoke(self, task_name, input_data):
    if task_name == "GetExecution":                       # 框架内置：查执行记录
        return self.executions.refresh(self.sdk, invocation_id).to_dict()
    if task_name == "StopExecution":                     # 框架内置：停止 + hold
        return self.executions.stop(self.sdk, invocation_id, reason).to_dict()
    task = self.tasks.get(task_name)                     # ★ 业务 Task：只查表
    return dict(task.execute(input_data))                # Task 内部用 self.sdk 调底层能力
```

### 10.6 完整时序

```text
AbilityFramework          能力进程(main.py)                  Robot SDK
      │ activate             │                                 │
      │ ── run_ability ─────▶│                                 │
      │                      │ ImplAbility(role, name)        │
      │                      │ role_task_types + Get/Stop      │
      │                      │ task_manager.register_tasks()   │
      │                      │ AbilityService().run()          │
      │ on_start ───────────▶│ 记录时间/预加载                  │
      │ ◀── 心跳(port) ──────│                                 │
      │ on_connect ─────────▶│ RuntimeConfig.load → find_cr    │
      │                      │ R1ProSDK.open(RobotProfile) ───▶│ 连接/复用
      │                      │ R1ProAbilityService(role, sdk)  │
      │                      │ ServerThread(port).start()      │
      │ MCP: start_ability ─▶│                                 │
      │ MCP: handle_gripper ─▶│ task/start → DispatchTask      │
      │                      │   └─ service.invoke             │
      │                      │       └─ Task.execute → sdk ───▶│ 发布指令
      │ ◀── {output,message}─│                                 │
      │ MCP: stop_ability ──▶│ on_disconnect（停服务/关 SDK）   │
```

### 10.7 关键点速查

| # | 关键点 | 位置 |
|---|---|---|
| ① | task 注册（taskType → DispatchTask） | `run_ability` |
| ② | 业务 task 声明（ROLE_TASK_SPECS / name / task_type） | `tasks/__init__.py`、各 Task 类 |
| ③ | SDK 创建（单例复用 + 断连自愈） | `ImplAbility.on_connect` |
| ④ | invoke 只引用 TaskRegistry | `R1ProAbilityService.invoke` |
| ⑤ | 任务代理端口上报 | `get_ability_port` |
| ⑥ | CR 配置解析 | `RuntimeConfig.load → utils.find_cr` |
