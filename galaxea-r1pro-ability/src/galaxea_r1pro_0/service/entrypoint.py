"""使用现有 ability_py.task_manager 注册 Task 的薄启动层（v3.0.0 registry 版）。

- 连接：Robot SDK（galaxea_r1pro）单例复用 + 断连自愈
- Task：由 ability/tasks 的 ROLE_TASK_SPECS 声明，本模块只负责注册框架 taskType
- invoke：R1ProAbilityService 只引用 TaskRegistry，不引用底层 SDK
- ServerThread / ImplAbility / DispatchTask 已在模块级抽离，run_ability 只做装配
"""

from __future__ import annotations

import logging
import os
import threading
import time
import yaml
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from ability_py import (
    AbilityInterface,
    AbilityService,
    TaskInterface,
    get_free_port,
    task_manager,
)
from ability_py.task_server import app
from galaxea_r1pro import R1ProSDK, RobotProfile
from werkzeug.serving import make_server

from ..tasks import role_task_types
from ..utils.utils_common import find_cr
from .ability_service import R1ProAbilityService
from .execution import ExecutionRegistry
from .models import ModelProfileRegistry
from .roles import AbilityRole

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class RuntimeConfig:
    robot_profile_path: str
    model_registry_path: str | None

    @classmethod
    def load(cls, ability_name: str) -> "RuntimeConfig":
        # 开发和测试允许环境变量覆盖；正式部署默认读取当前 Ability 的 CR spec.config。
        robot_profile = os.environ.get("ROBOT_PROFILE_PATH")
        model_registry = os.environ.get("MODEL_REGISTRY_PATH")

        if robot_profile and ability_name != "YoloDetect.Demo":
            return cls(robot_profile, model_registry)

        cr_path = find_cr(ability_name)
        config = {}
        if cr_path is not None:
            cr_path = Path(cr_path)
            data = yaml.safe_load(cr_path.read_text(encoding="utf-8")) or {}
            config = ((data.get("spec") or {}).get("config") or {})
        elif not robot_profile:
            raise RuntimeError(f"没有找到 {ability_name} 的 CR，也未设置 ROBOT_PROFILE_PATH")
        robot_profile = robot_profile or config.get("robotProfilePath")
        if not robot_profile:
            raise RuntimeError(f"{cr_path} 缺少 spec.config.robotProfilePath")
        model_registry = model_registry or config.get("modelRegistryPath")
        if not model_registry and ability_name == "YoloDetect.Demo":
            model_registry = Path(__file__).resolve().parents[1] / "config" / "models.example.json"
        return cls(str(robot_profile), str(model_registry) if model_registry else None)

class ServerThread(threading.Thread):
    """沿用 AbilityFramework 的任务服务，不创建第二套运行时。"""

    def __init__(self, port: int, host: str = "0.0.0.0") -> None:
        super().__init__(daemon=True)
        self.server = make_server(host, port, app)

    def run(self) -> None:
        self.server.serve_forever()

    def shutdown(self) -> None:
        self.server.shutdown()

class ImplAbility(AbilityInterface):
    """能力生命周期：连接 Robot SDK、组装 R1ProAbilityService、启动 task_server。"""

    def __init__(self, role: AbilityRole, ability_name: str) -> None:
        self.role = role
        self.ability_name = ability_name
        self.ability_port = 0
        self.task_server: ServerThread | None = None
        self.sdk: R1ProSDK | None = None
        self.service: R1ProAbilityService | None = None

    def on_start(self) -> None:
        """启动回调（借鉴 Mock-X ImplAbility.on_start）。"""

        logger.warning("%s ability on_start", self.ability_name)
        self._started_at = time.time()
        preload = getattr(self, "_preload_resources", None)
        if callable(preload):
            try:
                preload()
            except Exception as exc:
                logger.error("Resource preload failed in on_start: %s", exc)

    def on_connect(self) -> None:
        config = RuntimeConfig.load(self.ability_name)

        # TODO：跨机器人泛化ability：
        # Step1、SDK的打开需要修改
        # 星海图对应：self.sdk = R1ProSDK.open(RobotProfile.load(config.robot_profile_path))
        # 其他机器人 slf.sdk = XXXopen(RobotProfile.load(config.robot_profile_path))

        self.sdk = R1ProSDK.open(RobotProfile.load(config.robot_profile_path), self.ability_name)

        models = ModelProfileRegistry.load(config.model_registry_path) if config.model_registry_path else None
        if self.role == AbilityRole.yolo_detect:
            logger.info(
                "YoloDetect model source: modelRegistryPath=%s, YOLO_CHECKPOINT_PATH=%s",
                config.model_registry_path or "<unset>",
                "configured" if os.environ.get("YOLO_CHECKPOINT_PATH") else "<unset>",
            )

        self.service = R1ProAbilityService(self.role.value, self.sdk, ExecutionRegistry(), models, cameras=models.cameras if models is not None else None)
        
        if self.ability_port == 0:
            self.ability_port = get_free_port()
            self.task_server = ServerThread(self.ability_port)
            self.task_server.start()

    def on_disconnect(self) -> None:
        if self.service is not None:
            self.service.close()
        if self.task_server is not None:
            self.task_server.shutdown()
        self.task_server = None
        self.ability_port = 0
        if self.sdk is not None:
            self.sdk.close()
        self.sdk = None
        self.service = None

    def on_terminate(self) -> None:
        self.on_disconnect()

    def get_ability_port(self) -> int:
        return self.ability_port

class DispatchTask(TaskInterface):
    """把框架的 task 调用转发到 R1ProAbilityService.invoke。"""

    def __init__(self, implability: ImplAbility, task_name: str) -> None:
        self.implability = implability
        self.task_name = task_name

    def execute(self, input_data: dict[str, Any]) -> dict[str, Any]:

        if self.implability.service is None:
            raise RuntimeError("Ability 尚未连接 Robot SDK")

        return self.implability.service.invoke(self.task_name, input_data)

def run_ability(role: AbilityRole, ability_name: str) -> None:
    """注册 Task 并交还给官方 AbilityService；不接管进程或任务调度。"""

    lifecycle = ImplAbility(role, ability_name)

    # ★ 关键：task 注册 —— 业务 Task 来自 ROLE_TASK_SPECS（taskType 由 Task 类声明），
    # 末尾追加框架内置的 GetExecution / StopExecution

    registered = list(role_task_types(role.value))

    next_type = max((task_type for task_type, _ in registered), default = -1) + 1

    registered.append((next_type, "GetExecution"))

    registered.append((next_type + 1, "StopExecution"))

    task_manager.register_tasks({task_type: DispatchTask(lifecycle, task_name) for task_type, task_name in registered})

    AbilityService().run(lifecycle)
