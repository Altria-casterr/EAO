"""业务 Task 基类：Task 直接持有 sdk / executions / models，不依赖 service 与框架。"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any, Mapping

from galaxea_r1pro import R1ProSDK

from ..service.execution import ExecutionRegistry
from ..service.models import ModelProfileRegistry


class AbilityTask(ABC):
    """业务 Task 基类。

    子类声明 name / summary / task_type，并实现 execute()；
    SDK / 执行记录 / 模型在构造时直接注入，Task 内部可自由承载
    反馈控制、视觉检测等复杂业务逻辑。
    """

    name: str
    summary: str
    task_type: int | None = None

    def __init__(
        self,
        sdk: R1ProSDK,
        executions: ExecutionRegistry | None = None,
        models: ModelProfileRegistry | None = None,
        handler: Any | None = None,
    ) -> None:
        self.sdk = sdk
        self.executions = executions
        self.models = models
        self.handler = handler

    @abstractmethod
    def execute(self, input_data: Mapping[str, Any]) -> Mapping[str, Any]:
        """执行任务并返回标准化结果。"""
