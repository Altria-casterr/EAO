"""registry 版能力服务：invoke 只引用 TaskRegistry，不引用底层 SDK。"""

from __future__ import annotations

import threading
from typing import Any, Mapping

from galaxea_r1pro import CommandHandle, R1ProSDK

from ..tasks import build_role_tasks
from ..abilities.yolo_detect.handler import ObjectPerceptionHandler
from .execution import ExecutionRegistry
from .models import ModelProfileRegistry


class R1ProAbilityService:
    """Task 分派：业务 Task 走 TaskRegistry；GetExecution/StopExecution 走执行记录。"""

    def __init__(
        self,
        role: str,
        sdk: R1ProSDK,
        executions: ExecutionRegistry | None = None,
        models: ModelProfileRegistry | None = None,
        cameras: Mapping[str, Any] | None = None,
    ) -> None:
        self.role = role
        self.sdk = sdk
        self.executions = executions or ExecutionRegistry()
        self.models = models
        self.handler = ObjectPerceptionHandler(sdk, models, cameras) if role == "yolo_detect" else None
        self.tasks = build_role_tasks(role, sdk, self.executions, models, self.handler)
        self._start_lock = threading.RLock()

    def close(self) -> None:
        if self.handler is not None:
            self.handler.close()

    def invoke(self, task_name: str, input_data: Mapping[str, Any]) -> dict[str, Any]:
        if task_name == "GetExecution":
            return self.executions.refresh(self.sdk, str(input_data["invocation_id"])).to_dict()
        if task_name == "StopExecution":
            return self.executions.stop(
                self.sdk,
                str(input_data["invocation_id"]),
                str(input_data.get("reason", "pilot stop")),
            ).to_dict()
        return self._invoke_business_task(task_name, input_data)

    def _invoke_business_task(self, task_name: str, data: Mapping[str, Any]) -> dict[str, Any]:
        """业务 Task 分派（借鉴参考骨架 _invoke_business_task）。

        - invocation_id 幂等：已存在则刷新返回；task 名不一致则报错
        - 物理 Task 返回 CommandHandle → 登记执行记录（running，可 Get/Stop）
        - 同步 Task 返回 dict → register_completed
        """

        invocation_id = str(data.get("invocation_id") or "") or None
        with self._start_lock:
            if invocation_id:
                existing = self.executions.find(invocation_id)
                if existing is not None:
                    if existing.task_name != task_name:
                        raise ValueError(
                            f"invocation_id {invocation_id} 已用于 {existing.task_name}，不能改为 {task_name}"
                        )
                    return self.executions.refresh(self.sdk, invocation_id).to_dict()

            task = self.tasks.get(task_name)
            result = task.execute(data)

            if isinstance(result, CommandHandle):
                record = self.executions.register(result, invocation_id, task_name)
                return self.executions.refresh(self.sdk, record.invocation_id).to_dict()
            if isinstance(result, (list, tuple)) and result and isinstance(result[0], CommandHandle):
                record = self.executions.register(result[0], invocation_id, task_name)
                return self.executions.refresh(self.sdk, record.invocation_id).to_dict()

            observations = tuple(result.get("observations") or ())
            record = self.executions.register_completed(task_name, dict(result), invocation_id, observations)
            return record.to_dict()
