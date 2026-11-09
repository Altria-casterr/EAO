"""Ability 进程内的执行记录、反馈和真实停止（保留参考骨架实现，仅改 SDK 导入）。"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
from enum import StrEnum
from threading import RLock
from typing import Any, Mapping
from uuid import uuid4

from galaxea_r1pro import CommandHandle, CommandStatus, R1ProSDK


class ExecutionStatus(StrEnum):
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"
    STOPPED = "stopped"
    INTERRUPTED = "interrupted"


@dataclass
class ExecutionRecord:
    invocation_id: str
    task_name: str
    operation: str
    command_id: str
    resource: str
    status: ExecutionStatus
    progress: float
    phase: str
    feedback: Mapping[str, Any] = field(default_factory=dict)
    observations: tuple[Mapping[str, Any], ...] = ()
    result: Mapping[str, Any] = field(default_factory=dict)
    error: Mapping[str, Any] | None = None
    updated_at: datetime = field(default_factory=lambda: datetime.now(timezone.utc))

    def to_dict(self) -> dict[str, Any]:
        return {
            "invocation_id": self.invocation_id,
            "task_name": self.task_name,
            "operation": self.operation,
            "command_id": self.command_id,
            "resource": self.resource,
            "status": self.status.value,
            "progress": self.progress,
            "phase": self.phase,
            # semantic-pilot 的 AbilityFeedback 是数组契约；单条记录包装为一元素数组
            "feedback": [dict(self.feedback)] if self.feedback else [],
            "observations": [dict(item) for item in self.observations],
            "result": dict(self.result),
            "error": dict(self.error) if self.error else None,
            "updated_at": self.updated_at.isoformat(),
        }


class ExecutionRegistry:
    """只记录当前 Ability 实例启动的命令，不承担 AbilityFramework 生命周期。"""

    def __init__(self) -> None:
        self._records: dict[str, ExecutionRecord] = {}
        self._lock = RLock()

    def register(
        self,
        handle: CommandHandle,
        invocation_id: str | None = None,
        task_name: str | None = None,
    ) -> ExecutionRecord:
        invocation = invocation_id or str(uuid4())
        record = ExecutionRecord(
            invocation_id=invocation,
            task_name=task_name or handle.operation,
            operation=handle.operation,
            command_id=handle.command_id,
            resource=handle.resource,
            status=ExecutionStatus.RUNNING,
            progress=0.0,
            phase="accepted",
        )
        with self._lock:
            if invocation in self._records:
                raise ValueError(f"invocation_id 已存在: {invocation}")
            self._records[invocation] = record
        return record

    def find(self, invocation_id: str) -> ExecutionRecord | None:
        with self._lock:
            return self._records.get(invocation_id)

    def register_completed(
        self,
        task_name: str,
        result: Mapping[str, Any],
        invocation_id: str | None = None,
        observations: tuple[Mapping[str, Any], ...] = (),
    ) -> ExecutionRecord:
        invocation = invocation_id or str(uuid4())
        record = ExecutionRecord(
            invocation_id=invocation,
            task_name=task_name,
            operation=task_name,
            command_id="",
            resource="model_or_observation",
            status=ExecutionStatus.COMPLETED,
            progress=1.0,
            phase="completed",
            observations=observations,
            result=dict(result),
        )
        with self._lock:
            if invocation in self._records:
                raise ValueError(f"invocation_id 已存在: {invocation}")
            self._records[invocation] = record
        return record

    def refresh(self, sdk: R1ProSDK, invocation_id: str) -> ExecutionRecord:
        with self._lock:
            record = self._require(invocation_id)
            if record.status in {ExecutionStatus.STOPPED, ExecutionStatus.INTERRUPTED}:
                return record
            if not record.command_id:
                return record
            result = sdk.command.get(record.command_id)
            record.progress = result.feedback.progress
            record.phase = result.feedback.phase
            record.feedback = dict(result.feedback.metrics)
            record.updated_at = datetime.now(timezone.utc)
            if result.status is CommandStatus.SUCCEEDED:
                record.status = ExecutionStatus.COMPLETED
                record.result = dict(result.output)
            elif result.status is CommandStatus.FAILED:
                record.status = ExecutionStatus.FAILED
                record.error = {"code": result.error_code, "message": result.error_message}
            elif result.status is CommandStatus.UNKNOWN:
                record.status = ExecutionStatus.INTERRUPTED
            return record

    def stop(self, sdk: R1ProSDK, invocation_id: str, reason: str) -> ExecutionRecord:
        with self._lock:
            record = self._require(invocation_id)
            if record.status in {ExecutionStatus.COMPLETED, ExecutionStatus.FAILED, ExecutionStatus.STOPPED}:
                return record
            result = sdk.command.stop(record.command_id, reason)
            held = result.status is CommandStatus.CANCELLED and sdk.safety.hold((record.resource,), reason)
            record.status = ExecutionStatus.STOPPED if held else ExecutionStatus.INTERRUPTED
            record.phase = "held" if held else "stop_unconfirmed"
            record.progress = result.feedback.progress
            record.feedback = {"stop_reason": reason}
            record.updated_at = datetime.now(timezone.utc)
            return record

    def _require(self, invocation_id: str) -> ExecutionRecord:
        record = self._records.get(invocation_id)
        if record is None:
            raise KeyError(f"未知 invocation_id: {invocation_id}")
        return record
