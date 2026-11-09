"""确定性的内存后端，用于 SDK、Ability 和 Pilot 的并行开发。"""

from __future__ import annotations

from dataclasses import dataclass
from threading import RLock
from typing import Any, Mapping
from uuid import uuid4

from ..utils.errors import BackendUnavailableError, CommandNotFoundError
from ..utils.profile import RobotProfile
from ..utils.types import (
    CapabilityInfo,
    CommandFeedback,
    CommandHandle,
    CommandResult,
    CommandStatus,
    JointState,
    Pose,
    RobotState,
    SensorFrame,
    utc_now,
)


@dataclass
class _CommandRecord:
    handle: CommandHandle
    target: Mapping[str, Any]
    result: CommandResult


class InMemoryBackend:
    """只模拟状态变化，不包含任何拆码垛或流程语义。"""

    _RESOURCES = ("state", "base", "upper_body", "end_effector", "sensors", "safety")

    def __init__(self, profile: RobotProfile) -> None:
        self.profile = profile
        self._connected = False
        self._commands: dict[str, _CommandRecord] = {}
        self._held_resources: set[str] = set()
        self._lock = RLock()

    def connect(self, ability_name: str | None = None) -> None:
        self._connected = True

    def close(self) -> None:
        self._connected = False

    def capabilities(self) -> CapabilityInfo:
        return CapabilityInfo(
            robot_model="R1Pro",
            backend=self.profile.backend,
            firmware_profile=self.profile.firmware_profile,
            resources=self._RESOURCES,
        )

    def snapshot(self) -> RobotState:
        return RobotState(
            robot_id=self.profile.robot_id,
            backend=self.profile.backend,
            connected=self._connected,
            mode="hold" if self._held_resources else "ready",
            base_pose=Pose("map", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
            joints=JointState(names=(), positions_rad=()),
            held_resources=tuple(sorted(self._held_resources)),
        )

    def start_command(
        self,
        resource: str,
        operation: str,
        target: Mapping[str, Any],
    ) -> CommandHandle:

        self._require_connected("start_command")

        command_id = str(uuid4())

        handle = CommandHandle(command_id, self.profile.robot_id, resource, operation)

        deferred = bool(self.profile.options.get("defer_commands", False))

        status = CommandStatus.RUNNING if deferred else CommandStatus.SUCCEEDED

        feedback = CommandFeedback(
            command_id=command_id,
            status=status,
            progress=0.25 if deferred else 1.0,
            phase="executing" if deferred else "completed",
            metrics={"backend": self.profile.backend.value},
        )

        result = CommandResult(command_id, status, feedback, output={"target": dict(target)})

        with self._lock:
            self._commands[command_id] = _CommandRecord(handle, dict(target), result)
            self._held_resources.discard(resource)

        return handle

    def get_command(
        self,
        command_id: str | None = None,
        *,
        resource: str | None = None,
        operation: str | None = None,
        target: Mapping[str, Any] | None = None,
    ) -> CommandResult:
        if command_id is not None:
            with self._lock:
                record = self._commands.get(command_id)
                if record is None:
                    raise CommandNotFoundError(command_id=command_id, hint=f"未知命令: {command_id}")
                return record.result
        # 读取型：内存模式没有真实反馈，统一为 SUCCEEDED（与 real 后端一致）
        return CommandResult(
            command_id="",
            status=CommandStatus.SUCCEEDED,
            feedback=CommandFeedback(
                command_id="",
                status=CommandStatus.SUCCEEDED,
                progress=0.0,
                phase="no_feedback",
            ),
            output={"data": None},
        )

    def stop_command(self, command_id: str, reason: str) -> CommandResult:
        with self._lock:
            record = self._commands.get(command_id)

            if record is None:
                raise CommandNotFoundError(command_id=command_id, hint=f"未知命令: {command_id}")

            if not bool(self.profile.options.get("stop_confirmed", True)):
                status = CommandStatus.UNKNOWN
                phase = "stop_unconfirmed"

            else:
                status = CommandStatus.CANCELLED
                phase = "held"
                self._held_resources.add(record.handle.resource)

            feedback = CommandFeedback(
                command_id=command_id,
                status=status,
                progress=record.result.feedback.progress,
                phase=phase,
                metrics={"stop_reason": reason},
            )
            record.result = CommandResult(command_id, status, feedback, output=record.result.output)
            return record.result

    def hold(self, resources: tuple[str, ...], reason: str) -> bool:
        self._require_connected("hold")
        with self._lock:
            self._held_resources.update(resources)
        return bool(self.profile.options.get("stop_confirmed", True))

    def capture(self, sensor_id: str, encoding: str) -> SensorFrame:
        self._require_connected("capture")
        return SensorFrame(
            sensor_id=sensor_id,
            frame_id=f"{sensor_id}_frame",
            timestamp=utc_now(),
            encoding=encoding,
            data=b"fake-frame",
            metadata={"backend": self.profile.backend.value, "sample": True},
        )

    def complete_command(self, command_id: str, output: Mapping[str, Any] | None = None) -> None:
        """测试辅助方法：由测试明确推进命令，不使用定时器制造不稳定结果。"""

        with self._lock:
            record = self._commands.get(command_id)
            if record is None:
                raise CommandNotFoundError(command_id=command_id, hint=f"未知命令: {command_id}")
            feedback = CommandFeedback(command_id, CommandStatus.SUCCEEDED, 1.0, "completed")
            record.result = CommandResult(command_id, CommandStatus.SUCCEEDED, feedback, output or {})

    def _require_connected(self, operation: str) -> None:

        if not self._connected:

            raise BackendUnavailableError(
                backend=self.profile.backend,
                robot_id=self.profile.robot_id,
                reason=BackendUnavailableError.NOT_CONNECTED,
                operation=operation,
                hint="请先调用 R1ProSDK.open() 建立连接",
            )

