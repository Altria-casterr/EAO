"""停止、保持与命令状态接口。"""

from ..backends.base import RobotBackend
from .types import CommandResult


class CommandClient:
    def __init__(self, backend: RobotBackend) -> None:
        self._backend = backend

    def get(self, command_id: str) -> CommandResult:
        return self._backend.get_command(command_id)

    def stop(self, command_id: str, reason: str) -> CommandResult:
        return self._backend.stop_command(command_id, reason)


class SafetyClient:
    def __init__(self, backend: RobotBackend) -> None:
        self._backend = backend

    def hold(self, resources: tuple[str, ...], reason: str) -> bool:
        return self._backend.hold(resources, reason)

