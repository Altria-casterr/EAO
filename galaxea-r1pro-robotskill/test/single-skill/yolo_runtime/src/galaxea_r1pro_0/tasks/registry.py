"""Task 映射表：task_name → AbilityTask 实例。"""

from __future__ import annotations

from typing import TYPE_CHECKING, Mapping

if TYPE_CHECKING:
    from .base import AbilityTask


class TaskRegistry:
    """按 taskName（与 manifest 一致）索引业务 Task。"""

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

    def as_mapping(self) -> Mapping[str, AbilityTask]:
        return dict(self._tasks)
