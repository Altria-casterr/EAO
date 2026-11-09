"""R1 Pro 语义 Ability 实现（task 承载业务，invoke 只引用 Task）。"""

from .tasks.registry import TaskRegistry
from .tasks.base import AbilityTask

__all__ = ["AbilityTask", "TaskRegistry"]
