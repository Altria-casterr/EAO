"""In-memory index of manifests loaded from the framework."""

from __future__ import annotations

import fnmatch
from typing import Dict, Iterable, List, Optional, Tuple

from .models import Manifest, TaskDef


class ManifestRegistry:
    """Keeps Manifest objects keyed by abilityName.

    Each entry stores the manifest + a quick `taskName -> taskType` lookup
    so dynamic tool handlers can resolve `task_type` without rescanning.
    """

    def __init__(self) -> None:
        self._by_name: Dict[str, Manifest] = {}
        self._task_index: Dict[Tuple[str, str], int] = {}

    def add(self, manifest: Manifest) -> None:
        self._by_name[manifest.abilityName] = manifest
        for task in manifest.tasks:
            self._task_index[(manifest.abilityName, task.taskName)] = task.taskType

    def clear(self) -> None:
        self._by_name.clear()
        self._task_index.clear()

    # ---- queries ---------------------------------------------------------

    def ability_names(self) -> List[str]:
        return sorted(self._by_name.keys())

    def get(self, ability_name: str) -> Optional[Manifest]:
        return self._by_name.get(ability_name)

    def tasks(self) -> Iterable[Tuple[str, TaskDef]]:
        """Iterate over (ability_name, task_def) for every task in every manifest."""
        for name, manifest in self._by_name.items():
            for t in manifest.tasks:
                yield name, t

    def task_type_of(self, ability_name: str, task_name: str) -> Optional[int]:
        return self._task_index.get((ability_name, task_name))

    # ---- filtering -------------------------------------------------------

    def filter(self, patterns: str) -> None:
        """Keep only abilities matching one of the comma-separated glob patterns.

        `"*"` or empty string is a no-op. Example:
            registry.filter("EndpointControl.*,Perception.*")
        """
        patterns = (patterns or "").strip()
        if not patterns or patterns == "*":
            return
        globs = [p.strip() for p in patterns.split(",") if p.strip()]
        kept = {
            name: m
            for name, m in self._by_name.items()
            if any(fnmatch.fnmatchcase(name, g) for g in globs)
        }
        self._by_name = kept
        self._task_index = {
            (ab, tn): tt for (ab, tn), tt in self._task_index.items() if ab in kept
        }
