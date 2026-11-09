"""模型默认值和允许列表（保留参考骨架实现）。"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping


class ModelProfileError(ValueError):
    """模型未启用、类型不匹配或配置不可读。"""


@dataclass(frozen=True)
class ModelProfile:
    name: str
    ability_role: str
    provider: str
    settings: Mapping[str, Any]


class ModelProfileRegistry:
    def __init__(self, defaults: Mapping[str, str], profiles: Mapping[str, ModelProfile], cameras: Mapping[str, Any] | None = None) -> None:
        self._defaults = dict(defaults)
        self._profiles = dict(profiles)
        self.cameras = dict(cameras or {})

    @classmethod
    def load(cls, path: str | Path) -> "ModelProfileRegistry":
        try:
            data = json.loads(Path(path).read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise ModelProfileError(f"无法读取模型配置: {path}") from exc
        if not isinstance(data, Mapping):
            raise ModelProfileError('模型配置顶层必须是对象')
        cameras = data.get('cameras', {})
        if not isinstance(cameras, Mapping) or any(not isinstance(v, Mapping) for v in cameras.values()):
            raise ModelProfileError('cameras 必须是按 sensor_id 索引的相机配置对象')
        profiles = {
            name: ModelProfile(
                name=name,
                ability_role=str(value["ability_role"]),
                provider=str(value["provider"]),
                settings=dict(value.get("settings") or {}),
            )
            for name, value in (data.get("profiles") or {}).items()
            if bool(value.get("enabled", True))
        }
        return cls(data.get("defaults") or {}, profiles, cameras)

    def resolve(self, ability_role: str, requested: str | None) -> ModelProfile:
        profile_name = requested or self._defaults.get(ability_role)
        if not profile_name:
            raise ModelProfileError(f"{ability_role} 没有默认模型")
        profile = self._profiles.get(profile_name)
        if profile is None:
            raise ModelProfileError(f"模型不在允许列表: {profile_name}")
        if profile.ability_role != ability_role:
            raise ModelProfileError(f"模型 {profile_name} 不适用于 {ability_role}")
        return profile
