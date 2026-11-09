"""Pydantic models mirroring AbilityFramework JSON shapes."""

from __future__ import annotations

from typing import Any, List, Optional

from pydantic import BaseModel, ConfigDict, Field, field_validator


class LifecycleState:
    """Ability instance lifecycle state strings used by the framework."""

    Unknown = "Unknown"
    Inactive = "Inactive"
    Init = "Init"
    Standby = "Standby"
    Running = "Running"
    Suspend = "Suspend"
    Terminated = "Terminated"
    Error = "Error"

    ACTIVE = {Init, Standby, Running, Suspend}
    TERMINAL = {Terminated, Error}


# ---- Manifest / Task ---------------------------------------------------------


class ParamDef(BaseModel):
    """A single task parameter or return value declared in ability.manifest.yaml."""

    model_config = ConfigDict(extra="allow")

    name: str
    type: str
    optional: bool = False
    description: Optional[str] = None


class TaskDef(BaseModel):
    """A task entry under manifest.tasks[]."""

    model_config = ConfigDict(extra="allow")

    taskType: int
    taskName: str
    summary: Optional[str] = None
    params: List[ParamDef] = Field(default_factory=list)
    returns: List[ParamDef] = Field(default_factory=list)

    @field_validator("params", "returns", mode="before")
    @classmethod
    def _none_as_empty_list(cls, v: Any) -> Any:
        # C++ nlohmann::json serialises an empty list as null; accept both.
        if v is None:
            return []
        return v


class Manifest(BaseModel):
    """Full manifest for a single ability class."""

    model_config = ConfigDict(extra="allow", populate_by_name=True)

    abilityName: str
    kind: str
    tasks: List[TaskDef] = Field(default_factory=list)
    # `schema` is reserved by pydantic, expose with alias
    schema_: Optional[dict] = Field(default=None, alias="schema")


class ManifestSummary(BaseModel):
    """Summary entry returned by `GET /api/manifest`."""

    model_config = ConfigDict(extra="allow")

    abilityName: str
    kind: str
    package: str
    version: str
    taskCount: int = 0
    hasConfigSchema: bool = False


# ---- Heartbeat / Instance ----------------------------------------------------


class Heartbeat(BaseModel):
    """Live heartbeat entry from `GET /api/ability-heartbeat`."""

    model_config = ConfigDict(extra="allow")

    id: str
    instanceName: str
    abilityName: str
    version: str
    state: str
    IPCPort: int = 0
    abilityPort: int = 0


class AbilityInstance(BaseModel):
    """Runtime instance row from `GET /api/instance`."""

    model_config = ConfigDict(extra="allow")

    instance_id: str
    cr_id: Optional[str] = None
    cr_name: str = ""
    instance_name: str = ""
    ability_name: str = ""
    ability_version: str = ""
    state: str = LifecycleState.Unknown
    start_time: int = 0
    stop_time: int = 0
    detail: Optional[Any] = None
    spec_snapshot: Optional[Any] = None
