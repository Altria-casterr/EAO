"""Static + dynamic MCP tool handlers.

Static tools mirror framework HTTP endpoints 1:1 (no aggregation, no
narration). Dynamic tools are one per `(abilityName, taskName)` discovered
from the manifest at startup.

Every tool returns a plain dict / list; `server.py` is responsible for
JSON-serialising it into the MCP `CallToolResult.content`.
"""

from __future__ import annotations

import asyncio
import keyword
import time
from pathlib import Path
from typing import Any, Awaitable, Callable, Dict, List, Optional

from mcp.server.fastmcp import FastMCP

from ..core.framework_client import FrameworkClient, FrameworkError
from ..core.models import LifecycleState, TaskDef
from ..core.registry import ManifestRegistry
from ..core.schema import build_input_schema, tool_name
from .skills import SkillCache

DEFAULT_TASK_TIMEOUT = 120.0
POLL_INITIAL = 0.2
POLL_MAX = 2.0

# Terminal task statuses as reported by the ability's /api/task/status endpoint.
# (Distinct from ability lifecycle states; these come from service/task_manager.)
TASK_TERMINAL_STATUSES = {"completed", "failed", "cancelled", "error"}


def _model_dump_json_safe(obj: Any) -> Any:
    """Pydantic -> dict for tool return values."""
    if hasattr(obj, "model_dump"):
        return obj.model_dump()
    if isinstance(obj, list):
        return [_model_dump_json_safe(x) for x in obj]
    return obj


def _framework_error_payload(e: FrameworkError) -> Dict[str, Any]:
    return {"error": str(e), "status_code": e.status_code}


# ============================================================================
# Static tools
# ============================================================================


def register_static_tools(
    mcp: FastMCP,
    client: FrameworkClient,
    registry: ManifestRegistry,
    reload_cb,
    skill_cache: Optional[SkillCache] = None,
) -> None:
    """Register the fixed set of management tools.

    `reload_cb` is an async callable invoked by `refresh_tools` — it should
    rebuild the registry from the framework and re-register dynamic tools.
    `skill_cache` (optional) wires up the skill discovery / sync tools.
    """

    # ----------------------------------------------------------------------
    # Simplified "use a tool" lifecycle: start_ability → <task tool> → stop_ability
    #
    # These wrap the lower-level spawn_instance/terminate_instance flow so
    # an agent's mental model of "boot the tool, wait until ready, use it,
    # shut it down" is expressible in 2 management calls instead of 4.
    # ----------------------------------------------------------------------

    @mcp.tool(
        name="start_ability",
        description=(
            "One-click: start an ability and wait until it is Running and "
            "ready to accept task calls. Takes `template` (CR template name "
            "e.g. 'endpoint-control-1', or its UUID). Blocks up to `timeout` "
            "seconds (default 30). If an active instance for the template's "
            "ability_name already exists, reuse it instead of spawning a new "
            "one — making this call idempotent. Returns {instance_id, "
            "ability_name, state, ability_port, reused, ready_in_seconds}. "
            "This is the preferred entry point before calling any dynamic "
            "task tool; use spawn_instance only if you need finer control."
        ),
    )
    async def start_ability(template: str, timeout: float = 30.0) -> dict:
        try:
            return await client.ensure_ability_ready(template, timeout=timeout)
        except FrameworkError as e:
            return _framework_error_payload(e)

    @mcp.tool(
        name="stop_ability",
        description=(
            "One-click: stop ability instances and destroy their rows. "
            "Takes `target` which is either an instance_id (stops just that "
            "instance) or a template name/id (stops ALL active instances "
            "with the matching ability_name). Waits up to `timeout` seconds "
            "(default 10) for the rows to disappear. Returns {stopped: "
            "[instance_id...], count, elapsed_seconds}. This is the "
            "counterpart to start_ability."
        ),
    )
    async def stop_ability(target: str, timeout: float = 10.0) -> dict:
        try:
            return await client.stop_ability(target, timeout=timeout)
        except FrameworkError as e:
            return _framework_error_payload(e)

    # ----------------------------------------------------------------------
    # Low-level discovery / management — kept for power users, introspection,
    # and backward compatibility with earlier MCP flows.
    # ----------------------------------------------------------------------

    @mcp.tool(
        name="list_abilities",
        description=(
            "List all ability classes loaded into the framework. Each entry "
            "summarises a manifest (one per package+version+abilityName) and "
            "carries the task count. Use this to discover what the robot can "
            "do. Source: GET /api/manifest."
        ),
    )
    async def list_abilities() -> List[dict]:
        try:
            return _model_dump_json_safe(await client.list_manifests())
        except FrameworkError as e:
            return [_framework_error_payload(e)]

    @mcp.tool(
        name="list_templates",
        description=(
            "List all CR templates currently loaded from yaml files or "
            "package-bundled CRs. Templates are the launch configurations "
            "for abilities — you spawn a runtime instance from a template "
            "by its name. CR templates are read-only at runtime (POST "
            "/api/cr and DELETE /api/cr/:id return 410 Gone). Source: "
            "GET /api/cr."
        ),
    )
    async def list_templates() -> List[dict]:
        try:
            return await client.list_templates() or []
        except FrameworkError as e:
            return [_framework_error_payload(e)]

    @mcp.tool(
        name="list_instances",
        description=(
            "List all runtime ability instances. Each instance has a fresh "
            "instance_id (distinct from any CR template id) and is destroyed "
            "on termination. Source: GET /api/instance."
        ),
    )
    async def list_instances() -> List[dict]:
        try:
            return _model_dump_json_safe(await client.list_instances())
        except FrameworkError as e:
            return [_framework_error_payload(e)]

    @mcp.tool(
        name="list_heartbeats",
        description=(
            "List currently alive ability instances with their lifecycle "
            "state (Init / Standby / Running / Suspend) and IPC/ability "
            "ports. This is the source of truth for which instances are "
            "reachable right now. Source: GET /api/ability-heartbeat."
        ),
    )
    async def list_heartbeats() -> List[dict]:
        try:
            return _model_dump_json_safe(await client.list_heartbeats())
        except FrameworkError as e:
            return [_framework_error_payload(e)]

    @mcp.tool(
        name="spawn_instance",
        description=(
            "Spawn a new runtime ability instance from a CR template. "
            "Takes `template` (CR template name, e.g. 'endpoint-control-1', "
            "or its UUID). Optional `start=true` starts the process; "
            "optional `connect=true` drives it all the way to Running. "
            "Returns the task id for the framework-side start pipeline; "
            "the new instance_id appears in list_instances once the "
            "heartbeat arrives. Source: POST /api/instance."
        ),
    )
    async def spawn_instance(template: str, start: bool = True, connect: bool = True) -> dict:
        try:
            return await client.spawn_instance(template, start=start, connect=connect)
        except FrameworkError as e:
            return _framework_error_payload(e)

    @mcp.tool(
        name="terminate_instance",
        description=(
            "Terminate a running ability instance and destroy its DB row. "
            "Takes `instance_id` (from list_instances / list_heartbeats). "
            "Source: DELETE /api/instance/:id."
        ),
    )
    async def terminate_instance(instance_id: str) -> dict:
        try:
            return await client.terminate_instance(instance_id)
        except FrameworkError as e:
            return _framework_error_payload(e)

    @mcp.tool(
        name="lifecycle_command",
        description=(
            "Send a fine-grained lifecycle command (start / connect / "
            "disconnect / terminate) to an **already-created** instance. "
            "Use this to recover a Standby instance that failed to "
            "auto-connect, or to re-connect after a manual disconnect. "
            "For creating a brand-new instance, use spawn_instance instead. "
            "Source: POST /api/lifecycle-request."
        ),
    )
    async def lifecycle_command(instance_id: str, command: str) -> dict:
        allowed = {"start", "connect", "disconnect", "terminate"}
        if command not in allowed:
            return {"error": f"command must be one of {sorted(allowed)}"}
        try:
            return await client.lifecycle_request(instance_id, command)
        except FrameworkError as e:
            return _framework_error_payload(e)

    @mcp.tool(
        name="refresh_tools",
        description=(
            "Reload the tool list from framework manifests AND refresh the "
            "skill cache from /api/skill. Call this after uploading a new "
            "ability package, after changing yaml CRs, or whenever you "
            "suspect the local skill cache is stale. Returns the number of "
            "abilities, dynamic tools, and skills now available."
        ),
    )
    async def refresh_tools() -> dict:
        try:
            count = await reload_cb()
            return {
                "abilities": len(registry.ability_names()),
                "dynamic_tools": count,
                "skills": len(skill_cache.entries) if skill_cache else 0,
            }
        except FrameworkError as e:
            return _framework_error_payload(e)

    # ----------------------------------------------------------------------
    # Skill discovery / download / sync
    #
    # These let any MCP client (Claude Code, OpenClaw, MCP Inspector, a
    # custom agent runner, etc.) pull both the bundled framework skill and
    # the per-ability skills shipped inside packages, then materialise them
    # into whatever directory that client uses as its skill loader root.
    # The server is client-agnostic — it just exposes skills as MCP
    # resources and as cached files; which directory (if any) to sync
    # them into is entirely up to the agent.
    # ----------------------------------------------------------------------

    if skill_cache is not None:

        @mcp.tool(
            name="list_skills",
            description=(
                "List every cached skill document — both the bundled "
                "framework runtime skill and per-ability skills mirrored "
                "from packages via /api/skill. Each entry has a stable "
                "`uri` (`skill://...`), a `cache_path` on disk, plus "
                "package/version/filename/title metadata. Use this to see "
                "what guidance is available before planning a non-trivial "
                "task."
            ),
        )
        async def list_skills() -> List[dict]:
            return skill_cache.to_dicts()

        @mcp.tool(
            name="read_skill",
            description=(
                "Read the full markdown body of a cached skill, given its "
                "`uri` from list_skills. Returns the raw text. Useful when "
                "the agent wants the full contents of an ability's usage "
                "guide before deciding how to call its tools."
            ),
        )
        async def read_skill(uri: str) -> dict:
            entry = skill_cache.find(uri)
            if entry is None:
                return {"error": f"unknown skill uri: {uri}"}
            try:
                content = entry.cache_path.read_text(encoding="utf-8")
            except OSError as e:
                return {"error": f"failed to read {entry.cache_path}: {e}"}
            return {
                "uri": uri,
                "package": entry.package,
                "version": entry.version,
                "filename": entry.filename,
                "title": entry.title,
                "content": content,
            }

        @mcp.tool(
            name="sync_skills_to_dir",
            description=(
                "Copy every cached skill into the target directory, "
                "preserving the `<source>/<package>/<version>/<filename>` "
                "layout. Intended for one-shot installs into whatever "
                "directory the connected MCP client watches for skill "
                "files — Claude Code reads `~/.claude/skills/`, OpenClaw "
                "and other clients use their own conventions, so ask the "
                "user or check the client's docs before hardcoding a "
                "target. Returns the list of files written."
            ),
        )
        async def sync_skills_to_dir(target: str) -> dict:
            try:
                written = skill_cache.materialize_into(Path(target))
            except OSError as e:
                return {"error": f"failed to write to {target}: {e}"}
            return {
                "target": str(Path(target).expanduser().resolve()),
                "written": [str(p) for p in written],
                "count": len(written),
            }


# ============================================================================
# Dynamic tools
# ============================================================================


def register_dynamic_tools(
    mcp: FastMCP,
    client: FrameworkClient,
    registry: ManifestRegistry,
    task_timeout: float = DEFAULT_TASK_TIMEOUT,
) -> int:
    """Register one MCP tool per (ability, task). Returns the number of
    dynamic tools registered."""
    count = 0
    for ability_name, task in registry.tasks():
        name = tool_name(ability_name, task.taskName)
        description = task.summary or f"Call {task.taskName} on ability {ability_name}."
        # Add param hints to description so LLMs see the schema intent even
        # if they ignore the JSON Schema.
        if task.params:
            lines = [description, "", "Parameters:"]
            for p in task.params:
                optional = "?" if p.optional else ""
                desc = f" — {p.description}" if p.description else ""
                lines.append(f"  - {p.name}{optional} ({p.type}){desc}")
            if task.returns:
                lines.append("")
                lines.append("Returns:")
                for r in task.returns:
                    desc = f" — {r.description}" if r.description else ""
                    lines.append(f"  - {r.name} ({r.type}){desc}")
            description = "\n".join(lines)

        input_schema = build_input_schema(task)

        binding = {
            "ability": ability_name,
            "task_name": task.taskName,
            "task_type": task.taskType,
        }

        # FastMCP reads the function's inspect.signature() to build a pydantic
        # model for dispatch — so if we register `async def run_task(args=
        # {})` clients that send `position=..., orientation=...` never get
        # bound: there's no `position` parameter and `args` stays {}.
        # Compile a task-specific function whose formal parameters match
        # `task.params` 1:1, collect them back into the `{name: value}`
        # dict the ability expects, and forward to the dispatcher.
        fn = _build_task_tool_fn(
            task=task,
            binding=binding,
            client=client,
            task_timeout=task_timeout,
        )

        mcp.add_tool(
            fn,
            name=name,
            description=description,
            structured_output=True,
        )
        # Type-hint-driven inference gives a flat schema (Point3d → `dict`).
        # Override with the manifest's precise nested JSON Schema so clients
        # see the full composite structure (`{x,y,z}` / `{x,y,z,w}` / etc).
        tool_obj = mcp._tool_manager.get_tool(name)
        if tool_obj is not None:
            tool_obj.parameters = input_schema
        count += 1
    return count


# ============================================================================
# Dynamic function compilation: manifest params -> real function signature
# ============================================================================


# Map manifest type strings to Python type annotations used in the generated
# tool function. Composite types (Point3d, Quaternion, Pose, etc.) become
# `dict` at the Python level — the precise nested schema is attached later
# via `tool_obj.parameters = input_schema`.
_MANIFEST_TO_PY_TYPE: Dict[str, str] = {
    "string": "str",
    "str": "str",
    "int": "int",
    "integer": "int",
    "number": "float",
    "float": "float",
    "double": "float",
    "bool": "bool",
    "boolean": "bool",
    "array": "list",
    "list": "list",
    "object": "dict",
    "dict": "dict",
    "map": "dict",
}


def _py_type_hint_for(manifest_type: Optional[str]) -> str:
    """Best-effort mapping of manifest param type → Python type annotation."""
    if not manifest_type:
        return "Any"
    lowered = manifest_type.lower()
    if lowered in _MANIFEST_TO_PY_TYPE:
        return _MANIFEST_TO_PY_TYPE[lowered]
    # Custom composites (Point3d, Quaternion, Pose, Euler, Vector3, ...)
    return "dict"


def _py_safe_ident(name: str) -> str:
    """Turn an arbitrary manifest parameter name into a valid Python
    identifier: keep alnum/underscore, replace the rest with `_`, prepend
    underscore if it starts with a digit, suffix with `_` if it's a Python
    keyword. Starting with `_` is also rejected by FastMCP, so patch that."""
    if not name:
        return "arg_"
    safe = "".join(c if (c.isalnum() or c == "_") else "_" for c in name)
    if safe[0].isdigit():
        safe = "p_" + safe
    # FastMCP rejects params starting with underscore
    while safe.startswith("_"):
        safe = "p" + safe
    if keyword.iskeyword(safe):
        safe = safe + "_"
    return safe


def _build_task_tool_fn(
    *,
    task: TaskDef,
    binding: Dict[str, Any],
    client: FrameworkClient,
    task_timeout: float,
) -> Callable[..., Awaitable[Dict[str, Any]]]:
    """Compile and return an `async def run_task(...)` whose formal
    parameters match the manifest's `task.params` 1:1.

    The generated function body collects those parameters back into a
    `{orig_name: value}` dict (skipping optional params that were left at
    their default None) and forwards to `_dispatch_ability_task`. The
    original manifest names — which may contain dots / dashes / keywords —
    are preserved in the outgoing dict even when they had to be renamed
    for the Python signature.
    """
    required: List[tuple] = []
    optional: List[tuple] = []
    for p in task.params:
        py_name = _py_safe_ident(p.name)
        py_hint = _py_type_hint_for(p.type)
        if p.optional:
            optional.append((p.name, py_name, py_hint))
        else:
            required.append((p.name, py_name, py_hint))

    # required first, then optional with None default — keeps positional /
    # kwargs semantics consistent with Python norms.
    sig_parts: List[str] = []
    for _orig, py_name, hint in required:
        sig_parts.append(f"{py_name}: {hint}")
    for _orig, py_name, hint in optional:
        sig_parts.append(f"{py_name}: {hint} | None = None")
    sig = ", ".join(sig_parts)

    body_lines: List[str] = ["    args = {}"]
    for orig, py_name, _hint in required:
        # Required args always go in, even if the client sent `None` (the
        # ability's validator will reject it with a clear message).
        body_lines.append(f"    args[{orig!r}] = {py_name}")
    for orig, py_name, _hint in optional:
        body_lines.append(f"    if {py_name} is not None:")
        body_lines.append(f"        args[{orig!r}] = {py_name}")
    body_lines.append("    return await __dispatch__(args)")

    # FastMCP's structured_output path requires a concretely-typed return
    # annotation — bare `dict` is rejected as "not serializable for
    # structured output". `Dict[str, Any]` gives it a schema-able shape
    # while still accepting whatever payload the ability returns.
    src = (
        "async def run_task(" + sig + ") -> Dict[str, Any]:\n"
        + "\n".join(body_lines) + "\n"
    )

    async def _dispatch(args: Dict[str, Any]) -> Dict[str, Any]:
        return await _dispatch_ability_task(
            client=client,
            ability_name=binding["ability"],
            task_name=binding["task_name"],
            task_type=binding["task_type"],
            args=args,
            timeout=task_timeout,
        )

    namespace: Dict[str, Any] = {
        "__dispatch__": _dispatch,
        # Expose typing constructs + builtins used in annotations
        "Any": Any,
        "Dict": Dict,
        "dict": dict,
        "list": list,
        "str": str,
        "int": int,
        "float": float,
        "bool": bool,
    }
    # `dont_inherit=True` prevents this module's `from __future__ import
    # annotations` (PEP 563 lazy annotation evaluation) from leaking into
    # the generated function. We want the parameter annotations to be real
    # type objects so `inspect.signature(fn).parameters[n].annotation` gives
    # `float` rather than the string `'float'`. FastMCP's `func_metadata`
    # pipeline works either way, but callers (and tests) are simpler when
    # annotations are the real thing.
    code = compile(src, f"<ability_task:{binding['ability']}.{binding['task_name']}>",
                   "exec", dont_inherit=True)
    exec(code, namespace)  # noqa: S102 — controlled namespace, no user input
    fn = namespace["run_task"]
    fn.__name__ = tool_name(binding["ability"], binding["task_name"])
    fn.__doc__ = task.summary
    return fn


async def _dispatch_ability_task(
    client: FrameworkClient,
    ability_name: str,
    task_name: str,
    task_type: int,
    args: Dict[str, Any],
    timeout: float,
) -> Dict[str, Any]:
    """Core dynamic-tool handler:

    1. find a Running/Standby instance of this ability
    2. POST /api/ability/<instance_id>/api/task/start via framework proxy
    3. poll /api/task/status with exponential backoff until terminal or timeout
    4. return the raw task result dict
    """
    try:
        heartbeats = await client.list_heartbeats()
    except FrameworkError as e:
        return _framework_error_payload(e)
    inst = FrameworkClient.pick_active_instance(heartbeats, ability_name)
    if inst is None:
        return {
            "error": f"no active instance for ability {ability_name}",
            "hint": (
                f"call start_ability(template=<name>) first to boot an "
                f"instance of {ability_name}; list_templates shows which "
                "templates map to this ability."
            ),
        }
    if inst.state != LifecycleState.Running:
        return {
            "error": (
                f"instance {inst.id} of ability {ability_name} is in state "
                f"{inst.state}, not Running"
            ),
            "hint": (
                "task proxy needs Running (post-connect). Call "
                "start_ability(template=<name>) — it will wait until ready. "
                "lifecycle_command(instance_id, command='connect') is the "
                "low-level escape hatch."
            ),
        }

    try:
        start_resp = await client.ability_post(
            inst.id, "api/task/start", {"task_type": task_type, "input": args}
        )
    except FrameworkError as e:
        return _framework_error_payload(e)
    task_id = start_resp.get("task_id") if isinstance(start_resp, dict) else None
    if not task_id:
        return {
            "error": f"ability did not return task_id for {task_name}",
            "start_response": start_resp,
        }

    return await _poll_task(client, inst.id, task_id, timeout)


async def _poll_task(
    client: FrameworkClient,
    instance_id: str,
    task_id: str,
    timeout: float,
) -> Dict[str, Any]:
    deadline = time.monotonic() + timeout
    delay = POLL_INITIAL
    last: Dict[str, Any] = {"status": "pending", "task_id": task_id}
    while time.monotonic() < deadline:
        try:
            r = await client.ability_post(
                instance_id, "api/task/status", {"task_id": task_id}
            )
        except FrameworkError as e:
            return {"status": "error", "task_id": task_id, "error": str(e)}
        if isinstance(r, dict):
            last = r
            status = r.get("status")
            if status in TASK_TERMINAL_STATUSES:
                return r
        await asyncio.sleep(delay)
        delay = min(delay * 2.0, POLL_MAX)
    last.setdefault("task_id", task_id)
    last["status"] = "timeout"
    return last
