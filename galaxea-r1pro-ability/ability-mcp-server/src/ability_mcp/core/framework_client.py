"""Async HTTP client for AbilityFramework's :8080 API.

All routes here mirror the current C++ server after the CR/Instance refactor:

- `POST /api/cr` and `DELETE /api/cr/:id` are **410 Gone** — do not call them.
- Runtime lifecycle: create via `POST /api/instance`, destroy via
  `DELETE /api/instance/:id`, fine-grained transitions via
  `POST /api/lifecycle-request`.
- Ability task endpoints are proxied through `/api/ability/:id/<subpath>` —
  the framework routes to the ability process's abilityPort.
"""

from __future__ import annotations

import asyncio
import time
import uuid as _uuid
from typing import Any, Iterable, List, Optional

import httpx

from .models import AbilityInstance, Heartbeat, LifecycleState, Manifest, ManifestSummary


class FrameworkError(RuntimeError):
    """Raised when the framework returns a non-2xx response."""

    def __init__(self, message: str, status_code: Optional[int] = None):
        super().__init__(message)
        self.status_code = status_code


class FrameworkClient:
    """Thin async wrapper around the framework HTTP API.

    Instances own an `httpx.AsyncClient`; call `.aclose()` or use
    `async with FrameworkClient(...)` for cleanup.
    """

    def __init__(
        self,
        base_url: str = "http://localhost:8080",
        timeout: float = 10.0,
        client: Optional[httpx.AsyncClient] = None,
    ) -> None:
        self.base_url = base_url.rstrip("/")
        self._owns_client = client is None
        self._client = client or httpx.AsyncClient(base_url=self.base_url, timeout=timeout)

    async def __aenter__(self) -> "FrameworkClient":
        return self

    async def __aexit__(self, exc_type, exc, tb) -> None:
        await self.aclose()

    async def aclose(self) -> None:
        if self._owns_client:
            await self._client.aclose()

    # ---------- low-level ----------------------------------------------------

    async def _request(self, method: str, path: str, **kwargs) -> httpx.Response:
        try:
            resp = await self._client.request(method, path, **kwargs)
        except httpx.HTTPError as e:
            raise FrameworkError(f"framework unreachable at {self.base_url}{path}: {e}") from e
        if resp.status_code >= 400:
            raise FrameworkError(
                f"framework {method} {path} -> {resp.status_code}: {resp.text}",
                status_code=resp.status_code,
            )
        return resp

    async def _json(self, method: str, path: str, **kwargs) -> Any:
        resp = await self._request(method, path, **kwargs)
        if not resp.content:
            return None
        try:
            return resp.json()
        except ValueError:
            return resp.text

    # ---------- framework liveness / config ---------------------------------

    async def hello(self) -> str:
        resp = await self._request("GET", "/api/hello")
        return resp.text

    async def get_config(self) -> dict:
        return await self._json("GET", "/api/config")

    # ---------- manifests ----------------------------------------------------

    async def list_manifests(self) -> List[ManifestSummary]:
        data = await self._json("GET", "/api/manifest")
        return [ManifestSummary.model_validate(item) for item in data or []]

    async def get_manifest(self, name: str, version: str) -> Manifest:
        data = await self._json("GET", f"/api/manifest/{name}/{version}")
        return Manifest.model_validate(data)

    # ---------- CR templates (read-only) ------------------------------------

    async def list_templates(self) -> List[dict]:
        """Return all ability CR templates. (`GET /api/cr`)

        CR templates are loaded from `crs/*.yaml` and package-bundled
        `crs/_packages/<pkg>/<version>/*.yaml`; they cannot be created or
        deleted at runtime via API.
        """
        return await self._json("GET", "/api/cr")

    async def get_template(self, cr_id_or_instance_id: str) -> dict:
        """Fetch a CR template by its id.

        The framework's `GET /api/cr/:id` tolerates an instance id as well —
        it will resolve via `AbilityInstance.spec_snapshot` first, falling
        back to the template table. This is useful when all you have is an
        instance id and want the original spec.
        """
        return await self._json("GET", f"/api/cr/{cr_id_or_instance_id}")

    # ---------- runtime instances -------------------------------------------

    async def list_instances(self) -> List[AbilityInstance]:
        data = await self._json("GET", "/api/instance")
        return [AbilityInstance.model_validate(item) for item in data or []]

    async def get_instance(self, instance_id: str) -> AbilityInstance:
        data = await self._json("GET", f"/api/instance/{instance_id}")
        return AbilityInstance.model_validate(data)

    async def spawn_instance(
        self,
        template: str,
        start: bool = True,
        connect: bool = True,
    ) -> dict:
        """Create and (by default) start+connect a new runtime instance
        from an existing CR template. (`POST /api/instance`)

        `template` is either the CR template name (e.g. "endpoint-control-1")
        or its UUID. Returns a dict with `taskId` which tracks the async
        start pipeline (spawn → wait Standby → connect).
        """
        body = {"template": template, "start": start, "connect": connect}
        return await self._json("POST", "/api/instance", json=body)

    async def terminate_instance(self, instance_id: str) -> dict:
        """Terminate and destroy a runtime instance. (`DELETE /api/instance/:id`)"""
        return await self._json("DELETE", f"/api/instance/{instance_id}")

    # ---------- lifecycle commands for existing instances -------------------

    async def lifecycle_request(self, instance_id: str, command: str) -> dict:
        """Send a lifecycle command (start / connect / disconnect / terminate)
        to an **existing** instance. (`POST /api/lifecycle-request`)

        For creating a new instance from a template, use `spawn_instance`
        instead. This endpoint is for driving state transitions on an
        already-created instance (e.g. reconnecting after manual disconnect).
        """
        body = {"abilityInstanceId": instance_id, "command": command}
        return await self._json("POST", "/api/lifecycle-request", json=body)

    # ---------- heartbeats ---------------------------------------------------

    async def list_heartbeats(self) -> List[Heartbeat]:
        data = await self._json("GET", "/api/ability-heartbeat")
        return [Heartbeat.model_validate(item) for item in data or []]

    async def get_heartbeat(self, instance_id: str) -> Optional[Heartbeat]:
        try:
            data = await self._json("GET", f"/api/ability-heartbeat/{instance_id}")
        except FrameworkError as e:
            if e.status_code == 404:
                return None
            raise
        if data is None:
            return None
        return Heartbeat.model_validate(data)

    # ---------- ability proxy -----------------------------------------------

    async def ability_post(self, instance_id: str, subpath: str, body: Any) -> Any:
        """POST to `/api/ability/:id/<subpath>`.

        The framework forwards to the ability process's `abilityPort`.
        `subpath` should NOT start with a slash. Example:

            await c.ability_post(iid, "api/task/start", {"task_type": 0, "input": {}})
        """
        path = f"/api/ability/{instance_id}/{subpath.lstrip('/')}"
        return await self._json("POST", path, json=body)

    async def ability_get(self, instance_id: str, subpath: str) -> Any:
        path = f"/api/ability/{instance_id}/{subpath.lstrip('/')}"
        return await self._json("GET", path)

    # ---------- alerts / logs (best-effort, tolerate missing endpoints) -----

    async def get_alerts(self, latest: int = 20) -> List[dict]:
        try:
            return await self._json("GET", f"/api/ability-alert?latest={latest}")
        except FrameworkError:
            return []

    # ---------- skills (package-bundled agent docs) --------------------------

    async def list_skills(self) -> List[dict]:
        """Return all skill files mirrored under
        `<framework_home>/skills/_packages/<pkg>/<version>/`.

        Each entry is `{package, version, filename, size, title}`.
        Returns an empty list if the framework version doesn't expose
        `/api/skill` yet.
        """
        try:
            data = await self._json("GET", "/api/skill")
        except FrameworkError as e:
            if e.status_code == 404:
                return []
            raise
        return data or []

    async def download_skill(
        self, package: str, version: str, filename: str
    ) -> str:
        """Fetch the raw text content of a single skill file.
        (`GET /api/skill/:package/:version/:filename`)"""
        path = f"/api/skill/{package}/{version}/{filename}"
        resp = await self._request("GET", path)
        return resp.text

    # ---------- helpers ------------------------------------------------------

    @staticmethod
    def pick_active_instance(
        heartbeats: Iterable[Heartbeat], ability_name: str
    ) -> Optional[Heartbeat]:
        """From a list of heartbeats, pick one active (Running/Standby) instance
        matching the given ability_name. Running wins over Standby when both
        are present."""
        running: Optional[Heartbeat] = None
        standby: Optional[Heartbeat] = None
        for hb in heartbeats:
            if hb.abilityName != ability_name:
                continue
            if hb.state == LifecycleState.Running and running is None:
                running = hb
            elif hb.state == LifecycleState.Standby and standby is None:
                standby = hb
        return running or standby

    # ========================================================================
    # High-level orchestration helpers (for the simplified MCP tool set)
    #
    # These wrap multiple HTTP calls so a single client-side call maps to the
    # user's mental model of "start → wait until ready → use → stop". Framework
    # code is not modified; everything here is a composition of existing
    # primitives (spawn_instance / list_heartbeats / terminate_instance).
    # ========================================================================

    async def resolve_template(self, template_ref: str) -> dict:
        """Resolve a template by name or UUID, returning the full CR dict.

        Accepts either:
          - `<uuid>` — a CR template id (`GET /api/cr/:id`). Also tolerates
            an instance id since the framework resolves that via spec_snapshot.
          - `<metadata.name>` — scans `list_templates()` for the first matching row.
        """
        if not template_ref:
            raise FrameworkError("template name or id is required")
        try:
            _uuid.UUID(template_ref)
            return await self.get_template(template_ref)
        except (ValueError, FrameworkError):
            pass
        templates = await self.list_templates() or []
        for t in templates:
            meta = t.get("metadata") if isinstance(t, dict) else None
            if meta and meta.get("name") == template_ref:
                return t
        raise FrameworkError(f"template not found: {template_ref}")

    async def find_running_instance(self, ability_name: str) -> Optional[Heartbeat]:
        """Return a currently Running heartbeat for this ability_name, or None.

        Requires `abilityPort > 0` — without it, the framework proxy can't
        forward task calls, so the instance isn't actually usable.
        """
        hbs = await self.list_heartbeats()
        for hb in hbs:
            if (
                hb.abilityName == ability_name
                and hb.state == LifecycleState.Running
                and hb.abilityPort > 0
            ):
                return hb
        return None

    async def ensure_ability_ready(
        self,
        template: str,
        timeout: float = 30.0,
        poll_interval: float = 0.3,
        reuse_existing: bool = True,
    ) -> dict:
        """Ensure an ability instance derived from `template` is Running
        and ready to accept task calls. Blocks until either:
          - an active instance (Running, abilityPort > 0) exists, or
          - `timeout` seconds elapse (raises FrameworkError).

        If `reuse_existing=True` and an active instance for the template's
        ability_name already exists, return it immediately without spawning
        a new one. This makes the call idempotent — calling `start_ability`
        twice on the same template is safe.

        Returns a dict: {
            "instance_id": str,
            "ability_name": str,
            "template": str,          # resolved template metadata.name
            "state": "Running",
            "ipc_port": int,
            "ability_port": int,
            "reused": bool,           # True if no spawn happened
            "ready_in_seconds": float # wall-clock from call to readiness
        }
        """
        t0 = time.monotonic()
        cr = await self.resolve_template(template)
        template_name = cr.get("metadata", {}).get("name", template)
        ability_name = cr.get("spec", {}).get("abilityName")
        if not ability_name:
            raise FrameworkError(
                f"template {template_name!r} has no spec.abilityName"
            )

        reused = False
        if reuse_existing:
            existing = await self.find_running_instance(ability_name)
            if existing is not None:
                return {
                    "instance_id": existing.id,
                    "ability_name": ability_name,
                    "template": template_name,
                    "state": existing.state,
                    "ipc_port": existing.IPCPort,
                    "ability_port": existing.abilityPort,
                    "reused": True,
                    "ready_in_seconds": round(time.monotonic() - t0, 3),
                }

        # Spawn a new instance; framework drives create→start→wait Standby→connect
        await self.spawn_instance(template_name, start=True, connect=True)

        deadline = t0 + timeout
        last_seen_state: Optional[str] = None
        while time.monotonic() < deadline:
            hb = await self.find_running_instance(ability_name)
            if hb is not None:
                return {
                    "instance_id": hb.id,
                    "ability_name": ability_name,
                    "template": template_name,
                    "state": hb.state,
                    "ipc_port": hb.IPCPort,
                    "ability_port": hb.abilityPort,
                    "reused": reused,
                    "ready_in_seconds": round(time.monotonic() - t0, 3),
                }
            # record best state seen for diagnostics
            hbs = await self.list_heartbeats()
            for h in hbs:
                if h.abilityName == ability_name:
                    last_seen_state = h.state
                    break
            await asyncio.sleep(poll_interval)
        raise FrameworkError(
            f"ability {ability_name!r} (template {template_name!r}) not ready "
            f"after {timeout}s; last observed state={last_seen_state or 'none'}"
        )

    async def stop_ability(
        self,
        target: str,
        timeout: float = 10.0,
        poll_interval: float = 0.3,
    ) -> dict:
        """Terminate and destroy ability instances.

        `target` accepts either:
          - an `instance_id` — terminate that specific instance
          - a `template` name or id — find every active instance whose
            ability_name matches the template's ability_name and terminate
            all of them

        Returns: {
            "stopped": [instance_id, ...],
            "count": int,
            "elapsed_seconds": float
        }
        """
        t0 = time.monotonic()
        instance_ids: List[str] = []

        # Case 1: target is a UUID — could be instance_id or cr_id. Prefer
        # instance_id match first.
        is_uuid = False
        try:
            _uuid.UUID(target)
            is_uuid = True
        except ValueError:
            pass

        if is_uuid:
            # Try as instance_id first
            try:
                info = await self.get_instance(target)
                instance_ids.append(info.instance_id)
            except FrameworkError:
                # Fall back to "treat as template UUID and terminate all matching"
                cr = await self.resolve_template(target)
                ability_name = cr.get("spec", {}).get("abilityName")
                if ability_name:
                    instance_ids = [
                        i.instance_id
                        for i in await self.list_instances()
                        if i.ability_name == ability_name
                        and i.state not in LifecycleState.TERMINAL
                    ]
        else:
            # Template name → resolve → ability_name → find all
            cr = await self.resolve_template(target)
            ability_name = cr.get("spec", {}).get("abilityName")
            if not ability_name:
                raise FrameworkError(f"template {target!r} has no spec.abilityName")
            instance_ids = [
                i.instance_id
                for i in await self.list_instances()
                if i.ability_name == ability_name
                and i.state not in LifecycleState.TERMINAL
            ]

        stopped: List[str] = []
        for iid in instance_ids:
            try:
                await self.terminate_instance(iid)
                stopped.append(iid)
            except FrameworkError:
                pass

        # Best-effort: poll until rows disappear or timeout
        deadline = t0 + timeout
        while stopped and time.monotonic() < deadline:
            still_there = False
            for iid in stopped:
                try:
                    info = await self.get_instance(iid)
                    if info.state not in LifecycleState.TERMINAL:
                        still_there = True
                        break
                except FrameworkError as e:
                    if e.status_code == 404:
                        continue
                    still_there = True
                    break
            if not still_there:
                break
            await asyncio.sleep(poll_interval)

        return {
            "stopped": stopped,
            "count": len(stopped),
            "elapsed_seconds": round(time.monotonic() - t0, 3),
        }
