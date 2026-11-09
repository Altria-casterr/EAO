"""FastMCP server setup: wire FrameworkClient + ManifestRegistry + tools."""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Optional

from mcp.server.fastmcp import FastMCP
from mcp.server.fastmcp.resources import FunctionResource

from ..core.framework_client import FrameworkClient, FrameworkError
from ..core.registry import ManifestRegistry
from .skills import CachedSkill, SkillCache
from .tools import register_dynamic_tools, register_static_tools


def _log(msg: str) -> None:
    """Write a status message to stderr. stdout is reserved for the MCP
    stdio protocol frames — writing there corrupts the stream."""
    print(msg, file=sys.stderr, flush=True)


class NativeServer:
    """Lifetime container for the MCP server instance.

    Construct with a FrameworkClient and an (optional) glob pattern to
    limit which abilities become dynamic tools, then `await startup()`
    before invoking `run_stdio()` or `run_sse()`.
    """

    def __init__(
        self,
        client: FrameworkClient,
        expose_abilities: str = "*",
        task_timeout: float = 120.0,
        server_name: str = "ability-framework-native",
        skill_cache_dir: Optional[Path] = None,
    ) -> None:
        self.client = client
        self.expose_abilities = expose_abilities
        self.task_timeout = task_timeout
        self.mcp = FastMCP(server_name)
        self.registry = ManifestRegistry()
        self.skill_cache = SkillCache(client, cache_dir=skill_cache_dir)
        self._dynamic_tool_names: list[str] = []
        self._registered_skill_uris: list[str] = []

    async def _load_registry(self) -> int:
        """Pull manifests from the framework, build the registry, apply filter.

        Returns the number of dynamic tool candidates after filtering.
        """
        self.registry.clear()
        summaries = await self.client.list_manifests()
        for s in summaries:
            try:
                m = await self.client.get_manifest(s.abilityName, s.version)
                self.registry.add(m)
            except FrameworkError:
                # tolerate flaky manifest endpoint for a subset of packages
                continue
        self.registry.filter(self.expose_abilities)
        return sum(len(self.registry.get(n).tasks) for n in self.registry.ability_names())

    def _drop_dynamic_tools(self) -> None:
        for name in self._dynamic_tool_names:
            try:
                self.mcp._tool_manager.remove_tool(name)
            except Exception:
                pass
        self._dynamic_tool_names.clear()

    async def _reload(self) -> int:
        """refresh_tools callback: re-read manifests + skills, re-register dynamic tools + resources."""
        await self._load_registry()
        self._drop_dynamic_tools()
        new_names = [n for n in _collect_tool_names(self.registry)]
        register_dynamic_tools(
            self.mcp, self.client, self.registry, task_timeout=self.task_timeout
        )
        self._dynamic_tool_names = new_names
        # Refresh skill cache + re-publish as resources (best effort).
        try:
            await self.skill_cache.refresh()
            self._republish_skill_resources()
        except FrameworkError as e:
            _log(f"[ability-mcp] warning: skill refresh failed: {e}")
        # FastMCP's list_changed is sent automatically when tools/resources
        # are added/removed via the manager.
        return len(new_names)

    def _republish_skill_resources(self) -> None:
        """Re-register every cached skill as an MCP resource so MCP clients
        that consume resources directly (rather than scanning a local skill
        directory) can read them in-band."""
        # Drop previously registered ones first
        rm = getattr(self.mcp._resource_manager, "remove_resource", None)
        if callable(rm):
            for uri in self._registered_skill_uris:
                try:
                    rm(uri)
                except Exception:
                    pass
        self._registered_skill_uris.clear()
        for s in self.skill_cache.entries:
            cache_path = s.cache_path

            def _read_skill_body(_p: Path = cache_path) -> str:
                try:
                    return _p.read_text(encoding="utf-8")
                except OSError as e:
                    return f"<error reading skill: {e}>"

            try:
                self.mcp.add_resource(
                    FunctionResource(
                        uri=s.uri,
                        name=f"{s.package}/{s.filename}",
                        title=s.title or s.filename,
                        description=(
                            f"Skill document from {s.source} {s.package} "
                            f"{s.version}: {s.filename}"
                        ),
                        mime_type="text/markdown",
                        fn=_read_skill_body,
                    )
                )
                self._registered_skill_uris.append(s.uri)
            except Exception as e:
                _log(f"[ability-mcp] warning: failed to register resource {s.uri}: {e}")

    async def startup(self) -> None:
        # validate framework reachability eagerly so we fail fast
        try:
            await self.client.hello()
        except FrameworkError as e:
            # still register static tools so `list_abilities` etc. can report
            # the error instead of crashing the stdio transport
            _log(f"[ability-mcp] warning: framework handshake failed: {e}")
        await self._load_registry()
        # Pull skills (builtin + framework-mirrored package skills) into the
        # local cache. This always at least loads the bundled framework skill.
        try:
            await self.skill_cache.refresh()
        except FrameworkError as e:
            _log(f"[ability-mcp] warning: initial skill fetch failed: {e}")
        register_static_tools(
            self.mcp,
            self.client,
            self.registry,
            reload_cb=self._reload,
            skill_cache=self.skill_cache,
        )
        register_dynamic_tools(
            self.mcp, self.client, self.registry, task_timeout=self.task_timeout
        )
        self._dynamic_tool_names = list(_collect_tool_names(self.registry))
        self._republish_skill_resources()
        _log(
            f"[ability-mcp] ready: "
            f"{len(self.registry.ability_names())} abilities, "
            f"{len(self._dynamic_tool_names)} dynamic tools, "
            f"{len(self.skill_cache.entries)} skills cached at "
            f"{self.skill_cache.cache_dir}"
        )

    async def run_stdio(self) -> None:
        await self.mcp.run_stdio_async()


def _collect_tool_names(registry: ManifestRegistry):
    from .tools import tool_name as _tn

    for ability_name, task in registry.tasks():
        yield _tn(ability_name, task.taskName)
