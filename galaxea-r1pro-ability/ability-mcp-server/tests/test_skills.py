"""Tests for SkillCache + framework skill HTTP integration."""

from __future__ import annotations

from pathlib import Path

import httpx
import pytest
import respx

from ability_mcp.core.framework_client import FrameworkClient
from ability_mcp.native.skills import SkillCache, _builtin_skills_root


BASE = "http://localhost:8080"


@pytest.fixture
async def client():
    c = FrameworkClient(BASE)
    yield c
    await c.aclose()


@pytest.fixture
def cache_dir(tmp_path):
    return tmp_path / "skill-cache"


# ---------------------------------------------------------------------------
# builtin: bundled framework skill from <repo>/skills/abilityframework-runtime
# ---------------------------------------------------------------------------


async def test_builtin_skills_are_loaded(client, cache_dir):
    cache = SkillCache(client, cache_dir=cache_dir)
    # No package skills mocked -> framework call will fail / return empty
    with respx.mock(base_url=BASE) as r:
        r.get("/api/skill").mock(return_value=httpx.Response(200, json=[]))
        await cache.refresh()
    builtin = [e for e in cache.entries if e.source == "builtin"]
    assert len(builtin) >= 1
    af = next(
        (e for e in builtin if e.package == "abilityframework-runtime"), None
    )
    assert af is not None, "expected the bundled abilityframework-runtime skill"
    assert af.uri.startswith("skill://_builtin/")
    assert af.cache_path.exists()
    assert "AbilityFramework" in af.cache_path.read_text(encoding="utf-8")


# ---------------------------------------------------------------------------
# package skills: fetched via GET /api/skill + GET /api/skill/:pkg/:ver/:file
# ---------------------------------------------------------------------------


async def test_package_skills_downloaded_and_cached(client, cache_dir):
    skill_md = """---
name: robot-endpoint-control
description: drive the arm
---
# Robot Endpoint Control
"""
    with respx.mock(base_url=BASE) as r:
        r.get("/api/skill").mock(
            return_value=httpx.Response(
                200,
                json=[
                    {
                        "package": "robotics.endpoint.control",
                        "version": "1.0.0",
                        "filename": "SKILL.md",
                        "size": len(skill_md),
                        "title": "robot-endpoint-control",
                    }
                ],
            )
        )
        r.get(
            "/api/skill/robotics.endpoint.control/1.0.0/SKILL.md"
        ).mock(return_value=httpx.Response(200, text=skill_md))
        cache = SkillCache(client, cache_dir=cache_dir)
        await cache.refresh()

    pkg_entries = [e for e in cache.entries if e.source == "package"]
    assert len(pkg_entries) == 1
    e = pkg_entries[0]
    assert e.package == "robotics.endpoint.control"
    assert e.version == "1.0.0"
    assert e.filename == "SKILL.md"
    assert e.uri == "skill://robotics.endpoint.control/1.0.0/SKILL.md"
    assert e.cache_path.exists()
    body = e.cache_path.read_text(encoding="utf-8")
    assert "Robot Endpoint Control" in body
    assert "drive the arm" in body


async def test_materialize_into_target_dir(client, cache_dir, tmp_path):
    with respx.mock(base_url=BASE) as r:
        r.get("/api/skill").mock(
            return_value=httpx.Response(
                200,
                json=[
                    {
                        "package": "p1",
                        "version": "0.1",
                        "filename": "SKILL.md",
                        "size": 12,
                        "title": "p1 skill",
                    }
                ],
            )
        )
        r.get("/api/skill/p1/0.1/SKILL.md").mock(
            return_value=httpx.Response(200, text="# p1 skill\nbody")
        )
        cache = SkillCache(client, cache_dir=cache_dir)
        await cache.refresh()

    target = tmp_path / "claude-skills"
    written = cache.materialize_into(target)
    # at least one builtin + one package
    assert any(p.name == "SKILL.md" for p in written)
    pkg_path = target / "_packages" / "p1" / "0.1" / "SKILL.md"
    assert pkg_path.exists()
    assert "p1 skill" in pkg_path.read_text(encoding="utf-8")


async def test_framework_skill_endpoint_404_does_not_crash(client, cache_dir):
    with respx.mock(base_url=BASE) as r:
        r.get("/api/skill").mock(return_value=httpx.Response(404))
        cache = SkillCache(client, cache_dir=cache_dir)
        await cache.refresh()
    # Should still load builtin even if the framework endpoint is missing
    assert len(cache.entries) >= 1
    assert all(e.source == "builtin" for e in cache.entries)


def test_builtin_skills_root_resolvable():
    root = _builtin_skills_root()
    assert root.exists(), f"bundled skills dir missing: {root}"
    assert (root / "abilityframework-runtime" / "SKILL.md").exists()
