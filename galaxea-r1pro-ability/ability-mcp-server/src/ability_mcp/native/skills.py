"""Skill discovery, download, cache, and MCP resource exposure.

Skills come from two sources:

1. **Built-in framework skill** (`skills/abilityframework-runtime/SKILL.md`)
   shipped inside this repo. Always available.

2. **Per-ability skills** mirrored from each package by the framework into
   `<framework_home>/skills/_packages/<pkg>/<version>/`, fetched via
   `GET /api/skill`. Updated on every `refresh_skills()` call.

What this module does at startup:

- Pulls the latest skill catalog from the framework.
- Writes each skill into a local cache directory
  (`<cache_dir>/<source>/<pkg>/<version>/<filename>` or
  `<cache_dir>/_builtin/<name>/SKILL.md`), so **any MCP client** (Claude
  Code, OpenClaw, MCP Inspector, a bespoke agent runner, etc.) can mirror
  them into its own skill loader directory.
- Registers each skill as an MCP `resource` with a stable `skill://` URI,
  so clients that consume MCP resources directly can read them in-band
  without needing to understand the on-disk cache layout.
- Exposes a `sync_skills_to_dir(target_dir)` MCP tool for explicit
  user-driven installs. Each client's skill loader lives at a different
  path (Claude Code uses `~/.claude/skills/`, OpenClaw and others have
  their own conventions) — agents should ask the user or inspect
  environment before hardcoding a target.
"""

from __future__ import annotations

import asyncio
import shutil
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

from ..core.framework_client import FrameworkClient, FrameworkError


@dataclass
class CachedSkill:
    """One skill file resolved + cached locally."""

    source: str           # "builtin" or "package"
    package: str          # for builtin: skill folder name; for package: <pkg>
    version: str          # for builtin: "0.0.0"; for package: <ver>
    filename: str         # relative path inside the version dir, e.g. "SKILL.md"
    title: str            # extracted title (frontmatter or # heading)
    cache_path: Path      # absolute path on disk where this file lives
    uri: str              # stable MCP resource URI
    size_bytes: int = 0


def _builtin_skills_root() -> Path:
    """Find the bundled `skills/` directory shipped inside this package.

    Layout (as installed via pip):
        <site-packages>/ability_mcp/native/skills.py    <-- this file
        <repo>/skills/abilityframework-runtime/SKILL.md  <-- target

    We resolve relative to this file so editable installs work too.
    """
    here = Path(__file__).resolve()
    # ascend to the repo root: native/ -> ability_mcp/ -> src/ -> <repo>/
    repo_root = here.parents[3]
    return repo_root / "skills"


def _safe_filename(s: str) -> str:
    return "".join(c if c.isalnum() or c in "._-" else "_" for c in s)


class SkillCache:
    """In-memory + on-disk cache of skill files.

    `cache_dir` defaults to `~/.cache/ability-mcp/skills/` if not provided.
    Lifecycle:

        cache = SkillCache(framework_client, cache_dir=...)
        await cache.refresh()              # pulls + writes everything
        cache.entries                       # list[CachedSkill]
        cache.materialize_into(target)      # copy/symlink into a user dir
    """

    DEFAULT_CACHE_DIR = Path.home() / ".cache" / "ability-mcp" / "skills"

    def __init__(
        self,
        client: FrameworkClient,
        cache_dir: Optional[Path] = None,
    ) -> None:
        self.client = client
        self.cache_dir = (cache_dir or self.DEFAULT_CACHE_DIR).expanduser().resolve()
        self.entries: List[CachedSkill] = []

    # ---- builtin --------------------------------------------------------

    def _load_builtin_skills(self) -> List[CachedSkill]:
        """Mirror bundled `<repo>/skills/<name>/SKILL.md` into the cache."""
        out: List[CachedSkill] = []
        root = _builtin_skills_root()
        if not root.exists():
            return out
        for skill_dir in sorted(root.iterdir()):
            if not skill_dir.is_dir():
                continue
            name = skill_dir.name
            for src_file in sorted(skill_dir.rglob("*")):
                if not src_file.is_file():
                    continue
                rel = src_file.relative_to(skill_dir)
                dest = self.cache_dir / "_builtin" / name / rel
                dest.parent.mkdir(parents=True, exist_ok=True)
                if not dest.exists() or dest.read_bytes() != src_file.read_bytes():
                    shutil.copy2(src_file, dest)
                title = _extract_title(dest) or name
                out.append(
                    CachedSkill(
                        source="builtin",
                        package=name,
                        version="0.0.0",
                        filename=str(rel),
                        title=title,
                        cache_path=dest,
                        uri=f"skill://_builtin/{name}/{rel.as_posix()}",
                        size_bytes=dest.stat().st_size,
                    )
                )
        return out

    # ---- package skills via framework ----------------------------------

    async def _load_package_skills(self) -> List[CachedSkill]:
        out: List[CachedSkill] = []
        try:
            entries = await self.client.list_skills()
        except FrameworkError:
            return out
        for entry in entries:
            pkg = entry.get("package") or ""
            ver = entry.get("version") or ""
            fn = entry.get("filename") or ""
            if not (pkg and ver and fn):
                continue
            try:
                content = await self.client.download_skill(pkg, ver, fn)
            except FrameworkError:
                continue
            dest = self.cache_dir / "_packages" / _safe_filename(pkg) / _safe_filename(ver) / fn
            dest.parent.mkdir(parents=True, exist_ok=True)
            dest.write_text(content, encoding="utf-8")
            title = entry.get("title") or _extract_title(dest) or fn
            out.append(
                CachedSkill(
                    source="package",
                    package=pkg,
                    version=ver,
                    filename=fn,
                    title=title,
                    cache_path=dest,
                    uri=f"skill://{pkg}/{ver}/{fn}",
                    size_bytes=dest.stat().st_size,
                )
            )
        return out

    # ---- public --------------------------------------------------------

    async def refresh(self) -> int:
        """Reload everything (builtin + package) into the cache. Returns the
        total number of skill files cached."""
        self.cache_dir.mkdir(parents=True, exist_ok=True)
        self.entries = self._load_builtin_skills()
        self.entries += await self._load_package_skills()
        return len(self.entries)

    def find(self, uri: str) -> Optional[CachedSkill]:
        for s in self.entries:
            if s.uri == uri:
                return s
        return None

    def materialize_into(self, target_dir: Path) -> List[Path]:
        """Copy every cached skill into `target_dir`, preserving the
        `<source>/<pkg>/<version>/<filename>` layout.

        Useful for one-shot installs into whatever skill-loader directory
        the connected MCP client uses (Claude Code reads
        `~/.claude/skills/`; OpenClaw and other clients have their own
        paths — consult the client docs). Returns the list of files
        actually written.
        """
        target = Path(target_dir).expanduser().resolve()
        target.mkdir(parents=True, exist_ok=True)
        written: List[Path] = []
        for s in self.entries:
            sub = (
                Path("_builtin") / s.package / s.filename
                if s.source == "builtin"
                else Path("_packages") / s.package / s.version / s.filename
            )
            dest = target / sub
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(s.cache_path, dest)
            written.append(dest)
        return written

    def to_dicts(self) -> List[Dict[str, Any]]:
        return [
            {
                "source": s.source,
                "package": s.package,
                "version": s.version,
                "filename": s.filename,
                "title": s.title,
                "uri": s.uri,
                "cache_path": str(s.cache_path),
                "size_bytes": s.size_bytes,
            }
            for s in self.entries
        ]


def _extract_title(path: Path) -> str:
    """Pull a human title out of a skill file: prefer YAML frontmatter
    `name:` / `title:`, fall back to the first `# Heading`."""
    try:
        text = path.read_text(encoding="utf-8", errors="ignore")
    except OSError:
        return ""
    lines = text.splitlines()
    in_fm = False
    fm_name = ""
    fm_title = ""
    for i, line in enumerate(lines[:60]):
        if i == 0 and line.strip() == "---":
            in_fm = True
            continue
        if in_fm:
            if line.strip() == "---":
                in_fm = False
                continue
            for key, target in (("name:", "fm_name"), ("title:", "fm_title")):
                if line.startswith(key):
                    val = line[len(key):].strip().strip('"').strip("'")
                    if target == "fm_name":
                        fm_name = val
                    else:
                        fm_title = val
            continue
        if line.startswith("# "):
            return line[2:].strip()
    return fm_title or fm_name
