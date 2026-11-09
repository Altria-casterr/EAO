"""CLI entry: `python -m ability_mcp.native` or `ability-mcp-native`."""

from __future__ import annotations

import argparse
import asyncio
import sys
from pathlib import Path

from ..core.framework_client import FrameworkClient
from .server import NativeServer


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        prog="ability-mcp-native",
        description="Native MCP server exposing AbilityFramework HTTP API",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    ap.add_argument(
        "--framework-url",
        default="http://localhost:8080",
        help="AbilityFramework HTTP base URL",
    )
    ap.add_argument(
        "--expose-abilities",
        default="*",
        help=(
            "Comma-separated glob patterns of ability names to expose as dynamic "
            "tools. Use `*` for all. Example: 'EndpointControl.*,Perception.*'"
        ),
    )
    ap.add_argument(
        "--task-timeout",
        type=float,
        default=120.0,
        help="Per-task polling timeout in seconds",
    )
    ap.add_argument(
        "--http-timeout",
        type=float,
        default=10.0,
        help="HTTP read/connect timeout for framework calls",
    )
    ap.add_argument(
        "--skill-cache-dir",
        type=Path,
        default=None,
        help=(
            "Directory to cache downloaded ability skills + the bundled "
            "framework skill. Defaults to ~/.cache/ability-mcp/skills/ "
            "if not specified."
        ),
    )
    return ap


async def _amain(args: argparse.Namespace) -> int:
    client = FrameworkClient(args.framework_url, timeout=args.http_timeout)
    server = NativeServer(
        client,
        expose_abilities=args.expose_abilities,
        task_timeout=args.task_timeout,
        skill_cache_dir=args.skill_cache_dir,
    )
    try:
        await server.startup()
        await server.run_stdio()
    finally:
        await client.aclose()
    return 0


def main() -> None:
    args = build_parser().parse_args()
    try:
        sys.exit(asyncio.run(_amain(args)))
    except KeyboardInterrupt:
        sys.exit(130)


if __name__ == "__main__":
    main()
