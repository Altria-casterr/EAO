# ability-mcp-server

A **native** MCP server exposing the AbilityFramework HTTP API as MCP tools.
Serves as the baseline for the flagship "Enhanced" variant (streaming
Context Store + Snapshot + Narrator).

```
┌─────────────────────┐   stdio    ┌──────────────────────────┐
│ any MCP client      │ ◀────────▶ │  ability-mcp-native      │
│  Claude Code        │            │  (FastMCP, single proc)  │
│  OpenClaw           │            │                          │
│  MCP Inspector      │            │  static tools: 10        │
│  Claude API         │            │    (incl. skill tools)   │
│  custom agent …     │            │  dynamic tools: N        │
└─────────────────────┘            │    (one per manifest     │
                                   │     task)                │
                                   └──────────┬───────────────┘
                                              │ HTTP :8080
                                              ▼
                                    ┌───────────────────┐
                                    │  AbilityFramework │
                                    │  (workspace/      │
                                    │   AbilityFramework)│
                                    └───────────────────┘
```

The server speaks the standard MCP stdio protocol and does not assume
any particular client. It's been wired up against Claude Code and
OpenClaw in this repo; any other client that can exec a Python
stdio MCP server works the same.

## Features

- **Two tool tiers**:

  *Simplified (agent workflow)*
  - `start_ability(template, timeout)` — one-click boot, blocks until the
    instance is Running and reachable by the task proxy; idempotent — if
    an active instance already exists it's reused.
  - `stop_ability(target, timeout)` — one-click shutdown of a single
    instance_id or all instances of a template.
  - *(task tools)* — the dynamic `ability__task` tools generated from
    manifests also block until the task finishes (internal polling), so
    the agent's full flow fits in 3 calls: `start_ability → <task> →
    stop_ability`.

  *Lower level (power users / backward compat)*
  - `list_abilities` `list_templates` `list_instances` `list_heartbeats`
  - `spawn_instance` `terminate_instance` `lifecycle_command`
  - `refresh_tools`
- **N dynamic tools** generated at startup from every `(abilityName, taskName)`
  declared in each package's `ability.manifest.yaml`. JSON Schema for each
  tool is derived from `tasks[].params` (including `Point3d` / `Quaternion`
  composite shapes).
- Task execution polls via the framework's ability proxy
  `POST /api/ability/:id/api/task/start → task_id → /api/task/status`;
  exponential backoff with a configurable timeout.
- `start_ability` polls `list_heartbeats` until `state == Running &&
  abilityPort > 0`, then returns `{instance_id, ability_port, reused,
  ready_in_seconds}`. This is implemented entirely on the client side —
  the framework's `POST /api/instance` already performs
  `create → start → wait Standby → connect` internally; the client just
  correlates the resulting instance_id via heartbeats.
- No background work, no caches, no narration. Every tool call round-trips
  live against the framework. This is intentional: it's the "raw" baseline
  the Enhanced design measures against.

## Compatibility with recent framework changes

This server targets the **post-refactor** framework API:

| Endpoint | Status |
|---|---|
| `POST /api/cr` | ❌ `410 Gone` — CR templates are yaml / package-bundled only |
| `DELETE /api/cr/:id` | ❌ `410 Gone` — delete yaml and restart to reconcile |
| `GET /api/cr` | ✅ list templates |
| `GET /api/cr/:id` | ✅ resolves via `AbilityInstance.spec_snapshot` first, falls back to template table |
| `POST /api/instance` | ✅ **create + start + connect** a new runtime instance from a template |
| `DELETE /api/instance/:id` | ✅ terminate and destroy |
| `GET /api/instance[/:id]` | ✅ list / detail (now includes `spec_snapshot`) |
| `POST /api/lifecycle-request` | ✅ lifecycle transitions on existing instances |
| `/api/ability/:id/<path>` | ✅ proxies to the instance's `abilityPort` |

See `../AgentResearch/abilityframework-mcp-native-design.md` for full rationale.

## Install

```bash
cd ability-mcp-server/

# Create an isolated venv (uv is fastest but `python -m venv` also works)
uv venv --python 3.12 .venv
uv pip install --python .venv/bin/python -e '.[test]'

# Sanity check
.venv/bin/python -m pytest -q
```

## Run

```bash
# With the default workspace framework on :8080
.venv/bin/python -m ability_mcp.native

# Against a remote framework, expose only robotics-family abilities
.venv/bin/python -m ability_mcp.native \
    --framework-url http://robot-node-1:8080 \
    --expose-abilities 'EndpointControl.*,Perception.*'
```

The server uses **stdio transport** — standard MCP over the child
process's stdin/stdout. It is meant to be spawned by an MCP client
(Claude Code, OpenClaw, MCP Inspector, Claude API `tool_use` runner,
or any custom agent runner that implements the MCP client half).

## MCP client integration

The command line is always the same — `python -m ability_mcp.native
[flags]`. Each client has its own way of registering that command; see
`examples/clients.md` for fully-worked examples for Claude Code,
OpenClaw, and the MCP Inspector, plus a generic recipe for anything
else. The short version:

```bash
/abs/path/to/ability-mcp-server/.venv/bin/python \
    -m ability_mcp.native \
    --framework-url http://localhost:8080
```

Point your client at that command, give it a stable name like
`ability-framework-native`, and you're done. On first connect the
server registers:

- **10 static tools** — the skill management tools (`list_skills`,
  `read_skill`, `sync_skills_to_dir`, `refresh_tools`) plus the
  simplified lifecycle (`start_ability`, `stop_ability`) and the
  discovery / low-level management set.
- **N dynamic tools** — one per `(ability, task)` from the framework
  manifests.
- **2+ MCP resources** — one per cached skill, at stable URIs
  `skill://<source>/<package>/<version>/<filename>`.

Typical agent prompt:

> Move the robot end effector to (0.3, 0.1, 0.2), then shut it down.

The LLM is expected to use the **simplified 3-call flow**:

1. `start_ability(template="endpoint-control-1")` → waits for Running, returns instance_id
2. `endpoint_control_robotics__move_to_pose({position:..., orientation:...})` → blocks until task completes
3. `stop_ability(target="endpoint-control-1")` → terminates and destroys

The lower-level tools (`spawn_instance`, `lifecycle_command`,
`list_heartbeats`, …) are still there if the agent needs finer control or
diagnostic visibility, but the happy path is the 3-call sequence above.

## Skills

Two layers of skills are exposed via this MCP server:

1. **Framework runtime skill (built-in, common)** — `skills/abilityframework-runtime/SKILL.md`
   in this repo. Describes the CR/Instance model, the 3-call `start_ability →
   <task> → stop_ability` workflow, and recovery patterns. Always loaded.

2. **Per-ability skills (per-package)** — each ability package can ship its
   own skill files inside `<package>/skills/`. The framework mirrors them
   to `<framework_home>/skills/_packages/<pkg>/<version>/` on install (and
   on every `update()` cycle, idempotently for already-installed packages),
   and exposes them via:
   - `GET /api/skill` — list of `{package, version, filename, size, title}`
   - `GET /api/skill/:pkg/:version/:filename` — raw text
   - WebUI tab "技能文档" — browse and view inline

   The current workspace ships a real example:
   `workspace/packages/robotics.endpoint.control/1.0.0/skills/SKILL.md`.

The MCP server pulls both layers at startup into a local cache directory
(default `~/.cache/ability-mcp/skills/`, configurable via
`--skill-cache-dir`) and:

- Registers each as an MCP **resource** with a stable `skill://` URI, so
  any MCP client that consumes resources can read them in-band.
- Exposes three MCP tools: `list_skills`, `read_skill(uri)`,
  `sync_skills_to_dir(target)`.
- Refreshes them on `refresh_tools` calls (and re-registers the resources).

To wire the skills into your client's local skill-loader directory (each
client has its own path: Claude Code uses `~/.claude/skills/`, OpenClaw
and others have their own conventions), the agent or the user calls:

```text
sync_skills_to_dir(target="<your client's skill dir>/ability-framework")
```

That copies every cached skill (builtin + per-package) into the target
directory, preserving the `<source>/<package>/<version>/<filename>` layout.
Whatever watches that directory (Claude Code's skill loader, OpenClaw's,
etc.) picks them up automatically on next scan. After uploading new
ability packages, call `refresh_tools` then `sync_skills_to_dir` again.
See `examples/clients.md` for per-client target paths.

## Project layout

```
ability-mcp-server/
├── pyproject.toml
├── README.md
├── src/ability_mcp/
│   ├── core/
│   │   ├── framework_client.py   # async HTTP client, post-refactor API + skill endpoints
│   │   ├── models.py             # pydantic models (Manifest, Instance, Heartbeat, ...)
│   │   ├── registry.py           # in-memory manifest index + task lookup
│   │   └── schema.py             # manifest params -> JSON Schema, tool_name()
│   └── native/
│       ├── __main__.py           # CLI entry (python -m ability_mcp.native)
│       ├── server.py             # FastMCP glue + startup/reload lifecycle + skill resource publishing
│       ├── skills.py             # SkillCache (fetch + on-disk cache + materialize)
│       └── tools.py              # static tools + dynamic task dispatcher + skill tools
├── skills/
│   └── abilityframework-runtime/
│       └── SKILL.md              # built-in framework runtime skill (always loaded)
├── tests/
│   ├── test_schema.py            # schema conversion tests
│   ├── test_registry.py          # registry filter/index tests
│   ├── test_framework_client.py  # respx-mocked HTTP tests
│   ├── test_lifecycle_helpers.py # ensure_ability_ready / stop_ability tests
│   └── test_skills.py            # SkillCache builtin + package fetch + materialize tests
└── examples/
    ├── mcp.json                  # Claude Code `.mcp.json` template
    └── clients.md                # Wiring recipes: Claude Code, OpenClaw, MCP Inspector, generic
```

## Tests

```bash
.venv/bin/python -m pytest -v
```

All tests are offline (no framework required); HTTP paths are mocked with
`respx`.

## Non-goals

- No context store, no snapshot, no narrator — those belong to the Enhanced
  variant.
- No heuristic auto-start of missing instances; errors clearly hint
  `spawn_instance` as the next step for the LLM.
- No batch tools — the model is expected to string task calls together.
