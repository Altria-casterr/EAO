---
name: abilityframework-runtime
description: Drive the AbilityFramework runtime via the native MCP server. Use when the user asks anything about the robot's "abilities", running instances, ability templates, lifecycle (start/stop/connect), or task execution. This is the meta-skill that explains the framework's CR/Instance model and the recommended 3-call workflow; pair it with one or more ability-specific skills for domain knowledge.
---

# AbilityFramework Runtime

This skill describes the **framework-level** tools exposed by
`ability-framework-native` MCP server. It is independent of which ability
packages happen to be loaded — it always applies. For domain knowledge
(units, coordinate frames, task semantics) you should also load the
ability-specific skill that ships with the package being used.

## Mental model (read this first)

Three distinct entities, do not confuse them:

| Entity | What it is | Lifecycle | Source of truth |
|---|---|---|---|
| **Manifest** | Description of an ability *class* — its tasks, params, returns, schema | Loaded from a package's `ability.manifest.yaml` | `list_abilities` |
| **CR Template** | A named launch configuration for an ability (e.g. `endpoint-control-1`). Read-only at runtime. | Loaded from `crs/*.yaml` (workspace) or package-bundled `crs/*.yaml`. Reconciled on framework restart. | `list_templates` |
| **Instance** | A running Python process derived from a template. Has its own `instance_id ≠ cr_id`. Destroyed on terminate / exit. | Created by `start_ability` / `spawn_instance`. Destroyed by `stop_ability` / `terminate_instance` / process exit. | `list_instances` / `list_heartbeats` |

**Do not** try to `POST /api/cr` or `DELETE /api/cr/:id` — both return
`410 Gone`. Templates are package/yaml only.

## Canonical 3-call workflow

For 95% of agent task calls, the right pattern is exactly three tool calls:

```text
1. start_ability(template="<template-name>")
   → blocks until Running, returns {instance_id, ability_port, reused, ...}
   → idempotent: reuses existing Running instance if any

2. <ability>__<task>(...arguments per the task's JSON Schema...)
   → blocks until task completes, returns {task_id, status, payload}
   → can be called multiple times in a row on the same instance

3. stop_ability(target="<template-name-or-instance-id>")
   → terminates and destroys, returns {stopped: [...], count}
```

You do **not** need to call `list_heartbeats` or `lifecycle_command` in
the happy path. `start_ability` already polls for Running internally.

## Tool reference

### Simplified (preferred)

| Tool | Purpose |
|---|---|
| `start_ability(template, timeout=30)` | Boot an ability instance from a template, block until Running. Idempotent. |
| `stop_ability(target, timeout=10)` | Terminate one instance (by id) or all matching instances of a template. |
| `<ability>__<task>(...)` | Dynamic per-manifest task tools. Each blocks until terminal status. Names are derived as `<snake_ability_name>__<snake_task_name>`. |

### Discovery

| Tool | Purpose |
|---|---|
| `list_abilities` | All ability classes (manifest summaries) loaded into the framework. Use to see what's available. |
| `list_templates` | All CR templates (read-only). Each has a `metadata.name` (use this with `start_ability`) and a `spec.abilityName`. |
| `list_instances` | All runtime instance rows. State, start_time, instance_id, the spec snapshot. |
| `list_heartbeats` | Live heartbeats — actually-reachable instances. Includes IPC/abilityPort. |

### Power user / recovery

| Tool | When to use |
|---|---|
| `spawn_instance(template, start, connect)` | Lower-level non-blocking variant of `start_ability`. Returns immediately with a `taskId`. |
| `terminate_instance(instance_id)` | Lower-level direct delete by id (no template→batch resolution). |
| `lifecycle_command(instance_id, command)` | `start` / `connect` / `disconnect` / `terminate` on an existing instance. Use `connect` to recover a stuck-in-Standby instance. |
| `refresh_tools` | Reload manifests + dynamic tools **and skill cache** from the framework, then re-publish MCP resources. Call after uploading a new ability package or whenever you suspect stale tool / skill state. See the "session bootstrap" and "package just got updated" recipes below. |

## Common patterns

### Pattern: one shot — "do X with the robot"

```text
start_ability(template="endpoint-control-1")
endpoint_control_robotics__move_to_pose(position={x,y,z}, orientation={x,y,z,w})
stop_ability(target="endpoint-control-1")
```

### Pattern: chain of tasks on the same instance

```text
start_ability(template="endpoint-control-1")
endpoint_control_robotics__get_current_pose()
# inspect result, decide next action
endpoint_control_robotics__move_to_pose(...)
endpoint_control_robotics__set_gripper(opening=0.0)
stop_ability(target="endpoint-control-1")
```

You can call `start_ability` again at any point — it's idempotent and
returns `reused: true` in one round-trip if the instance is still alive.

### Pattern: recovery from stuck Standby

```text
list_heartbeats()
# spot an instance with state=Standby and abilityPort=0 — it failed to auto-connect
lifecycle_command(instance_id="<id>", command="connect")
# poll with list_heartbeats until state=Running
```

## Errors and what they mean

- **`{error: "no active instance for ability X", hint: "call start_ability..."}`** — you tried to call a task tool without booting the ability. Call `start_ability(template=<name>)` first.
- **`{error: "instance ... is in state Standby, not Running"}`** — the ability is up but its IPC port isn't listening. Almost always means a transient race; `start_ability` should have waited, but if you used `spawn_instance` you have to either re-issue or call `lifecycle_command(connect)`.
- **`status: timeout`** in a task tool result — the task did not reach a terminal status within the polling window (default 120s). Either the ability is hung or the task is genuinely longer than the window.
- **`410 Gone`** on `/api/cr` POST/DELETE — you tried to create or delete a CR template at runtime; not allowed. Templates come from yaml.

## Skill discovery and updates

### The picture

Each ability package can ship its own `skills/*.md` agent guide inside
the zip. On install (and on every periodic `update()` scan) the
framework mirrors those files into
`<framework_home>/skills/_packages/<pkg>/<version>/` and exposes them at:

- `GET /api/skill` — list of `{package, version, filename, size, title}`
- `GET /api/skill/:pkg/:version/:filename` — raw markdown

This MCP server pulls the list on **startup** and on every
`refresh_tools` call, caches each file under
`~/.cache/ability-mcp/skills/` (override with `--skill-cache-dir`), and
publishes them two ways — which one the connected MCP client consumes
depends on the client:

1. As MCP **resources** with stable URIs `skill://<pkg>/<version>/<filename>`,
   readable via the MCP `resources/read` protocol. This is the
   in-protocol, client-agnostic path — anything that implements the MCP
   client half (Claude Code, OpenClaw, MCP Inspector, a custom agent
   runner) can consume them without touching the filesystem.
2. As files inside the cache dir, so a client that has its own
   disk-based skill loader (e.g. Claude Code reads `~/.claude/skills/`,
   OpenClaw and others have their own per-install roots) can sync them
   into whatever directory it watches. The `sync_skills_to_dir` tool
   below is the entry point for that.

On top of those, there's always one **built-in** skill — this one, the
"abilityframework-runtime" skill — under the synthetic package `_builtin`.
It never comes from the framework and is always present.

### Skill-related tools you have

| Tool | Purpose |
|---|---|
| `list_skills()` | Enumerate everything in the cache. Each entry has `source` (`builtin` / `package`), `package`, `version`, `filename`, `title`, `uri`, `cache_path`, `size_bytes`. Call this before planning if you're unsure what guidance is available. |
| `read_skill(uri)` | Return the full markdown body of one skill, given a `skill://...` URI from `list_skills`. |
| `sync_skills_to_dir(target)` | Copy every cached skill into `target`, preserving the `<source>/<package>/<version>/<filename>` layout. Useful for writing skills into a client's skill loader directory so they survive across sessions. The path is client-specific: Claude Code watches `~/.claude/skills/`, OpenClaw and others have their own conventions — pick the one for your client (or ask the user) rather than assuming a default. |
| `refresh_tools()` | Re-fetch manifests **and** skills from the framework, re-register dynamic tools, and re-publish resources. Call this after any ability package upload / uninstall, or whenever you suspect the cache is stale. Returns `{abilities, dynamic_tools, skills}` counts. |

### Recipe: session bootstrap (run this once after `/mcp` connects)

Do this **before** attempting any non-trivial task on an unfamiliar robot:

```text
1. list_abilities()
   → see what ability classes the framework knows about

2. list_skills()
   → see which ones ship usage guides. You should always see at least
     one entry with source="builtin" (this skill) plus one per ability
     package that bundled a skills/ dir.

3. For each package-source skill relevant to the user's request:
     read_skill(uri=<from step 2>)
   → learn the domain-specific conventions, coordinate frames, safety
     constraints, and recommended compositions

4. (Optional) sync_skills_to_dir(target="<client skill dir>/ability-framework")
   → persist the cache into your MCP client's local skill loader so
     future sessions can match these skills without re-fetching. The
     target path is client-specific — Claude Code uses
     `~/.claude/skills/`, OpenClaw and other clients have their own
     conventions. Ask the user or check the client's docs. Skip this
     step entirely if the client consumes MCP resources directly via
     `resources/read` — then `read_skill(uri)` is enough.
```

If `list_skills()` returns only the builtin entry, either no package
ships a skill yet, or the framework's `/api/skill` endpoint is missing
(older framework). Fall back to calling `list_abilities` + the dynamic
tool descriptions, and ask the user for domain details you need.

### Recipe: ability package just got updated

Trigger this when: the user uploads a new ability package, mentions
installing/upgrading/uninstalling an ability, or you call a dynamic
tool that unexpectedly returns `{"error": "no active instance for
ability X"}` where X was supposed to exist.

```text
1. refresh_tools()
   → rebuilds the dynamic tool list + re-pulls the skill cache +
     re-publishes MCP resources. Returns fresh counts:
     {abilities, dynamic_tools, skills}.

2. Compare the counts with what you remember. If `abilities` or
   `skills` went up, there are new things to learn:

3. list_abilities()  # if abilities went up
   → identify the new ability class(es) and their packages

4. list_skills()     # if skills went up
   → see which new guides are available

5. For each new package skill:
     read_skill(uri=...)

6. (Optional) sync_skills_to_dir(target=...)
   → only if the user wants the updated skills persisted to their
     local skill loader dir
```

If `refresh_tools()` itself returns an error (e.g. framework
unreachable, `410 Gone` on some internal call), the tool counts stay
stale — surface the error to the user instead of silently proceeding.

### Important rules

- **Never invent task arguments.** Always read the relevant package
  skill first (or the dynamic tool's description / param list) before
  calling a task tool with non-trivial inputs.
- **Skill content beats tool descriptions.** The dynamic tool's
  description comes from the manifest and is deliberately terse.
  Package skills have the surrounding knowledge (units, frames,
  compositions, safety). When in doubt, `read_skill(uri=...)`.
- **Built-in skill is authoritative for framework semantics.** This
  file is the source of truth for lifecycle / CR-vs-instance /
  error-handling. If a package skill contradicts it on framework
  mechanics, this file wins. Package skills only override their own
  domain.
