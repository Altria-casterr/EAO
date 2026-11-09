# Wiring `ability-mcp-native` into MCP clients

The server is client-agnostic: it speaks standard MCP over stdio. The
only thing each client needs is the command to spawn, plus (optionally)
a place to drop skill files on disk. Everything else — tool discovery,
resource discovery, dynamic refresh — is handled by the MCP protocol
itself.

The canonical spawn command is always:

```bash
/abs/path/to/ability-mcp-server/.venv/bin/python \
    -m ability_mcp.native \
    --framework-url http://localhost:8080
```

Flags:

| Flag | Purpose |
|---|---|
| `--framework-url URL` | Where to reach the AbilityFramework HTTP API (default `http://localhost:8080`). |
| `--expose-abilities PATTERN` | Comma-separated glob list, e.g. `"EndpointControl.*,Perception.*"`, to filter which abilities become dynamic tools. Default `*`. |
| `--task-timeout SECONDS` | Per-task polling timeout (default 120). |
| `--http-timeout SECONDS` | Framework HTTP read/connect timeout (default 10). |
| `--skill-cache-dir PATH` | Override the local skill cache. Default `~/.cache/ability-mcp/skills`. |

Below: wiring recipes for the clients that have been exercised against
this server in the repo, plus a generic recipe for anything else.

---

## 1. Claude Code (Anthropic)

Claude Code reads `.mcp.json` from the project (or the user's
`~/.claude.json`). Put this file at the root of whichever workspace
you run `claude` in — see also `examples/mcp.json`:

```json
{
  "mcpServers": {
    "ability-framework-native": {
      "command": "/home/YOU/.../ability-mcp-server/.venv/bin/python",
      "args": [
        "-m", "ability_mcp.native",
        "--framework-url", "http://localhost:8080"
      ]
    }
  }
}
```

**Skill loader directory**: `~/.claude/skills/` (global) or
`.claude/skills/` (project-local). After bringing up the server the
agent can run:

```text
sync_skills_to_dir(target="~/.claude/skills/ability-framework")
```

to materialise every cached skill into Claude Code's loader. Restart
the session or let the loader rescan to pick them up.

**Validate in session**: type `/mcp` — you should see
`ability-framework-native` listed with its tools and resources counts.

---

## 2. OpenClaw

OpenClaw is an open-source Claude-Code-like MCP client
([github.com/openclaw/openclaw](https://github.com/openclaw/openclaw)).
It follows the same `.mcp.json` convention for MCP server registration
and has a similar on-disk skill loader.

Point it at the same spawn command:

```json
{
  "mcpServers": {
    "ability-framework-native": {
      "command": "/home/YOU/.../ability-mcp-server/.venv/bin/python",
      "args": [
        "-m", "ability_mcp.native",
        "--framework-url", "http://localhost:8080"
      ]
    }
  }
}
```

**Skill loader directory**: consult your OpenClaw installation's docs
for the exact path (OpenClaw's skill layout mirrors Claude Code's, but
the root directory is configurable and may differ per install). Once
you know it, the sync call is:

```text
sync_skills_to_dir(target="<openclaw skill root>/ability-framework")
```

Alternatively, if your OpenClaw build reads MCP resources directly (via
`resources/list` + `resources/read` in-protocol), you do **not** need
to sync to disk at all — the skills are already reachable as
`skill://<source>/<package>/<version>/<filename>` URIs exposed by this
server.

---

## 3. MCP Inspector (Anthropic)

Useful for manual debugging: it shows you every tool, resource, and
prompt registered by the server, and lets you invoke them by hand.

```bash
npx @modelcontextprotocol/inspector \
    /abs/path/to/ability-mcp-server/.venv/bin/python \
    -m ability_mcp.native \
    --framework-url http://localhost:8080
```

Use it to verify:

- **Tools list** includes the 10 static tools + N dynamic
  `<ability>__<task>` tools from your framework manifest.
- **Resources list** includes `skill://_builtin/abilityframework-runtime/SKILL.md`
  plus one per package skill.
- `read_skill(uri="...")` returns the full markdown.

---

## 4. Generic / custom agent runner

Any program that can:

1. Spawn a child process with a pipe for stdin/stdout.
2. Speak MCP over JSON-RPC on those pipes (`initialize` → `tools/list`
   → `tools/call` → `resources/list` → `resources/read` → …).

…can drive this server. There's nothing client-specific in the
server's behaviour, and nothing that requires file-system side effects
unless the agent itself chooses to call `sync_skills_to_dir`. The
default skill cache lives at `~/.cache/ability-mcp/skills/` and is
readable by anyone; if your agent loader wants to watch a filesystem
directory, point it there (or at the directory you sync to) and you're
done.

**Tip**: if you're writing your own MCP client, call the bundled
`abilityframework-runtime` skill first (URI
`skill://_builtin/abilityframework-runtime/SKILL.md`) — it explains the
tool surface, the canonical 3-call workflow, and the recovery patterns.

---

## Checklist for any client

- [ ] Child process can find `python` and `ability_mcp.native` (use
      an absolute path to the venv-installed interpreter).
- [ ] `--framework-url` matches wherever the framework is actually
      listening. Default is `http://localhost:8080`.
- [ ] The framework is running **before** the client spawns the MCP
      server; otherwise the startup handshake logs a warning and the
      dynamic tool list comes up empty.
- [ ] If you want skill files on disk for a watch-based loader, call
      `sync_skills_to_dir(target=...)` once at session start. Otherwise
      use `resources/list` + `resources/read` in-protocol.
- [ ] After uploading / upgrading an ability package, call
      `refresh_tools()` — that re-pulls manifests **and** skills and
      re-publishes MCP resources.
