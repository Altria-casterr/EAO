# End-to-End Workflow: AbilityFramework + Native MCP

The fastest path from a blank shell to "LLM calls a task on the workspace
endpoint-control-1 instance".

## 1. Build and launch the framework

```bash
cd abilityframework/
xmake build
cp build/linux/x86_64/release/AbilityFramework ../workspace/AbilityFramework

cd ../workspace/
./AbilityFramework -c config.yaml
# framework listens on http://localhost:8080
```

Check:

```bash
curl -s http://localhost:8080/api/hello            # → "hello"
curl -s http://localhost:8080/api/manifest | jq .  # → [ {abilityName: EndpointControl.Robotics, ...} ]
curl -s http://localhost:8080/api/cr | jq .        # → [ {metadata.name: endpoint-control-1, ...} ]
```

The workspace currently ships with one CR template
(`workspace/crs/endpoint-control.cr.yaml`) and one package
(`workspace/packages/robotics.endpoint.control/1.0.0/`).

## 2. Install the MCP server

```bash
cd ability-mcp-server/
uv venv --python 3.12 .venv
uv pip install --python .venv/bin/python -e '.[test]'
.venv/bin/python -m pytest -q          # 15 passed
```

## 3. Wire into your MCP client

The server speaks standard MCP over stdio. Any client that can spawn a
child process and exchange JSON-RPC frames over its pipes can drive it.
The two clients exercised in this repo are **Claude Code** and
**OpenClaw**; both consume the same `.mcp.json` shape, and the same
spawn command works for the MCP Inspector and any custom agent runner
too. Full per-client wiring lives in `examples/clients.md`. For the
common case:

```json
{
  "mcpServers": {
    "ability-framework-native": {
      "command": "/abs/path/to/ability-mcp-server/.venv/bin/python",
      "args": [
        "-m", "ability_mcp.native",
        "--framework-url", "http://localhost:8080"
      ]
    }
  }
}
```

Drop that into the project root as `.mcp.json` (Claude Code, OpenClaw)
and re-launch your client. You should see `ability-framework-native`
listed with **10 static tools** plus **N dynamic tools** (one per
`(ability, task)` from the framework manifest — the workspace ships
`endpoint_control_robotics__*`, 5 of them).

## 4. Smoke test prompts

### Cold start

> List the abilities currently loaded into the framework.

Expected: the model calls `list_abilities`, returns the `EndpointControl.Robotics`
summary.

### Spawn and move

> Spawn an endpoint-control-1 instance and move the end effector to
> (0.3, 0.1, 0.2) with a neutral orientation.

Expected sequence:

1. `list_heartbeats` (no running instance)
2. `spawn_instance({template: "endpoint-control-1"})` → taskId
3. `list_heartbeats` (verify Running)
4. `endpoint_control_robotics__move_to_pose({position: {x:0.3,y:0.1,z:0.2}, orientation: {x:0,y:0,z:0,w:1}})`

### Graceful cleanup

> Terminate the running endpoint-control-1 instance.

Expected: the model finds the running `instance_id` from `list_heartbeats`
and calls `terminate_instance(instance_id=...)`.

## 5. Batch stress

Use `abilityframework/test/test-batch-instance-start.py` to fire off N
concurrent instance starts (with random jitter) for race-condition testing.
The MCP server doesn't need to be involved for this — it hits the framework
HTTP API directly.

```bash
python3 abilityframework/test/test-batch-instance-start.py --count 8
```

Every instance should reach Running within ~10s after the SDK and framework
fixes from the `wait_once(Standby)` rework.

## 6. Upgrade cycle (hot reload)

1. Modify `ability.manifest.yaml`, `crs/*.yaml`, or `skills/*.md` in a package
2. Repackage as zip
3. `POST /api/package` via webui's upload form (or curl)
4. Framework `update()` ticks within 5s; new manifests + mirrored CRs +
   mirrored skills loaded
5. From your MCP client (Claude Code, OpenClaw, or any other), call
   `refresh_tools` — this re-pulls manifests **and** the skill cache,
   re-registers dynamic tools and MCP resources, and returns the new
   `{abilities, dynamic_tools, skills}` counts.

`update_interval` in `workspace/config.yaml` controls how often step 4
fires even without an explicit upload.
