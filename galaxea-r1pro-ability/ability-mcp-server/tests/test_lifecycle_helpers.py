"""Tests for FrameworkClient.ensure_ability_ready / stop_ability.

These exercise the "start → wait ready → use → stop" orchestration layer
without touching a real framework; all HTTP routes are respx-mocked.
"""

from __future__ import annotations

import httpx
import pytest
import respx

from ability_mcp.core.framework_client import FrameworkClient, FrameworkError


BASE = "http://localhost:8080"

TEMPLATE_CR = {
    "metadata": {"name": "endpoint-control-1"},
    "spec": {
        "abilityName": "EndpointControl.Robotics",
        "package": "robotics.endpoint.control",
        "version": "1.0.0",
    },
    "kind": "AtomAbility",
}


def _hb(
    id: str = "inst-1",
    state: str = "Running",
    ability_port: int = 50501,
    ability_name: str = "EndpointControl.Robotics",
    version: str = "1.0.0",
) -> dict:
    return {
        "id": id,
        "instanceName": f"endpoint-control-1-{id}",
        "abilityName": ability_name,
        "version": version,
        "state": state,
        "IPCPort": 40000,
        "abilityPort": ability_port,
    }


def _instance_row(id: str, state: str = "Running", ability_name: str = "EndpointControl.Robotics") -> dict:
    return {
        "instance_id": id,
        "cr_id": "cr-1",
        "cr_name": "endpoint-control-1",
        "instance_name": f"endpoint-control-1-{id}",
        "ability_name": ability_name,
        "ability_version": "1.0.0",
        "state": state,
        "start_time": 0,
        "stop_time": 0,
        "detail": None,
        "spec_snapshot": None,
    }


@pytest.fixture
async def client():
    c = FrameworkClient(BASE)
    yield c
    await c.aclose()


# -----------------------------------------------------------------------------
# ensure_ability_ready
# -----------------------------------------------------------------------------


@respx.mock
async def test_ensure_ability_ready_reuses_existing(client):
    # No template resolution needed when a Running instance already exists,
    # but start_ability still reads /api/cr to get the ability_name. Mock both.
    respx.get(f"{BASE}/api/cr").mock(return_value=httpx.Response(200, json=[TEMPLATE_CR]))
    respx.get(f"{BASE}/api/ability-heartbeat").mock(
        return_value=httpx.Response(200, json=[_hb(id="already-running")])
    )

    info = await client.ensure_ability_ready("endpoint-control-1", timeout=2.0)
    assert info["instance_id"] == "already-running"
    assert info["reused"] is True
    assert info["state"] == "Running"
    assert info["ability_name"] == "EndpointControl.Robotics"


@respx.mock
async def test_ensure_ability_ready_spawns_then_waits_until_running(client):
    respx.get(f"{BASE}/api/cr").mock(return_value=httpx.Response(200, json=[TEMPLATE_CR]))

    # First heartbeat probe: the instance is still in Standby (ability_port=0)
    # Second heartbeat probe: Running with abilityPort > 0.
    calls = {"n": 0}

    def heartbeat_handler(request):
        calls["n"] += 1
        if calls["n"] == 1:
            # first call inside reuse_existing check — nothing running yet
            return httpx.Response(200, json=[])
        if calls["n"] == 2:
            return httpx.Response(200, json=[_hb(id="new-1", state="Standby", ability_port=0)])
        return httpx.Response(200, json=[_hb(id="new-1", state="Running", ability_port=50501)])

    respx.get(f"{BASE}/api/ability-heartbeat").mock(side_effect=heartbeat_handler)
    respx.post(f"{BASE}/api/instance").mock(
        return_value=httpx.Response(200, json={"taskId": "task-xyz"})
    )

    info = await client.ensure_ability_ready(
        "endpoint-control-1", timeout=5.0, poll_interval=0.01
    )
    assert info["instance_id"] == "new-1"
    assert info["reused"] is False
    assert info["ability_port"] == 50501
    assert info["state"] == "Running"
    assert calls["n"] >= 3  # reuse check + at least 2 polls


@respx.mock
async def test_ensure_ability_ready_times_out(client):
    respx.get(f"{BASE}/api/cr").mock(return_value=httpx.Response(200, json=[TEMPLATE_CR]))
    # Always return Standby with ability_port=0 — never becomes ready
    respx.get(f"{BASE}/api/ability-heartbeat").mock(
        return_value=httpx.Response(
            200, json=[_hb(id="stuck", state="Standby", ability_port=0)]
        )
    )
    respx.post(f"{BASE}/api/instance").mock(
        return_value=httpx.Response(200, json={"taskId": "task-xyz"})
    )

    with pytest.raises(FrameworkError) as ei:
        await client.ensure_ability_ready(
            "endpoint-control-1", timeout=0.2, poll_interval=0.05
        )
    assert "not ready" in str(ei.value)
    assert "Standby" in str(ei.value)


@respx.mock
async def test_ensure_ability_ready_unknown_template_raises(client):
    respx.get(f"{BASE}/api/cr").mock(return_value=httpx.Response(200, json=[]))

    with pytest.raises(FrameworkError) as ei:
        await client.ensure_ability_ready("no-such-template", timeout=0.2)
    assert "template not found" in str(ei.value)


# -----------------------------------------------------------------------------
# stop_ability
# -----------------------------------------------------------------------------


@respx.mock
async def test_stop_ability_by_template_name_stops_all_matching(client):
    respx.get(f"{BASE}/api/cr").mock(return_value=httpx.Response(200, json=[TEMPLATE_CR]))
    respx.get(f"{BASE}/api/instance").mock(
        return_value=httpx.Response(
            200,
            json=[
                _instance_row("inst-a", state="Running"),
                _instance_row("inst-b", state="Standby"),
                _instance_row("inst-c", state="Terminated"),  # already dead, skipped
                _instance_row("inst-d", state="Running", ability_name="Other.Ability"),
            ],
        )
    )
    # DELETE /api/instance/:id for each of the two active matches
    respx.delete(f"{BASE}/api/instance/inst-a").mock(
        return_value=httpx.Response(200, json={"status": "ok"})
    )
    respx.delete(f"{BASE}/api/instance/inst-b").mock(
        return_value=httpx.Response(200, json={"status": "ok"})
    )
    # Post-delete poll: both return 404 immediately
    respx.get(f"{BASE}/api/instance/inst-a").mock(return_value=httpx.Response(404))
    respx.get(f"{BASE}/api/instance/inst-b").mock(return_value=httpx.Response(404))

    result = await client.stop_ability("endpoint-control-1", timeout=0.5, poll_interval=0.05)
    assert sorted(result["stopped"]) == ["inst-a", "inst-b"]
    assert result["count"] == 2


@respx.mock
async def test_stop_ability_by_instance_id(client):
    uuid_42 = "00000000-0000-0000-0000-000000000042"
    # First GET /api/instance/:id returns the row; subsequent GETs return 404
    # (simulating the row being removed after DELETE).
    get_calls = {"n": 0}

    def get_handler(request):
        get_calls["n"] += 1
        if get_calls["n"] == 1:
            return httpx.Response(200, json=_instance_row(uuid_42, state="Running"))
        return httpx.Response(404)

    respx.get(f"{BASE}/api/instance/{uuid_42}").mock(side_effect=get_handler)
    respx.delete(f"{BASE}/api/instance/{uuid_42}").mock(
        return_value=httpx.Response(200, json={"status": "ok"})
    )

    result = await client.stop_ability(uuid_42, timeout=0.5, poll_interval=0.05)
    assert result["stopped"] == [uuid_42]
    assert result["count"] == 1


@respx.mock
async def test_stop_ability_no_active_instances_noop(client):
    respx.get(f"{BASE}/api/cr").mock(return_value=httpx.Response(200, json=[TEMPLATE_CR]))
    respx.get(f"{BASE}/api/instance").mock(return_value=httpx.Response(200, json=[]))

    result = await client.stop_ability("endpoint-control-1")
    assert result["stopped"] == []
    assert result["count"] == 0
