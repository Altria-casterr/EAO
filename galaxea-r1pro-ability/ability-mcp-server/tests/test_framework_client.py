"""FrameworkClient tests using respx mocking (no real framework needed)."""

from __future__ import annotations

import httpx
import pytest
import respx

from ability_mcp.core.framework_client import FrameworkClient, FrameworkError


@pytest.fixture
async def client():
    c = FrameworkClient("http://localhost:8080")
    yield c
    await c.aclose()


@respx.mock
async def test_hello(client):
    respx.get("http://localhost:8080/api/hello").mock(
        return_value=httpx.Response(200, text="Hello")
    )
    assert await client.hello() == "Hello"


@respx.mock
async def test_list_manifests(client):
    respx.get("http://localhost:8080/api/manifest").mock(
        return_value=httpx.Response(
            200,
            json=[
                {
                    "abilityName": "EndpointControl.Robotics",
                    "kind": "AtomAbility",
                    "package": "robotics.endpoint.control",
                    "version": "1.0.0",
                    "taskCount": 5,
                    "hasConfigSchema": True,
                }
            ],
        )
    )
    items = await client.list_manifests()
    assert len(items) == 1
    assert items[0].abilityName == "EndpointControl.Robotics"
    assert items[0].taskCount == 5


@respx.mock
async def test_spawn_and_terminate_instance(client):
    respx.post("http://localhost:8080/api/instance").mock(
        return_value=httpx.Response(200, json={"taskId": "task-abc", "template": "t1"})
    )
    r = await client.spawn_instance("t1", start=True, connect=True)
    assert r["taskId"] == "task-abc"

    respx.delete("http://localhost:8080/api/instance/iid-1").mock(
        return_value=httpx.Response(200, json={"status": "ok"})
    )
    r2 = await client.terminate_instance("iid-1")
    assert r2["status"] == "ok"


@respx.mock
async def test_post_api_cr_returns_410(client):
    respx.post("http://localhost:8080/api/cr").mock(
        return_value=httpx.Response(410, json={"error": "endpoint removed"})
    )
    # Client itself doesn't have a create_cr method anymore; make sure a
    # raw call still surfaces the 410 as FrameworkError.
    with pytest.raises(FrameworkError) as ei:
        await client._json("POST", "/api/cr", json={"kind": "AtomAbility"})
    assert ei.value.status_code == 410


@respx.mock
async def test_ability_post_proxy(client):
    respx.post("http://localhost:8080/api/ability/iid-42/api/task/start").mock(
        return_value=httpx.Response(200, json={"task_id": "task-xy"})
    )
    r = await client.ability_post("iid-42", "api/task/start", {"task_type": 0, "input": {}})
    assert r["task_id"] == "task-xy"


@respx.mock
async def test_list_heartbeats_and_pick_active(client):
    respx.get("http://localhost:8080/api/ability-heartbeat").mock(
        return_value=httpx.Response(
            200,
            json=[
                {
                    "id": "a",
                    "instanceName": "ec-1-a",
                    "abilityName": "EndpointControl.Robotics",
                    "version": "1.0.0",
                    "state": "Standby",
                    "IPCPort": 10001,
                    "abilityPort": 0,
                },
                {
                    "id": "b",
                    "instanceName": "ec-1-b",
                    "abilityName": "EndpointControl.Robotics",
                    "version": "1.0.0",
                    "state": "Running",
                    "IPCPort": 10002,
                    "abilityPort": 20002,
                },
            ],
        )
    )
    hbs = await client.list_heartbeats()
    picked = FrameworkClient.pick_active_instance(hbs, "EndpointControl.Robotics")
    assert picked.id == "b"
    assert picked.state == "Running"


@respx.mock
async def test_framework_unreachable_raises_error(client):
    respx.get("http://localhost:8080/api/hello").mock(
        side_effect=httpx.ConnectError("refused")
    )
    with pytest.raises(FrameworkError) as ei:
        await client.hello()
    assert "unreachable" in str(ei.value)
