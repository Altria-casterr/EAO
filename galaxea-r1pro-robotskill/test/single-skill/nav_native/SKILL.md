---
name: nav-native
description: 验证新栈导航链路的两段 Skill（navigate_to_map_pose 地图导航 → verify_arrival 复核，仅 navigation 角色）
category: robot_skill
version: 0.1.0
runtime:
  api_version: 1
  python: ">=3.11"
  entrypoint: scripts.skill:run
  stop_entrypoint: scripts.skill:on_stop
  input_model: scripts.models:NavNativeInput
  state_model: scripts.models:NavNativeState
  result_model: scripts.models:NavNativeResult
  controllers: {}
required_actions:
  - { type: navigation.follow_route, schema_version: 2 }
  - { type: navigation.verify_arrival, schema_version: 2 }
stop_actions:
  - { type: navigation.stop, schema_version: 2 }
debug_input:
  navigate:
    target_x: 0.0
    target_y: 0.0
    target_orientation: [0.0, 0.0, 0.0, 1.0]
    target_point_type: 0
    frame_id: map
    timeout: 60.0
  verify:
    tool_refs: []
    carrying_force_axis: z
    carrying_min_load_n: 3.0
---

# 导航链路验证 Skill（nav-native）

用两段 Stage 验证「skill → pilot → WooshNavigation.Demo → SDK → 真机」这条导航链路打通。

## Action 与 ability task 的对应

| Stage | actionType@schemaVersion | ability taskName | 说明 |
|---|---|---|---|
| navigate | `navigation.follow_route@2` | `navigate_to_map_pose` | 导航到 frame_id 坐标系下的 (target_x, target_y)，朝向取 target_orientation 的 yaw |
| verify | `navigation.verify_arrival@2` | `verify_arrival` | 读腕部力传感，判断手上是否仍有搬运物体 |
| （停止） | `navigation.stop@2` | `stop` | 零速发布 + 急停置位 |

参数是**扁平原生格式**，逐字段与 ability manifest 对齐：
`target_x` / `target_y` / `target_orientation` / `target_point_type` / `frame_id` / `timeout`；
复核段为 `tool_refs` / `carrying_force_axis` / `carrying_min_load_n`。

## 三个必须知道的点

1. **`verify_arrival` 名不副实**：当前 ability 实现的是**腕力持物检查**，不做位姿/距离到位校验。
   空手跑纯导航测试时 `holding_object=false`、`verify_verdict=not_achieved` 都是正常观测，
   Skill 不会因此报失败（终态仍 completed）。要真正校验到位，需要能力侧新增到位判定 task。
2. **导航是同步阻塞的**：`navigate_to_map_pose` 在 ability 内部阻塞等待导航完成（解锁刹车 →
   发 goal → 轮询 `is_navigating`），期间 `GetExecution` / `StopExecution` 拿不到执行记录。
   真正的超时由入参 `timeout`（默认 60s）决定，Action 信封的 timeout 只是留了 10s 余量。
3. **`navigation.stop` 有闩锁**：调用后 ability 实例的急停标志永久置位，后续所有导航立即被取消，
   必须重启 WooshNavigation 实例。因此 `on_stop` 只用于必须立刻停住的场景；
   常规 action 停止走 pilot 的 StopExecution（取消 action goal），不触发闩锁。

## 角色与部署

`WooshNavigation.Demo` 声明 `abilityRole: navigation`，`WooshMove.Demo` 声明 `abilityRole: navigation_move`
（两者都有 `stop`，但角色已拆开，不再互斥，可在同一 Deployment 里同时绑定，例如
`navigation: {}` + `navigation_move: {}`）。同一角色存在多个健康实例时 pilot 拒绝路由，
可用 `navigation: { instance_id: "<实例 UUID>" }` 精确指定。

## 本地运行

```bash
cd /home/sam/workspace/semantic/semantic-skill/real_galaxea_robotskill
./start-shell/run_skill.sh nav-native@0.1.0
```

输入取 `inputs/nav_native.json`。**跑之前必须按现场地图把 target_x / target_y / frame_id
改成真实可达目标**（默认 (0,0) 只是占位，等价于导航到地图原点做冒烟测试）。
