# Robot Skill 导航 Solver

`solver.Nav.navigation` 是供 Robot Skill 直接使用的纯 Python 站位解算模块。它把地图系
箱子目标点转换成 `WooshNavigation.navigate_to_map_pose` 所需的底盘目标，不访问
Robot、ROS、网络或文件系统。

```python
from solver.Nav.navigation import solve_navigation_pose

station = solve_navigation_pose(
    [1.2, 0.8, 0.9, 1.5708],
    frame_id="map",
)
parameters = station.to_action_parameters(timeout=60.0)
```

输入支持 `[x,y,z]`、`[x,y,z,yaw]`、`[x,y,z,roll,pitch,yaw]`，也支持包含
`x/y/z/yaw_rad` 的字典。yaw 始终是输入坐标系中的绝对作业朝向，单位为弧度。

Solver 检查箱子中心是否对左右臂都名义可达，但不包含地图、障碍物或完整抓取 IK，
最终路线仍由导航 Ability 规划，抓取可达性仍由抓取 Skill 验证。

发布 Robot Skill 时，必须把整个 `solver` 包复制到 Skill ZIP 根目录（含 `Nav/`）；
只发布 `scripts/` 和 `SKILL.md` 会导致 `from solver.Nav.navigation ...` 导入失败。

目录说明：`Nav/navigation.py` 是本 solver 的生产实现；`Nav/Galaxea/packages/` 保留
历史的 NavSolver 能力描述（NavSolver.yaml、README.md），其中旧的 `Nav/Utils`、
`Nav/Galaxea/scripts` 规划代码已删除。URDF 资料位于 `src/assest/r1_pro_with_hand.urdf`。
