# Solver 来源

两个 solver 来自仓库共享包 `src/solver/`，由 `package_skill.py` 打包到 Skill ZIP 根的 `solver/`：

- `src/solver/capture_point_solver/capture_point_solver.py` ← `semantic-ability/galaxea-r1pro-ability/src/galaxea_r1pro_0/solver/capture_point_solver/capture_point_solver.py`
- `src/solver/target_point_solver/target_point_solver.py` ← `semantic-ability/galaxea-r1pro-ability/src/galaxea_r1pro_0/solver/target_point_solver/target_point_solver.py`

快照日期：2026-09-30。ZIP 内包含 `solver/__init__.py`、`solver/Nav/navigation.py` 与两个子包，运行时无需访问源码仓库。
`scripts/solver.py` 负责参数适配、有限值/四元数校验与米单位输出，不改变原算法。

## 需要重新同步

`src/solver/` 在快照之后继续被修改过（2026-10-05 ~ 10-07），本 Skill 的 ZIP 里是旧快照：

- `move_box_to_center` 的终点公式在 `2d5831e`（2026-10-05）被改写为
  `x = 当前左末端x - width/4`、`z = 当前左末端z`，`end_pose_x_offset/end_pose_z_offset`
  不再被函数体引用（即输入的 `center_x_offset_m/center_z_offset_m` 目前无效）；
- `get_capture_point_place`、`move_to_retract_point` 在 `6d78537`（2026-10-07）有改动，
  属放置侧（`place_object_force`），本 Skill 不使用。

本 Skill 走到的 `get_capture_point` / `move_box_to_center` 在部署包与 `src/solver/`
之间**没有差异**，因此真机行为与源码一致；但下次打包前仍应重新同步，并同步更新
`SKILL.md` 中“上游接口的实际语义”一节。
