# Solver 来源

两个 solver 直接从根 `solver/` 包导入，不随 Skill 分发副本：

- `solver/capture_point_solver/capture_point_solver.py` ← `semantic-ability/galaxea-r1pro-ability/src/galaxea_r1pro_0/solver/capture_point_solver/capture_point_solver.py`
- `solver/target_point_solver/target_point_solver.py` ← `semantic-ability/galaxea-r1pro-ability/src/galaxea_r1pro_0/solver/target_point_solver/target_point_solver.py`

快照日期：2026-09-30。本 Skill 通过绝对导入 `from solver.capture_point_solver import ...` 访问根包，部署时须把仓库根加入 PYTHONPATH。`scripts/solver.py` 负责参数适配、有限值/四元数校验与米单位输出，不改变原算法。
