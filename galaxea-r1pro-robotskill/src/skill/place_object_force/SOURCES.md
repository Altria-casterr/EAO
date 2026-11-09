# 实现依据

- `/mnt/d/workspace/skill/skill_library/Behavioral/Put_Down/Agibot_G2/scripts/main.py`：
  偏移预定位、分侧撤手顺序；当前 Skill 按任务要求改为 X→Z 调整。0.4.0 按最终箱体目标计算 X/Z，
  不再使用原行为 Z 下移 0.02 m、X 前移 0.05 m 的固定距离。
- 0.4.5 按任务要求保持首次手臂运动前左右手各自的 Y：预定位和 X/Z 放置不再计算
  箱缘 Y 安全带，撤辅助手后才由主手沿 Y 力控推靠，默认触发阈值为 1.5 N。
- 0.4.6 收紧超时默认值：`y_force.timeout_s` 60→30 秒，预定位/位置/撤手动作 timeout 统一到 10 秒，
  使失败能更快暴露，而不是长时间占用前台物理锁。
- 同目录 `main_crrc.py`：撤辅助手后主手 Y 力反馈推靠的顺序；
  托举工具无需夹爪开闭。采用其几何撤手公式；Z/X 使用绝对位置目标。
- morefine（`172.16.70.143`）项目
  `/home/morefine/ws/SEMF/workspace/semantic-skill/galaxea-r1pro-robotskill`：
  `src/skill/grasp_object_bilateral` 的 prepare 躯干动作与随后读取变换的顺序、
  四关节姿态输入、独立 approach_motion、自动双手轨迹点数、
  逐轴顺序、Action 等待预算、显式坐标变换及 SkillContext 约定；
  `src/skill/grasp_object_unilateral` 的单手轨迹参数、显式点数回退、
  既有动作保存和旧版检查点升级边界。本次基于该机器当前工作区源码对齐。
- 同项目 `src/solver/target_point_solver`：现场标定的目标转换，
  以米制 `mm_scale=1.0` 使用，并随独立 ZIP 打包。
- 当前 `semantic-ability/galaxea-r1pro-ability/src/galaxea_r1pro_0/abilities/robot_observation`、
  `torso_control` 和 `joint_eef_pose_control` 的 manifest、utils：schema-v2 躯干控制、读状态、双手轨迹、
  双手点位、Y 单手小步和单手轨迹。离线契约测试直接对照该机器当前清单。
- `semantic-robotsdk/galaxea-r1pro-sdk/src`：读取 `force_data` 结构、双臂提交方式、
  坐标系和取消边界。Skill 无该 SDK 的运行时依赖。

阈值、传感器监测轴、最大行程、超时、工具中心偏移必须由现场标定提供。
本交付经过离线测试，未通过真机运动验证；未新增 Ability/Robot SDK 接口。
