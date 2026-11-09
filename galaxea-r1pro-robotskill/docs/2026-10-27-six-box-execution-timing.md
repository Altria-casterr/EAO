# Leader 规划模式六箱搬运执行耗时统计（2026-10-27）

本轮完成 6 个箱子、30 次 Skill 执行。执行阶段总耗时 **16 分 42.59 秒**，平均每箱 **2 分 47.10 秒**。模型调用占 **18.48%**，真机执行（扣除显式检查）占 **74.48%**，检查占 **2.48%**，其他开销占 **4.55%**。

机器人：`r1pro-real-1`。

Workflow：`wf-ca4729aa-5e8a-46ba-8ad1-59f7e47f8056`。

范围：2026-10-27 16:49:29.543 至 17:06:12.131（北京时间），c1–c6，30 次 Skill，均 completed。排除 c7/c8 和后续状态查询。

## 口径

- 总耗时：Workflow started_at 到 c6 ended_at。逐箱从上一箱完成开始计时，第一箱从 Workflow 开始计时。
- 模型：69 个外层 ChatModel span 的 duration_ms 总和；嵌套 OpenAI span 不重复累计。包含请求等待、推理和生成，不等同于可单独测量的纯思考。
- Skill：skill.started 到 execution.terminal；从中拆出 stage=verify 的 stage.running 到 stage.completed。
- 真机执行（不含 verify）：Skill 区间扣除 verify；含 Skill 内部求解、动作等待、反馈控制，不是纯电机运动时间。
- 检查：显式 verify 阶段，含抓取力检测、携物力检测、放置结果核对。动作内部的闭环检测仍在对应动作中。
- 其他：总耗时扣除模型和 Skill 区间；包括工具调用、启动、事件传播、状态确认及调度。30 次执行 terminal 到 SubTask completed 累计 1.66 秒，已计入其他；该区间不是独立埋点的复核计算时间。
- 未发现独立执行后模型复核调用。模型区间与 Skill 区间无重叠。毫秒精度，分项四舍五入可能存在尾差。

## 总计

| 类别 | 秒 | 占比 |
|---|---:|---:|
| 模型调用 | 185.25 | 18.48% |
| 真机执行（不含 verify） | 746.77 | 74.48% |
| 显式 verify 检查 | 24.91 | 2.48% |
| 其他开销 | 45.66 | 4.55% |
| 总计 | 1002.59 | 100% |

## 逐箱

| Task / 箱子 | 总秒数 | 模型 | 真机不含检查 | 检查 | 其他 |
|---|---:|---:|---:|---:|---:|
| c1 / gray_box-l5-1 | 152.34 | 28.54 | 111.90 | 4.45 | 7.45 |
| c2 / gray_box-l5-2 | 143.93 | 26.53 | 106.65 | 3.63 | 7.13 |
| c3 / gray_box-l4-1 | 181.52 | 31.95 | 137.12 | 5.14 | 7.32 |
| c4 / gray_box-l4-2 | 168.18 | 36.60 | 120.55 | 3.52 | 7.51 |
| c5 / gray_box-l3-1 | 183.47 | 33.95 | 136.90 | 4.82 | 7.81 |
| c6 / gray_box-l3-2 | 173.16 | 27.68 | 133.66 | 3.37 | 8.45 |

## 按 SubTask 类型累计

下表单位均为秒。Skill 总时间已包含检查，不能与检查列相加。

| 类型 | 模型秒数 | Skill 总秒数 | 其中 verify 秒数 |
|---|---:|---:|---:|
| 空手预备 | 22.33 | 26.71 | 0.00 |
| 接近导航 | 17.74 | 169.94 | 0.00 |
| 抓取 | 29.83 | 249.10 | 20.58 |
| 携物导航 | 19.10 | 213.38 | 1.00 |
| 放置 | 29.86 | 112.55 | 3.32 |

六次 Task 分解另计模型 66.385 秒，已包含在模型总计中。

## 执行前 Leader 规划

Leader run：`run-e7e69e44-283e-459d-a62f-12dedf2b58c5`。

16:48:18.531–16:49:19.715，共 61.18 秒，其中模型 61.09 秒。

Leader 完成到 Workflow 开始间隔 9.83 秒。

若从 Leader 开始计到第六箱结束，总计 **17 分 53.60 秒（1073.60 秒）**；模型 246.34 秒，占 22.94%。

## 30 次执行明细

| Task | 步 | Skill | execution_id | Skill秒 | verify秒 |
|---|---:|---|---|---:|---:|
| c1 | 1 | robot-prepare@0.1.2 | rex-cafc0e81-7baa-407d-8bc4-2388dccd5f4c | 0.34 | 0.00 |
| c1 | 2 | semantic-navigation-real@0.1.4 | rex-d4a529f9-3e4c-4278-9f26-cc214b2f01eb | 25.52 | 0.00 |
| c1 | 3 | grasp-object-unilateral@0.1.11 | rex-c2ea6bd7-f6d7-43d7-b1d1-c8fc5d3e2e2b | 46.50 | 3.72 |
| c1 | 4 | semantic-navigation-real@0.1.4 | rex-5dcde73a-394d-452f-a950-840063c44bdb | 24.11 | 0.18 |
| c1 | 5 | place-object-force@0.4.15 | rex-52902d30-9212-4218-b401-873bdf493e10 | 19.89 | 0.55 |
| c2 | 1 | robot-prepare@0.1.2 | rex-13e355c9-faed-4ec1-9145-485e30de4afe | 5.76 | 0.00 |
| c2 | 2 | semantic-navigation-real@0.1.4 | rex-37f87f59-50f5-43f8-94ea-3ed5732a3e24 | 26.36 | 0.00 |
| c2 | 3 | grasp-object-bilateral@0.2.7 | rex-5c1d21bc-02f0-4996-9053-2a41265982f7 | 28.65 | 2.90 |
| c2 | 4 | semantic-navigation-real@0.1.4 | rex-1c906533-c60f-47c8-a565-d453c7ede75e | 29.58 | 0.18 |
| c2 | 5 | place-object-force@0.4.15 | rex-e6341198-cd6c-499c-a5b7-65525fc583c2 | 19.92 | 0.55 |
| c3 | 1 | robot-prepare@0.1.2 | rex-f47afb3e-7621-4017-9964-c23d94ee821b | 6.02 | 0.00 |
| c3 | 2 | semantic-navigation-real@0.1.4 | rex-6c251f50-d010-45df-bae9-ddfc6635d377 | 30.43 | 0.00 |
| c3 | 3 | grasp-object-unilateral@0.1.11 | rex-5dc8847c-474f-4e5b-a2e7-457e55fb433c | 51.89 | 4.40 |
| c3 | 4 | semantic-navigation-real@0.1.4 | rex-db45b18d-f9b9-483b-b2b6-83a34b3b0fad | 34.70 | 0.18 |
| c3 | 5 | place-object-force@0.4.15 | rex-cde8ab13-c546-47bd-820d-a70ff49aa131 | 19.22 | 0.55 |
| c4 | 1 | robot-prepare@0.1.2 | rex-0701696d-23f7-437b-b13f-2a505ff3dd4d | 5.15 | 0.00 |
| c4 | 2 | semantic-navigation-real@0.1.4 | rex-d0f23122-2aca-4662-8d51-e8bb4d6e0a57 | 26.85 | 0.00 |
| c4 | 3 | grasp-object-bilateral@0.2.7 | rex-6115a5b0-8c68-4c2b-b1df-e88a9f829096 | 34.14 | 2.78 |
| c4 | 4 | semantic-navigation-real@0.1.4 | rex-4421012f-32be-4cc1-9dfe-7954e6e502d3 | 39.17 | 0.18 |
| c4 | 5 | place-object-force@0.4.15 | rex-5b36fcb1-62c6-43f6-b7e9-9db99b8903ae | 18.74 | 0.55 |
| c5 | 1 | robot-prepare@0.1.2 | rex-7f814c65-23a3-4a48-a2b3-d93367e0109c | 4.96 | 0.00 |
| c5 | 2 | semantic-navigation-real@0.1.4 | rex-853dca2f-aa8c-41c8-ac53-f849c329cdaf | 31.41 | 0.00 |
| c5 | 3 | grasp-object-unilateral@0.1.11 | rex-067c733e-8a7d-4505-b7be-6cda85502dcd | 52.52 | 4.09 |
| c5 | 4 | semantic-navigation-real@0.1.4 | rex-c51b486e-d86d-4c6a-80df-d283824d7149 | 35.22 | 0.18 |
| c5 | 5 | place-object-force@0.4.15 | rex-6eee91ea-9498-49ca-b1a3-826e73116971 | 17.61 | 0.55 |
| c6 | 1 | robot-prepare@0.1.2 | rex-2e11bea8-3045-47b2-9584-4d206953020d | 4.48 | 0.00 |
| c6 | 2 | semantic-navigation-real@0.1.4 | rex-e11cb12a-770e-4932-9fe3-7a5d8a6c4fe2 | 29.37 | 0.00 |
| c6 | 3 | grasp-object-bilateral@0.2.7 | rex-38a94976-82fb-470f-bdff-b17d6f2075c7 | 35.40 | 2.69 |
| c6 | 4 | semantic-navigation-real@0.1.4 | rex-2ac9ca5a-ecf8-4200-8aa2-6995f77dfa45 | 50.60 | 0.10 |
| c6 | 5 | place-object-force@0.4.15 | rex-64829619-de06-46d0-9c92-ea561061e7db | 17.17 | 0.57 |

## 数据来源与复核方法

数据来自 `semantic-framework/.output/data/semantic.db` 的只读查询，使用以下记录交叉核对：

- `workflows`、`tasks`、`subtasks`：执行范围、任务状态及完成时间。
- `run_sessions`、`trace_spans`：模型调用归属及耗时，只累计外层 `ChatModel` span。
- `robot_executions`、`robot_execution_events`：30 次执行的 Skill 版本、开始与结束事件，以及 `verify` 阶段边界。

执行阶段的毫秒级分项校验：`185248 + 746770 + 24909 + 45661 = 1002588 ms`。已核对 6 个 Task、30 次 Skill 全部完成，模型与 Skill 计时区间无重叠。本文保留各次 `execution_id`，用于回查原始记录。
