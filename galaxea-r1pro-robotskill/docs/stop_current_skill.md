# 停止当前占用机器人的 Skill

脚本：`start-shell/stop_current_skill.sh`。只有这一个运行文件，依赖 Bash 和 Python 3 标准库。

在运行 Pilot 的机器上执行。自动从本机 Pilot 获取 Server 地址、Pilot ID 和日志路径，再查询该机器人当前的执行 ID；不需要填写 Skill 名称。

先设置框架网页的登录 token，也可通过 `--token-file /path/to/token` 提供。SSH 密码、Git token 和 Pilot 接入 token 都不能替代网页登录 token。

```bash
export SEMANTIC_TOKEN='框架网页登录 token'
bash start-shell/stop_current_skill.sh --dry-run
bash start-shell/stop_current_skill.sh
```

脚本调用 `execution.stop`，由 Pilot 停止 Ability、调用 Skill 的停止入口并结束对应 Worker；随后等待停止证据和机器人当前执行指针清除。停止过程可能调用 Skill 的 hold/stop 动作。不要用它做无副作用的探测，探测使用 `--dry-run`。

遇到 `interrupted`，默认停止并报错，保留尚未确认的占用。操作员确认现场已停止、物理状态安全后，可以用下面这条命令重试停止并释放：

```bash
bash start-shell/stop_current_skill.sh --confirm-safe
```

`--confirm-safe` 表示调用者已经完成现场确认。脚本不通过杀死 Python 进程推断机器人已停止，也不自动提交虚假的现场确认。确认接口返回后，还会读取本次操作之后的 Pilot 日志，核实同一个执行 ID 的释放回执；不能只凭 HTTP 202 判定释放成功。

归属于 Workflow 的执行，需要显式添加 `--stop-workflow`，这会停止**整个所属 Workflow**，并检查该机器人的 Task 占用也已结束。默认拒绝这种扩大范围的停止，避免只结束一个 Skill 后调度器继续派发。

其他选项：

```bash
bash start-shell/stop_current_skill.sh --help
# 多个 Pilot 时明确选择一个
bash start-shell/stop_current_skill.sh --pilot-pid 12345
# 显式指定连接；确认中断占用时还需可读取的本机 Pilot 日志
bash start-shell/stop_current_skill.sh --server http://127.0.0.1:9090 --robot r1pro-real-1
```

每个等待阶段默认最多 30 秒，可用 `--timeout` 调整。超时、鉴权失败、停止证据不足、释放回执缺失、新执行已接管都会返回非零退出码，不继续清理新执行。不改数据库，不重启 Pilot，不按名称批量杀 Worker。仅支持 Server 管理的 Pilot 执行；独立的 `semantic-pilot skill run` 不属于该接口的管理范围。

本次验证只使用离线假 HTTP 服务，覆盖正常停止、中断重试、确认回执、竞争、超时、鉴权以及 Workflow 占用；没有在真机执行清理。
