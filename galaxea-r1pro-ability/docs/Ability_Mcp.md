# MCP Playground — AbilityFramework 能力开发全流程教程

## Phase 1 — 通过 MCP + OpenClaw 用自然语言调用能力
本阶段接入 MCP Server，让 AI agent（如 OpenClaw / Claude Code）通过自然语言操控你的能力。

### 1.1 解压并安装 MCP Server
环境切换,切换到上述创建的conda环境

```bash
conda activate xx
```

验证安装：

```bash
ability-mcp-server/xxx/bin/python -m ability_mcp.native --help
```

### 1.2 在 OpenClaw 中注册 MCP Server

在你运行 OpenClaw 的工作目录下创建或编辑 `.mcp.json`：

```json
{
  "mcpServers": {
    "ability-framework": {
      "command": "/absolute/path/to/mcp-playground/ability-mcp-server/.venv/bin/python",
      "args": [
        "-m", "ability_mcp.native",
        "--framework-url", "http://localhost:8080"
      ]
    }
  }
}
```

> **注意**：`command` 必须是**绝对路径**，指向 MCP Server venv 里的 Python。  
> `--framework-url` 指向 Phase 4 启动的 AbilityFramework 地址。

Claude Code 用户也使用同样的 `.mcp.json` 格式，放在项目根目录或 `~/.claude.json` 中。

### 1.3 确保框架和能力就绪

```bash
# 确保 Phase 4 的框架还在跑 (如果已停，重新启动)
ability_tool framework activate
```

```bash
# 确认能力包已加载
ability_tool list
# 应该看到: mockarm-demo-1 -> MockArm.Demo
```

### 1.4 启动 OpenClaw 并用自然语言操控

在配置了 `.mcp.json` 的目录下启动 OpenClaw：

```bash
openclaw    # 或 claude (如果用 Claude Code)
```

MCP Server 会被 OpenClaw 自动拉起。此时 agent 具备以下 MCP 工具：


| 工具                                         | 作用                            |
| ------------------------------------------ | ----------------------------- |
| `refresh_tools()`                          | 重新从框架拉取 manifest，更新动态 task 工具 |
| `start_ability(template="mockarm-demo-1")` | 启动能力实例                        |
| `stop_ability(target="mockarm-demo-1")`    | 停止能力实例                        |
| `mock_arm_demo__move_to(x, y, z, speed)`   | 调用 MoveTo task                |
| `mock_arm_demo__get_pose()`                | 调用 GetPose task               |
| `list_skills()` / `read_skill(uri)`        | 查看/读取 Skill 文档                |


### 5.5 自然语言调用示例

在 OpenClaw / Claude Code 会话中输入：

```
请帮我把机械臂移动到 x=0.3, y=0.1, z=0.2 的位置，速度设为 0.5
```

Agent 会自动执行：

1. `start_ability(template="mockarm-demo-1")` — 启动能力实例
2. `mock_arm_demo__move_to(x=0.3, y=0.1, z=0.2, speed=0.5)` — 调用 MoveTo
3. 返回结果：`{success: true, error_distance: 0.002}`

继续对话：

```
现在告诉我机械臂在什么位置
```

Agent 执行：

- `mock_arm_demo__get_pose()` → `{x: 0.29, y: 0.10, z: 0.20}`

```
好了，关闭机械臂
```

Agent 执行：

- `stop_ability(target="mockarm-demo-1")` — 停止实例

### 5.6 Skill 文档如何生效

Phase 3 编写的 `skills/SKILL.md` 在包上传后被框架自动镜像到 `skills/_packages/` 目录。MCP Server 通过 `/api/skill` 接口读取，作为 agent 的先验知识：

- Agent 通过 `list_skills()` 发现可用 skill
- 通过 `read_skill(uri="skill://mock.arm.demo/1.0.0/SKILL.md")` 读取完整内容
- Skill 里的 "When to use" + "Canonical workflow" 指导 agent 选择正确的工具序列

这就是为什么 agent 知道要先 `start_ability` 再调用 task 最后 `stop_ability` —— 这个三步工作流写在 Skill 文档里。


