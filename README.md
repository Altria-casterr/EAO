# EAO

RobotSkill evaluation and optimization

本仓库包含三个项目的快照，分别放在三个文件夹里。每个文件夹就是对应项目的根目录。

| 目录 | 对应项目 | 内容 |
|------|----------|------|
| [`galaxea-r1pro-ability/`](/Altria-casterr/EAO/tree/main/galaxea-r1pro-ability) | `galaxea-r1pro-ability` | AbilityFramework、ability-mcp-server、docs、requirments 等（131 个文件） |
| [`galaxea-r1pro-robotskill/`](/Altria-casterr/EAO/tree/main/galaxea-r1pro-robotskill) | `galaxea-r1pro-robotskill` | RobotSkill 技能实现、测试、运行脚本（305 个文件） |
| [`galaxea-r1pro-sdk/`](/Altria-casterr/EAO/tree/main/galaxea-r1pro-sdk) | `galaxea-r1pro-sdk` | R1 Pro SDK 源码、示例、文档（53 个文件，含 Git LFS 模型） |

## 克隆

```bash
git lfs install
git clone https://github.com/Altria-casterr/EAO.git
```

## 目录结构

```text
EAO/
├── README.md
├── galaxea-r1pro-ability/
├── galaxea-r1pro-robotskill/
└── galaxea-r1pro-sdk/
```

## Git LFS

`galaxea-r1pro-sdk/src/galaxea_r1pro/galaxea_r1pro_0/utils/` 下的两个模型权重（`*.pt`，合计约 180 MB）通过 Git LFS 存储。克隆或拉取前请先执行：

```bash
git lfs install
```

否则这两个文件只会是 LFS 指针文本，不是可用的模型。

## 更新

```bash
git add -A
git commit -m "描述这次改动"
git push origin main
```
