# EAO

RobotSkill evaluation and optimization

本仓库把三个独立项目的快照放在三个分支里，**每个分支的根目录就是项目根目录**（没有外层文件夹）。`main` 只作为导航页，不存放代码；三个分支是互不相关的独立历史（orphan branch），不要互相合并。

| 分支 | 对应项目 | 内容 |
|------|----------|------|
| [`ability`](/Altria-casterr/EAO/tree/ability) | `galaxea-r1pro-ability` | AbilityFramework、ability-mcp-server、docs、requirments 等（131 个文件） |
| [`robotskill`](/Altria-casterr/EAO/tree/robotskill) | `galaxea-r1pro-robotskill` | RobotSkill 技能实现、测试、运行脚本（305 个文件） |
| [`sdk`](/Altria-casterr/EAO/tree/sdk) | `galaxea-r1pro-sdk` | R1 Pro SDK 源码、示例、文档（53 个文件，含 Git LFS 模型） |

## 克隆

只取其中一个分支（推荐，避免下载无关项目）：

```bash
# ability
git clone --branch ability --single-branch https://github.com/Altria-casterr/EAO.git

# robotskill
git clone --branch robotskill --single-branch https://github.com/Altria-casterr/EAO.git

# sdk
git clone --branch sdk --single-branch https://github.com/Altria-casterr/EAO.git
```

需要同时查看多个分支时，先完整克隆，再切换：

```bash
git clone https://github.com/Altria-casterr/EAO.git
cd EAO
git branch -a
git switch ability
```

## Git LFS

`sdk` 分支里的 `src/galaxea_r1pro/galaxea_r1pro_0/utils/*.pt` 模型权重通过 Git LFS 存储（合计约 180 MB）。克隆或拉取前先安装并初始化：

```bash
git lfs install
```

否则这两个文件只会是指针文本，不是可用的模型。`ability` 和 `robotskill` 不含 LFS 文件。

## 更新某个分支

```bash
git switch ability
# 修改文件后
git add -A
git commit -m "描述这次改动"
git push origin ability
```
