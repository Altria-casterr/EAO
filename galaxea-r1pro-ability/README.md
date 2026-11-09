# 从零开始，完成一个机械臂控制能力的**定义 → 生成 → 实现 → 打包 → 部署 → 自然语言调用**全流程。

## Step1、拉取当前能力仓库并切换分支
- 分支具体规范如下：
  - master-real (对外部暴露的真机能力，稳定可用)
  - master-simu (对外部暴露的仿真能力，稳定可用)
  - v3.0.0 (能力开发组内部开发使用，与技能库2.0对应，正在更新和维护, 当前的能力框架版本是最新版本，起名2.0的原因是为了和skill_library进行对应)
  - v3.0.1 (能力开发组内部开发使用，与技能库1.0对应，不在更新和维护)
  - dev-v1-20260405 (能力开发组内部开发使用，是之前的能力仓库版本，不在进行任何更新和维护)

```bash
git clone https://git.insightos.cn:40080/Liu.Xin/skill-Ability.git
git checkout v3.0.0
```
- 当前分支下项目结构如下
  - ability (能力源码目录-必须包含)
  - ability-mcp-server (mcp目录)
  - docs (相关文档说明-必须包含)
  - requirments (依赖库-必须包含)
  - AbilityFramework (能力仓库二进制代码-必须包含)

- ！！！缺少上述任何文件请与能力开发组人员联系

## Step2、安装依赖项

### 硬件环境
Linux x86_64（Ubuntu 20.04+） 

### python环境安装
```bash
# 切换到相关环境路径
cd requirments

# 使用Conda创建环境
conda create -n xxx python=3.12

# 启用Conda环境
conda activate xxx

# 安装python环境依赖
pip install ability_py-0.4.0-py3-none-any.whl
pip install ability_scaffold-1.2.0-py3-none-any.whl
pip install -r requirements.txt
```

### 能力创建快捷工具安装

```bash
# 切换到相关环境路径
cd requirments

chmod +x openapi-tool
```

### 能力框架快捷使用工具安装

```bash
# 切换到相关环境路径
cd requirments

chmod +x ability_tool
sudo cp ability_tool /usr/local/bin/
```

## Step3、编写新能力
- 新能力（abilities/xxx）创建具体流程请参考[Ability创建流程](docs/Ability_Create.md)

## Step4、能力调用
- 能力调用请参考[Ability能力调用](docs/Galaxea_Ability_Test.md)

## Step5（可选）、MCP调用
- 能力调用请参考[MCP能力调用](docs/Ability_Mcp.md)

# 开发过程中遇到任何问题请联系能力开发组人员进行修改



