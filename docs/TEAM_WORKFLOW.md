# 团队协作规范

## 1. 开发方式

本项目采用功能分支开发，不建议所有人直接提交到 `main`。

推荐分支：

```text
main                  稳定展示版本
feature/simulation    2号：仿真场景
feature/motion        3号：动作与路径
feature/scheduler     4号：调度算法
feature/dashboard     5号：数据看板
feature/docs          1号：报告与PPT
```

## 2. 提交流程

```bash
git checkout main
git pull

git checkout -b feature/scheduler
# 修改代码
git add .
git commit -m "feat: add dynamic scheduler prototype"
git push -u origin feature/scheduler
```

然后在 GitHub 上创建 Pull Request，由负责人检查后合并。

## 3. 提交信息

格式：

```text
<type>: <description>
```

类型：

| 类型 | 含义 |
|---|---|
| `feat` | 新功能 |
| `fix` | 修复问题 |
| `docs` | 文档修改 |
| `refactor` | 代码重构 |
| `test` | 测试或实验 |
| `chore` | 配置、依赖、整理 |

示例：

```text
feat: add order parser
feat: add region lock collision avoidance
fix: correct robot target point name
docs: update project plan
```

## 4. 每周集成检查

每周至少进行一次集成，检查以下内容：

- 是否能成功 `colcon build`；
- 是否能启动 RViz 或仿真场景；
- 调度算法是否能独立运行；
- 日志格式是否符合 `docs/INTERFACES.md`；
- README 是否需要同步更新。

## 5. 不要提交的内容

不要提交以下内容：

- `build/`；
- `install/`；
- `log/`；
- `__pycache__/`；
- `.pyc`；
- 临时视频、大体积实验数据；
- 个人账号、密钥、Token、局域网敏感配置。

## 6. 参赛提交前检查清单

- [ ] README 能让评委 3 分钟内看懂项目；
- [ ] 有一键运行或清晰运行说明；
- [ ] 有仿真演示视频；
- [ ] 有固定顺序调度和动态调度对比数据；
- [ ] 有系统架构图和算法流程图；
- [ ] 有技术方案报告和答辩 PPT；
- [ ] 代码目录清晰，无大量临时文件；
- [ ] 所有成员分工明确。
