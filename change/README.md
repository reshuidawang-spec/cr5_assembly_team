# change 文件夹说明

本文件夹整理的是 GitHub 仓库 `qixunqiwo/cr5_assembly_team` 中 `2` 分支相对 `main` 分支多出的内容。

比较基准：

- `main`: `origin/main`，提交 `938f15958db7`
- `2`: `origin/2`，提交 `61dd4de9efa7`
- 比较方式：`git diff origin/main..origin/2`

整理结果：

- `files/`：从 `origin/2` 复制出的新增和修改文件，保留原始路径结构。
- `name-status.txt`：每个差异文件的 Git 状态清单。
- `diff-stat.txt`：差异统计。
- `branch-2-vs-main.patch`：`2` 分支相对 `main` 的文本 patch。二进制场景文件请以 `files/` 中的完整文件为准。

差异概要：

- 共整理 139 个差异文件，其中新增 113 个，修改 26 个。
- 主要新增五机械臂协同控制相关代码，包括 `robot_control/` 下的 R1-R5 执行器、五臂协调器、笛卡尔运行时和周期运行入口。
- 新增/更新场景与配置文件，包括五臂场景审计基线、障碍物配置、机器人点位配置和 CoppeliaSim 场景文件。
- 新增大量 RViz 轨迹采集、人工示教 waypoint、运行日志和五臂流程验证数据。
- 新增使用文档和接口说明，覆盖 R1/R2 协调、机器人控制用法、五臂协调器、R1-R5 任务执行器等内容。
- 新增调度器、仿真桥接、ROS2/MoveIt 配置相关改动，并补充了对应测试文件。

注意：

- 本地仓库当前存在一些未跟踪日志和 waypoint 文件；这些文件没有提交到 `origin/2` 或 `origin/main`，因此没有纳入本次整理。
- 本文件夹只做差异内容归档，不会把 `2` 分支代码直接覆盖到仓库根目录。
