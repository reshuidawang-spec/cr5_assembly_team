# CR5 + RMP60 测头 — 调试会话记录

> 日期：2026-05-16
> 项目路径：`/home/zhu/dobot_ws/src/cr5_rmp60_measurement/`

## 项目背景
CR5 机械臂法兰末端通过自制夹具搭载 Renishaw RMP60 无线电测头，接收器信号线已接入 CR5 控制柜 **DI1** 端口。

## 会话目标
验证 ROS2 侧能否读取 RMP60 测头的 DI1 触发信号。

## 技术栈
| 项目 | 版本/说明 |
|------|-----------|
| 机械臂 | Dobot CR5 |
| ROS2 | Humble |
| 控制柜 IP | 192.168.5.1 |
| 测头 | Renishaw RMP60 |
| 信号端口 | DI1 |
| RMP60 接收器 → 控制柜 | DI1（触发信号），DO1（测头开启） |
| 现有 ROS2 驱动 | `dobot_bringup_v4` → 编译为 `cr_robot_ros2` |

## 关键文件
| 文件 | 说明 |
|------|------|
| `scripts/probe_di_monitor.py` | DI1 FeedInfo 实时监控节点 |
| `scripts/direct_di_monitor.py` | 直连 29999/30004 的 DI1 监控 |
| `scripts/raw_feed_dump.py` | Port 30004 原始二进制 dump |
| `scripts/probe_touch.py` | 竖直测针触碰流程（默认 dry-run） |
| `PROJECT_MEMORY.md` | 项目状态持续更新文档 |

## 最新结论（2026-05-16 09:55）

问题已定位为连接 IP 错误。DobotStudio Pro 4.6 页面显示当前控制柜 IP 为 `192.168.5.1`，原调试记录使用的 `192.168.1.6` 不是当前 CR5 控制器地址。

已验证：
- 关闭/断开 DobotStudio 后，RMP60 接收器仍能正常触发，DI1 硬件链路正常。
- `ping 192.168.5.1` 正常。
- `printf 'RobotMode()\n' | nc 192.168.5.1 29999` 返回 `0,{5},RobotMode();`。
- `printf 'DI(1)\n' | nc 192.168.5.1 29999` 返回 `0,{0},DI(1);` 或触发时对应状态。
- 直连 `192.168.5.1:30004` 可收到完整 `1440` 字节 feedback 帧，`len_field=1440`，`robot_mode=5`。
- ROS2 驱动使用 `IP_address=192.168.5.1` 启动后，`/dobot_bringup_ros2/srv/DI` 可正常返回，`/dobot_bringup_ros2/msg/FeedInfo` 可正常发布。

正确启动方式：
```bash
source /opt/ros/humble/setup.bash
source ~/dobot_ws/install/setup.bash
export IP_address=192.168.5.1 DOBOT_TYPE=cr5
ros2 launch cr_robot_ros2 dobot_bringup_ros2.launch.py
```

直连 DI1 监控：
```bash
./scripts/direct_di_monitor.py --ip 192.168.5.1
```

## 调试过程

### 1. 环境确认
- ROS2 工作空间：`/home/zhu/dobot_ws/`
- 驱动包：`DOBOT_6Axis_ROS2_V4/dobot_bringup_v4`（package.xml 内名为 `cr_robot_ros2`）
- DOBOT 驱动已有完整 I/O 接口（DI/DIGroup/ToolDI/MovLIO/MovJIO）

### 2. 启动驱动
```bash
source /opt/ros/humble/setup.bash
source ~/dobot_ws/install/setup.bash
export IP_address=192.168.1.6 DOBOT_TYPE=cr5
ros2 launch cr_robot_ros2 dobot_bringup_ros2.launch.py
```
- 连接成功：`192.168.1.6:30004`（反馈）和 `192.168.1.6:29999`（命令）
- 但频繁出现 `tcpDoCmd failed`、`tcp recv error`

### 3. DI Service 调用测试
```bash
ros2 service call /dobot_bringup_ros2/srv/DI dobot_msgs_v4/srv/DI "{index: 1}"
```
返回 `res=0`，但日志显示 `tcpDoCmd failed` — 命令实际执行失败。

### 4. FeedInfo 数据分析
Port 30004 的实时反馈数据中所有字段均为 **0**：
- `digital_input_bits: 0`
- `q_actual: [0,0,0,0,0,0]`（关节角度全零）
- `EnableStatus: 0`
- `len: 0`

ROS2 驱动代码（`command.cpp`）中有关键校验：`if (real_time_data_->len != 1440) continue;` — len=0 导致所有数据被丢弃。

### 5. 原始 TCP 直连测试
用 Python socket 直连 port 30004：
```python
s.connect(("192.168.1.6", 30004))
# 收到 0 字节
```
Port 30004 **完全没有推送数据**。

用 netcat 测试 port 29999：
```bash
printf 'DI(1)\n' | nc 192.168.1.6 29999
# 无任何响应
```

### 6. 官方 Python Demo 测试
使用 Dobot 官方 [TCP-IP-Python-V4](https://github.com/Dobot-Arm/TCP-IP-Python-V4) 测试：
```python
dashboard = DobotApiDashboard("192.168.1.6", 29999)
dashboard.EnableRobot()  # 返回 b'' （空）
dashboard.RobotMode()    # 返回 b'' （空）
dashboard.DI(1)          # Broken pipe （断连）
```
官方 Demo 同样失败。

## 当前结论

**旧结论已推翻：CR5 控制器 TCP 模式可用，异常原因是连接了错误 IP。**

可能原因：
1. **DobotStudio 端口独占** — DobotStudio 已占用 29999/30004，导致外部客户端连上但收到空数据
2. **固件版本过旧** — 标准 CR TCP/IP 协议需要固件 v3.5.1.19+
3. **TCP 模式类型不匹配** — 控制器可能不在正确的协议模式下

### 待验证
- 关闭 Windows 端 DobotStudio 后，用官方 Python Demo 重连测试
- 确认控制器固件版本
- 确认"TCP 模式"的具体协议类型

## ROS2 驱动通信架构（供参考）

```
CR5 控制柜 (192.168.5.1)
├── Port 29999 (Dashboard): 命令端口
│   - EnableRobot(), RobotMode()
│   - DI(index), DO(index,status)
│   - GetPose(), GetAngle()
│   - MovJ(), MovL(), MovLIO(), MovJIO()
├── Port 30004 (Feedback): 8ms 推送
│   - digital_input_bits (offset 8-15)
│   - q_actual[6] (offset 432-479)
│   - tool_vector_actual[6] (offset 624-671)
│   - robot_mode, controller_timer 等
└── Port 30003 (Motion): 备选命令端口

DOBOT ROS2 驱动 (cr_robot_ros2)
├── CRCommanderRos2
│   ├── dash_board_tcp_ → Port 29999
│   └── real_time_tcp_ → Port 30004
├── CRRobotRos2 Node
│   ├── Service: ~/srv/DI (→ port 29999)
│   └── Topic: ~/msg/FeedInfo (← port 30004, 100Hz)
└── 关键校验: command.cpp:47 — len != 1440 则丢弃数据
```

## 下一步计划
- [x] 关闭 DobotStudio，排除端口独占
- [x] 验证真实 TCP 协议模式
- [x] 确认 DI1 信号可读取
- [x] 编写竖直测针触碰流程
- [ ] TCP 标定（测头几何参数）
- [ ] 真机测量流程闭环

## 安全注意
- 当前触碰停止链路为 ROS2/Python 软件链路：`30004 FeedInfo` → DI1 判断 → `29999 Stop()`，不是硬实时安全回路。
- 为保护 RMP60，真实接触前应使用 `SpeedFactor=1`、短下探距离、足够回退空间，并先用手动触发测头验证 Stop 响应。
- `probe_touch.py` 默认 dry-run；真实运动必须加 `--execute`。
- 脚本默认下探 `3 mm`，超过 `5 mm` 必须显式添加 `--allow-long-approach`。
