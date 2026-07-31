# Ubuntu 运行步骤

## 1. 安装依赖

```bash
sudo apt update
sudo apt install -y python3 python3-venv python3-pip python3-tk

python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

如果找不到 CoppeliaSim ZMQ 客户端，可补装：

```bash
pip install coppeliasim-zmqremoteapi-client
```

## 2. 设置路径

```bash
export COPPELIASIM_ROOT=/opt/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04
export CR5_SCENE_PATH=$PWD/scenes/compact_cell1ttt.ttt
```

如果使用 ROS2，请先执行：

```bash
source /opt/ros/humble/setup.bash
source ~/dobot_ws/install/setup.bash
```

## 3. 启动 CoppeliaSim 场景

```bash
bash scripts/start_coppelia_ubuntu.sh
```

## 4. 启动动态订单窗口

另开一个终端：

```bash
source .venv/bin/activate
bash scripts/run_dynamic_order_window_ubuntu.sh
```

## 5. 只预览调度结果

```bash
python scripts/run_coppelia_order_demo.py \
  --preview-only \
  --urgent-type C \
  --insert-time 20 \
  --speed 2
```

## 6. 运行测试

```bash
python -m pytest tests/test_scheduler_v2.py
```

当前预期：

```text
24 passed
```

