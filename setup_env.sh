#!/bin/bash
# ================================================
# Sentinel 哨兵系统 — 环境配置脚本
# GOAI 具身未来赛道 | 产业园区全地形巡逻挑战赛
# 目标: Ubuntu 24.04 / ROS2 Jazzy
# ================================================

set -e

echo "=========================================="
echo " Sentinel 环境配置 (Ubuntu 24.04 / ROS2 Jazzy)"
echo "=========================================="

# ---- 1. ROS2 Jazzy 安装 (如未安装) ----
if ! command -v ros2 &> /dev/null; then
    echo "[1/6] 安装 ROS2 Jazzy..."
    sudo apt update
    sudo apt install -y curl
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update
    sudo apt install -y ros-jazzy-desktop python3-colcon-common-extensions python3-rosdep
    sudo rosdep init || true
    rosdep update
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
else
    echo "[1/6] ROS2 已安装: $(ros2 --version)"
fi

# ---- 2. 系统依赖 ----
echo "[2/6] 安装系统依赖..."
sudo apt update
sudo apt install -y \
    python3-pip python3-opencv python3-numpy \
    python3-venv python3-dev \
    ros-jazzy-cv-bridge ros-jazzy-vision-msgs \
    ros-jazzy-sensor-msgs ros-jazzy-visualization-msgs \
    ros-jazzy-std-srvs ros-jazzy-rviz2

# ---- 3. Python 依赖 ----
echo "[3/6] 安装 Python 依赖..."
pip install --upgrade pip

# ultralytics (YOLO)
pip install ultralytics>=8.0.0

# llama-cpp-python (VLM GGUF)
# 如需 GPU 加速: CMAKE_ARGS="-DGGML_CUDA=on" pip install llama-cpp-python
pip install llama-cpp-python>=0.2.0

# 基础依赖
pip install opencv-python>=4.5.0 numpy>=1.21.0 Pillow>=9.0.0

echo "[3/6] Python 依赖安装完成"

# ---- 4. 构建工作空间 ----
echo "[4/6] 构建 Sentinel 工作空间..."
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WS_DIR="$SCRIPT_DIR/sentinel_ws"

if [ -d "$WS_DIR" ]; then
    cd "$WS_DIR"
    # 安装 rosdep 依赖
    rosdep install --from-paths src -y --ignore-src 2>/dev/null || true
    # 构建
    colcon build --symlink-install
    echo "[4/6] 构建完成"
else
    echo "[4/6] 未找到 sentinel_ws 目录, 跳过构建"
fi

# ---- 5. 下载 YOLO 模型 ----
echo "[5/6] 检查 YOLO 模型..."
YOLO_MODEL="$HOME/.cache/torch/hub/ultralytics_yolov8_master/yolov8n.pt"
if [ ! -f "$YOLO_MODEL" ]; then
    echo "  首次运行 fast_detector 时会自动下载 yolov8n.pt"
fi

# ---- 6. 环境变量 ----
echo "[6/6] 写入环境变量到 ~/.bashrc..."
SOURCE_LINE="source $WS_DIR/install/setup.bash"
if ! grep -qF "$SOURCE_LINE" ~/.bashrc 2>/dev/null; then
    echo "$SOURCE_LINE  # Sentinel" >> ~/.bashrc
    echo "  已追加到 ~/.bashrc"
else
    echo "  已存在"
fi

echo ""
echo "=========================================="
echo " 环境配置完成!"
echo ""
echo " 启动方式:"
echo "   source install/setup.bash"
echo "   ros2 launch sentinel_bringup sentinel.launch.py"
echo ""
echo " 带 VLM 启动:"
echo "   ros2 launch sentinel_bringup sentinel.launch.py model_path:=/path/to/model.gguf"
echo ""
echo " 生成巡检报告:"
echo "   ros2 service call /report_generator/generate_report std_srvs/srv/Trigger"
echo ""
echo " 查看任务计划:"
echo "   ros2 service call /task_scheduler/get_next_plan std_srvs/srv/Trigger"
echo ""
echo " 可视化 (RViz):"
echo "   ros2 run rviz2 rviz2"
echo "   Add -> By topic -> /sentinel/viz/markers (MarkerArray)"
echo "=========================================="
