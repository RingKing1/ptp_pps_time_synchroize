#!/bin/bash

set -e

# =============================================================================
# ROS2 Bag 同步导出运行脚本（xterm 双终端版）
# 开启两个 xterm 终端：一个启动 sync_export 节点，一个延迟 10s 后播放 rosbag
# =============================================================================
#
# 用法:
#   ./run.sh <output_folder_name> <rosbag_path>
#
# 示例:
#   ./run.sh scene1 data/rosbag2_2026_04_27-16_44_35
#   ./run.sh scene2 data/rosbag2_2026_04_27-16_50_02
# =============================================================================

# 参数检查
if [ $# -lt 2 ]; then
    echo "Usage: $0 <output_folder_name> <rosbag_path>"
    echo ""
    echo "Arguments:"
    echo "  output_folder_name   导出数据保存的文件夹名称"
    echo "  rosbag_path          rosbag 文件夹路径"
    echo ""
    echo "Examples:"
    echo "  $0 scene1 data/rosbag2_2026_04_27-16_44_35"
    echo "  $0 scene2 data/rosbag2_2026_04_27-16_50_02"
    exit 1
fi

OUTPUT_DIR="$1"
BAG_PATH="$2"

# 获取脚本所在目录（项目根目录）
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# 检查 xterm 是否安装
if ! command -v xterm &> /dev/null; then
    echo "Error: xterm is not installed."
    echo "Please install it with: sudo apt install xterm"
    exit 1
fi

# source ROS2 环境
source /opt/ros/jazzy/setup.bash

# 可选：如果有虚拟环境，取消下面一行的注释
# source ~/ros_venv/bin/activate

# 检查工作空间是否已编译
if [ ! -f "${SCRIPT_DIR}/install/setup.bash" ]; then
    echo "Error: install/setup.bash not found."
    echo "Please build the project first with:"
    echo "  colcon build --packages-up-to sync_export"
    exit 1
fi

source "${SCRIPT_DIR}/install/setup.bash"

# 检查 rosbag 路径是否存在
if [ ! -d "${SCRIPT_DIR}/${BAG_PATH}" ] && [ ! -d "${BAG_PATH}" ]; then
    echo "Error: Rosbag path not found: ${BAG_PATH}"
    echo "Also tried: ${SCRIPT_DIR}/${BAG_PATH}"
    exit 1
fi

# 使用绝对路径
if [ -d "${SCRIPT_DIR}/${BAG_PATH}" ]; then
    BAG_ABS_PATH="${SCRIPT_DIR}/${BAG_PATH}"
else
    BAG_ABS_PATH="$(cd "$(dirname "$BAG_PATH")" && pwd)/$(basename "$BAG_PATH")"
fi

echo "=========================================="
echo "Terminal:          xterm"
echo "Output directory:  ${OUTPUT_DIR}"
echo "ROS bag path:      ${BAG_ABS_PATH}"
echo "=========================================="

# ---------------------------------------------------------------------------
# 终端1：启动 sync_export 导出节点
# ---------------------------------------------------------------------------
echo "[Terminal 1] Starting sync_export node..."
xterm -title "Sync Export" -fa "Monospace" -fs 10 \
    -e bash -c "
        echo '[Sync Export] Starting...'
        source /opt/ros/jazzy/setup.bash
        source '${SCRIPT_DIR}/install/setup.bash'
        ros2 launch sync_export sync_export.launch.py output_dir:=${OUTPUT_DIR}
        echo '[Sync Export] Press Enter to close...'
        read
    " &

# ---------------------------------------------------------------------------
# 等待 10 秒让 sync_export 节点完成初始化
# ---------------------------------------------------------------------------
echo "Waiting 10 seconds for sync_export node to initialize..."
sleep 10

# ---------------------------------------------------------------------------
# 终端2：播放 rosbag（0.4 倍速）
# ---------------------------------------------------------------------------
echo "[Terminal 2] Starting rosbag playback at 0.4x rate..."
xterm -title "Rosbag Play" -fa "Monospace" -fs 10 \
    -e bash -c "
        echo '[Rosbag Play] Starting playback at 0.4x rate...'
        source /opt/ros/jazzy/setup.bash
        source '${SCRIPT_DIR}/install/setup.bash'
        ros2 bag play --rate 0.4 '${BAG_ABS_PATH}'
        echo '[Rosbag Play] Playback finished. Press Enter to close...'
        read
    " &

echo ""
echo "Both xterm terminals launched."
echo ""
echo "终端说明:"
echo "  - 'Sync Export' 终端: sync_export 节点日志"
echo "  - 'Rosbag Play' 终端: rosbag 播放日志（延迟 10s 后启动）"
echo ""
echo "按 Enter 键退出此脚本（xterm 终端会继续运行）"
read
