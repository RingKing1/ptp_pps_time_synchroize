#!/bin/bash

set -e

# =============================================================================
# 时间同步测试脚本 - 运行 test_sync_node 输出每帧相机与雷达时间差
#
# 用法:
#   ./run_test_sync.sh <rosbag_path> [--rate 0.4]
#
# 示例:
#   ./run_test_sync.sh data/rosbag2_2026_04_27-16_44_35
#   ./run_test_sync.sh data/rosbag2_2026_04_27-16_44_35 --rate 1.0
# =============================================================================

if [ $# -lt 1 ]; then
    echo "Usage: $0 <rosbag_path> [--rate RATE]"
    echo ""
    echo "Examples:"
    echo "  $0 data/rosbag2_2026_04_27-16_44_35"
    echo "  $0 data/rosbag2_2026_04_27-16_44_35 --rate 1.0"
    exit 1
fi

BAG_PATH="$1"
shift

RATE="0.4"
while [[ $# -gt 0 ]]; do
    case "$1" in
        --rate)
            RATE="$2"
            shift 2
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

source /opt/ros/jazzy/setup.bash

if [ ! -f "${SCRIPT_DIR}/install/setup.bash" ]; then
    echo "Error: install/setup.bash not found. Please build first."
    exit 1
fi

source "${SCRIPT_DIR}/install/setup.bash"

# 检查 rosbag 路径
if [ -d "${SCRIPT_DIR}/${BAG_PATH}" ]; then
    BAG_ABS_PATH="${SCRIPT_DIR}/${BAG_PATH}"
elif [ -d "${BAG_PATH}" ]; then
    BAG_ABS_PATH="$(cd "$(dirname "$BAG_PATH")" && pwd)/$(basename "$BAG_PATH")"
else
    echo "Error: Rosbag path not found: ${BAG_PATH}"
    exit 1
fi

CAMERAS_CONFIG="${SCRIPT_DIR}/config/cameras.yaml"

echo "=========================================="
echo " Time Sync Test Node"
echo "=========================================="
echo "Cameras config:  ${CAMERAS_CONFIG}"
echo "ROS bag path:    ${BAG_ABS_PATH}"
echo "Playback rate:   ${RATE}"
echo "=========================================="

# 启动 test_sync_node
echo "[1/2] Starting test_sync_node..."
xterm -title "Test Sync Node" -fa "Monospace" -fs 10 \
    -e bash -c "
        echo '[Test Sync] Starting...'
        source /opt/ros/jazzy/setup.bash
        source '${SCRIPT_DIR}/install/setup.bash'
        ros2 run sync_export test_sync_node --ros-args \
            -p cameras_config:='${CAMERAS_CONFIG}'
    " &
SYNC_PID=$!

echo "Waiting 5 seconds for node to initialize..."
sleep 5

# 播放 rosbag
echo "[2/2] Starting rosbag playback at ${RATE}x rate..."
xterm -title "Rosbag Play" -fa "Monospace" -fs 10 \
    -e bash -c "
        echo '[Rosbag Play] Starting at ${RATE}x...'
        source /opt/ros/jazzy/setup.bash
        source '${SCRIPT_DIR}/install/setup.bash'
        ros2 bag play --rate ${RATE} '${BAG_ABS_PATH}'
    " &
BAG_PID=$!

# 双向检测：任一方先退出，等 2s 后关闭另一方
echo ""
echo "Monitoring: whichever finishes first will stop the other after 2s..."
while true; do
    if ! kill -0 $SYNC_PID 2>/dev/null; then
        echo "test_sync_node finished. Stopping bag play in 2s..."
        sleep 2
        kill $BAG_PID 2>/dev/null || true
        break
    fi
    if ! kill -0 $BAG_PID 2>/dev/null; then
        echo "Bag play finished. Stopping test_sync_node in 2s..."
        sleep 2
        kill $SYNC_PID 2>/dev/null || true
        break
    fi
    sleep 1
done

echo "Done."
