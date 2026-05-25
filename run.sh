#!/bin/bash

set -e

# =============================================================================
# ROS2 Bag 同步导出运行脚本（xterm 双终端版）
# 顺序：
#   1) 调用 convert_calibration.py 生成 <OUTPUT_DIR>/calib/...
#   2) 启动 sync_export 节点（读 calib_dir 加载内参并去畸变）
#   3) 延迟 10s 后播放 rosbag
#   4) 双向检测：sync_export 先退出或 bag 先播完，都等 2s 后关闭另一方
# =============================================================================
#
# 用法:
#   ./run.sh <output_folder_name> <rosbag_path> [--max-sync-frames N] [--no-undistort]
#
# 示例:
#   ./run.sh scene1 data/rosbag2_2026_04_27-16_44_35
#   ./run.sh scene1 data/rosbag2_2026_04_27-16_44_35 --max-sync-frames 1000
#   ./run.sh scene1 data/rosbag2_2026_04_27-16_44_35 --no-undistort
# =============================================================================

if [ $# -lt 2 ]; then
    echo "Usage: $0 <output_folder_name> <rosbag_path> [--max-sync-frames N] [--no-undistort]"
    echo ""
    echo "Arguments:"
    echo "  output_folder_name   导出 sample 文件夹（zip 时即样本名）"
    echo "  rosbag_path          rosbag 文件夹路径"
    echo "  --max-sync-frames N  最大同步帧数（默认 0 = 无限制）"
    echo "  --no-undistort       关闭相机图像去畸变（默认开启）"
    echo ""
    echo "Examples:"
    echo "  $0 scene1 data/rosbag2_2026_04_27-16_44_35"
    echo "  $0 scene1 data/rosbag2_2026_04_27-16_44_35 --max-sync-frames 1000"
    echo "  $0 scene1 data/rosbag2_2026_04_27-16_44_35 --no-undistort"
    exit 1
fi

OUTPUT_DIR="$1"
BAG_PATH="$2"
shift 2

MAX_SYNC_FRAMES=0
ENABLE_UNDISTORT="true"
while [[ $# -gt 0 ]]; do
    case "$1" in
        --max-sync-frames)
            MAX_SYNC_FRAMES="$2"
            shift 2
            ;;
        --no-undistort)
            ENABLE_UNDISTORT="false"
            shift
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

if ! command -v xterm &> /dev/null; then
    echo "Error: xterm is not installed."
    echo "Please install it with: sudo apt install xterm"
    exit 1
fi

source /opt/ros/jazzy/setup.bash

if [ ! -f "${SCRIPT_DIR}/install/setup.bash" ]; then
    echo "Error: install/setup.bash not found."
    echo "Please build the project first with:"
    echo "  colcon build --packages-up-to sync_export"
    exit 1
fi

source "${SCRIPT_DIR}/install/setup.bash"

# ---------------------------------------------------------------------------
# 关键检查：ros2 bag play 需要本地工作空间的消息定义才能创建 publisher。
# 如果 autoware_localization_msgs 找不到，bag play 会静默失败（topic 不发布）。
# ---------------------------------------------------------------------------
if ! ros2 interface show autoware_localization_msgs/msg/KinematicState &> /dev/null; then
    echo "Error: autoware_localization_msgs 消息定义未找到。"
    echo "请先编译工作空间并 source install/setup.bash:"
    echo "  cd ${SCRIPT_DIR} && colcon build --packages-up-to sync_export"
    echo "  source install/setup.bash"
    exit 1
fi
if ! ros2 interface show beidou_ins_driver/msg/Inspva &> /dev/null; then
    echo "Error: beidou_ins_driver/msg/Inspva 消息定义未找到。"
    echo "请先编译工作空间并 source install/setup.bash:"
    exit 1
fi

if [ ! -d "${SCRIPT_DIR}/${BAG_PATH}" ] && [ ! -d "${BAG_PATH}" ]; then
    echo "Error: Rosbag path not found: ${BAG_PATH}"
    echo "Also tried: ${SCRIPT_DIR}/${BAG_PATH}"
    exit 1
fi

if [ -d "${SCRIPT_DIR}/${BAG_PATH}" ]; then
    BAG_ABS_PATH="${SCRIPT_DIR}/${BAG_PATH}"
else
    BAG_ABS_PATH="$(cd "$(dirname "$BAG_PATH")" && pwd)/$(basename "$BAG_PATH")"
fi

# 输出目录绝对路径（让 OUTPUT_DIR 既可是相对也可是绝对）
if [[ "${OUTPUT_DIR}" = /* ]]; then
    OUTPUT_ABS_DIR="${OUTPUT_DIR}"
else
    OUTPUT_ABS_DIR="${SCRIPT_DIR}/${OUTPUT_DIR}"
fi
mkdir -p "${OUTPUT_ABS_DIR}"

CAMERAS_CONFIG="${SCRIPT_DIR}/config/cameras.yaml"
CALIB_DIR="${OUTPUT_ABS_DIR}/calib/camera"

# ---------------------------------------------------------------------------
# 步骤 1：生成 calib/  （camera/*.json + lidar/Lidarbase_to_Chassis.yaml）
# ---------------------------------------------------------------------------
echo "[1/3] Generating calib via convert_calibration.py..."
python3 "${SCRIPT_DIR}/convert_calibration.py" --output "${OUTPUT_ABS_DIR}" --config "${CAMERAS_CONFIG}"

echo "=========================================="
echo "Terminal:          xterm"
echo "Sample directory:  ${OUTPUT_ABS_DIR}"
echo "Cameras config:    ${CAMERAS_CONFIG}"
echo "Calib dir:         ${CALIB_DIR}"
echo "ROS bag path:      ${BAG_ABS_PATH}"
if [ "${MAX_SYNC_FRAMES}" -gt 0 ]; then
    echo "Max sync frames:   ${MAX_SYNC_FRAMES}"
else
    echo "Max sync frames:   unlimited"
fi
echo "Undistort:         ${ENABLE_UNDISTORT}"
echo "=========================================="

# ---------------------------------------------------------------------------
# 步骤 2：启动 sync_export 节点
# ---------------------------------------------------------------------------
echo "[2/3] Starting sync_export node..."
xterm -title "Sync Export" -fa "Monospace" -fs 10 \
    -e bash -c "
        echo '[Sync Export] Starting...'
        source /opt/ros/jazzy/setup.bash
        source '${SCRIPT_DIR}/install/setup.bash'
        ros2 launch sync_export sync_export.launch.py \
            output_dir:='${OUTPUT_ABS_DIR}' \
            cameras_config:='${CAMERAS_CONFIG}' \
            calib_dir:='${CALIB_DIR}' \
            max_sync_frames:='${MAX_SYNC_FRAMES}' \
            enable_undistort:='${ENABLE_UNDISTORT}'
    " &
SYNC_PID=$!

echo "Waiting 10 seconds for sync_export node to initialize..."
sleep 10

# ---------------------------------------------------------------------------
# 步骤 3：播放 rosbag（0.4 倍速）
# ---------------------------------------------------------------------------
echo "[3/3] Starting rosbag playback at 0.4x rate..."
xterm -title "Rosbag Play" -fa "Monospace" -fs 10 \
    -e bash -c "
        echo '[Rosbag Play] Starting playback at 0.4x rate...'
        source /opt/ros/jazzy/setup.bash
        source '${SCRIPT_DIR}/install/setup.bash'
        ros2 bag play --rate 0.4 '${BAG_ABS_PATH}'
    " &
BAG_PID=$!

# ---------------------------------------------------------------------------
# 步骤 4：双向检测进程生命周期
#   - sync_export 先退出（达到帧数限制）→ 停 2s → 关闭 bag play
#   - bag play 先播完                         → 停 2s → 关闭 sync_export
# ---------------------------------------------------------------------------
echo ""
echo "Monitoring: whichever finishes first triggers shutdown of the other after 2s..."
while true; do
    if ! kill -0 $SYNC_PID 2>/dev/null; then
        echo "Sync export finished. Stopping bag play in 2s..."
        sleep 2
        kill $BAG_PID 2>/dev/null || true
        break
    fi
    if ! kill -0 $BAG_PID 2>/dev/null; then
        echo "Bag play finished. Stopping sync export in 2s..."
        sleep 2
        kill $SYNC_PID 2>/dev/null || true
        break
    fi
    sleep 1
done

echo "Done."
