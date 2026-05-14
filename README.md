# ROS2 Bag 同步导出工具

本项目用于将 ROS2 rosbag 中录制的多传感器数据同步导出为**腾讯数据闭环平台**规范的数据集格式。支持 6 路相机图像（去畸变）、1 路激光雷达点云、车辆位姿 YAML 以及 INS sqlite3 的近似时间同步导出。

## 功能介绍

- **多传感器同步**：使用 `message_filters::ApproximateTime` 策略对 6 路相机 + 1 路激光雷达 + 位姿进行近似时间同步
- **图像去畸变**：节点启动时加载相机内参与畸变参数，对每帧 JPEG 进行 `cv::remap` 去畸变后保存
- **统一文件名**：所有输出文件以**激光雷达时间戳**（10ms 精度）命名，确保各传感器数据一一对应
- **INS 导出**：独立订阅 `/localization/kinematicstate`，逐条写入 `INS/ins_export.sqlite3`（odometry + accel 两张表）
- **帧数限制**：支持 `--max-sync-frames N`，达到指定帧数后节点自动退出
- **标定自动生成**：运行前自动调用 `convert_calibration.py` 生成 `calib/` 目录（相机内外参 + lidar 外参）

## 输入话题

| ROS Topic | 消息类型 | 说明 |
|-----------|----------|------|
| `/cam5/compressed` | `sensor_msgs/CompressedImage` | 前相机（camera_0_front100） |
| `/cam2/compressed` | `sensor_msgs/CompressedImage` | 前左相机（camera_3_left_front） |
| `/cam1/compressed` | `sensor_msgs/CompressedImage` | 前右相机（camera_4_right_front） |
| `/cam4/compressed` | `sensor_msgs/CompressedImage` | 后相机（camera_5_back） |
| `/cam3/compressed` | `sensor_msgs/CompressedImage` | 后左相机（camera_6_left_back） |
| `/cam0/compressed` | `sensor_msgs/CompressedImage` | 后右相机（camera_7_right_back） |
| `/rslidar_points` | `sensor_msgs/PointCloud2` | 激光雷达点云 |
| `/localization/kinematicstate` | `autoware_localization_msgs/KinematicState` | 车辆运动状态 |

> 话题与规范名的映射真源位于 [`config/cameras.yaml`](config/cameras.yaml)，修改此处即可调整相机顺序或增减相机。

## 输出目录结构

按腾讯数据闭环平台规范组织：

```
<output_dir>/                              # 样本文件夹（zip 时即样本名）
├── camera/                                # 去畸变后的图像
│   ├── camera_0_front100/<ts>.jpg
│   ├── camera_3_left_front/<ts>.jpg
│   ├── camera_4_right_front/<ts>.jpg
│   ├── camera_5_back/<ts>.jpg
│   ├── camera_6_left_back/<ts>.jpg
│   └── camera_7_right_back/<ts>.jpg
├── calib/
│   ├── camera/
│   │   ├── camera_0_front100.json         # extrinsic + intrinsic + distortion + translation
│   │   ├── camera_3_left_front.json
│   │   ├── camera_4_right_front.json
│   │   ├── camera_5_back.json
│   │   ├── camera_6_left_back.json
│   │   └── camera_7_right_back.json
│   └── lidar/
│       ├── Lidarbase_to_Chassis.yaml      # lidar -> Chassis 外参
│       └── Lidarbase_to_IMU.yaml          # lidar -> IMU 外参（可选）
├── lidar/<ts>.pcd                         # binary PCD，含 intensity
├── localization/<ts>.yaml                 # pose + vel + acc + cov
└── INS/
    └── ins_export.sqlite3                 # t_export_odometry + t_export_accel
```

## 文件命名规则

所有文件以**激光雷达时间戳**（Unix 时间戳，10 毫秒精度）命名：

```
<unix_timestamp_10ms_precision>.<ext>

示例：
- camera/camera_0_front100/173742825010.jpg
- lidar/173742825010.pcd
- localization/173742825010.yaml
```

其中时间戳计算方式为：`floor(timestamp_seconds * 100)`

## 依赖项

- ROS 2 Jazzy
- PCL (Point Cloud Library)
- OpenCV
- yaml-cpp
- SQLite3
- `autoware_localization_msgs`（本地自定义消息包，位于 `src/autoware_localization_msgs/`）
- `autoware_common_msgs`（`autoware_localization_msgs` 的依赖，位于 `src/autoware_common_msgs/`）

## 编译步骤

```bash
source /opt/ros/jazzy/setup.bash
cd "你的项目路径"
colcon build --packages-up-to sync_export
source install/setup.bash
```

## 运行方法

### 一键运行（推荐）

```bash
# 全部导出
./run.sh scene1 data/rosbag2_2026_04_27-16_44_35

# 只导出前 1000 帧
./run.sh scene1 data/rosbag2_2026_04_27-16_44_35 --max-sync-frames 1000
```

`run.sh` 会自动完成以下步骤：
1. 调用 `convert_calibration.py` 生成 `<output>/calib/`
2. 启动 `sync_export` 节点
3. 延迟 10s 后播放 rosbag
4. 双向检测： whichever 先结束，停 2s 后自动关闭另一方

### 标定文件准备

首次使用前，请将真实外参填入占位模板：

```bash
# 编辑后保存
calibration/lidar_to_chassis.yaml   # Lidar -> Chassis
calibration/lidar_to_imu.yaml       # Lidar -> IMU（可选，不提供则不生成）
```

相机内外参已放在 `calibration/intrinsic/` 和 `calibration/extrinsic/` 下，如需更新请直接替换文件，并确保 [`config/cameras.yaml`](config/cameras.yaml) 中的 `intrinsic_file` / `extrinsic_file` 字段指向正确的文件名。

### 手动运行（高级）

```bash
# 1. 生成标定
python3 convert_calibration.py --output scene1 --config config/cameras.yaml

# 2. 启动节点
ros2 launch sync_export sync_export.launch.py \
    output_dir:=$(pwd)/scene1 \
    cameras_config:=$(pwd)/config/cameras.yaml \
    calib_dir:=$(pwd)/scene1/calib/camera \
    max_sync_frames:=0

# 3. 另开终端播放 rosbag
ros2 bag play data/rosbag2_2026_04_27-16_44_35 --rate 0.4
```

### Launch 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `output_dir` | string | `export_sync` | 样本根目录 |
| `cameras_config` | string | - | `config/cameras.yaml` 路径 |
| `calib_dir` | string | - | `calib/camera/` 路径 |
| `lidar_topic` | string | `/rslidar_points` | 激光雷达话题 |
| `kinematic_state_topic` | string | `/localization/kinematicstate` | 定位话题 |
| `queue_size` | int | 20 | 同步缓存深度 |
| `max_sync_frames` | int | 0 | 最大同步帧数，0 = 无限制 |

## 项目结构

```
.
├── calibration/                         # 原始标定文件
│   ├── intrinsic/*.json
│   ├── extrinsic/*.txt
│   ├── lidar_to_chassis.yaml
│   └── lidar_to_imu.yaml
├── config/
│   └── cameras.yaml                     # topic -> 规范名映射
├── convert_calibration.py               # 生成规范 calib/ 目录
├── run.sh                               # 一键运行脚本
├── data/                                # rosbag 数据
├── src/
│   ├── autoware_common_msgs/
│   ├── autoware_localization_msgs/
│   └── sync_export/
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── launch/
│       │   └── sync_export.launch.py
│       └── src/
│           └── sync_export_node.cpp
└── README.md
```

## 注意事项

1. **同步策略**：采用 `ApproximateTime` 近似时间同步，位姿数据频率远高于相机/雷达，每个同步窗口仅保留与雷达时间戳最接近的一帧位姿
2. **去畸变**：节点加载 `calib/camera/*.json` 中的 `intrinsic` 与 `distortion`，使用 `cv::initUndistortRectifyMap` + `cv::remap` 对每帧进行去畸变，输出图像分辨率与原图保持一致
3. **INS sqlite3**：采用 WAL 模式 + 批提交（每 100 条 COMMIT），独立订阅器以原始频率写入，不受同步降采影响
4. **坐标系**：位姿 YAML 中的 `position` 直接输出 ROS 消息原始值，当前环境已为 WGS84 经纬高，无需额外转换
