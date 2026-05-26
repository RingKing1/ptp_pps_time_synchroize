# ROS2 Bag 同步导出工具

本项目用于将 ROS2 rosbag 中录制的多传感器数据同步导出为**腾讯数据闭环平台**规范的数据集格式。支持 6 路相机图像（去畸变，可选开关）、1 路激光雷达点云、车辆位姿 YAML（WGS84）以及 INS sqlite3 的近似时间同步导出。

## 功能介绍

- **多传感器同步**：使用 `message_filters::ApproximateTime` 策略对 6 路相机 + 1 路激光雷达进行近似时间同步（7 输入同步器）。位姿数据通过独立高频订阅缓存，按时间最近查找，避免加入同步导致 ApproximateTime 死锁
- **图像去畸变**：节点启动时加载相机内参与畸变参数，对每帧 JPEG 进行 `cv::remap` 去畸变后保存。支持通过 `--no-undistort` 关闭去畸变
- **WGS84 位姿输出**：通过独立订阅 `/beidou/inspva` 获取北斗 INS 提供的 WGS84 经纬高，写入 `localization/<ts>.yaml` 的 `pose.position`
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
| `/localization/kinematicstate` | `autoware_localization_msgs/KinematicState` | 车辆运动状态（独立高频订阅） |
| `/beidou/inspva` | `beidou_ins_driver/msg/Inspva` | 北斗 INS WGS84 经纬高 |

> 话题与规范名的映射真源位于 [`config/cameras.yaml`](config/cameras.yaml)，修改此处即可调整相机顺序或增减相机。

## 输出目录结构

按腾讯数据闭环平台规范组织：

```
<output_dir>/                              # 样本文件夹（zip 时即样本名）
├── camera/                                # 去畸变后的图像（可关闭）
│   ├── camera_0_front100/<ts>.jpg
│   ├── camera_3_left_front/<ts>.jpg
│   ├── camera_4_right_front/<ts>.jpg
│   ├── camera_5_back/<ts>.jpg
│   ├── camera_6_left_back/<ts>.jpg
│   └── camera_7_right_back/<ts>.jpg
├── calib/
│   ├── camera/
│   │   ├── camera_0_front100.json         # extrinsic + intrinsic + distortion(8) + translation
│   │   ├── camera_3_left_front.json
│   │   ├── camera_4_right_front.json
│   │   ├── camera_5_back.json
│   │   ├── camera_6_left_back.json
│   │   └── camera_7_right_back.json
│   └── lidar/
│       ├── Lidarbase_to_Chassis.yaml      # lidar -> Chassis 外参
│       └── Lidarbase_to_IMU.yaml          # lidar -> IMU 外参（可选）
├── lidar/<ts>.pcd                         # binary PCD，含 intensity
├── localization/<ts>.yaml                 # pose(WGS84) + vel + acc + cov
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
- `beidou_ins_driver`（北斗 INS 消息包，位于 `src/beidou_ins_driver/`）

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
# 全部导出（去畸变默认开启）
./run.sh scene1 data/rosbag2_2026_04_27-16_44_35

# 只导出前 1000 帧
./run.sh scene1 data/rosbag2_2026_04_27-16_44_35 --max-sync-frames 1000

# 关闭去畸变
./run.sh scene1 data/rosbag2_2026_04_27-16_44_35 --no-undistort
```

`run.sh` 会自动完成以下步骤：
1. 调用 `convert_calibration.py` 生成 `<output>/calib/`
2. 启动 `sync_export` 节点
3. 延迟 10s 后播放 rosbag
4. 双向检测： whichever 先结束，停 2s 后自动关闭另一方

### 标定文件准备

本分支使用**原始标定 TXT 文件**作为输入源（含 8 参数畸变）：

1. 将相机厂商提供的标定 TXT 文件放入 `calibration/intrinsic/`
2. 确保 `config/cameras.yaml` 中 `intrinsic_file` 指向正确的 TXT 文件名
3. `convert_calibration.py` 会自动解析 TXT 并生成 `calib/camera/*.json`

TXT 文件格式示例：
```
SN码:H60FA-G12221669
FX:1950.2933412435
FY:1950.1008224996
CX:953.5413262951
CY:543.0735017008
K1:35.5988436352
K2:60.6725471701
P1:-0.0000134649
P2:0.0000713496
K3:6.5825207363
K4:36.1637560345
K5:80.1604550530
K6:38.9128048330
RMS:0.0047
```

外参文件放在 `calibration/extrinsic/` 下，如需更新请直接替换。

另外请填入 Lidar 外参模板：
```bash
# 编辑后保存
calibration/lidar_to_chassis.yaml   # Lidar -> Chassis
calibration/lidar_to_imu.yaml       # Lidar -> IMU（可选，不提供则不生成）
```

### 手动运行（高级）

```bash
# 1. 生成标定
python3 convert_calibration.py --output scene1 --config config/cameras.yaml

# 2. 启动节点
ros2 launch sync_export sync_export.launch.py \
    output_dir:=$(pwd)/scene1 \
    cameras_config:=$(pwd)/config/cameras.yaml \
    calib_dir:=$(pwd)/scene1/calib/camera \
    max_sync_frames:=0 \
    enable_undistort:=true

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
| `inspva_topic` | string | `/beidou/inspva` | 北斗 INS WGS84 话题 |
| `queue_size` | int | `20` | 同步缓存深度 |
| `max_sync_frames` | int | `0` | 最大同步帧数，0 = 无限制 |
| `enable_undistort` | bool | `true` | 是否开启相机去畸变 |

## 项目结构

```
.
├── calibration/                         # 原始标定文件
│   ├── intrinsic/*.txt                  # 8参数标定源文件
│   ├── extrinsic/*.txt                  # Lidar->Camera 外参
│   ├── lidar_to_chassis.yaml
│   └── lidar_to_imu.yaml
├── config/
│   └── cameras.yaml                     # topic -> 规范名映射
├── convert_calibration.py               # 生成规范 calib/ 目录（支持8参数TXT）
├── run.sh                               # 一键运行脚本
├── run_test_sync.sh                     # 时间同步调试脚本
├── data/                                # rosbag 数据
├── claude.md                            # 项目文档
├── src/
│   ├── autoware_common_msgs/
│   ├── autoware_localization_msgs/
│   ├── beidou_ins_driver/               # 北斗 INS 消息定义
│   └── sync_export/
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── launch/
│       │   └── sync_export.launch.py
│       └── src/
│           ├── sync_export_node.cpp     # 主节点
│           └── test_sync_node.cpp       # 同步测试节点
└── README.md
```

## 时间同步调试工具

`test_sync_node` 是一个轻量的时间同步调试节点，用于实时查看各传感器与雷达之间的时间偏差。它同样使用 `ApproximateTime` 对 6 路相机 + 1 路雷达做同步，同时独立订阅 `kinematicstate` 和 `inspva`，在每帧同步回调中打印各传感器与雷达的时间差。

### 输出示例

```
[SYNC #0] lidar=1737428250.100s | cam0=12.3ms cam1=-8.7ms cam2=5.1ms cam3=15.2ms cam4=-3.4ms cam5=7.8ms | ks=2.1ms inspva=45.6ms
```

- 正数表示该传感器时间戳**晚于**雷达（传感器数据比雷达晚到）
- 负数表示该传感器时间戳**早于**雷达
- `-999.0ms` 表示该时刻缓冲区中无对应数据

### 运行方法

```bash
# 使用便捷脚本
./run_test_sync.sh data/rosbag2_2026_04_27-16_44_35

# 可指定播放速率
./run_test_sync.sh data/rosbag2_2026_04_27-16_44_35 --rate 1.0
```

该脚本自动完成：
1. 启动 `test_sync_node`（加载 `config/cameras.yaml`）
2. 延迟 5s 后播放 rosbag
3. 任一方退出后自动关闭另一方

### 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `cameras_config` | string | - | `config/cameras.yaml` 路径 |
| `lidar_topic` | string | `/rslidar_points` | 激光雷达话题 |
| `kinematic_state_topic` | string | `/localization/kinematicstate` | 定位话题 |
| `inspva_topic` | string | `/beidou/inspva` | 北斗 INS WGS84 话题 |
| `queue_size` | int | `20` | 同步缓存深度 |

## 注意事项

1. **同步策略**：采用 `ApproximateTime` 近似时间同步（7 输入：6 相机 + 1 雷达）。`kinematic_state` 不加入同步器，而是通过独立高频订阅缓存，在回调中按时间最近查找，避免多输入 ApproximateTime 死锁
2. **去畸变**：节点加载 `calib/camera/*.json` 中的 `intrinsic` 与 `distortion`，支持 8 参数畸变模型（k1, k2, p1, p2, k3, k4, k5, k6）。使用 `cv::initUndistortRectifyMap` + `cv::remap` 对每帧进行去畸变，可通过 `--no-undistort` 关闭
3. **INS sqlite3**：采用 WAL 模式 + 批提交（每 100 条 COMMIT），独立订阅器以原始频率写入，不受同步降采影响
4. **WGS84 坐标**：`localization/<ts>.yaml` 中的 `pose.position` 优先使用 `/beidou/inspva` 提供的 WGS84 经纬高（经时间最近查找匹配）。若该时刻无 inspva 数据，则回退到 `kinematic_state` 的原始局部坐标
