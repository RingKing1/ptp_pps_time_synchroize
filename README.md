# ROS2 Bag 同步导出工具

本项目用于将 ROS2 rosbag 中录制的多传感器数据同步导出为标准化的数据集格式。支持 6 路相机图像、1 路激光雷达点云以及车辆位姿数据的近似时间同步导出。

## 功能介绍

- **多传感器同步**：使用 `message_filters::ApproximateTime` 策略对 8 个话题进行近似时间同步
- **统一文件名**：所有输出文件以**激光雷达时间戳**为基准命名，确保各传感器数据一一对应
- **位姿导出**：将 `KinematicState` 消息解析为结构化 YAML 文件，包含位姿、速度、角速度、加速度及协方差矩阵
- **直接可用**：C++ 节点直接生成最终文件名，无需额外的后处理重命名脚本

## 输入话题

| ROS Topic | 消息类型 | 说明 |
|-----------|----------|------|
| `/cam0/compressed` | `sensor_msgs/CompressedImage` | 相机0（后右） |
| `/cam1/compressed` | `sensor_msgs/CompressedImage` | 相机1（前右） |
| `/cam2/compressed` | `sensor_msgs/CompressedImage` | 相机2（前左） |
| `/cam3/compressed` | `sensor_msgs/CompressedImage` | 相机3（后左） |
| `/cam4/compressed` | `sensor_msgs/CompressedImage` | 相机4（后） |
| `/cam5/compressed` | `sensor_msgs/CompressedImage` | 相机5（前） |
| `/rslidar_points` | `sensor_msgs/PointCloud2` | 激光雷达点云 |
| `/localization/kinematicstate` | `autoware_localization_msgs/KinematicState` | 车辆运动状态（位姿+速度+加速度） |

## 输出目录结构

```
<output_dir>/
├── cam_back/           # 后相机 JPG 图像
├── cam_back_left/      # 后左相机 JPG 图像
├── cam_back_right/     # 后右相机 JPG 图像
├── cam_front/          # 前相机 JPG 图像
├── cam_front_left/     # 前左相机 JPG 图像
├── cam_front_right/    # 前右相机 JPG 图像
├── lidar/              # 激光雷达 PCD 点云
└── localization/       # 车辆位姿 YAML 文件
```

## 文件命名规则

所有文件以**激光雷达时间戳**（Unix 时间戳，10 毫秒精度）命名：

```
<unix_timestamp_10ms_precision>.<ext>

示例：
- cam_back/173742825010.jpg
- lidar/173742825010.pcd
- localization/173742825010.yaml
```

其中时间戳计算方式为：`floor(timestamp_seconds * 100)`

## 位姿 YAML 格式

`localization/` 目录下的 YAML 文件包含以下字段：

```yaml
header:
  frameId: Chassis           # 坐标系名称，取自消息 header.frame_id
  timestampSec: 1739498509.0 # 位姿消息自身的 Unix 时间戳（秒）
worldFrame: WGS84
poseConfidence: 1.0
status: 2
mode: 1
consistencyToMap: 1.0
pose:
  orientation:
    w: 0.6347339892067021
    x: -0.0016194361946210098
    y: 0.02093424644864628
    z: 0.7724452345876188
  position:
    x: 116.26878997599766    # 经度或 ENU 坐标系 X
    y: 40.0397848703284      # 纬度或 ENU 坐标系 Y
    z: 39.07551461313173     # 高度或 ENU 坐标系 Z
posCov:
  - 0.0                      # 36 个元素，来自 PoseWithCovariance.covariance
  - ...
vel:
  x: -0.26303324862161914   # 线速度（m/s）
  y: 1.289909567531137
  z: -0.04197598147769784
angularV:
  x: 0.0009304552346556005  # 角速度（rad/s）
  y: 0.0073151708822296515
  z: 0.002870088706778395
velCov:
  - 0.0                      # 36 个元素，来自 TwistWithCovariance.covariance
  - ...
acc:
  x: 0.2863556115171435     # 线加速度（m/s^2）
  y: -0.02831622223033823
  z: 9.808668938394934
```

## 依赖项

- ROS 2 Jazzy
- PCL (Point Cloud Library)
- OpenCV
- `autoware_localization_msgs`（本地自定义消息包，位于 `src/autoware_localization_msgs/`）
- `autoware_common_msgs`（`autoware_localization_msgs` 的依赖，位于 `src/autoware_common_msgs/`）

## 编译步骤

### 1. 环境准备

确保已 source ROS 2 环境：

```bash
source /opt/ros/jazzy/setup.bash
```

### 2. 编译消息包和节点

```bash
cd "你的项目路径"

# 完整构建（自动处理依赖顺序）
colcon build --packages-up-to sync_export
```

如果系统中存在多个 Python 版本导致 cmake 找不到正确的 Python，可显式指定：

```bash
colcon build --packages-up-to sync_export --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

### 3. 安装缺失的 Python 依赖（如报错）

编译过程中若提示缺少 Python 模块，例如：
- `No module named 'catkin_pkg'`
- `No module named 'em'`

请根据报错安装对应依赖：

```bash
# 对于系统默认 Python
sudo apt install python3-catkin-pkg

# 若 cmake 实际调用的是其他 Python 版本（如 python3.10）
python3.10 -m pip install catkin_pkg empy --break-system-packages
```

### 4. 加载编译结果

```bash
source install/setup.bash
```

## 运行方法

### 启动导出节点

```bash
ros2 run sync_export sync_export_node --ros-args --params-file src/sync_export/config/sync_export.yaml
```

或启动 launch 文件（如有配置）：

```bash
ros2 launch sync_export sync_export.launch.py
```

### 回放 rosbag

在另一个终端回放 bag 数据，节点会自动接收并导出：

```bash
ros2 bag play data/rosbag2_2026_04_27-16_44_35
```

### 参数配置

修改 `src/sync_export/config/sync_export.yaml`：

```yaml
sync_export_node:
  ros__parameters:
    output_dir: "export_sync"   # 输出目录路径
```

## 项目结构

```
.
├── data/
│   ├── rosbag2_2026_04_27-16_44_35/   # rosbag 数据示例
│   └── rosbag2_2026_04_27-16_50_02/
├── src/
│   ├── autoware_common_msgs/          # 自定义消息依赖
│   ├── autoware_localization_msgs/    # KinematicState 消息定义
│   │   └── msg/
│   │       └── KinematicState.msg
│   └── sync_export/
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── config/
│       │   └── sync_export.yaml       # 节点参数配置
│       ├── launch/
│       │   └── sync_export.launch.py
│       └── src/
│           └── sync_export_node.cpp   # 主节点源码
└── README.md
```

## 注意事项

1. **同步策略**：采用 `ApproximateTime` 近似时间同步，位姿数据频率（~28kHz）远高于相机/雷达（~2.9kHz），每个同步窗口仅保留与雷达时间戳最接近的一帧位姿
2. **坐标系**：位姿 YAML 中的 `position` 直接输出 ROS 消息原始值，如需转换为 WGS84 经纬高，请在下游处理
3. **协方差矩阵**：若消息中协方差字段未填充，YAML 中将输出 36 个 `0.0`
4. **图像格式**：节点直接保存 `CompressedImage` 的原始压缩数据，确保输入话题发布的是 JPEG 压缩图像
