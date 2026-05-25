# Project Overview

ROS2 workspace for parsing self-built sensor datasets. Synchronizes and exports camera images, LiDAR point clouds, and localization data from ROS2 bags into a structured sample format aligned with the Tencent data closed-loop specification.

This branch (`局部地图坐标解析`) adds **WGS84 coordinate conversion** and **8-parameter camera distortion** compared to the base `1dc79c0` commit.

## Key Components

- **sync_export_node** (`src/sync_export/src/sync_export_node.cpp`): Main node that subscribes to 6 camera compressed image topics, 1 LiDAR topic, and KinematicState. Uses `message_filters` ApproximateTime synchronizer (8-input) to align camera + LiDAR + pose frames, then exports them to disk.
- **UTM Coordinate Conversion** (`src/sync_export/src/utm/`): Embedded UTM/Transverse Mercator C library for converting local ENU coordinates to WGS84 lat/lon/height.
- **run.sh**: Entry point script. Generates calibration files, launches sync_export_node, and plays the rosbag.
- **convert_calibration.py**: Converts raw calibration TXT files (with 8-param distortion) to JSON format expected by sync_export_node.

## Branch-Specific Features

### WGS84 Coordinate Output
Local ENU plane coordinates from `/localization/kinematicstate` are converted to WGS84 longitude/latitude/height before export:
- **Local map origin** (UTM Zone 50N): `Easting=630224.58, Northing=3480549.44, Height=10.32`
- Conversion formula:
  ```
  UTM_easting  = local_x + origin_easting
  UTM_northing = local_y + origin_northing
  height       = local_z + origin_height
  // Then Convert_UTM_To_Geodetic → WGS84(lat, lon)
  ```
- Applied to both `localization/<ts>.yaml` (pose.position) and `INS/ins_export.sqlite3` (t_export_odometry).

### 8-Parameter Camera Distortion
Calibration source files (`calibration/intrinsic/*.txt`) contain 8 distortion coefficients:
```
[k1, k2, p1, p2, k3, k4, k5, k6]
```
OpenCV `cv::initUndistortRectifyMap` supports this directly via dynamic-length D matrix.

## Build

```bash
colcon build --packages-up-to sync_export
source install/setup.bash
```

## Run

```bash
./run.sh <output_folder> <rosbag_path> [--max-sync-frames N]
```

- `--max-sync-frames N`: Limit the number of synced frames to export (default: 0 = unlimited).

## Output Structure

```
<output_dir>/
  camera/<camera_name>/      # Undistorted .jpg images
  lidar/                     # .pcd point clouds
  localization/              # .yaml pose files (WGS84 coordinates)
  INS/                       # ins_export.sqlite3 (odometry + accel, WGS84 coords)
  calib/camera/              # .json intrinsic files (generated from TXT)
  calib/lidar/               # Lidarbase_to_Chassis.yaml
```

## Input Topics

| ROS Topic | Message Type | Description |
|-----------|-------------|-------------|
| `/cam5/compressed` | `sensor_msgs/CompressedImage` | Front camera |
| `/cam2/compressed` | `sensor_msgs/CompressedImage` | Front-left camera |
| `/cam1/compressed` | `sensor_msgs/CompressedImage` | Front-right camera |
| `/cam4/compressed` | `sensor_msgs/CompressedImage` | Back camera |
| `/cam3/compressed` | `sensor_msgs/CompressedImage` | Back-left camera |
| `/cam0/compressed` | `sensor_msgs/CompressedImage` | Back-right camera |
| `/rslidar_points` | `sensor_msgs/PointCloud2` | LiDAR point cloud |
| `/localization/kinematicstate` | `autoware_localization_msgs/KinematicState` | Vehicle pose (local ENU) |

> Topic-to-name mapping is in `config/cameras.yaml`.

## Key Parameters (sync_export.launch.py)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `output_dir` | `export_sync` | Export root directory |
| `cameras_config` | (required) | Path to cameras.yaml |
| `calib_dir` | (required) | Path to generated camera calibration JSONs |
| `lidar_topic` | `/rslidar_points` | LiDAR topic |
| `kinematic_state_topic` | `/localization/kinematicstate` | Localization topic |
| `queue_size` | `20` | ApproximateTime sync buffer size |
| `max_sync_frames` | `0` | Max frames to export (0 = unlimited) |

## Calibration Pipeline

1. Place raw calibration TXT files in `calibration/intrinsic/` (8-param format with FX, FY, CX, CY, K1-K6, P1-P2).
2. Place extrinsic TXT files in `calibration/extrinsic/`.
3. Edit `config/cameras.yaml` to map topics to the correct intrinsic/extrinsic files.
4. `run.sh` automatically calls `convert_calibration.py` to generate `<output>/calib/` before launching the node.

## Coordinate System Notes

- **Raw input**: KinematicState provides vehicle pose in local ENU coordinates (origin at UTM Easting=630224.58, Northing=3480549.44).
- **Exported output**: All position data is converted to WGS84 (longitude = x, latitude = y, height = z).
- The conversion is hardcoded in `sync_export_node.cpp` via `local_to_wgs84()` using the embedded UTM library.
