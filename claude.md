# Project Overview

ROS2 workspace for parsing self-built sensor datasets. Synchronizes and exports camera images, LiDAR point clouds, and localization/INS data from ROS2 bags into a structured sample format aligned with the Tencent data closed-loop specification.

## Key Components

- **sync_export_node** (`src/sync_export/src/sync_export_node.cpp`): Main node that subscribes to 6 camera compressed image topics, 1 LiDAR topic, KinematicState, and Inspva. Uses `message_filters` ApproximateTime synchronizer to align camera + LiDAR frames, then exports them to disk.
- **run.sh**: Entry point script. Generates calibration files, launches sync_export_node, and plays the rosbag.
- **convert_calibration.py**: Converts calibration data to JSON format expected by sync_export_node.

## Build

```bash
colcon build --packages-up-to sync_export
source install/setup.bash
```

## Run

```bash
./run.sh <output_folder> <rosbag_path> [--max-sync-frames N] [--no-undistort]
```

- `--max-sync-frames N`: Limit the number of synced frames to export (default: 0 = unlimited).
- `--no-undistort`: Disable camera image undistortion (default: enabled).

## Output Structure

```
<output_dir>/
  camera/<camera_name>/   # .jpg images (undistorted by default)
  lidar/                  # .pcd point clouds
  localization/           # .yaml pose files
  INS/                    # ins_export.sqlite3 (odometry + accel)
  calib/camera/           # .json intrinsic files
  calib/lidar/            # Lidarbase_to_Chassis.yaml
```

## Key Parameters (sync_export.launch.py)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `output_dir` | `export_sync` | Export root directory |
| `cameras_config` | (required) | Path to cameras.yaml |
| `calib_dir` | (required) | Path to camera calibration JSONs |
| `lidar_topic` | `/rslidar_points` | LiDAR topic |
| `kinematic_state_topic` | `/localization/kinematicstate` | Localization topic |
| `inspva_topic` | `/beidou/inspva` | INS WGS84 topic |
| `queue_size` | `20` | ApproximateTime sync buffer size |
| `max_sync_frames` | `0` | Max frames to export (0 = unlimited) |
| `enable_undistort` | `true` | Enable/disable camera undistortion |
