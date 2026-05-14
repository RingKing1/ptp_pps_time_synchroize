#!/usr/bin/env python3
"""
按数据闭环规范生成 <sample>/calib/ 目录。

输入:
  - config/cameras.yaml              # 相机 topic/规范名 与原始标定文件的映射真源
  - calibration/intrinsic/<file>     # 由 cameras.yaml 引用
  - calibration/extrinsic/<file>     # 由 cameras.yaml 引用
  - calibration/lidar_to_chassis.yaml  # Lidar->Chassis 真实外参 (identity 占位)
  - calibration/lidar_to_imu.yaml      # Lidar->IMU  真实外参 (identity 占位, 可选)

输出:
  - <output>/calib/camera/<spec_name>.json     # 6 路相机
  - <output>/calib/lidar/Lidarbase_to_Chassis.yaml
  - <output>/calib/lidar/Lidarbase_to_IMU.yaml (若提供 lidar_to_imu.yaml)

用法:
  python3 convert_calibration.py --output <sample_dir>
"""

import argparse
import json
import sys
from pathlib import Path

import yaml


def parse_extrinsic_txt(filepath: Path) -> list:
    """从 extrinsics_*.txt 解析 T_lidar_to_camera 的 4x4 矩阵，返回长度 16 行优先列表。"""
    with open(filepath, 'r') as f:
        content = f.read()

    matrix = []
    in_matrix = False
    for line in content.splitlines():
        line = line.strip()
        if 'T_lidar_to_camera:' in line:
            in_matrix = True
            continue
        if in_matrix:
            vals = [float(v.strip()) for v in line.split(',') if v.strip()]
            if len(vals) == 4:
                matrix.extend(vals)
            if len(matrix) >= 16:
                break

    if len(matrix) != 16:
        raise ValueError(f"Could not parse 4x4 matrix from {filepath}")
    return matrix


def parse_intrinsic_json(filepath: Path) -> tuple:
    """从 intrinsic JSON 中读 K (9) 和 D (5)。"""
    with open(filepath, 'r') as f:
        data = json.load(f)
    return data.get('intrinsic', []), data.get('distortion', [])


def write_camera_calib(sample_dir: Path, base_dir: Path, cameras_cfg: list) -> None:
    out_dir = sample_dir / 'calib' / 'camera'
    out_dir.mkdir(parents=True, exist_ok=True)
    intrinsic_dir = base_dir / 'calibration' / 'intrinsic'
    extrinsic_dir = base_dir / 'calibration' / 'extrinsic'

    for cam in cameras_cfg:
        name = cam['name']
        intrinsic_path = intrinsic_dir / cam['intrinsic_file']
        extrinsic_path = extrinsic_dir / cam['extrinsic_file']
        out_path = out_dir / f'{name}.json'

        print(f"[camera] {name}")
        print(f"  intrinsic : {intrinsic_path}")
        print(f"  extrinsic : {extrinsic_path}")

        extrinsic = parse_extrinsic_txt(extrinsic_path)
        intrinsic, distortion = parse_intrinsic_json(intrinsic_path)
        translation = [extrinsic[3], extrinsic[7], extrinsic[11]]

        result = {
            'extrinsic': extrinsic,
            'intrinsic': intrinsic,
            'distortion': distortion,
            'translation': translation,
        }
        with open(out_path, 'w') as f:
            json.dump(result, f, indent=2)
        print(f"  -> {out_path}")


def _write_lidar_transform_yaml(sample_dir: Path, base_dir: Path,
                                 src_name: str, out_name: str,
                                 base_frame: str, object_frame: str) -> None:
    """通用函数：从 calibration/<src_name> 读取，输出 calib/lidar/<out_name>。"""
    src = base_dir / 'calibration' / src_name
    if not src.exists():
        raise FileNotFoundError(f"Missing {src}; please create it (identity template OK).")

    with open(src, 'r') as f:
        cfg = yaml.safe_load(f) or {}

    t = cfg.get('translation', {'x': 0.0, 'y': 0.0, 'z': 0.0})
    q = cfg.get('quaternion', {'qw': 1.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0})
    e = cfg.get('euler_angle', {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0})

    out_dir = sample_dir / 'calib' / 'lidar'
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / out_name

    # 严格按规范字面顺序与 "euler angle" 空格键名写出
    lines = [
        f"base: {base_frame}",
        f"object: {object_frame}",
        "extrinsic:",
        "  euler angle:",
        f"    pitch: {float(e['pitch'])}",
        f"    roll: {float(e['roll'])}",
        f"    yaw: {float(e['yaw'])}",
        "  quaternion:",
        f"    qw: {float(q['qw'])}",
        f"    qx: {float(q['qx'])}",
        f"    qy: {float(q['qy'])}",
        f"    qz: {float(q['qz'])}",
        "  translation:",
        f"    x: {float(t['x'])}",
        f"    y: {float(t['y'])}",
        f"    z: {float(t['z'])}",
        "",
    ]
    out_path.write_text("\n".join(lines))
    print(f"[lidar]  -> {out_path}")


def write_lidar_calib(sample_dir: Path, base_dir: Path) -> None:
    _write_lidar_transform_yaml(
        sample_dir, base_dir,
        src_name='lidar_to_chassis.yaml',
        out_name='Lidarbase_to_Chassis.yaml',
        base_frame='Chassis',
        object_frame='lidar_base')

    # Lidar->IMU 是可选的：若 calibration/lidar_to_imu.yaml 存在则生成
    imu_src = base_dir / 'calibration' / 'lidar_to_imu.yaml'
    if imu_src.exists():
        _write_lidar_transform_yaml(
            sample_dir, base_dir,
            src_name='lidar_to_imu.yaml',
            out_name='Lidarbase_to_IMU.yaml',
            base_frame='IMU',
            object_frame='lidar_base')
    else:
        print(f"[lidar]  calibration/lidar_to_imu.yaml 不存在，跳过 Lidarbase_to_IMU.yaml")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output', required=True, help='样本根目录 (生成 <output>/calib/...)')
    ap.add_argument('--config', default=None,
                    help='相机配置 YAML 路径，默认 <repo>/config/cameras.yaml')
    args = ap.parse_args()

    base_dir = Path(__file__).resolve().parent
    cfg_path = Path(args.config) if args.config else base_dir / 'config' / 'cameras.yaml'
    if not cfg_path.exists():
        print(f"Error: cameras config not found: {cfg_path}", file=sys.stderr)
        sys.exit(1)

    with open(cfg_path, 'r') as f:
        cameras_cfg = (yaml.safe_load(f) or {}).get('cameras', [])
    if not cameras_cfg:
        print(f"Error: no cameras defined in {cfg_path}", file=sys.stderr)
        sys.exit(1)

    sample_dir = Path(args.output).resolve()
    sample_dir.mkdir(parents=True, exist_ok=True)

    write_camera_calib(sample_dir, base_dir, cameras_cfg)
    write_lidar_calib(sample_dir, base_dir)

    print(f"\nDone. calib written under {sample_dir}/calib/")


if __name__ == '__main__':
    main()
