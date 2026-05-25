#!/usr/bin/env python3
"""
验证 8 参数去畸变效果。

输入:
  - rosbag db3 (压缩图像)
  - calib/camera/*.json (K + 8-param D)

输出:
  - test/output/<camera_name>_frame<N>.jpg  (side-by-side 对比图)

每路相机均匀采样 5 帧，左侧原始 JPEG，右侧去畸变后图像，
左上角标注 distortion 参数。

用法:
  source /opt/ros/jazzy/setup.bash
  python3 test/verify_undistort.py
"""

import json
import sqlite3
import sys
from pathlib import Path

import cv2
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage

# ---------------------------------------------------------------------------
# 配置
# ---------------------------------------------------------------------------
REPO_ROOT = Path(__file__).resolve().parent.parent
BAG_PATH = REPO_ROOT / "data/wanghuan_0519_data/rosbag2_2026_05_19-16_25_29/rosbag2_2026_05_19-16_25_29_0.db3"
CALIB_DIR = REPO_ROOT / "data_output/data_0519/calib/camera"
OUTPUT_DIR = REPO_ROOT / "test/output"
FRAMES_PER_CAMERA = 5
FONT = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE = 0.5
FONT_COLOR = (0, 255, 0)   # BGR 绿色
LINE_THICKNESS = 1

CAMERA_NAMES = [
    "camera_0_front100",
    "camera_3_left_front",
    "camera_4_right_front",
    "camera_5_back",
    "camera_6_left_back",
    "camera_7_right_back",
]

# topic -> camera name (与 cameras.yaml 一致)
TOPIC_MAP = {
    "/cam5/compressed": "camera_0_front100",
    "/cam2/compressed": "camera_3_left_front",
    "/cam1/compressed": "camera_4_right_front",
    "/cam4/compressed": "camera_5_back",
    "/cam3/compressed": "camera_6_left_back",
    "/cam0/compressed": "camera_7_right_back",
}


def load_calib(camera_name: str):
    path = CALIB_DIR / f"{camera_name}.json"
    with open(path) as f:
        d = json.load(f)
    K = np.array(d["intrinsic"], dtype=np.float64).reshape(3, 3)
    D = np.array(d["distortion"], dtype=np.float64).reshape(1, -1)
    return K, D


def read_frames_from_bag(bag_path: Path, topic: str, num_frames: int):
    """从 rosbag 读取指定 topic 的压缩图像，均匀采样 num_frames 帧。"""
    db = sqlite3.connect(str(bag_path))
    cur = db.cursor()

    # 获取 topic_id
    cur.execute(
        "SELECT id FROM topics WHERE name = ?",
        (topic,),
    )
    row = cur.fetchone()
    if not row:
        raise ValueError(f"Topic {topic} not found in bag")
    topic_id = row[0]

    # 统计总数
    cur.execute(
        "SELECT COUNT(*) FROM messages WHERE topic_id = ?",
        (topic_id,),
    )
    total = cur.fetchone()[0]
    if total == 0:
        raise ValueError(f"No messages for topic {topic}")

    # 均匀采样索引
    if total <= num_frames:
        indices = list(range(total))
    else:
        step = total / num_frames
        indices = [int(i * step + step / 2) for i in range(num_frames)]

    frames = []
    for idx in indices:
        cur.execute(
            "SELECT data FROM messages WHERE topic_id = ? ORDER BY timestamp LIMIT 1 OFFSET ?",
            (topic_id, idx),
        )
        raw_data = cur.fetchone()[0]
        # CDR 反序列化 ROS2 CompressedImage 消息
        msg = deserialize_message(raw_data, CompressedImage)
        img = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
        if img is not None:
            frames.append((idx, img))

    db.close()
    return frames


def undistort(img: np.ndarray, K: np.ndarray, D: np.ndarray):
    """使用 8 参数去畸变。"""
    h, w = img.shape[:2]
    map1, map2 = cv2.initUndistortRectifyMap(
        K, D, None, K, (w, h), cv2.CV_16SC2
    )
    return cv2.remap(img, map1, map2, cv2.INTER_LINEAR)


def annotate_params(img: np.ndarray, camera_name: str, K: np.ndarray, D: np.ndarray):
    """在图像左上角标注相机名和 distortion 参数。"""
    h, w = img.shape[:2]
    overlay = img.copy()
    # 半透明黑色背景
    cv2.rectangle(overlay, (0, 0), (w, 90), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.6, img, 0.4, 0, img)

    k1, k2, p1, p2, k3, k4, k5, k6 = D[0]
    text_lines = [
        f"{camera_name}",
        f"fx={K[0,0]:.1f} fy={K[1,1]:.1f} cx={K[0,2]:.1f} cy={K[1,2]:.1f}",
        f"k1={k1:.3f} k2={k2:.3f} p1={p1:.6f} p2={p2:.6f}",
        f"k3={k3:.3f} k4={k4:.3f} k5={k5:.3f} k6={k6:.3f}",
    ]
    y = 20
    for line in text_lines:
        cv2.putText(img, line, (10, y), FONT, FONT_SCALE, FONT_COLOR, LINE_THICKNESS)
        y += 18
    return img


def make_side_by_side(orig: np.ndarray, undist: np.ndarray, camera_name: str, frame_idx: int,
                      K: np.ndarray, D: np.ndarray):
    """生成 side-by-side 对比图。"""
    # 确保尺寸一致
    h, w = orig.shape[:2]
    if undist.shape[:2] != (h, w):
        undist = cv2.resize(undist, (w, h))

    # 左侧标注 "Original"，右侧标注 "Undistorted"
    orig_label = orig.copy()
    undist_label = undist.copy()
    cv2.putText(orig_label, "Original", (10, h - 15), FONT, 0.7, (0, 0, 255), 2)
    cv2.putText(undist_label, "Undistorted", (10, h - 15), FONT, 0.7, (0, 255, 0), 2)

    combined = np.hstack((orig_label, undist_label))

    # 全局标注参数（底部叠加）
    combined = annotate_params(combined, camera_name, K, D)

    return combined


def main():
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    if not BAG_PATH.exists():
        print(f"Error: rosbag not found: {BAG_PATH}", file=sys.stderr)
        sys.exit(1)

    for camera_name in CAMERA_NAMES:
        print(f"\n[{camera_name}]")
        topic = None
        for t, name in TOPIC_MAP.items():
            if name == camera_name:
                topic = t
                break
        if topic is None:
            print(f"  Skip: no topic mapping", file=sys.stderr)
            continue

        K, D = load_calib(camera_name)
        print(f"  K={K[0,0]:.1f},{K[1,1]:.1f} D={D.shape[1]} coeffs")

        try:
            frames = read_frames_from_bag(BAG_PATH, topic, FRAMES_PER_CAMERA)
        except ValueError as e:
            print(f"  Error reading bag: {e}", file=sys.stderr)
            continue

        for frame_idx, img in frames:
            undist_img = undistort(img, K, D)
            combined = make_side_by_side(img, undist_img, camera_name, frame_idx, K, D)

            out_path = OUTPUT_DIR / f"{camera_name}_frame{frame_idx:04d}.jpg"
            cv2.imwrite(str(out_path), combined)
            print(f"  -> {out_path.name}  ({img.shape[1]}x{img.shape[0]})")

    print(f"\nDone. Output: {OUTPUT_DIR}/")


if __name__ == "__main__":
    main()
