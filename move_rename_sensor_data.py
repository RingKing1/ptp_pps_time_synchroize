import os
import shutil
from pathlib import Path

def list_files_sorted(folder: Path):
    """列出目录下所有文件（不含子目录），按文件名排序。"""
    return sorted([p for p in folder.iterdir() if p.is_file()], key=lambda x: x.name)

def move_and_rename(src_dir: Path, dst_dir: Path, num_files: int, start_index: int = 1, dry_run: bool = False):
    """
    从 src_dir 取 num_files 个文件移动到 dst_dir，并从 start_index 开始重命名，保留后缀。
    返回：实际移动文件数
    """
    if not src_dir.exists():
        raise FileNotFoundError(f"源目录不存在: {src_dir}")

    dst_dir.mkdir(parents=True, exist_ok=True)

    files = list_files_sorted(src_dir)
    if num_files is None:
        num_files = len(files)

    files = files[:num_files]

    moved = 0
    for i, f in enumerate(files, start=start_index):
        ext = f.suffix  # 保留后缀
        new_name = f"{i}{ext}"
        dst_path = dst_dir / new_name

        # 防止覆盖：若已存在则追加 _1, _2 ...
        if dst_path.exists():
            k = 1
            while True:
                candidate = dst_dir / f"{i}_{k}{ext}"
                if not candidate.exists():
                    dst_path = candidate
                    break
                k += 1

        if dry_run:
            print(f"[DRY RUN] {f}  ->  {dst_path}")
        else:
            shutil.move(str(f), str(dst_path))
            print(f"[MOVED]   {f.name}  ->  {dst_path.name}")

        moved += 1

    return moved

def duplicate_config_file(
    src_file: Path,
    dst_dir: Path,
    count: int = 100,
    start_index: int = 1,
    dry_run: bool = False
):
    """
    将 src_file 复制 count 份到 dst_dir，命名为 start_index..start_index+count-1，保留后缀
    例：1.json ... 100.json
    """
    if not src_file.exists():
        raise FileNotFoundError(f"配置文件不存在: {src_file}")

    dst_dir.mkdir(parents=True, exist_ok=True)

    ext = src_file.suffix  # .json
    for i in range(start_index, start_index + count):
        dst_path = dst_dir / f"{i}{ext}"

        if dry_run:
            print(f"[DRY RUN] {src_file}  ->  {dst_path}")
        else:
            shutil.copy2(str(src_file), str(dst_path))
            print(f"[COPIED]  {src_file.name}  ->  {dst_path.name}")

def main():
    # ================== 你需要改的路径（按你的实际目录） ==================
    # 数据源根目录（第一张图那些 CAM_* / LIDAR_TOP 所在）
    src_root = Path(r"/home/nvidia/Self-Built Dataset Parsing/ptp_pps_time_synchroize/export_sync")

    # 目标根目录（第二张图 data_xtrem1/scence1 下 camera_image_* 等所在）
    dst_root = Path(r"/home/nvidia/Self-Built Dataset Parsing/ptp_pps_time_synchroize/data_xtrem1/scence1")
    # ==================================================================

    # 1) 先做 camera_config 的复制与重命名（可选步骤）
    # 例如：源文件在 scence1/camera_config/102007.json
    #      目标目录在 data_xtrem1/scence1/camera_config
    config_src_file = Path(r"/home/nvidia/Self-Built Dataset Parsing/ptp_pps_time_synchroize/scence1/camera_config/102007.json")  # 改成你的真实路径
    config_dst_dir  = dst_root / "camera_config"

    # 复制数量（可选）：改成你想要的数量；不想执行就设为 0
    config_copy_count = 100

    # 是否只预演（不真的复制/移动）
    dry_run = False

    if config_copy_count > 0:
        duplicate_config_file(
            src_file=config_src_file,
            dst_dir=config_dst_dir,
            count=config_copy_count,
            start_index=1,
            dry_run=dry_run
        )

    # 2) 再做原来的：各传感器文件夹移动 + 重命名
    folder_map = {
        "CAM_BACK_RIGHT": "camera_image_0",   # /cam0
        "CAM_FRONT_RIGHT": "camera_image_1",  # /cam1
        "CAM_FRONT_LEFT": "camera_image_2",   # /cam2
        "CAM_BACK_LEFT": "camera_image_3",    # /cam3
        "CAM_BACK": "camera_image_4",         # /cam4
        "CAM_FRONT": "camera_image_5",        # /cam5
        "LIDAR_TOP": "lidar_point_cloud_0",   # /rslidar_points
    }

    # 每个源文件夹要移动多少个文件（None=全部）
    num_files_per_folder = {
        "CAM_BACK_RIGHT": 100,
        "CAM_FRONT_RIGHT": 100,
        "CAM_FRONT_LEFT": 100,
        "CAM_BACK_LEFT": 100,
        "CAM_BACK": 100,
        "CAM_FRONT": 100,
        "LIDAR_TOP": 100,
    }

    start_index = 1

    total = 0
    for src_name, dst_name in folder_map.items():
        src_dir = src_root / src_name
        dst_dir = dst_root / dst_name
        n = num_files_per_folder.get(src_name, None)

        moved = move_and_rename(
            src_dir=src_dir,
            dst_dir=dst_dir,
            num_files=n,
            start_index=start_index,
            dry_run=dry_run
        )
        total += moved
        print(f"== {src_name} -> {dst_name}: moved {moved} files ==")

    print(f"\nDONE. Total moved: {total}")

if __name__ == "__main__":
    main()
