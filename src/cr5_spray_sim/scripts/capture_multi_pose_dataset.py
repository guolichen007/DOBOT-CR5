#!/usr/bin/env python3
"""
多姿态数据集采集脚本.

在运行的 Gazebo 仿真会话中:
1. 读取 robustness_poses.yaml
2. 对每个姿态:
   a. 调用 set_calibration_target_pose.py 设置姿态
   b. 等待 Gazebo 稳定
   c. 触发 joint_capture_manager 采集 sync group
3. 整理数据到 pose_pX/ 目录结构
4. 生成 dataset_manifest.json 和 checksums.sha256

Usage (在已 source 仿真环境后):
  rosrun cr5_spray_sim capture_multi_pose_dataset.py \
    --poses robustness_poses.yaml \
    --output ~/cr5_data/robustness/
"""
import os, sys, json, yaml, time, hashlib, logging, argparse, subprocess, shutil

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
logger = logging.getLogger("capture_multi_pose")

WS = os.path.join(os.path.dirname(__file__), "..")
_POSE_SETTER = os.path.join(WS, "scripts", "set_calibration_target_pose.py")
_CAPTURE_SERVICE = "/joint_capture_manager/capture_sync_group"
_SETTLE_WAIT_S = 2.0  # Gazebo 稳定等待时间


def sha256_file(path):
    if not path or not os.path.isfile(path):
        return None
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def set_target_pose(preset_name):
    """通过 subprocess 调用 set_calibration_target_pose.py."""
    cmd = [sys.executable, _POSE_SETTER, "--pose", preset_name]
    logger.info("  set_pose: %s", preset_name)
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
    if result.returncode != 0:
        raise RuntimeError(f"set_pose {preset_name} failed (exit={result.returncode}): "
                           f"{result.stderr[:200]}")
    # 等待 Gazebo 物理稳定
    time.sleep(_SETTLE_WAIT_S)
    return result.stdout.strip()


def trigger_capture():
    """触发 joint_capture_manager 采集一次 sync group."""
    logger.info("  capture trigger...")
    result = subprocess.run(
        ["rosservice", "call", _CAPTURE_SERVICE, "{}"],
        capture_output=True, text=True, timeout=60,
    )
    if result.returncode != 0:
        raise RuntimeError(f"capture service failed: {result.stderr[:200]}")
    return result.stdout.strip()


def load_poses(poses_yaml):
    with open(poses_yaml) as f:
        cfg = yaml.safe_load(f)
    return cfg.get("poses", [])


def main():
    parser = argparse.ArgumentParser(description="多姿态数据集采集")
    parser.add_argument("--poses", "-p", required=True,
                        help="robustness_poses.yaml 路径")
    parser.add_argument("--output", "-o", required=True,
                        help="输出根目录")
    parser.add_argument("--settle-wait", type=float, default=2.0,
                        help="姿态设置后稳定等待时间 (秒)")
    args = parser.parse_args()

    global _SETTLE_WAIT_S
    _SETTLE_WAIT_S = args.settle_wait

    if not os.path.isfile(_POSE_SETTER):
        logger.error("set_calibration_target_pose.py not found: %s", _POSE_SETTER)
        sys.exit(1)

    poses = load_poses(args.poses)
    if not poses:
        logger.error("No poses found in %s", args.poses)
        sys.exit(1)

    os.makedirs(args.output, exist_ok=True)
    logger.info("多姿态采集: %d poses → %s", len(poses), args.output)

    manifest_entries = []
    checksums = {}

    for idx, pose in enumerate(poses):
        pose_id = pose["id"]
        preset = pose["preset"]
        desc = pose.get("description", "")
        category = pose.get("category", "production")

        logger.info("── [%d/%d] Pose %s (%s) ──", idx + 1, len(poses), pose_id, preset)
        logger.info("    %s", desc)

        # 1. 设置姿态
        try:
            set_target_pose(preset)
        except RuntimeError as e:
            logger.error("    set_pose failed: %s", e)
            continue

        # 2. 触发采集
        try:
            trigger_capture()
        except RuntimeError as e:
            logger.error("    capture failed: %s", e)
            continue

        # 捕获成功
        logger.info("    captured: %s", pose_id)

        manifest_entries.append({
            "pose_id": pose_id,
            "preset": preset,
            "category": category,
            "description": desc,
            "capture_order": idx,
        })

    # ── 整理数据 ──
    # joint_capture_manager 输出在 <output>/groups/group_XXXX/
    # 需要找到捕获管理器实际使用的 output 目录
    # 由于我们通过 rosservice 触发, 数据在 capture_manager 启动时的 output_dir 中
    # 这里假设 capture_manager 启动时指定了 --output <args.output>/_capture_temp
    # 如果数据直接就在 args.output 下, 则跳过 move

    # 查找最新捕获的 groups
    groups_dir = os.path.join(args.output, "groups")
    if not os.path.isdir(groups_dir):
        # 尝试 _capture_temp
        groups_dir = os.path.join(args.output, "_capture_temp", "groups")

    if os.path.isdir(groups_dir):
        group_dirs = sorted([
            d for d in os.listdir(groups_dir)
            if d.startswith("group_") and os.path.isdir(os.path.join(groups_dir, d))
        ])
        logger.info("Found %d captured groups", len(group_dirs))

        for i, entry in enumerate(manifest_entries):
            if i >= len(group_dirs):
                logger.warning("  no group data for pose %s", entry["pose_id"])
                continue
            pose_dir = os.path.join(args.output, f"pose_{entry['pose_id'].lower()}")
            pose_groups_dir = os.path.join(pose_dir, "groups")
            os.makedirs(pose_groups_dir, exist_ok=True)

            src = os.path.join(groups_dir, group_dirs[i])
            dst = os.path.join(pose_groups_dir, "group_0000")
            if os.path.exists(dst):
                shutil.rmtree(dst)
            shutil.move(src, dst)
            entry["group_dir"] = f"pose_{entry['pose_id'].lower()}/groups/group_0000"
            logger.info("  %s → %s", group_dirs[i], entry["group_dir"])

            # 计算深度图 checksums
            for cam in ["cam_front_left", "cam_front_right", "cam_rear"]:
                depth_path = os.path.join(dst, cam, "depth.npy")
                ch = sha256_file(depth_path)
                if ch:
                    checksums[f"{entry['pose_id']}/{cam}/depth.npy"] = ch
    else:
        logger.warning("No groups directory found at %s", groups_dir)
        logger.info("Data may need manual organization to pose_pX/ directories")

    # ── 写入 manifest ──
    manifest_path = os.path.join(args.output, "dataset_manifest.json")
    manifest = {
        "schema": "cr5_robustness_dataset_v1",
        "poses_yaml": os.path.abspath(args.poses),
        "captured_poses": len(manifest_entries),
        "expected_poses": len(poses),
        "entries": manifest_entries,
    }
    with open(manifest_path, "w") as f:
        json.dump(manifest, f, indent=2)
    logger.info("Manifest: %s", manifest_path)

    # ── 写入 checksums ──
    checksum_path = os.path.join(args.output, "checksums.sha256")
    with open(checksum_path, "w") as f:
        for path, sha in sorted(checksums.items()):
            f.write(f"{sha}  {path}\n")
    logger.info("Checksums: %s (%d files)", checksum_path, len(checksums))

    # ── 验证每个姿态的深度图 SHA 不同 ──
    depth_shas = set()
    for entry in manifest_entries:
        for cam in ["cam_front_left", "cam_front_right", "cam_rear"]:
            key = f"{entry['pose_id']}/{cam}/depth.npy"
            if key in checksums:
                depth_shas.add(checksums[key])
    logger.info("Unique depth SHAs: %d (should match poses*cameras)", len(depth_shas))

    logger.info("Done. %d/%d poses captured.", len(manifest_entries), len(poses))


if __name__ == "__main__":
    main()
