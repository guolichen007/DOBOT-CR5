#!/usr/bin/env python3
"""
多姿态数据集采集脚本 (V2 — rospy ServiceProxy + pose evidence).

在运行的 Gazebo 仿真会话中:
1. 读取 robustness_poses.yaml
2. 对每个姿态:
   a. 调用 set_calibration_target_pose.py 设置姿态
   b. 调用 /gazebo/get_model_state 验证实际位姿并等待稳定
   c. 触发 joint_capture_manager 采集 sync group
   d. 保存 pose_evidence.json (requested + actual pose)
   e. 生成 per-pose visible_union.ply
3. 整理数据到 pose_pX/ 目录结构
4. 生成 dataset_manifest.json 和 checksums.sha256
5. 验证深度图跨姿态唯一性

Usage (在已 source 仿真环境后):
  rosrun cr5_spray_sim capture_multi_pose_dataset.py \
    --poses robustness_poses.yaml \
    --output ~/cr5_data/robustness/
"""
import os, sys, json, yaml, time, math, hashlib, logging, argparse, subprocess, shutil

import rospy
from std_srvs.srv import Trigger, TriggerRequest
from gazebo_msgs.srv import GetModelState, GetModelStateRequest

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
logger = logging.getLogger("capture_multi_pose")

WS = os.path.join(os.path.dirname(__file__), "..")
_POSE_SETTER = os.path.join(WS, "scripts", "set_calibration_target_pose.py")

# 稳定参数
SETTLE_MIN_SAMPLES = 5
SETTLE_POS_THRESHOLD_MM = 0.5
SETTLE_ROT_THRESHOLD_DEG = 0.05
SETTLE_MAX_WAIT_S = 10.0
SETTLE_INTERVAL_S = 0.3


def sha256_file(path):
    if not path or not os.path.isfile(path):
        return None
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def sha256_dir_files(dir_path, extensions=(".png", ".npy", ".yaml", ".json")):
    """目录下所有匹配文件的 SHA256 (排序)."""
    results = {}
    if not os.path.isdir(dir_path):
        return results
    for root, _, files in sorted(os.walk(dir_path)):
        for fname in sorted(files):
            if any(fname.endswith(ext) for ext in extensions):
                fpath = os.path.join(root, fname)
                results[fpath] = sha256_file(fpath)
    return results


def load_poses(poses_yaml):
    with open(poses_yaml) as f:
        cfg = yaml.safe_load(f)
    return cfg.get("poses", [])


def set_target_pose(preset_name):
    """通过 subprocess 调用 set_calibration_target_pose.py."""
    cmd = [sys.executable, _POSE_SETTER, "--pose", preset_name]
    logger.info("  set_pose: %s", preset_name)
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
    if result.returncode != 0:
        raise RuntimeError(
            f"set_pose {preset_name} failed (exit={result.returncode}): "
            f"{result.stderr[:200]}")
    return result.stdout.strip()


def verify_and_wait_stable(model_name="simple_hanging_workpiece",
                            reference_frame="world"):
    """等待模型稳定, 返回 (actual_position, actual_orientation_xyzw, settle_info)."""
    rospy.wait_for_service("/gazebo/get_model_state", timeout=5.0)
    svc = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

    prev_pos = None
    prev_quat = None
    stable_count = 0
    samples = []

    t_start = time.time()
    while time.time() - t_start < SETTLE_MAX_WAIT_S:
        req = GetModelStateRequest()
        req.model_name = model_name
        req.relative_entity_name = reference_frame
        resp = svc(req)

        if not resp.success:
            raise RuntimeError(f"get_model_state failed: {resp.status_message}")

        pos = [resp.pose.position.x, resp.pose.position.y, resp.pose.position.z]
        quat = [resp.pose.orientation.x, resp.pose.orientation.y,
                resp.pose.orientation.z, resp.pose.orientation.w]
        samples.append({"pos": pos, "quat": quat, "t": time.time() - t_start})

        if prev_pos is not None:
            dx = pos[0] - prev_pos[0]
            dy = pos[1] - prev_pos[1]
            dz = pos[2] - prev_pos[2]
            dist_mm = math.sqrt(dx*dx + dy*dy + dz*dz) * 1000.0

            # 旋转变化 (近似)
            dot = abs(quat[0]*prev_quat[0] + quat[1]*prev_quat[1] +
                      quat[2]*prev_quat[2] + quat[3]*prev_quat[3])
            dot = min(dot, 1.0)
            rot_deg = math.degrees(2.0 * math.acos(dot))

            if dist_mm < SETTLE_POS_THRESHOLD_MM and rot_deg < SETTLE_ROT_THRESHOLD_DEG:
                stable_count += 1
                if stable_count >= SETTLE_MIN_SAMPLES:
                    logger.info("    stable after %.1fs (%d samples)",
                               time.time() - t_start, len(samples))
                    settle_info = {
                        "samples": len(samples),
                        "duration_s": round(time.time() - t_start, 2),
                        "final_pos_mm": [round(p * 1000.0, 2) for p in pos],
                        "stable_count": stable_count,
                        "status": "STABLE",
                    }
                    return pos, quat, settle_info
            else:
                stable_count = 0
        else:
            stable_count = 0

        prev_pos = pos
        prev_quat = quat
        time.sleep(SETTLE_INTERVAL_S)

    # timeout
    settle_info = {
        "samples": len(samples),
        "duration_s": round(time.time() - t_start, 2),
        "final_pos_mm": [round(p * 1000.0, 2) for p in (prev_pos or [0,0,0])],
        "stable_count": stable_count,
        "status": "TIMEOUT",
    }
    logger.warning("    settle TIMEOUT after %.1fs (stable=%d/%d)",
                   time.time() - t_start, stable_count, SETTLE_MIN_SAMPLES)
    return prev_pos, prev_quat, settle_info


def trigger_capture():
    """触发 joint_capture_manager 采集. 返回 (group_dir, response_message)."""
    rospy.wait_for_service("/joint_capture_manager/capture_sync_group", timeout=10.0)
    svc = rospy.ServiceProxy("/joint_capture_manager/capture_sync_group", Trigger)

    req = TriggerRequest()
    resp = svc(req)

    if not resp.success:
        raise RuntimeError(f"capture_sync_group failed: {resp.message}")

    # 从 message 解析 GROUP_DIR
    group_dir = None
    for line in resp.message.split("\n"):
        line = line.strip()
        if line.startswith("GROUP_DIR:"):
            group_dir = line.split(":", 1)[1].strip()
            break

    if not group_dir or not os.path.isdir(group_dir):
        raise RuntimeError(
            f"capture_sync_group succeeded but GROUP_DIR not found in message: "
            f"{resp.message[:200]}")

    return group_dir, resp.message


def default_target_pose_from_scene():
    """从 simulation_scene.yaml 读取基准目标位姿."""
    try:
        from cr5_spray_sim.scene_config import get_base_target_pose
        return get_base_target_pose()
    except Exception:
        return (0.72, 0.0, 0.62)


def main():
    parser = argparse.ArgumentParser(description="多姿态数据集采集 V2")
    parser.add_argument("--poses", "-p", required=True,
                        help="robustness_poses.yaml 路径")
    parser.add_argument("--output", "-o", required=True,
                        help="输出根目录")
    parser.add_argument("--model", default="simple_hanging_workpiece",
                        help="Gazebo model name")
    args = parser.parse_args()

    if not os.path.isfile(_POSE_SETTER):
        logger.error("set_calibration_target_pose.py not found: %s", _POSE_SETTER)
        sys.exit(1)

    poses = load_poses(args.poses)
    if not poses:
        logger.error("No poses found in %s", args.poses)
        sys.exit(1)

    os.makedirs(args.output, exist_ok=True)
    logger.info("多姿态采集: %d poses → %s", len(poses), args.output)

    # 默认目标位姿 (基准)
    default_pos = default_target_pose_from_scene()

    manifest_entries = []
    checksums = {}
    # 按相机收集所有 depth SHA 用于唯一性验证
    per_camera_depth_shas = {
        "cam_front_left": {},
        "cam_front_right": {},
        "cam_rear": {},
    }

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

        # 2. 验证实际位姿并等待稳定
        try:
            actual_pos, actual_quat, settle_info = verify_and_wait_stable(
                model_name=args.model)
        except RuntimeError as e:
            logger.error("    stabilize failed: %s", e)
            continue

        # 3. 触发采集
        try:
            group_dir, cap_msg = trigger_capture()
        except RuntimeError as e:
            logger.error("    capture failed: %s", e)
            continue

        logger.info("    captured: %s → %s", pose_id, os.path.basename(group_dir))

        # 4. 整理数据到 pose 目录
        pose_dir = os.path.join(args.output, f"pose_{pose_id.lower()}")
        pose_groups_dir = os.path.join(pose_dir, "groups")
        os.makedirs(pose_groups_dir, exist_ok=True)

        dst_group = os.path.join(pose_groups_dir, "group_0000")
        if os.path.exists(dst_group):
            shutil.rmtree(dst_group)
        shutil.move(group_dir, dst_group)

        # 5. 保存 pose_evidence.json
        evidence = {
            "pose_id": pose_id,
            "preset": preset,
            "category": category,
            "description": desc,
            "capture_order": idx,
            "timestamp": rospy.Time.now().to_sec(),
            "model_name": args.model,
            "default_base_position": list(default_pos),
            "requested_pose": {
                "preset": preset,
            },
            "actual_pose": {
                "position_xyz": actual_pos,
                "orientation_xyzw": actual_quat,
            },
            "settle": settle_info,
        }
        evidence_path = os.path.join(pose_dir, "pose_evidence.json")
        with open(evidence_path, "w") as f:
            json.dump(evidence, f, indent=2, default=str)

        # 6. 保存 pose_manifest.json
        pose_manifest = {
            "pose_id": pose_id,
            "dataset_root": args.output,
            "group_dir": f"pose_{pose_id.lower()}/groups/group_0000",
            "group_manifest_sha256": sha256_file(
                os.path.join(dst_group, "group_manifest.json")),
        }
        with open(os.path.join(pose_dir, "pose_manifest.json"), "w") as f:
            json.dump(pose_manifest, f, indent=2)

        # 7. 收集 checksums
        group_files = sha256_dir_files(dst_group)
        for fpath, sha in group_files.items():
            rel_path = os.path.relpath(fpath, args.output)
            checksums[rel_path] = sha

            # 按相机收集 depth SHA
            for cam in ["cam_front_left", "cam_front_right", "cam_rear"]:
                if f"pose_{pose_id.lower()}/groups/group_0000/{cam}/depth.npy" == rel_path:
                    per_camera_depth_shas[cam][pose_id] = sha

        # evidence checksum
        ev_rel = os.path.relpath(evidence_path, args.output)
        checksums[ev_rel] = sha256_file(evidence_path)

        manifest_entries.append({
            "pose_id": pose_id,
            "preset": preset,
            "category": category,
            "description": desc,
            "pose_dir": f"pose_{pose_id.lower()}",
            "group_dir": f"pose_{pose_id.lower()}/groups/group_0000",
            "evidence_path": ev_rel,
            "actual_position_xyz": [round(v, 6) for v in actual_pos],
            "actual_orientation_xyzw": [round(v, 6) for v in actual_quat],
            "group_manifest_sha256": pose_manifest["group_manifest_sha256"],
        })

    # ── 写入 dataset manifest ──
    manifest_path = os.path.join(args.output, "dataset_manifest.json")
    manifest = {
        "schema": "cr5_robustness_dataset_v2",
        "poses_yaml": os.path.abspath(args.poses),
        "captured_poses": len(manifest_entries),
        "expected_poses": len(poses),
        "entries": manifest_entries,
        "depth_uniqueness": {},
    }
    with open(manifest_path, "w") as f:
        json.dump(manifest, f, indent=2)
    logger.info("Manifest: %s", manifest_path)

    # ── 写入 checksums ──
    checksum_path = os.path.join(args.output, "checksums.sha256")
    with open(checksum_path, "w") as f:
        for path in sorted(checksums.keys()):
            f.write(f"{checksums[path]}  {path}\n")
    logger.info("Checksums: %s (%d files)", checksum_path, len(checksums))

    # ── 深度图唯一性验证 ──
    for cam, pose_shas in per_camera_depth_shas.items():
        unique_count = len(set(pose_shas.values()))
        n_poses = len(pose_shas)
        manifest["depth_uniqueness"][cam] = {
            "unique": unique_count,
            "expected": n_poses,
        }
        if unique_count < n_poses and n_poses > 1:
            logger.error("DEPTH UNIQUENESS FAIL: %s: %d unique / %d poses",
                        cam, unique_count, n_poses)
            # 找出重复
            seen = {}
            for pid, sha in pose_shas.items():
                if sha in seen:
                    logger.error("  DUPLICATE: %s == %s", pid, seen[sha])
                else:
                    seen[sha] = pid
        else:
            logger.info("Depth uniqueness OK: %s: %d/%d unique",
                       cam, unique_count, n_poses)

    # 更新 manifest 含 depth_uniqueness
    with open(manifest_path, "w") as f:
        json.dump(manifest, f, indent=2)

    # ── 验证 checksums 可校验 ──
    logger.info("Verifying checksums...")
    result = subprocess.run(
        ["sha256sum", "-c", checksum_path],
        capture_output=True, text=True, cwd=args.output,
    )
    if result.returncode == 0:
        logger.info("checksums verification: PASS")
    else:
        logger.error("checksums verification: FAIL")
        logger.error(result.stderr[:500] if result.stderr else result.stdout[:500])

    logger.info("Done. %d/%d poses captured.", len(manifest_entries), len(poses))


if __name__ == "__main__":
    rospy.init_node("capture_multi_pose_dataset", anonymous=True)
    main()
