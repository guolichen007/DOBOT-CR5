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


def parse_group_dir_from_trigger_message(message):
    """从 joint_capture_manager 的 Trigger response.message 解析 GROUP_DIR.

    真实格式: GROUP_DIR:/abs/path/group_0000|SYNC_GROUP_3_OF_3_PASS: 3/3 cameras
    只提取 | 前的绝对路径.
    """
    prefix = "GROUP_DIR:"
    if prefix not in message:
        raise ValueError(f"GROUP_DIR prefix missing in message: {message[:200]}")

    payload = message[message.index(prefix) + len(prefix):]
    group_dir = payload.split("|", 1)[0].strip()

    if not group_dir:
        raise ValueError("empty GROUP_DIR in message")
    if not os.path.isdir(group_dir):
        raise FileNotFoundError(f"GROUP_DIR not a directory: {group_dir}")
    return group_dir


def trigger_capture():
    """触发 joint_capture_manager 采集. 返回 (group_dir, response_message)."""
    rospy.wait_for_service("/joint_capture_manager/capture_sync_group", timeout=10.0)
    svc = rospy.ServiceProxy("/joint_capture_manager/capture_sync_group", Trigger)

    req = TriggerRequest()
    resp = svc(req)

    if not resp.success:
        raise RuntimeError(f"capture_sync_group failed: {resp.message}")

    group_dir = parse_group_dir_from_trigger_message(resp.message)
    return group_dir, resp.message


def default_target_pose_from_scene():
    """从 simulation_scene.yaml 读取基准目标位姿."""
    try:
        from cr5_spray_sim.scene_config import get_base_target_pose
        return get_base_target_pose()
    except Exception:
        return (0.72, 0.0, 0.62)


def get_requested_pose_from_preset(preset_name):
    """从 set_calibration_target_pose.py 的 POSE_DELTAS 解析请求位姿."""
    # 直接导入避免 subprocess 开销
    sys.path.insert(0, WS)
    from cr5_spray_sim.scene_config import get_base_target_pose
    base = get_base_target_pose()
    # 读取 POSE_DELTAS (与 set_calibration_target_pose.py 共享定义)
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "set_pose", os.path.join(WS, "scripts", "set_calibration_target_pose.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    delta = mod.POSE_DELTAS.get(preset_name, {})
    return {
        "preset": preset_name,
        "position_xyz": [base[0] + delta.get("dx", 0),
                         base[1] + delta.get("dy", 0),
                         base[2] + delta.get("dz", 0)],
        "rpy_deg": delta.get("rp_deg", [0, 0, 0]),
        "base_xyz": list(base),
    }


def generate_visible_gt(pose_dir, pose_evidence_path):
    """为单个姿态生成 visible_union.ply."""
    gt_script = os.path.join(WS, "..", "cr5_spray_sim", "scripts", "generate_visible_gt.py")
    if not os.path.isfile(gt_script):
        logger.warning("generate_visible_gt.py not found at %s, skipping", gt_script)
        return False, None

    out_dir = os.path.join(pose_dir, "visible_gt")
    cmd = [
        sys.executable, gt_script,
        "--dataset", pose_dir,
        "--output-dir", out_dir,
        "--model-pose-json", pose_evidence_path,
    ]
    logger.info("  generating visible GT: %s", " ".join(cmd))
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=300)
    if result.returncode != 0:
        logger.error("  visible GT generation failed: %s", result.stderr[:300])
        return False, None

    gt_ply = os.path.join(out_dir, "visible_union.ply")
    if not os.path.isfile(gt_ply):
        logger.error("  visible_union.ply not generated")
        return False, None

    return True, gt_ply


def main():
    parser = argparse.ArgumentParser(description="多姿态数据集采集 V3")
    parser.add_argument("--poses", "-p", required=True)
    parser.add_argument("--output", "-o", required=True)
    parser.add_argument("--model", default="simple_hanging_workpiece")
    parser.add_argument("--skip-visible-gt", action="store_true",
                        help="跳过 per-pose visible GT 生成")
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

    fatal_errors = []
    manifest_entries = []
    checksums = {}
    per_camera_depth_shas = {
        "cam_front_left": {}, "cam_front_right": {}, "cam_rear": {},
    }

    for idx, pose in enumerate(poses):
        pose_id = pose["id"]
        preset = pose["preset"]
        category = pose.get("category", "production")
        desc = pose.get("description", "")

        logger.info("── [%d/%d] Pose %s (%s) ──", idx + 1, len(poses), pose_id, preset)
        logger.info("    %s", desc)

        # 1. 获取请求位姿
        try:
            requested = get_requested_pose_from_preset(preset)
        except Exception as e:
            logger.error("    get_requested_pose failed: %s", e)
            if category == "production":
                fatal_errors.append(f"{pose_id}: get_requested_pose: {e}")
            continue

        # 2. 设置姿态
        try:
            set_target_pose(preset)
        except RuntimeError as e:
            logger.error("    set_pose failed: %s", e)
            if category == "production":
                fatal_errors.append(f"{pose_id}: set_pose: {e}")
            continue

        # 3. 验证稳定
        try:
            actual_before, actual_quat_before, settle_info = verify_and_wait_stable(
                model_name=args.model)
        except RuntimeError as e:
            logger.error("    stabilize failed: %s", e)
            if category == "production":
                fatal_errors.append(f"{pose_id}: stabilize: {e}")
            continue

        if settle_info.get("status") != "STABLE":
            logger.error("    settle TIMEOUT, skipping capture")
            if category == "production":
                fatal_errors.append(f"{pose_id}: settle timeout")
            continue

        # 计算请求 vs 实际误差
        req_pos = requested["position_xyz"]
        dx = actual_before[0] - req_pos[0]
        dy = actual_before[1] - req_pos[1]
        dz = actual_before[2] - req_pos[2]
        trans_err_mm = math.sqrt(dx*dx + dy*dy + dz*dz) * 1000.0

        # 旋转误差 (请求角度 vs 实际四元数)
        from scipy.spatial.transform import Rotation
        req_rpy_deg = requested.get("rpy_deg", [0, 0, 0])
        req_quat = Rotation.from_euler("xyz", [math.radians(r) for r in req_rpy_deg]).as_quat()
        dot = abs(sum(a*b for a, b in zip(req_quat, actual_quat_before)))
        dot = min(dot, 1.0)
        rot_err_deg = math.degrees(2.0 * math.acos(dot))

        logger.info("    trans_err=%.2fmm rot_err=%.2fdeg", trans_err_mm, rot_err_deg)

        if trans_err_mm > 2.0 or rot_err_deg > 0.2:
            logger.error("    POSE TOLERANCE EXCEEDED: trans=%.2fmm rot=%.2fdeg",
                        trans_err_mm, rot_err_deg)
            if category == "production":
                fatal_errors.append(
                    f"{pose_id}: tolerance exceeded "
                    f"(trans={trans_err_mm:.2f}mm rot={rot_err_deg:.2f}deg)")
            continue

        # 4. 触发采集
        try:
            group_dir, cap_msg = trigger_capture()
        except RuntimeError as e:
            logger.error("    capture failed: %s", e)
            if category == "production":
                fatal_errors.append(f"{pose_id}: capture: {e}")
            continue

        # 采集后位姿
        try:
            actual_after, actual_quat_after, _ = verify_and_wait_stable(
                model_name=args.model)
            adx = actual_after[0] - actual_before[0]
            ady = actual_after[1] - actual_before[1]
            adz = actual_after[2] - actual_before[2]
            capture_motion_mm = math.sqrt(adx*adx + ady*ady + adz*adz) * 1000.0
            dot_a = abs(sum(a*b for a, b in zip(actual_quat_before, actual_quat_after)))
            dot_a = min(dot_a, 1.0)
            capture_rot_deg = math.degrees(2.0 * math.acos(dot_a))
        except Exception:
            actual_after, actual_quat_after = actual_before, actual_quat_before
            capture_motion_mm, capture_rot_deg = 0, 0

        logger.info("    captured: %s", os.path.basename(group_dir))

        # 5. 整理数据
        pose_dir = os.path.join(args.output, f"pose_{pose_id.lower()}")
        pose_groups_dir = os.path.join(pose_dir, "groups")
        os.makedirs(pose_groups_dir, exist_ok=True)
        dst_group = os.path.join(pose_groups_dir, "group_0000")
        if os.path.exists(dst_group):
            shutil.rmtree(dst_group)
        shutil.move(group_dir, dst_group)

        # 6. 写入完整的 pose_evidence.json
        evidence = {
            "pose_id": pose_id,
            "preset": preset,
            "category": category,
            "description": desc,
            "capture_order": idx,
            "timestamp": rospy.Time.now().to_sec(),
            "model_name": args.model,
            "requested_pose": {
                "preset": preset,
                "position_xyz": [round(v, 6) for v in req_pos],
                "orientation_rpy_deg": req_rpy_deg,
                "base_xyz": [round(v, 6) for v in requested.get("base_xyz", [])],
            },
            "actual_pose_before_capture": {
                "position_xyz": [round(v, 6) for v in actual_before],
                "orientation_xyzw": [round(v, 6) for v in actual_quat_before],
            },
            "actual_pose_after_capture": {
                "position_xyz": [round(v, 6) for v in actual_after],
                "orientation_xyzw": [round(v, 6) for v in actual_quat_after],
            },
            "translation_error_mm": round(trans_err_mm, 3),
            "rotation_error_deg": round(rot_err_deg, 4),
            "capture_motion_mm": round(capture_motion_mm, 3),
            "capture_rotation_deg": round(capture_rot_deg, 4),
            "settle": settle_info,
            "group_manifest_sha256": sha256_file(
                os.path.join(dst_group, "group_manifest.json")),
        }
        evidence_path = os.path.join(pose_dir, "pose_evidence.json")
        with open(evidence_path, "w") as f:
            json.dump(evidence, f, indent=2, default=str)

        # 7. 生成 per-pose visible GT
        if not args.skip_visible_gt:
            gt_ok, gt_path = generate_visible_gt(pose_dir, evidence_path)
            if not gt_ok and category == "production":
                fatal_errors.append(f"{pose_id}: visible GT generation failed")
            if gt_ok:
                gt_rel = os.path.relpath(gt_path, args.output)
                checksums[gt_rel] = sha256_file(gt_path)

        # 8. Checksums
        group_files = sha256_dir_files(dst_group)
        for fpath, sha in group_files.items():
            rel_path = os.path.relpath(fpath, args.output)
            checksums[rel_path] = sha
            for cam in ["cam_front_left", "cam_front_right", "cam_rear"]:
                if f"pose_{pose_id.lower()}/groups/group_0000/{cam}/depth.npy" == rel_path:
                    per_camera_depth_shas[cam][pose_id] = sha

        ev_rel = os.path.relpath(evidence_path, args.output)
        checksums[ev_rel] = sha256_file(evidence_path)

        manifest_entries.append({
            "pose_id": pose_id, "preset": preset, "category": category,
            "description": desc,
            "pose_dir": f"pose_{pose_id.lower()}",
            "group_dir": f"pose_{pose_id.lower()}/groups/group_0000",
            "evidence_path": ev_rel,
            "actual_position_xyz": [round(v, 6) for v in actual_before],
            "translation_error_mm": round(trans_err_mm, 3),
            "rotation_error_deg": round(rot_err_deg, 4),
            "group_manifest_sha256": evidence["group_manifest_sha256"],
        })

    # ── Checksum 验证 ──
    checksum_path = os.path.join(args.output, "checksums.sha256")
    with open(checksum_path, "w") as f:
        for path, sha_val in sorted(checksums.items()):
            f.write(f"{sha_val}  {path}\n")
    result = subprocess.run(["sha256sum", "-c", checksum_path],
                            capture_output=True, text=True, cwd=args.output)
    if result.returncode != 0:
        fatal_errors.append("checksum verification failed")

    # ── 深度唯一性 ──
    for cam, pose_shas in per_camera_depth_shas.items():
        unique_count = len(set(pose_shas.values()))
        n_poses = len(pose_shas)
        if n_poses > 1 and unique_count < n_poses:
            fatal_errors.append(f"{cam}: depth SHA duplicate ({unique_count}/{n_poses} unique)")

    # ── Manifest ──
    manifest = {
        "schema": "cr5_robustness_dataset_v3",
        "poses_yaml": os.path.abspath(args.poses),
        "captured_poses": len(manifest_entries),
        "expected_poses": len(poses),
        "entries": manifest_entries,
        "depth_uniqueness": {
            cam: {"unique": len(set(vals.values())), "expected": len(vals)}
            for cam, vals in per_camera_depth_shas.items()
        },
    }
    with open(os.path.join(args.output, "dataset_manifest.json"), "w") as f:
        json.dump(manifest, f, indent=2)

    # ── 最终判定 ──
    if len(manifest_entries) != len(poses):
        fatal_errors.append(
            f"captured={len(manifest_entries)} != expected={len(poses)}")

    if fatal_errors:
        logger.error("FATAL ERRORS (%d):", len(fatal_errors))
        for e in fatal_errors:
            logger.error("  %s", e)
        failure_report = {"fatal_errors": fatal_errors}
        with open(os.path.join(args.output, "capture_failure_report.json"), "w") as f:
            json.dump(failure_report, f, indent=2)
        sys.exit(1)

    logger.info("Done. %d/%d poses captured. All checks PASS.",
                len(manifest_entries), len(poses))


if __name__ == "__main__":
    rospy.init_node("capture_multi_pose_dataset", anonymous=True)
    main()
