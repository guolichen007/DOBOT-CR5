#!/usr/bin/env python3
"""
V8.4 PnP Truth Audit — Per-group per-camera PnP error vs Gazebo truth.

输出 pnp_truth_audit.csv 和详细 JSON。
"""

import sys, os, math, json, csv
from collections import defaultdict
import yaml
import numpy as np
import cv2


# ---------------------------------------------------------------------------
# Geometry (mirrors export_simulation_truth.py)
# ---------------------------------------------------------------------------
def euler_matrix(ai, aj, ak):
    Rx = np.array([[1, 0, 0], [0, math.cos(ai), -math.sin(ai)], [0, math.sin(ai), math.cos(ai)]])
    Ry = np.array([[math.cos(aj), 0, math.sin(aj)], [0, 1, 0], [-math.sin(aj), 0, math.cos(aj)]])
    Rz = np.array([[math.cos(ak), -math.sin(ak), 0], [math.sin(ak), math.cos(ak), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def look_at_rotation(cam_pos, target_pos):
    cam = np.array(cam_pos, dtype=np.float64)
    tgt = np.array(target_pos, dtype=np.float64)
    d_raw = tgt - cam
    dist = float(np.linalg.norm(d_raw))
    d = d_raw / max(dist, 1e-9)
    world_z = np.array([0., 0., 1.])
    cam_x = d
    cam_y = np.cross(world_z, cam_x)
    ny = float(np.linalg.norm(cam_y))
    if ny < 1e-9:
        cam_y = np.array([0., 1., 0.])
    else:
        cam_y /= ny
    cam_z = np.cross(cam_x, cam_y)
    nz = float(np.linalg.norm(cam_z))
    if nz > 1e-9:
        cam_z /= nz
    return np.column_stack([cam_x, cam_y, cam_z]), dist


LINK_TO_OPTICAL_RPY = (-math.pi/2, 0, -math.pi/2)


def compute_camera_optical_pose(cam_pos, target_pos, roll_offset_deg=0.0):
    R_wl, _dist = look_at_rotation(cam_pos, target_pos)
    if roll_offset_deg:
        roll_r = math.radians(roll_offset_deg)
        Rx_roll = np.array([[1, 0, 0], [0, math.cos(roll_r), -math.sin(roll_r)],
                            [0, math.sin(roll_r), math.cos(roll_r)]])
        R_wl = R_wl @ Rx_roll
    R_link_optical = euler_matrix(*LINK_TO_OPTICAL_RPY)[:3, :3]
    T_wo = np.eye(4)
    T_wo[:3, :3] = R_wl @ R_link_optical
    T_wo[:3, 3] = cam_pos
    return T_wo


def T_from_xyz_rpy(xyz, rpy_deg):
    T = np.eye(4)
    T[:3, :3] = euler_matrix(*[math.radians(r) for r in rpy_deg])
    T[:3, 3] = xyz
    return T


def T_from_qt(qt):
    T = np.eye(4)
    qw, qx, qy, qz = qt[:4]
    T[:3, :3] = np.array([
        [1-2*qy**2-2*qz**2, 2*qx*qy-2*qz*qw, 2*qx*qz+2*qy*qw],
        [2*qx*qy+2*qz*qw, 1-2*qx**2-2*qz**2, 2*qy*qz-2*qx*qw],
        [2*qx*qz-2*qy*qw, 2*qy*qz+2*qx*qw, 1-2*qx**2-2*qy**2],
    ])
    T[:3, 3] = qt[4:]
    return T


def translation_error(T_est, T_truth):
    return float(np.linalg.norm(T_est[:3, 3] - T_truth[:3, 3])) * 1000  # mm


def rotation_error_deg(T_est, T_truth):
    R_diff = T_truth[:3, :3].T @ T_est[:3, :3]
    trace = np.clip((np.trace(R_diff) - 1) / 2, -1, 1)
    return float(math.degrees(math.acos(trace)))


def rpy_from_rotation(R):
    sy = math.sqrt(R[0, 0]**2 + R[1, 0]**2)
    if sy > 1e-6:
        return math.atan2(R[2, 1], R[2, 2]), math.atan2(-R[2, 0], sy), math.atan2(R[1, 0], R[0, 0])
    return math.atan2(-R[1, 2], R[1, 1]), math.atan2(-R[2, 0], sy), 0.0


def classify_face(x, y, z):
    if abs(x - 0.171) < 0.012:
        return 'front'
    if abs(x + 0.171) < 0.012:
        return 'back'
    if abs(y - 0.141) < 0.012:
        return 'left'
    if abs(y + 0.141) < 0.012:
        return 'right'
    if abs(z - 0.121) < 0.012:
        return 'top'
    return 'unknown'


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def run_pnp_audit(run_dir, output_csv=None, output_json=None):
    # Load BA input
    ba_path = os.path.join(run_dir, 'pipeline', 'ceres_ba_input_ba1.json')
    with open(ba_path) as f:
        ba = json.load(f)

    # Load scene config for camera truth
    scene_yaml = os.path.join(
        os.path.dirname(__file__), '..', '..', 'cr5_spray_sim',
        'config', 'simulation_scene.yaml')
    with open(scene_yaml) as f:
        scene_cfg = yaml.safe_load(f)

    cam_cfgs = scene_cfg['cameras']['cameras']
    look_target = scene_cfg['cameras']['target']
    tgt_pos = [look_target['x'], look_target['y'], look_target['z']]

    # Camera truth poses
    camera_truth = {}
    for cc in cam_cfgs:
        name = cc['name']
        pos = [cc['position']['x'], cc['position']['y'], cc['position']['z']]
        roll = cc.get('roll_offset_deg', 0.0)
        T_wo = compute_camera_optical_pose(pos, tgt_pos, roll)
        camera_truth[name] = T_wo

    camera_idx_to_name = {0: 'cam_front_left', 1: 'cam_front_right', 2: 'cam_rear'}
    camera_name_to_idx = {v: k for k, v in camera_idx_to_name.items()}

    # Camera intrinsics
    fx = fy = 462.138
    cx, cy = 320., 240.
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)

    # Target poses
    capture_configs = [
        ('01_center',      [0.68, 0.0, 0.60],  [0, 0, 0]),
        ('02_yaw_p15',     [0.68, 0.0, 0.60],  [0, 0, 15]),
        ('03_yaw_m15',     [0.68, 0.0, 0.60],  [0, 0, -15]),
        ('04_yaw_p25',     [0.68, 0.0, 0.60],  [0, 0, 25]),
        ('05_yaw_m25',     [0.68, 0.0, 0.60],  [0, 0, -25]),
        ('06_pitch_p10',   [0.68, 0.0, 0.60],  [0, 10, 0]),
        ('07_pitch_m10',   [0.68, 0.0, 0.60],  [0, -10, 0]),
        ('08_pitch_p18',   [0.68, 0.0, 0.60],  [0, 18, 0]),
        ('09_left',        [0.68, 0.06, 0.58], [0, 0, 10]),
        ('10_right',       [0.68, -0.06, 0.62], [0, 0, -10]),
        ('11_combo_pp',    [0.68, 0.04, 0.56], [0, 12, 15]),
        ('12_combo_mm',    [0.68, -0.04, 0.64], [0, -10, -15]),
        ('13_high',        [0.68, 0.0, 0.68],  [0, -5, 0]),
        ('14_low',         [0.68, 0.0, 0.50],  [0, 8, 0]),
    ]

    rows = []

    for obs in ba['observations']:
        cam_idx = obs['camera_idx']
        target_idx = obs['target_idx']
        cam_name = camera_idx_to_name[cam_idx]

        # Get object points and image points
        obj_flat = obs['obj_pts']
        img_flat = obs['img_pts']
        n_pts = len(obj_flat) // 3
        obj_pts = np.array(obj_flat).reshape(-1, 1, 3).astype(np.float64)
        img_pts = np.array(img_flat).reshape(-1, 1, 2).astype(np.float64)

        # Classify faces present in this observation
        faces = set()
        for i in range(n_pts):
            faces.add(classify_face(*obj_flat[i*3:i*3+3]))
        face_label = '+'.join(sorted(faces))
        is_multi_face = len(faces) > 1

        # Gazebo truth
        label, xyz, rpy_deg = capture_configs[target_idx]
        T_world_target = T_from_xyz_rpy(xyz, rpy_deg)
        T_world_cam = camera_truth[cam_name]
        T_cam_world = np.linalg.inv(T_world_cam)
        T_cam_target_truth = T_cam_world @ T_world_target

        # Run PnP
        try:
            success, rvec, tvec = cv2.solvePnP(
                obj_pts, img_pts, K, None,
                flags=cv2.SOLVEPNP_ITERATIVE)
            if not success:
                # Fallback to EPNP
                success, rvec, tvec = cv2.solvePnP(
                    obj_pts, img_pts, K, None,
                    flags=cv2.SOLVEPNP_EPNP)
        except Exception as e:
            rows.append({
                'group': target_idx, 'label': label,
                'camera': cam_name, 'faces': face_label,
                'multi_face': is_multi_face,
                'n_pts': n_pts,
                'PnP_T_mm': None, 'PnP_R_deg': None,
                'PnP_RMSE_px': None,
                'reproj_median_px': None,
                'status': f'PnP_FAILED: {e}',
            })
            continue

        T_est = np.eye(4)
        T_est[:3, :3] = cv2.Rodrigues(rvec)[0]
        T_est[:3, 3] = tvec.flatten()

        t_err = translation_error(T_est, T_cam_target_truth)
        r_err = rotation_error_deg(T_est, T_cam_target_truth)

        # Reprojection RMSE
        proj_pts, _ = cv2.projectPoints(obj_pts, rvec, tvec, K, None)
        residuals = np.linalg.norm(img_pts.reshape(-1, 2) - proj_pts.reshape(-1, 2), axis=1)
        rmse_px = float(np.sqrt(np.mean(residuals**2)))
        median_px = float(np.median(residuals))

        rows.append({
            'group': target_idx, 'label': label,
            'camera': cam_name, 'faces': face_label,
            'multi_face': is_multi_face,
            'n_pts': n_pts,
            'PnP_T_mm': round(t_err, 2),
            'PnP_R_deg': round(r_err, 3),
            'PnP_RMSE_px': round(rmse_px, 3),
            'reproj_median_px': round(median_px, 3),
            'status': 'OK',
        })

    # Summary statistics
    print("=" * 80)
    print("  V8.4 PNP TRUTH AUDIT")
    print("=" * 80)

    print(f"\n## Per-Camera PnP Error Summary")
    print(f"{'Camera':<20s} {'N':>5s} {'T_median':>10s} {'T_P95':>10s} {'R_median':>10s} {'R_P95':>10s} {'RMSE':>8s}")
    print("-" * 70)

    for cam_name in ['cam_front_left', 'cam_front_right', 'cam_rear']:
        cam_rows = [r for r in rows if r['camera'] == cam_name and r['status'] == 'OK']
        if not cam_rows:
            continue
        t_errs = sorted([r['PnP_T_mm'] for r in cam_rows])
        r_errs = sorted([r['PnP_R_deg'] for r in cam_rows])
        t_median = t_errs[len(t_errs)//2]
        t_p95 = t_errs[int(len(t_errs)*0.95)] if len(t_errs) > 1 else t_errs[0]
        r_median = r_errs[len(r_errs)//2]
        r_p95 = r_errs[int(len(r_errs)*0.95)] if len(r_errs) > 1 else r_errs[0]
        avg_rmse = np.mean([r['PnP_RMSE_px'] for r in cam_rows])

        print(f"{cam_name:<20s} {len(cam_rows):>5d} {t_median:>9.1f}mm {t_p95:>9.1f}mm "
              f"{r_median:>9.2f}° {r_p95:>9.2f}° {avg_rmse:>7.3f}px")

    # By face type
    print(f"\n## By Face Type")
    by_face = defaultdict(list)
    for r in rows:
        if r['status'] != 'OK':
            continue
        by_face[r['faces']].append(r)

    for face_label in sorted(by_face.keys()):
        f_rows = by_face[face_label]
        t_errs = [r['PnP_T_mm'] for r in f_rows]
        r_errs = [r['PnP_R_deg'] for r in f_rows]
        print(f"  {face_label:<30s} N={len(f_rows):>2d}  "
              f"T: median={np.median(t_errs):.1f}mm  max={np.max(t_errs):.1f}mm  "
              f"R: median={np.median(r_errs):.2f}°  max={np.max(r_errs):.2f}°")

    # Detail per group
    print(f"\n## Per-Group Detail")
    print(f"{'Grp':>4s} {'Label':<16s} {'Camera':<20s} {'Faces':<25s} {'N':>4s} "
          f"{'T(mm)':>8s} {'R(°)':>8s} {'RMSE(px)':>9s}")
    print("-" * 100)
    for r in rows:
        if r['status'] != 'OK':
            print(f"{r['group']:4d} {r['label']:<16s} {r['camera']:<20s} "
                  f"{r['faces']:<25s} {r['n_pts']:>4d} FAILED: {r['status']}")
        else:
            print(f"{r['group']:4d} {r['label']:<16s} {r['camera']:<20s} "
                  f"{r['faces']:<25s} {r['n_pts']:>4d} "
                  f"{r['PnP_T_mm']:>7.1f}  {r['PnP_R_deg']:>7.2f}  "
                  f"{r['PnP_RMSE_px']:>8.3f}")

    # Output
    if output_csv:
        with open(output_csv, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=[
                'group', 'label', 'camera', 'faces', 'multi_face', 'n_pts',
                'PnP_T_mm', 'PnP_R_deg', 'PnP_RMSE_px', 'reproj_median_px', 'status'])
            writer.writeheader()
            writer.writerows(rows)
        print(f"\nSaved CSV to {output_csv}")

    if output_json:
        import json as _json
        with open(output_json, 'w') as f:
            _json.dump(rows, f, indent=2)
        print(f"Saved JSON to {output_json}")

    return rows


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('run_dir', nargs='?',
                    default='/home/ydkj/cr5_data/calibration/runs/sim_v8_e2e_001')
    ap.add_argument('--csv', type=str, default=None)
    ap.add_argument('--json', type=str, default=None)
    args = ap.parse_args()

    run_pnp_audit(args.run_dir, args.csv, args.json)
    return 0


if __name__ == '__main__':
    sys.exit(main())
