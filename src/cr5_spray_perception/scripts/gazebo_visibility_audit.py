#!/usr/bin/env python3
"""
V8.4 Visibility-Aware Truth Audit.

三级可见性判断:
  IN_FRUSTUM  — 3D 点投影落进图像范围
  FRONT_FACING — face outward normal 朝向相机
  VISIBLE      — IN_FRUSTUM + FRONT_FACING + 未被遮挡 + 足够投影面积

用法:
  python3 gazebo_visibility_audit.py [run_dir] [--group GROUP_ID]
"""

import sys, os, math, json, hashlib
from collections import defaultdict
import yaml
import numpy as np
import cv2

# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def euler_matrix(ai, aj, ak):
    """R = Rz(ak) @ Ry(aj) @ Rx(ai)"""
    Rx = np.array([[1, 0, 0], [0, math.cos(ai), -math.sin(ai)], [0, math.sin(ai), math.cos(ai)]])
    Ry = np.array([[math.cos(aj), 0, math.sin(aj)], [0, 1, 0], [-math.sin(aj), 0, math.cos(aj)]])
    Rz = np.array([[math.cos(ak), -math.sin(ak), 0], [math.sin(ak), math.cos(ak), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def rpy_from_rotation(R):
    sy = math.sqrt(R[0, 0]**2 + R[1, 0]**2)
    if sy > 1e-6:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0
    return roll, pitch, yaw


def T_from_xyz_rpy(xyz, rpy_deg):
    """Build 4x4 transform from xyz (m) and rpy (degrees)."""
    T = np.eye(4)
    T[:3, :3] = euler_matrix(*[math.radians(r) for r in rpy_deg])
    T[:3, 3] = xyz
    return T


def T_from_qt(qt):
    """qt = [qw, qx, qy, qz, tx, ty, tz] → 4x4."""
    T = np.eye(4)
    qw, qx, qy, qz = qt[:4]
    T[:3, :3] = np.array([
        [1-2*qy**2-2*qz**2, 2*qx*qy-2*qz*qw, 2*qx*qz+2*qy*qw],
        [2*qx*qy+2*qz*qw, 1-2*qx**2-2*qz**2, 2*qy*qz-2*qx*qw],
        [2*qx*qz-2*qy*qw, 2*qy*qz+2*qx*qw, 1-2*qx**2-2*qy**2],
    ])
    T[:3, 3] = qt[4:]
    return T


def look_at_rotation(cam_pos, target_pos):
    """Gazebo link convention: link +X = camera forward."""
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


def compute_camera_optical_pose(cam_pos, target_pos, roll_offset_deg=0.0):
    """Compute T_world_optical for a fixed camera with look-at."""
    R_wl, dist = look_at_rotation(cam_pos, target_pos)
    if roll_offset_deg:
        roll_r = math.radians(roll_offset_deg)
        Rx_roll = np.array([[1, 0, 0], [0, math.cos(roll_r), -math.sin(roll_r)],
                            [0, math.sin(roll_r), math.cos(roll_r)]])
        R_wl = R_wl @ Rx_roll
    # Link → optical: RPY = -π/2, 0, -π/2
    R_link_optical = euler_matrix(-math.pi/2, 0, -math.pi/2)[:3, :3]
    T_wo = np.eye(4)
    T_wo[:3, :3] = R_wl @ R_link_optical
    T_wo[:3, 3] = cam_pos
    return T_wo


# ---------------------------------------------------------------------------
# Face corner generation
# ---------------------------------------------------------------------------

def generate_face_corners_3d(panel_cfg, target_frame_origin=(0., 0., 0.)):
    """
    Generate 3D coordinates of face corners in target frame.
    Returns dict: face_name → {corners_3d, marker_corners, marker_ids, object_points}
    """
    main_body = np.array([0.34, 0.28, 0.24])  # from calibration_target.yaml

    faces = {}
    for face_name in ['front', 'back', 'left', 'right', 'top']:
        panel = panel_cfg.get(face_name, {})
        pose = panel.get('pose_target', {})
        xyz = pose.get('xyz', [0, 0, 0])
        rpy_deg = [math.degrees(r) if abs(r) < 10 else math.degrees(r)
                   for r in pose.get('rpy', [0, 0, 0])]
        rpy_rad = pose.get('rpy', [0, 0, 0])

        T_target_face = T_from_xyz_rpy(xyz, rpy_deg)

        # Physical canvas size
        canvas = panel.get('physical_canvas_m', [0.24, 0.18])

        # Face local coordinates: panel is in XY plane with +Z as outward normal
        hw, hh = canvas[0] / 2, canvas[1] / 2
        face_corners_local = np.array([
            [-hw, -hh, 0.],
            [hw, -hh, 0.],
            [hw, hh, 0.],
            [-hw, hh, 0.],
        ])

        # Transform corners to target frame
        face_corners_target = (T_target_face[:3, :3] @ face_corners_local.T).T + T_target_face[:3, 3]

        # Generate marker/tag object points
        pattern = panel.get('pattern', '')
        object_points = []
        marker_corners = {}  # marker_id → [4 corners in target frame]
        marker_ids = []

        if pattern == 'charuco':
            board = panel.get('board', {})
            sx, sy = board.get('squares_x', 8), board.get('squares_y', 6)
            sq = board.get('square_length_m', 0.027)
            ml = board.get('marker_length_m', 0.02)
            ids = board.get('marker_ids', [])

            # ChArUco board corners in OpenCV convention
            # Board origin at top-left corner of chessboard in face local frame
            # Face local: +X right, +Y down (looking at the face)
            # Board starts at (-hw_panel, -hh_panel) in face local XY

            # Marker centers and corners in face local frame
            # Board layout: (sx-1) × (sy-1) chessboard corners
            board_w = (sx - 1) * sq  # total chessboard width
            board_h = (sy - 1) * sq  # total chessboard height
            offset_x = -board_w / 2
            offset_y = -board_h / 2

            # For each marker, compute its 4 corners in face local
            # Marker centers are at integer square coordinates (col, row)
            # Marker size = ml, offset from square corner = (sq - ml) / 2
            half_offset = (sq - ml) / 2

            for mid_idx, mid in enumerate(ids):
                # Find which square this marker is in
                # ChArUco boards place markers inside chessboard squares
                # In OpenCV's convention, markers are placed in squares
                # Square (col, row) where col ∈ [0, sx-1), row ∈ [0, sy-1)
                # Marker ID placement follows the order in marker_ids
                col = mid_idx % (sx - 1)
                row = mid_idx // (sx - 1)

                # Square top-left corner in face local
                sq_x = offset_x + col * sq
                sq_y = offset_y + row * sq

                # Marker center = square center
                mx = sq_x + sq / 2
                my = sq_y + sq / 2

                # Marker 4 corners in face local (XY plane)
                hm = ml / 2
                m_corners_local = np.array([
                    [mx - hm, my - hm, 0.],
                    [mx + hm, my - hm, 0.],
                    [mx + hm, my + hm, 0.],
                    [mx - hm, my + hm, 0.],
                ])
                m_corners_target = (T_target_face[:3, :3] @ m_corners_local.T).T + T_target_face[:3, 3]
                marker_corners[mid] = m_corners_target
                marker_ids.append(mid)
                object_points.extend(m_corners_target.tolist())

            # Also generate Charuco chessboard corners (interpolated)
            charuco_corners_target = []
            for row in range(sy - 1):
                for col in range(sx - 1):
                    cx = offset_x + col * sq
                    cy = offset_y + row * sq
                    cp_local = np.array([cx, cy, 0.])
                    cp_target = T_target_face[:3, :3] @ cp_local + T_target_face[:3, 3]
                    charuco_corners_target.append(cp_target)
            object_points.extend([c.tolist() for c in charuco_corners_target])

        elif pattern in ('apriltag_grid', 'apriltag_single', 'aruco'):
            # Right face uses 'marker_centers_face_m', left/top use 'tag_centers_face_m'
            tags = panel.get('tag_centers_face_m',
                   panel.get('marker_centers_face_m', {}))
            tag_size = panel.get('tag_size_m', panel.get('marker_size_m', 0.07))
            ht = tag_size / 2
            for tid_str, center in tags.items():
                tid = int(tid_str)
                cx, cy, cz = center
                m_corners_local = np.array([
                    [cx - ht, cy - ht, 0.],
                    [cx + ht, cy - ht, 0.],
                    [cx + ht, cy + ht, 0.],
                    [cx - ht, cy + ht, 0.],
                ])
                m_corners_target = (T_target_face[:3, :3] @ m_corners_local.T).T + T_target_face[:3, 3]
                marker_corners[tid] = m_corners_target
                marker_ids.append(tid)
                object_points.extend(m_corners_target.tolist())

        faces[face_name] = {
            'T_target_face': T_target_face,
            'face_center_target': np.array(xyz),
            'outward_normal_target': T_target_face[:3, 2],  # local +Z after rotation
            'corners_target': face_corners_target,
            'marker_corners': marker_corners,
            'marker_ids': marker_ids,
            'object_points': np.array(object_points),
            'pattern': pattern,
            'canvas_m': canvas,
        }

    return faces


# ---------------------------------------------------------------------------
# Visibility classification
# ---------------------------------------------------------------------------

def classify_visibility(pts_3d_cam, corners_2d, img_w, img_h,
                        face_normal_world, cam_pos_world, face_center_world,
                        depth_map=None, fx=None, fy=None, cx=None, cy=None):
    """
    Classify visibility of a set of points.

    Returns:
        status: one of OUT_OF_FRAME, BACK_FACING, OCCLUDED,
                VISIBLE_OBLIQUE, VISIBLE_SMALL, VISIBLE_GOOD
        details: dict with metrics
    """
    n = len(pts_3d_cam)

    # 1. IN_FRUSTUM check
    in_frame = [(0 <= u < img_w and 0 <= v < img_h and z > 0)
                for (u, v), z in zip(corners_2d, pts_3d_cam[:, 2])]
    n_in_frame = sum(in_frame)
    if n_in_frame == 0:
        return 'OUT_OF_FRAME', {'n_in_frame': 0, 'n_total': n}

    # 2. FRONT_FACING check
    v_cam_to_face = cam_pos_world - face_center_world
    v_dist = np.linalg.norm(v_cam_to_face)
    if v_dist < 1e-6:
        v_dir = np.zeros(3)
    else:
        v_dir = v_cam_to_face / v_dist
    cos_incidence = float(np.dot(face_normal_world, v_dir))
    incidence_deg = math.degrees(math.acos(max(-1., min(1., cos_incidence))))

    # Outward normal should face camera: cos_incidence > 0
    if cos_incidence <= 0:
        return 'BACK_FACING', {
            'cos_incidence': cos_incidence,
            'incidence_deg': incidence_deg,
            'n_in_frame': n_in_frame,
        }

    # 3. Projected area
    in_frame_corners = [c for c, ok in zip(corners_2d, in_frame) if ok]
    if len(in_frame_corners) >= 3:
        hull = cv2.convexHull(np.array(in_frame_corners, dtype=np.float32))
        area_px2 = float(cv2.contourArea(hull))
    else:
        area_px2 = 0.0

    # Min edge length in pixels
    if len(in_frame_corners) >= 2:
        pts_arr = np.array(in_frame_corners)
        edges = np.linalg.norm(pts_arr - np.roll(pts_arr, 1, axis=0), axis=1)
        min_edge_px = float(np.min(edges))
    else:
        min_edge_px = 0.0

    # 4. Occlusion check using depth
    occluded = False
    depth_diff_mm = None
    if depth_map is not None and fx is not None:
        center_u = int(round(np.mean([c[0] for c in in_frame_corners])))
        center_v = int(round(np.mean([c[1] for c in in_frame_corners])))
        if 0 <= center_u < depth_map.shape[1] and 0 <= center_v < depth_map.shape[0]:
            z_depth_m = depth_map[center_v, center_u] * 0.001  # mm → m
            z_truth = float(np.mean(pts_3d_cam[in_frame, 2]))
            if z_depth_m > 0.01:
                depth_diff_mm = abs(z_truth - z_depth_m) * 1000
                if depth_diff_mm > 50:  # 50mm tolerance
                    occluded = True

    # 5. Final classification
    if occluded:
        status = 'OCCLUDED'
    elif incidence_deg > 75:
        status = 'VISIBLE_OBLIQUE'
    elif min_edge_px < 10 or area_px2 < 100:
        status = 'VISIBLE_SMALL'
    else:
        status = 'VISIBLE_GOOD'

    details = {
        'cos_incidence': cos_incidence,
        'incidence_deg': incidence_deg,
        'n_in_frame': n_in_frame,
        'n_total': n,
        'area_px2': area_px2,
        'min_edge_px': min_edge_px,
        'occluded': occluded,
        'depth_diff_mm': depth_diff_mm,
    }
    return status, details


# ---------------------------------------------------------------------------
# Main audit
# ---------------------------------------------------------------------------

def run_visibility_audit(run_dir, target_groups=None):
    """Run full visibility audit on a calibration run directory."""

    # Load target config
    target_yaml = os.path.join(
        os.path.dirname(__file__), '..', '..', 'cr5_spray_sim',
        'config', 'calibration', 'calibration_target.yaml')
    with open(target_yaml) as f:
        target_cfg = yaml.safe_load(f)
    panels = target_cfg['panels']

    # Load scene config (camera truth)
    scene_yaml = os.path.join(
        os.path.dirname(__file__), '..', '..', 'cr5_spray_sim',
        'config', 'simulation_scene.yaml')
    with open(scene_yaml) as f:
        scene_cfg = yaml.safe_load(f)

    # Compute camera optical poses
    cam_cfgs = scene_cfg['cameras']['cameras']
    look_target = scene_cfg['cameras']['target']
    tgt_pos = [look_target['x'], look_target['y'], look_target['z']]

    camera_poses = {}  # name → T_world_optical
    camera_names = []
    for cc in cam_cfgs:
        name = cc['name']
        camera_names.append(name)
        pos = [cc['position']['x'], cc['position']['y'], cc['position']['z']]
        roll = cc.get('roll_offset_deg', 0.0)
        T_wo = compute_camera_optical_pose(pos, tgt_pos, roll)
        camera_poses[name] = T_wo

    # Camera intrinsics (quality profile)
    fx = fy = 462.138
    cx, cy = 320., 240.
    img_w, img_h = 640, 480
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
    D = np.zeros(5)

    # Generate face geometry
    faces = generate_face_corners_3d(panels)

    # Load BA input for detection reference
    ba_input_path = os.path.join(run_dir, 'pipeline', 'ceres_ba_input_ba1.json')
    if os.path.exists(ba_input_path):
        with open(ba_input_path) as f:
            ba_data = json.load(f)
    else:
        ba_data = None

    # Target pose captures
    capture_configs = [
        # (label, xyz, rpy_deg)
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

    if target_groups is not None:
        capture_configs = [c for i, c in enumerate(capture_configs) if i in target_groups]

    # -------------------------------------------------------------------
    # Per-group audit
    # -------------------------------------------------------------------
    results = []

    for group_idx, (label, xyz, rpy_deg) in enumerate(capture_configs):
        T_world_target = T_from_xyz_rpy(xyz, rpy_deg)

        # Load depth for this group if available
        group_dir = os.path.join(
            run_dir.replace('/runs/', '/raw/'), 'groups',
            f'group_{group_idx:04d}')
        depth_maps = {}
        for cam_name in camera_names:
            depth_path = os.path.join(group_dir, cam_name, 'depth.npy')
            if os.path.exists(depth_path):
                depth_maps[cam_name] = np.load(depth_path)

        group_result = {
            'group_idx': group_idx,
            'label': label,
            'xyz': xyz,
            'rpy_deg': rpy_deg,
            'cameras': {},
        }

        for cam_name in camera_names:
            T_world_cam = camera_poses[cam_name]
            T_cam_world = np.linalg.inv(T_world_cam)
            T_cam_target = T_cam_world @ T_world_target
            cam_pos_world = T_world_cam[:3, 3]

            cam_result = {'faces': {}}

            for face_name, face_data in faces.items():
                T_target_face = face_data['T_target_face']
                T_world_face = T_world_target @ T_target_face
                face_center_world = T_world_face[:3, 3]
                face_normal_local = np.array([0., 0., 1.])
                face_normal_world = T_world_face[:3, :3] @ face_normal_local

                # Classify each marker's corners
                marker_statuses = {}
                for mid, corners_target in face_data['marker_corners'].items():
                    # Transform marker corners to world then camera frame
                    corners_world = (T_world_target[:3, :3] @ corners_target.T).T + T_world_target[:3, 3]
                    corners_cam = (T_cam_world[:3, :3] @ corners_world.T).T + T_cam_world[:3, 3]

                    # Project to image
                    rvec = np.zeros(3)  # points already in camera frame
                    tvec = np.zeros(3)
                    corners_2d, _ = cv2.projectPoints(
                        corners_cam.reshape(-1, 1, 3).astype(np.float32),
                        rvec, tvec, K, D)
                    corners_2d = corners_2d.reshape(-1, 2)

                    depth_map = depth_maps.get(cam_name)
                    status, details = classify_visibility(
                        corners_cam, corners_2d, img_w, img_h,
                        face_normal_world, cam_pos_world, face_center_world,
                        depth_map, fx, fy, cx, cy)

                    marker_statuses[mid] = {
                        'status': status,
                        'details': details,
                        'corners_2d': corners_2d.tolist(),
                    }

                # Aggregate face-level stats
                status_counts = defaultdict(int)
                visible_markers = []
                for mid, ms in marker_statuses.items():
                    status_counts[ms['status']] += 1
                    if ms['status'] in ('VISIBLE_GOOD', 'VISIBLE_SMALL', 'VISIBLE_OBLIQUE'):
                        visible_markers.append(mid)

                cam_result['faces'][face_name] = {
                    'marker_statuses': marker_statuses,
                    'status_counts': dict(status_counts),
                    'visible_marker_ids': sorted(visible_markers),
                    'n_markers_total': len(marker_statuses),
                    'n_markers_visible': len(visible_markers),
                }

            group_result['cameras'][cam_name] = cam_result

        results.append(group_result)

    return {
        'camera_names': camera_names,
        'face_names': list(faces.keys()),
        'camera_poses_truth': {n: camera_poses[n].tolist() for n in camera_names},
        'fx': fx, 'fy': fy, 'cx': cx, 'cy': cy,
        'img_w': img_w, 'img_h': img_h,
        'groups': results,
        'ba_data_available': ba_data is not None,
    }


def print_summary(audit):
    """Print human-readable summary."""
    print("=" * 80)
    print("  V8.4 VISIBILITY-AWARE TRUTH AUDIT")
    print("=" * 80)

    camera_names = audit['camera_names']
    face_names = audit['face_names']

    # Per-face expected maximum
    print("\n## Pattern Expected Maximums")
    print(f"  ChArUco front (8×6):  max 35 chessboard corners, 24 marker IDs")
    print(f"  ChArUco back (8×6):   max 35 chessboard corners, 24 marker IDs")
    print(f"  left (4 AprilTags):   max 16 corners")
    print(f"  right (4 ArUco):      max 16 corners")
    print(f"  top (1 AprilTag):     max 4 corners")

    print(f"\n## Per-Group Visibility Summary (marker-level)")
    print(f"{'Grp':>4s} {'Label':<16s}", end='')
    for cam in camera_names:
        print(f" | {cam[-5:]:>8s}", end='')
    print()
    print("-" * 80)

    for g in audit['groups']:
        print(f"{g['group_idx']:4d} {g['label']:<16s}", end='')
        for cam in camera_names:
            cam_data = g['cameras'].get(cam, {})
            face_info = cam_data.get('faces', {})
            visible_faces = []
            for fn in face_names:
                fd = face_info.get(fn, {})
                n_vis = fd.get('n_markers_visible', 0)
                n_tot = fd.get('n_markers_total', 0)
                if n_vis > 0:
                    visible_faces.append(f"{fn[0]}:{n_vis}/{n_tot}")
            info_str = ','.join(visible_faces) if visible_faces else 'NONE'
            print(f" | {info_str:>8s}", end='')
        print()

    # Per-camera per-face stats across all groups
    print(f"\n## Per-Camera Per-Face Aggregate (marker-level)")
    print(f"{'Camera':<20s} {'Face':<8s}", end='')
    for status in ['VISIBLE_GOOD', 'VISIBLE_SMALL', 'VISIBLE_OBLIQUE', 'OCCLUDED', 'BACK_FACING', 'OUT_OF_FRAME']:
        print(f" | {status:>16s}", end='')
    print()
    print("-" * 100)

    for cam in camera_names:
        for fn in face_names:
            counts = defaultdict(int)
            for g in audit['groups']:
                fd = g['cameras'].get(cam, {}).get('faces', {}).get(fn, {})
                sc = fd.get('status_counts', {})
                for s, c in sc.items():
                    counts[s] += c
            print(f"{cam:<20s} {fn:<8s}", end='')
            for status in ['VISIBLE_GOOD', 'VISIBLE_SMALL', 'VISIBLE_OBLIQUE', 'OCCLUDED', 'BACK_FACING', 'OUT_OF_FRAME']:
                print(f" | {counts.get(status, 0):>16d}", end='')
            print()

    # Comparison with actual detections if BA data available
    if audit.get('ba_data_available'):
        print(f"\n## Detection vs Visibility (marker-level)")
        print("  (BA data available — comparison requires detector output parsing)")
        print("  Run with --compare to enable full comparison.")

    print("\n" + "=" * 80)


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('run_dir', nargs='?',
                    default='/home/ydkj/cr5_data/calibration/runs/sim_v8_e2e_001')
    ap.add_argument('--group', type=int, default=None,
                    help='Single group index to audit')
    ap.add_argument('--groups', type=str, default=None,
                    help='Comma-separated group indices')
    ap.add_argument('--compare', action='store_true',
                    help='Compare with actual detector output')
    ap.add_argument('--json-out', type=str, default=None,
                    help='Save full audit to JSON')
    args = ap.parse_args()

    run_dir = args.run_dir
    if not os.path.isdir(run_dir):
        print(f"FATAL: run_dir not found: {run_dir}")
        sys.exit(1)

    target_groups = None
    if args.group is not None:
        target_groups = [args.group]
    elif args.groups is not None:
        target_groups = [int(x) for x in args.groups.split(',')]

    audit = run_visibility_audit(run_dir, target_groups)
    print_summary(audit)

    if args.json_out:
        import json as _json
        # Convert numpy types for JSON serialization
        class NpEncoder(_json.JSONEncoder):
            def default(self, obj):
                if isinstance(obj, np.integer):
                    return int(obj)
                if isinstance(obj, np.floating):
                    return float(obj)
                if isinstance(obj, np.ndarray):
                    return obj.tolist()
                return super().default(obj)

        with open(args.json_out, 'w') as f:
            _json.dump(audit, f, cls=NpEncoder, indent=2)
        print(f"\nSaved audit to {args.json_out}")

    return 0


if __name__ == '__main__':
    sys.exit(main())
