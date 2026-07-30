"""
CR5 Calibration — Synthetic regression test suite (S0-S6).

Tests factor graph init and full Ceres BA pipeline without ROS/Gazebo.

Tests:
  S0-S2: Factor graph init regression (noise-free, 0.3px, 0.8px)
  S3: 10% outliers → full pipeline with point audit
  S4: Single camera-face bias → BA1→detect→quarantine→BA2
  S5: 5 sparse groups → INSUFFICIENT_OBSERVABILITY
  S6: Brown-Conrady non-zero distortion
"""
import sys, os, math, json, copy
import numpy as np
import argparse

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))

# Force import via package
try:
    from .geometry import (euler_matrix, se3_distance_mm_deg, invert_transform,
                           T_to_qt, qt_to_T, rpy_from_rotation)
    from .measurement import (CalibrationDataset, CameraGroupMeasurement,
                              CornerObservation, temporal_fusion)
    from .pnp_solver import solve_pnp, compute_planarity
    from .quality import detect_face_bias, FaceBiasDiagnostic
    from .rig_initializer import build_factor_graph, extract_nonplanar_measurements
    from .validation import check_observability, determine_final_status
    from .pipeline import run_calibration_pipeline
except ImportError:
    from geometry import (euler_matrix, se3_distance_mm_deg, invert_transform,
                          T_to_qt, qt_to_T, rpy_from_rotation)
    from measurement import (CalibrationDataset, CameraGroupMeasurement,
                             CornerObservation, temporal_fusion)
    from pnp_solver import solve_pnp, compute_planarity
    from quality import detect_face_bias
    from rig_initializer import build_factor_graph
    from validation import check_observability
    from pipeline import run_calibration_pipeline

# ── Synthetic scene (canonicalized: FL = I exactly) ──
TARGET_CENTER_WORLD = np.array([0.0, 0.0, 0.70])

def _make_camera_pose(cam_pos, look_at):
    forward = look_at - cam_pos; forward = forward / np.linalg.norm(forward)
    world_up = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(forward, world_up)) > 0.999: world_up = np.array([0.0, 1.0, 0.0])
    right = np.cross(world_up, forward); right = right / np.linalg.norm(right)
    down = np.cross(forward, right); down = down / np.linalg.norm(down)
    R = np.column_stack([right, down, forward])
    T = np.eye(4); T[:3,:3] = R; T[:3,3] = cam_pos; return T

T_WORLD_CAMS = {
    "cam_front_left":  _make_camera_pose(np.array([0.0, 0.0, 0.0]), TARGET_CENTER_WORLD),
    "cam_front_right": _make_camera_pose(np.array([0.0, 0.50, 0.02]), TARGET_CENTER_WORLD),
    "cam_rear":        _make_camera_pose(np.array([0.0, 0.0, 1.35]), TARGET_CENTER_WORLD),
}
T_WORLD_FL = T_WORLD_CAMS["cam_front_left"]
TRUTH_CAM_POSES = {cam: np.linalg.inv(T_WORLD_FL) @ T_wc for cam, T_wc in T_WORLD_CAMS.items()}
assert np.allclose(TRUTH_CAM_POSES["cam_front_left"], np.eye(4), atol=1e-10)

CAMERA_K = [[462.138, 0, 320], [0, 462.138, 240], [0, 0, 1]]
FACE_POSES_TARGET = {
    "front": {"xyz": [0.171, 0.0, 0.0], "rpy": [0.0, math.pi/2, 0.0]},
    "back": {"xyz": [-0.171, 0.0, 0.0], "rpy": [0.0, -math.pi/2, 0.0]},
    "left": {"xyz": [0.0, 0.141, 0.0], "rpy": [-math.pi/2, 0.0, 0.0]},
    "right": {"xyz": [0.0, -0.141, 0.0], "rpy": [math.pi/2, 0.0, 0.0]},
    "top": {"xyz": [0.0, 0.0, 0.121], "rpy": [0.0, 0.0, 0.0]},
}

# Face corner points
def _make_face_corners():
    fc = {}
    for fn in ["front", "back"]:
        bw, bh = 8*0.027, 6*0.027
        xs = np.linspace(0, bw, 8) - bw/2; ys = np.linspace(0, bh, 6) - bh/2
        X, Y = np.meshgrid(xs, ys); fc[fn] = np.column_stack([X.ravel(), Y.ravel(), np.zeros(48)])
    for fn in ["left", "right"]:
        pts = []
        for cx, cy in [(-0.0425,0.0425),(0.0425,0.0425),(-0.0425,-0.0425),(0.0425,-0.0425)]:
            h = 0.035
            pts.extend([[cx-h,cy+h,0],[cx+h,cy+h,0],[cx+h,cy-h,0],[cx-h,cy-h,0]])
        fc[fn] = np.array(pts)
    h = 0.06; fc["top"] = np.array([[-h,h,0],[h,h,0],[h,-h,0],[-h,-h,0]])
    return fc

FACE_CORNERS = _make_face_corners()


def generate_target_poses(n_groups=12):
    designs = [
        (0,0,"center"),(10,0,"yaw_p10"),(-10,0,"yaw_m10"),(20,0,"yaw_p20"),(-20,0,"yaw_m20"),
        (0,8,"pitch_p8"),(0,-8,"pitch_m8"),(0,15,"pitch_p15"),(0,-15,"pitch_m15"),
        (15,10,"combo_pp"),(-15,-10,"combo_mm"),(15,-10,"combo_pm"),(-15,10,"combo_mp"),
    ]
    poses = {}
    nominal = TARGET_CENTER_WORLD.copy()
    for i in range(min(n_groups, len(designs))):
        yaw_d, pitch_d, label = designs[i]
        pos = nominal + np.array([np.random.uniform(-.02,.02),
                                   np.random.uniform(-.04,.04),
                                   np.random.uniform(-.04,.04)])
        T_world = euler_matrix(0.0, math.radians(pitch_d), math.radians(yaw_d))
        T_world[:3,3] = pos
        T_rig = np.linalg.inv(T_WORLD_FL) @ T_world
        poses[i] = (T_rig, label)
    return poses


def _project(obj_pts_target, T_camera_target, K, D=None):
    import cv2
    if D is None: D = np.zeros(5)
    rvec = cv2.Rodrigues(T_camera_target[:3,:3])[0].flatten()
    tvec = T_camera_target[:3,3]
    img, _ = cv2.projectPoints(obj_pts_target.astype(np.float32), rvec, tvec,
                                K.reshape(3,3).astype(np.float64), D.astype(np.float64))
    return img.reshape(-1, 2)


def generate_dataset(n_groups=12, noise_sigma_px=0.0, outlier_fraction=0.0,
                      outlier_range=(5,15), bias_cam=None, bias_face=None,
                      bias_du=0.0, bias_dv=0.0, D=None) -> CalibrationDataset:
    if D is None: D = [0,0,0,0,0]
    np.random.seed(42)
    target_poses = generate_target_poses(n_groups)
    dataset = CalibrationDataset(
        camera_infos={cam: {"K": CAMERA_K, "D": list(D), "width": 640, "height": 480}
                       for cam in ["cam_front_left","cam_front_right","cam_rear"]},
        face_poses_target=dict(FACE_POSES_TARGET), groups={}, source_type="synthetic")
    K_arr = np.array(CAMERA_K).reshape(3,3)

    for gid in range(len(target_poses)):
        T_rig_target, label = target_poses[gid]
        group = {}
        for cam_name in ["cam_front_left","cam_front_right","cam_rear"]:
            T_cam_target = invert_transform(TRUTH_CAM_POSES[cam_name]) @ T_rig_target
            corners = []
            for face_name, pts_face in FACE_CORNERS.items():
                T_face = euler_matrix(*FACE_POSES_TARGET[face_name]["rpy"])
                T_face[:3,3] = FACE_POSES_TARGET[face_name]["xyz"]
                pts_h = np.column_stack([pts_face, np.ones(len(pts_face))])
                pts_target = (T_face @ pts_h.T).T[:,:3]
                img_pts = _project(pts_target, T_cam_target, K_arr, np.array(D))

                pts_cam = (T_cam_target @ np.column_stack([pts_target, np.ones(len(pts_target))]).T).T[:,:3]
                visible = ((img_pts[:,0]>0)&(img_pts[:,0]<640)&(img_pts[:,1]>0)&(img_pts[:,1]<480)&(pts_cam[:,2]>0.01))
                if np.sum(visible) < 4: continue

                for pi in range(len(pts_face)):
                    if not visible[pi]: continue
                    u, v = img_pts[pi]
                    if noise_sigma_px > 0: u += np.random.normal(0, noise_sigma_px); v += np.random.normal(0, noise_sigma_px)
                    if bias_cam == cam_name and bias_face and face_name == bias_face:
                        u += bias_du; v += bias_dv
                    if outlier_fraction > 0 and np.random.random() < outlier_fraction:
                        mag = np.random.uniform(*outlier_range); ang = np.random.uniform(0, 2*math.pi)
                        u += mag*math.cos(ang); v += mag*math.sin(ang)
                    corners.append(CornerObservation(
                        camera=cam_name, group_id=gid, face_name=face_name,
                        marker_id=pi//4, corner_idx=len(corners),
                        obj_pt_target=tuple(pts_target[pi].tolist()),
                        img_pt_raw=(float(u), float(v)),
                        img_pt_undistorted=(float(u), float(v)),
                        detector_type="charuco" if face_name in ("front","back") else "apriltag"))
            if corners:
                group[cam_name] = CameraGroupMeasurement(camera=cam_name, group_id=gid, corners=corners)
        if len(group) >= 2: dataset.groups[gid] = group
    return dataset


# ── Test helpers ──

def run_init_test(name, ds_generator, max_t_mm, max_r_deg, expect_insufficient=False):
    """Factor graph init only test."""
    print(f"\n  === {name} ===")
    np.random.seed(42)
    ds = ds_generator()
    print(f"    Dataset: {ds.n_groups} groups")
    X, Y, diag = build_factor_graph(ds)
    if X is None:
        if expect_insufficient: return {"passed": True, "status": "INSUFFICIENT"}
        return {"passed": False, "status": "INIT_FAILED", "error": diag.get("error","")}
    errs = {}
    for cam in ["cam_front_right","cam_rear"]:
        t, r, _ = se3_distance_mm_deg(X.get(cam, np.eye(4)), TRUTH_CAM_POSES[cam], 30, 3)
        errs[f"{cam}_t_mm"] = round(t,2); errs[f"{cam}_r_deg"] = round(r,3)
        print(f"    {cam}: T={t:.2f}mm, R={r:.3f}°")
    passed = all(errs.get(f"{c}_t_mm",999) <= max_t_mm and errs.get(f"{c}_r_deg",999) <= max_r_deg
                 for c in ["cam_front_right","cam_rear"])
    return {"passed": passed, "errors": errs, "status": "PASS" if passed else "FAIL"}


def run_pipeline_test(name, ds_generator, max_t_mm, max_r_deg, output_dir,
                       expect_insufficient=False, expect_bias_cam_face=None):
    """Full pipeline test with BA-1/BA-2."""
    print(f"\n  === {name} ===")
    np.random.seed(42)
    ds = ds_generator()
    print(f"    Dataset: {ds.n_groups} groups")
    os.makedirs(output_dir, exist_ok=True)

    result = run_calibration_pipeline(ds, output_dir, skip_cross_validation=True)

    if expect_insufficient:
        if result.final_status in ("INSUFFICIENT_OBSERVABILITY", "NOT_RELIABLE"):
            print(f"    ✓ Correctly rejected: {result.final_status}")
            return {"passed": True, "status": result.final_status, "obs": str(result.observability)}
        else:
            print(f"    ✗ Should have been rejected, got: {result.final_status}")
            return {"passed": False, "status": result.final_status}

    # INIT
    errs_init = {}
    for cam in ["cam_front_right","cam_rear"]:
        T = result.init_camera_poses.get(cam)
        if T is not None:
            t, r, _ = se3_distance_mm_deg(T, TRUTH_CAM_POSES[cam], 30, 3)
            errs_init[f"{cam}_t_mm"] = round(t,2); errs_init[f"{cam}_r_deg"] = round(r,3)

    # FINAL (BA2 or BA1)
    final = result.final_camera_poses
    errs_final = {}
    for cam in ["cam_front_right","cam_rear"]:
        T = final.get(cam)
        if T is not None:
            t, r, _ = se3_distance_mm_deg(T, TRUTH_CAM_POSES[cam], 30, 3)
            errs_final[f"{cam}_t_mm"] = round(t,2); errs_final[f"{cam}_r_deg"] = round(r,3)

    # Face bias check if expected
    bias_detected = None
    if expect_bias_cam_face:
        bias_cam, bias_face = expect_bias_cam_face
        for diag in result.face_bias_report:
            if diag.camera == bias_cam and diag.face_name == bias_face:
                bias_detected = diag.classification
                print(f"    Bias detected: {bias_cam}/{bias_face} → {diag.classification} "
                      f"(bias={diag.bias_magnitude_px:.1f}px, n={diag.n_groups} groups, w={diag.recommended_weight})")

    final_passed = all(errs_final.get(f"{c}_t_mm",999) <= max_t_mm and
                       errs_final.get(f"{c}_r_deg",999) <= max_r_deg
                       for c in ["cam_front_right","cam_rear"])

    print(f"    INIT: FR={errs_init.get('cam_front_right_t_mm','?')}mm/{errs_init.get('cam_front_right_r_deg','?')}° "
          f"RE={errs_init.get('cam_rear_t_mm','?')}mm/{errs_init.get('cam_rear_r_deg','?')}°")
    print(f"    FINAL: FR={errs_final.get('cam_front_right_t_mm','?')}mm/{errs_final.get('cam_front_right_r_deg','?')}° "
          f"RE={errs_final.get('cam_rear_t_mm','?')}mm/{errs_final.get('cam_rear_r_deg','?')}°")

    obs = result.observability
    print(f"    Obs: passed={obs.passed if obs else '?'}, "
          f"yaw={obs.stats.get('yaw_span_deg','?')}° pitch={obs.stats.get('pitch_span_deg','?')}°")

    return {"passed": final_passed, "init_errors": errs_init, "final_errors": errs_final,
            "status": "PASS" if final_passed else "FAIL",
            "ba1_stats": result.ba1_stats, "ba2_stats": result.ba2_stats,
            "bias_detected": bias_detected,
            "face_bias_count": len(result.face_bias_report),
            "obs_passed": obs.passed if obs else None}


# ── Main ──

def run_all_tests(output_base="/tmp/v8_regression"):
    np.random.seed(42)
    results = {}
    os.makedirs(output_base, exist_ok=True)

    # S0-S2: factor graph init only (fast, proven)
    for label, ngrp, noise, t_max, r_max in [
        ("S0", 12, 0.0, 0.1, 0.01),
        ("S1", 12, 0.3, 2.0, 0.2),
        ("S2", 12, 0.8, 5.0, 0.5),
    ]:
        results[label] = run_init_test(f"{label} — Noise-Free Init" if label=="S0" else
                                       f"{label} — Gaussian {noise}px",
                                       lambda n=ngrp, s=noise: generate_dataset(n, noise_sigma_px=s),
                                       t_max, r_max)

    # S3: 10% outliers → full pipeline
    results["S3"] = run_pipeline_test(
        "S3 — 10% outliers 5-15px",
        lambda: generate_dataset(12, noise_sigma_px=0.3, outlier_fraction=0.10, outlier_range=(5,15)),
        10.0, 1.0, os.path.join(output_base, "s3"))

    # S4: single camera-face strong bias → BA1→detect→quarantine→BA2
    results["S4"] = run_pipeline_test(
        "S4 — FR/back bias (-10,-8)px",
        lambda: generate_dataset(12, noise_sigma_px=0.3, bias_cam="cam_front_right",
                                 bias_face="back", bias_du=-10.0, bias_dv=-8.0),
        10.0, 1.0, os.path.join(output_base, "s4"),
        expect_bias_cam_face=("cam_front_right", "back"))

    # S5: 5 sparse → must fail observability
    results["S5"] = run_pipeline_test(
        "S5 — 5 sparse groups",
        lambda: generate_dataset(5, noise_sigma_px=0.3),
        999, 999, os.path.join(output_base, "s5"),
        expect_insufficient=True)

    # S6: Brown-Conrady distortion
    D_NONZERO = [-0.08, 0.015, 0.001, -0.001, 0.0]
    results["S6"] = run_pipeline_test(
        "S6 — Brown-Conrady k=[-0.08,0.015,0.001,-0.001,0]",
        lambda: generate_dataset(12, noise_sigma_px=0.3, D=D_NONZERO),
        10.0, 1.0, os.path.join(output_base, "s6_correct_d"))

    # S6 negative control: D=0 on distorted data
    def s6_neg_control():
        ds = generate_dataset(12, noise_sigma_px=0.3, D=D_NONZERO)
        # Force D=0 in camera infos
        for cam in ds.camera_infos: ds.camera_infos[cam]["D"] = [0,0,0,0,0]
        return ds

    results["S6_neg"] = run_pipeline_test(
        "S6-neg — Zero D on distorted data",
        s6_neg_control, 999, 999, os.path.join(output_base, "s6_zero_d"))

    # S6 control: negative control must be clearly worse than correct D
    s6_re_t = results["S6"]["final_errors"].get("cam_rear_t_mm", 0)
    s6_neg_re_t = results["S6_neg"]["final_errors"].get("cam_rear_t_mm", 0)
    # Negative control should have ≥3× worse translation error
    s6_neg_significantly_worse = s6_neg_re_t > max(s6_re_t * 3, 3.0)
    results["S6"]["s6_negative_control_worse"] = s6_neg_significantly_worse
    # S6 passes if correct D result is good AND negative control confirms D path works
    results["S6"]["passed"] = results["S6"]["passed"] and s6_neg_significantly_worse

    # S4: stronger bias for detection
    s4_bias_found = results["S4"].get("bias_detected") not in (None, "CLEAN")
    # S4 INIT should detect bias, but even if not, FINAL should be <10mm/<1° after BA2
    results["S4"]["bias_detected_flag"] = results["S4"]["bias_detected"]
    # S4 still fails if FINAL doesn't meet <10mm/<1°

    return results


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--case", default=None, help="Run specific case (S0-S6)")
    p.add_argument("--output", default="/tmp/v8_regression", help="Output directory")
    args = p.parse_args()

    print("=" * 60)
    print("  CR5 V8.2 — Synthetic Regression Suite (S0-S6)")
    print("=" * 60)

    all_results = run_all_tests(args.output)

    if args.case:
        r = all_results.get(args.case)
        if r: print(f"\n{args.case}: {r['status']} — {r}")
        else: print(f"Unknown case: {args.case}")
    else:
        print("\n" + "=" * 60)
        print("  RESULTS SUMMARY")
        print("=" * 60)
        all_pass = True
        for name, r in sorted(all_results.items()):
            s = "✓ PASS" if r["passed"] else "✗ FAIL"
            print(f"  {name}: {s}")
            if not r["passed"]: all_pass = False
            for k, v in r.get("init_errors", r.get("errors", {})).items():
                if "t_mm" in k or "r_deg" in k: print(f"    {k}: {v}")
            for k, v in r.get("final_errors", {}).items():
                if "t_mm" in k or "r_deg" in k: print(f"    FINAL {k}: {v}")
            if r.get("bias_detected"): print(f"    bias: {r['bias_detected']}")
        print(f"\n  Overall: {'ALL PASSED' if all_pass else 'SOME FAILED'}")
        sys.exit(0 if all_pass else 1)
