"""
CR5 Calibration — Ceres C++ BA backend JSON I/O.

Converts CalibrationDataset to Ceres JSON input format.
Handles Brown-Conrady distortion parameters for the C++ backend.
"""
import json
import os
import subprocess
import numpy as np
from typing import Dict, List, Tuple, Optional

from .measurement import CalibrationDataset


def build_ceres_input(dataset: CalibrationDataset,
                       camera_poses: Optional[Dict[str, np.ndarray]] = None,
                       target_poses: Optional[Dict[int, np.ndarray]] = None,
                       options: Optional[dict] = None) -> Tuple[dict, List[str]]:
    """Build Ceres BA input JSON from CalibrationDataset.

    Features vs legacy build_ceres_input:
      - Passes raw pixel coordinates + optional distortion params
      - Per-observation composite weight
      - Distortion model: [k1,k2,p1,p2,k3] per observation
      - Staged optimization options

    Args:
        dataset: CalibrationDataset
        camera_poses: {cam_name: 4x4 T_rig_camera} initial camera poses
        target_poses: {group_id: 4x4 T_rig_target} initial target poses
        options: Ceres solver options dict

    Returns:
        (ceres_input_json_dict, camera_names_list)
    """
    if options is None:
        options = {}

    cam_names = sorted(dataset.camera_infos.keys())

    # ── Cameras JSON ──
    cameras_json = []
    for cam_name in cam_names:
        K = np.array(dataset.camera_infos[cam_name]["K"]).reshape(3, 3)

        if camera_poses and cam_name in camera_poses:
            from .geometry import T_to_qt
            qt = T_to_qt(camera_poses[cam_name])
        else:
            qt = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        cameras_json.append({
            "name": cam_name,
            "fx": float(K[0, 0]), "fy": float(K[1, 1]),
            "cx": float(K[0, 2]), "cy": float(K[1, 2]),
            "initial_pose": [float(v) for v in qt],
        })

    # ── Targets JSON ──
    targets_json = []
    group_ids = sorted(dataset.groups.keys())
    for gid in group_ids:
        if target_poses and gid in target_poses:
            from .geometry import T_to_qt
            qt = T_to_qt(target_poses[gid])
        else:
            qt = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        targets_json.append({
            "group_id": int(gid),
            "initial_pose": [float(v) for v in qt],
        })

    # ── Observations JSON ──
    obs_json = []
    for gid in group_ids:
        gdata = dataset.groups[gid]
        for cam_name in cam_names:
            if cam_name not in gdata:
                continue

            meas = gdata[cam_name]
            K = np.array(dataset.camera_infos[cam_name]["K"]).reshape(3, 3)
            D = dataset.camera_infos[cam_name].get("D", [0, 0, 0, 0, 0])

            cam_idx = cam_names.index(cam_name)
            tgt_idx = group_ids.index(gid)

            # Use raw pixels + distortion (V8: no pre-undistortion)
            obj_flat = []
            for c in meas.corners:
                obj_flat.extend([float(c.obj_pt_target[0]),
                                float(c.obj_pt_target[1]),
                                float(c.obj_pt_target[2])])
            img_flat = []
            for c in meas.corners:
                img_flat.extend([float(c.img_pt_raw[0]),
                                float(c.img_pt_raw[1])])

            # Composite weight
            w = float(np.mean([c.weight for c in meas.corners])) if meas.corners else 1.0

            obs_entry = {
                "camera_idx": cam_idx,
                "target_idx": tgt_idx,
                "fx": float(K[0, 0]), "fy": float(K[1, 1]),
                "cx": float(K[0, 2]), "cy": float(K[1, 2]),
                "obj_pts": obj_flat,
                "img_pts": img_flat,
                "weight": w,
            }

            # Add distortion if present
            if D and len(D) >= 5:
                obs_entry["distortion"] = [float(D[0]), float(D[1]),
                                           float(D[2]), float(D[3]), float(D[4])]
            else:
                obs_entry["distortion"] = [0.0, 0.0, 0.0, 0.0, 0.0]

            obs_json.append(obs_entry)

    # ── Options ──
    ceres_opts = {
        "max_iterations": options.get("max_iterations", 500),
        "fix_first_camera": True,
        "accept_degraded_quality": True,
        "huber_threshold_px": options.get("huber_threshold_px", 2.0),
        "fix_all_cameras": options.get("fix_all_cameras", False),
        "fix_all_targets": options.get("fix_all_targets", False),
        "use_raw_pixels": True,  # V8: C++ applies distortion internally
    }

    sigma_t = options.get("camera_prior_sigma_translation_m", -1)
    sigma_r = options.get("camera_prior_sigma_rotation_rad", -1)
    if sigma_t > 0:
        ceres_opts["camera_prior_sigma_translation_m"] = float(sigma_t)
    if sigma_r > 0:
        ceres_opts["camera_prior_sigma_rotation_rad"] = float(sigma_r)

    return {
        "cameras": cameras_json,
        "targets": targets_json,
        "observations": obs_json,
        "options": ceres_opts,
    }, cam_names


def run_ceres_ba(input_json: dict, output_dir: str,
                  label: str = "ba", timeout_seconds: int = 120
                  ) -> Tuple[Optional[dict], List[str]]:
    """Run Ceres BA subprocess.

    Finds ceres_ba_optimizer executable, writes input JSON,
    runs it, parses output.

    Returns:
        (ba_output_dict, error_list)
        ba_output=None on failure.
    """
    input_path = os.path.join(output_dir, f"ceres_ba_input_{label}.json")
    output_path = os.path.join(output_dir, f"ceres_ba_output_{label}.json")

    with open(input_path, "w") as f:
        json.dump(input_json, f, indent=2)

    # Find executable
    exe_path = None
    search_paths = [
        os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "..",
                     "devel", "lib", "cr5_spray_perception", "ceres_ba_optimizer"),
    ]
    try:
        import rospkg
        rp = rospkg.RosPack()
        pkg_path = rp.get_path("cr5_spray_perception")
        search_paths.insert(0, os.path.join(
            os.path.dirname(os.path.dirname(pkg_path)), "devel", "lib",
            "cr5_spray_perception", "ceres_ba_optimizer"))
    except Exception:
        pass

    for p in search_paths:
        if os.path.isfile(p) and os.access(p, os.X_OK):
            exe_path = p
            break

    if exe_path is None:
        return None, ["ceres_ba_optimizer not found. Build with: catkin_make"]

    try:
        result = subprocess.run(
            [exe_path, input_path, output_path],
            capture_output=True, text=True, timeout=timeout_seconds)
    except subprocess.TimeoutExpired:
        return None, [f"Ceres BA timed out (>{timeout_seconds}s)"]
    except Exception as e:
        return None, [f"Ceres BA subprocess error: {e}"]

    if result.returncode != 0:
        return None, [f"Ceres BA exit code {result.returncode}: {result.stderr[:500]}"]

    if not os.path.isfile(output_path):
        return None, [f"Output file not found: {output_path}"]

    with open(output_path) as f:
        output = json.load(f)

    return output, []


def parse_ceres_output(output: dict, cam_names: List[str]) -> Dict[str, np.ndarray]:
    """Parse Ceres output into T_rig_camera dict."""
    from .geometry import qt_to_T

    result = {}
    for cam in output.get("cameras", []):
        name = cam["name"]
        qt = cam["optimized_pose"]
        result[name] = qt_to_T(qt)
    return result


def build_extrinsics_yaml(T_rig_cameras: Dict[str, np.ndarray],
                           ba_stats: dict,
                           cam_names: List[str],
                           status: str,
                           method: str,
                           rig_frame: Optional[str] = None) -> dict:
    """Build standard initial_extrinsics.yaml dict (schema v2)."""
    import cv2
    from datetime import datetime

    first_cam = cam_names[0]
    if rig_frame is None:
        rig_frame = f"{first_cam}_color_optical_frame"

    cameras_dict = {}
    for cam_name in cam_names:
        T = T_rig_cameras[cam_name]
        T_cam_rig = np.linalg.inv(T)
        rvec = cv2.Rodrigues(T[:3, :3])[0].flatten().tolist()
        from .geometry import T_to_qt
        qt = T_to_qt(T)

        cameras_dict[cam_name] = {
            "optical_frame": f"{cam_name}_color_optical_frame",
            "T_rig_camera": [[float(v) for v in row] for row in T.tolist()],
            "T_camera_rig": [[float(v) for v in row] for row in T_cam_rig.tolist()],
            "T_rig_camera_rvec": [float(v) for v in rvec],
            "T_rig_camera_tvec": [float(T[0, 3]), float(T[1, 3]), float(T[2, 3])],
            "T_rig_camera_qt": [float(v) for v in qt],
        }

    return {
        "schema_version": 2,
        "calibration_id": f"v8_{datetime.now().strftime('%Y%m%d_%H%M%S')}",
        "generated_at": datetime.now().strftime("%Y-%m-%dT%H:%M:%S"),
        "method": method,
        "optimization_framework": "Ceres Solver (SE(3) LM, Huber 2px, Brown-Conrady distortion)",
        "status": status,
        "rig_frame": rig_frame,
        "rig_definition": f"first camera ({first_cam}) color optical frame, gauge-fixed at identity",
        "transform_contract": {
            "primary_transform": "T_rig_camera",
            "inverse_transform": "T_camera_rig",
            "primary_equation": "p_rig = T_rig_camera @ p_camera",
        },
        "ba_stats": ba_stats,
        "cameras": cameras_dict,
    }
