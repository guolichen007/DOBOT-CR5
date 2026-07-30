"""
CR5 Calibration — Observation quality assessment.

- Angle-aware incidence classification
- Per-face systematic bias detection (post BA-1)
- Composite weight computation
"""
import math
import numpy as np
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass
from enum import Enum

from .geometry import invert_transform


class FaceQuality(Enum):
    GOOD = "GOOD"        # incidence < 35°
    NORMAL = "NORMAL"    # 35-50°
    WEAK = "WEAK"        # 50-60°
    REJECT = "REJECT"    # > 60°


INCIDENCE_WEIGHTS = {
    FaceQuality.GOOD: 1.0,
    FaceQuality.NORMAL: 0.7,
    FaceQuality.WEAK: 0.3,
    FaceQuality.REJECT: 0.0,
}


def compute_incidence_angle(face_normal_world: np.ndarray,
                            camera_position: np.ndarray,
                            face_center: np.ndarray) -> float:
    """Compute incidence angle (degrees) between face normal and camera view direction.

    The incidence angle is the angle between the face normal and the vector from
    the face center toward the camera. Values near 0° mean the camera is looking
    straight at the face (best quality). Values near 90° mean extreme oblique view.

    Args:
        face_normal_world: 3D normal vector of the face (in world or target frame)
        camera_position: position of the camera optical center
        face_center: position of the face center

    Returns:
        incidence angle in degrees [0, 180]
    """
    view_dir = camera_position - face_center
    view_norm = np.linalg.norm(view_dir)
    if view_norm < 1e-9:
        return 90.0
    view_dir = view_dir / view_norm

    normal_norm = np.linalg.norm(face_normal_world)
    if normal_norm < 1e-9:
        return 90.0
    normal = face_normal_world / normal_norm

    # cos(theta) = |dot(normal, view_dir)|
    # incidence = angle between normal and view direction
    cos_angle = abs(np.dot(normal, view_dir))
    cos_angle = max(-1.0, min(1.0, cos_angle))
    return float(math.degrees(math.acos(cos_angle)))


def classify_face_quality(incidence_angle_deg: float) -> FaceQuality:
    """Classify observation quality based on incidence angle."""
    if incidence_angle_deg < 35.0:
        return FaceQuality.GOOD
    elif incidence_angle_deg < 50.0:
        return FaceQuality.NORMAL
    elif incidence_angle_deg < 60.0:
        return FaceQuality.WEAK
    else:
        return FaceQuality.REJECT


def compute_face_quality_from_pose(face_poses_target: Dict[str, dict],
                                    T_rig_camera: np.ndarray,
                                    T_rig_target: np.ndarray) -> Dict[str, Tuple[FaceQuality, float]]:
    """Compute incidence angle and quality factor for all faces from a given camera pose.

    Args:
        face_poses_target: {face_name: {xyz: [3], rpy: [3]}} face geometry
        T_rig_camera: 4x4 T_rig_camera (camera pose in rig frame)
        T_rig_target: 4x4 T_rig_target (target pose in rig frame)

    Returns:
        {face_name: (FaceQuality, weight_factor)}
    """
    from .geometry import euler_matrix, transform_points

    # Camera position in target frame
    T_target_camera = invert_transform(T_rig_target) @ T_rig_camera
    cam_pos_target = T_target_camera[:3, 3]

    result = {}
    for face_name, face_pose in face_poses_target.items():
        xyz = face_pose["xyz"]
        rpy = face_pose["rpy"]

        # Face transform in target frame
        T_target_face = euler_matrix(rpy[0], rpy[1], rpy[2])
        T_target_face[:3, 3] = xyz

        # Face normal in target frame (face local +z = outward normal)
        face_normal_local = np.array([0.0, 0.0, 1.0])
        face_normal_target = T_target_face[:3, :3] @ face_normal_local

        # Face center in target frame
        face_center = np.array(xyz, dtype=np.float64)

        incidence = compute_incidence_angle(face_normal_target, cam_pos_target, face_center)
        quality = classify_face_quality(incidence)
        result[face_name] = (quality, INCIDENCE_WEIGHTS[quality])

    return result


@dataclass
class FaceBiasDiagnostic:
    """Diagnostic for a specific camera-face pair after BA."""
    camera: str
    face_name: str
    mean_du: float           # mean residual in u (pixels)
    mean_dv: float           # mean residual in v (pixels)
    median_bias_px: float    # median of |residual| per corner
    rmse_px: float           # RMS error for this face group
    p95_px: float            # 95th percentile error
    n_corners: int            # total corners across groups
    n_groups: int             # number of independent groups
    bias_magnitude_px: float  # sqrt(mean_du² + mean_dv²)
    bias_direction_deg: float # direction of bias in image plane
    classification: str = "CLEAN"      # "CLEAN" | "SYSTEMATIC_BIAS" | "QUARANTINE"
    recommended_weight: float = 1.0     # 1.0 → 0.1 → 0.0


def detect_face_bias(per_face_residuals: Dict[Tuple[str, str], Dict[str, np.ndarray]],
                     systematic_threshold_px: float = 2.0,
                     quarantine_threshold_px: float = 3.0,
                     min_groups: int = 3) -> List[FaceBiasDiagnostic]:
    """Detect systematic face bias from BA-1 per-camera×face residuals.

    Args:
        per_face_residuals: {(camera, face_name): {"du": array, "dv": array, "group_ids": set}}
        systematic_threshold_px: if bias > this → SYSTEMATIC_BIAS
        quarantine_threshold_px: if bias > this → QUARANTINE (weight=0)
        min_groups: minimum independent groups for reliable detection

    Returns:
        List of FaceBiasDiagnostic for each camera-face pair.
    """
    diagnostics = []

    for (camera, face_name), data in sorted(per_face_residuals.items()):
        du = np.asarray(data.get("du", []), dtype=np.float64)
        dv = np.asarray(data.get("dv", []), dtype=np.float64)
        group_ids = data.get("group_ids", set())

        n_corners = len(du)
        n_groups = len(group_ids)

        if n_corners < 4:
            diagnostics.append(FaceBiasDiagnostic(
                camera=camera, face_name=face_name,
                mean_du=0, mean_dv=0, median_bias_px=0, rmse_px=0, p95_px=0,
                n_corners=n_corners, n_groups=n_groups,
                bias_magnitude_px=0, bias_direction_deg=0,
                classification="CLEAN", recommended_weight=1.0))
            continue

        mean_du = float(np.mean(du))
        mean_dv = float(np.mean(dv))
        residuals_px = np.sqrt(du**2 + dv**2)
        median_bias = float(np.median(residuals_px))
        rmse = float(np.sqrt(np.mean(residuals_px**2)))
        p95 = float(np.percentile(residuals_px, 95))

        bias_magnitude = float(math.sqrt(mean_du**2 + mean_dv**2))
        bias_direction = float(math.degrees(math.atan2(mean_dv, mean_du)))

        # Classification
        if n_groups >= min_groups and bias_magnitude > quarantine_threshold_px:
            classification = "QUARANTINE"
            rec_weight = 0.0
        elif n_groups >= min_groups and bias_magnitude > systematic_threshold_px:
            classification = "SYSTEMATIC_BIAS"
            rec_weight = 0.1
        else:
            classification = "CLEAN"
            rec_weight = 1.0

        diagnostics.append(FaceBiasDiagnostic(
            camera=camera, face_name=face_name,
            mean_du=mean_du, mean_dv=mean_dv,
            median_bias_px=median_bias, rmse_px=rmse, p95_px=p95,
            n_corners=n_corners, n_groups=n_groups,
            bias_magnitude_px=bias_magnitude,
            bias_direction_deg=bias_direction,
            classification=classification,
            recommended_weight=rec_weight))

    return diagnostics


def compute_composite_weight(n_points: int,
                              w_angle: float = 1.0,
                              w_temporal: float = 1.0,
                              w_face_health: float = 1.0,
                              w_geometry: float = 1.0) -> float:
    """Compute composite observation weight.

    w = w_group * w_angle * w_temporal * w_face_health * w_geometry

    Where:
      w_group = 1 / sqrt(n_points) — prevents large groups from dominating BA
      w_angle from incidence classification (GOOD=1.0, NORMAL=0.7, WEAK=0.3, REJECT=0)
      w_temporal from multi-frame fusion quality (default 1.0)
      w_face_health from bias detection (CLEAN=1.0, BIASED=0.1, QUARANTINE=0)
      w_geometry: multi-face=1.0, single-face=0.5

    Args:
        n_points: number of corner points in this camera-group observation
        w_angle: incidence angle factor
        w_temporal: temporal stability factor
        w_face_health: face health factor (from bias detection)
        w_geometry: geometry diversity factor

    Returns:
        composite weight float
    """
    w_group = 1.0 / math.sqrt(max(n_points, 1))
    return w_group * w_angle * w_temporal * w_face_health * w_geometry


def compute_weight_factors(face_counts: Dict[str, int],
                            face_qualities: Dict[str, Tuple[FaceQuality, float]],
                            face_health_weights: Optional[Dict[str, float]] = None,
                            is_multi_face: bool = False) -> Dict[str, float]:
    """Compute all weight factors for a camera-group observation.

    Returns:
        {w_group, w_angle, w_geometry, w_face_health, composite}
    """
    n_points = sum(face_counts.values())
    w_group = 1.0 / math.sqrt(max(n_points, 1))

    # w_angle: average quality factor across detected faces
    if face_qualities:
        w_angle = np.mean([w for q, w in face_qualities.values()])
    else:
        w_angle = 1.0

    # w_geometry
    n_faces = len([f for f, c in face_counts.items() if c >= 4])
    w_geometry = 1.0 if n_faces >= 2 else 0.5

    # w_face_health: min health weight across detected faces
    if face_health_weights:
        w_face_health = min(
            face_health_weights.get(f, 1.0) for f in face_counts if face_counts[f] >= 4)
    else:
        w_face_health = 1.0

    composite = w_group * w_angle * w_geometry * w_face_health

    return {
        "w_group": w_group,
        "w_angle": w_angle,
        "w_geometry": w_geometry,
        "w_face_health": w_face_health,
        "composite": composite,
    }
