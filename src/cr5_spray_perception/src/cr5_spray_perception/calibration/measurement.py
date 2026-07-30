"""
CR5 Calibration — Rich per-corner measurement data structures.

Preserves full metadata (face, marker ID, detector type, incidence angle)
through the entire calibration pipeline, unlike the old approach which
merged all faces into flat obj_pts/img_pts arrays.
"""
import math
import dataclasses as dc
import numpy as np
from typing import Dict, List, Optional, Tuple

from .geometry import transform_points


@dc.dataclass
class CornerObservation:
    """Single detected corner with full provenance metadata.

    This is the fundamental unit of measurement data. Every corner keeps
    its face name, marker ID, detector type, and all quality metadata.
    """
    camera: str                       # "cam_front_left" etc.
    group_id: int                     # integer group identifier
    face_name: str                    # "front", "left", "right", "top", "back"
    marker_id: int                    # ArUco/ChArUco/AprilTag marker ID
    corner_idx: int                   # 0-N for ChArUco boards, 0-3 for tag markers
    obj_pt_target: Tuple[float, float, float]  # 3D in calibration_target_frame
    img_pt_raw: Tuple[float, float]            # raw pixel (with distortion, if any)
    img_pt_undistorted: Optional[Tuple[float, float]] = None  # undistorted pixel
    detector_type: str = "unknown"     # "charuco" | "aruco" | "apriltag"
    subpixel_displacement: float = 0.0  # pixels, from corner refinement
    face_normal: Optional[Tuple[float, float, float]] = None  # in target frame
    incidence_angle_deg: float = 0.0   # computed later
    temporal_sigma_u: float = 0.0      # temporal std dev in u (from multi-frame fusion)
    temporal_sigma_v: float = 0.0      # temporal std dev in v
    weight: float = 1.0                # composite weight (set by quality module)

    @property
    def obj_pt(self) -> np.ndarray:
        return np.array(self.obj_pt_target, dtype=np.float64)

    @property
    def img_pt(self) -> np.ndarray:
        """Return the primary image point (undistorted if available, else raw)."""
        if self.img_pt_undistorted is not None:
            return np.array(self.img_pt_undistorted, dtype=np.float64)
        return np.array(self.img_pt_raw, dtype=np.float64)

    @property
    def img_pt_for_pnp(self) -> np.ndarray:
        """Return undistorted image point for PnP (always prefer undistorted)."""
        if self.img_pt_undistorted is not None:
            return np.array(self.img_pt_undistorted, dtype=np.float64)
        return np.array(self.img_pt_raw, dtype=np.float64)

    @property
    def temporal_quality(self) -> float:
        """Temporal quality: 1/sqrt(sigma_u^2 + sigma_v^2). 0 = no data."""
        sigma_sq = self.temporal_sigma_u**2 + self.temporal_sigma_v**2
        if sigma_sq < 1e-12:
            return 1.0
        return 1.0 / math.sqrt(sigma_sq)


@dc.dataclass
class CameraGroupMeasurement:
    """All corner observations for one camera in one target pose group."""
    camera: str
    group_id: int
    corners: List[CornerObservation]

    @property
    def n_corners(self) -> int:
        return len(self.corners)

    @property
    def obj_pts(self) -> np.ndarray:
        """Nx3 array of object points in target frame."""
        return np.array([c.obj_pt_target for c in self.corners], dtype=np.float64)

    @property
    def img_pts_raw(self) -> np.ndarray:
        """Nx2 array of raw image points."""
        return np.array([c.img_pt_raw for c in self.corners], dtype=np.float64)

    @property
    def img_pts_undistorted(self) -> np.ndarray:
        """Nx2 array of undistorted image points (or raw if not available)."""
        return np.array([c.img_pt_for_pnp for c in self.corners], dtype=np.float64)

    @property
    def weights(self) -> np.ndarray:
        """N-element array of per-corner weights."""
        return np.array([c.weight for c in self.corners], dtype=np.float64)

    @property
    def face_counts(self) -> Dict[str, int]:
        """{face_name: corner_count} for faces with >= 4 corners."""
        counts: Dict[str, int] = {}
        for c in self.corners:
            counts[c.face_name] = counts.get(c.face_name, 0) + 1
        return {k: v for k, v in counts.items() if v >= 4}

    @property
    def detected_faces(self) -> List[str]:
        """List of face names with >= 4 detected corners."""
        return sorted(self.face_counts.keys())

    @property
    def n_faces(self) -> int:
        return len(self.detected_faces)

    @property
    def is_multi_face(self) -> bool:
        return self.n_faces >= 2

    def get_corners_by_face(self, face_name: str) -> List[CornerObservation]:
        return [c for c in self.corners if c.face_name == face_name]

    def get_marker_ids(self) -> set:
        return {c.marker_id for c in self.corners}


@dc.dataclass
class CalibrationDataset:
    """Complete multi-camera calibration dataset.

    Holds camera intrinsics, face geometry, and all per-group/camera observations.
    Supports loading from live capture, recorded data, or synthetic generation.
    """
    camera_infos: Dict[str, dict]        # {cam: {K: 3x3_list, D: list, width, height}}
    face_poses_target: Dict[str, dict]   # {face: {xyz: [3], rpy: [3]}}
    groups: Dict[int, Dict[str, CameraGroupMeasurement]]  # group_id → {cam → meas}
    source_type: str = "unknown"         # "gazebo" | "recorded" | "live" | "synthetic"
    metadata: dict = dc.field(default_factory=dict)

    CAMERAS = ["cam_front_left", "cam_front_right", "cam_rear"]
    REFERENCE_CAMERA = "cam_front_left"

    @property
    def n_groups(self) -> int:
        return len(self.groups)

    @property
    def n_cameras(self) -> int:
        return len(self.camera_infos)

    def get_corners_by_face(self, camera: str, face_name: str) -> List[CornerObservation]:
        """Get all detected corners for a specific camera-face across all groups."""
        result = []
        for gid, gdata in self.groups.items():
            if camera in gdata:
                result.extend(gdata[camera].get_corners_by_face(face_name))
        return result

    def get_nonplanar_measurements(self) -> Dict[int, Dict[str, CameraGroupMeasurement]]:
        """Return only measurements that are multi-face and non-planar.

        A measurement is non-planar if it has >= 2 detected faces and its
        3D points span significantly outside a single plane (s3/s2 > 0.01).
        """
        result = {}
        for gid, gdata in self.groups.items():
            group_nonplanar = {}
            for cam, meas in gdata.items():
                if not meas.is_multi_face:
                    continue
                # Quick SVD planar check
                pts = meas.obj_pts
                if len(pts) < 6:
                    continue
                pts_c = pts - np.mean(pts, axis=0)
                _, S, _ = np.linalg.svd(pts_c, full_matrices=False)
                s3, s2 = S[2] if len(S) > 2 else 0, S[1] if len(S) > 1 else 1
                if s2 > 1e-12 and s3 / s2 > 0.01:
                    group_nonplanar[cam] = meas
            if group_nonplanar:
                result[gid] = group_nonplanar
        return result

    def get_camera_groups(self, camera: str) -> List[int]:
        """Get list of group IDs where a camera has valid measurements."""
        return sorted(gid for gid, gdata in self.groups.items() if camera in gdata)

    def get_all_faces_detected(self) -> set:
        """Get set of all face names ever detected."""
        faces = set()
        for gdata in self.groups.values():
            for meas in gdata.values():
                faces.update(meas.detected_faces)
        return faces

    def to_legacy_observations(self) -> dict:
        """Convert to accumulated_observations.yaml format (backward compat).

        WARNING: This LOSES per-corner metadata (face name, marker ID, temporal stats).
        Only use for backward compatibility with old scripts.
        """
        from .geometry import T_to_qt, T_to_rvec_tvec

        data = {"cameras": {}, "observations": {}}

        for cam_name, info in self.camera_infos.items():
            cam_entry = {
                "K": info["K"],
                "D": info.get("D", [0, 0, 0, 0, 0]),
                "width": info.get("width", 640),
                "height": info.get("height", 480),
                "initial_pose": [1, 0, 0, 0, 0, 0, 0],
            }
            data["cameras"][cam_name] = cam_entry

        for gid in sorted(self.groups.keys()):
            gdata = self.groups[gid]
            group_entry = {}
            for cam_name in self.CAMERAS:
                if cam_name not in gdata:
                    continue
                meas = gdata[cam_name]
                group_entry[cam_name] = {
                    "object_points_3d": [list(c.obj_pt_target) for c in meas.corners],
                    "image_points_2d": [list(c.img_pt_for_pnp) for c in meas.corners],
                    "corner_count": meas.n_corners,
                    "face_counts": meas.face_counts,
                }
            if group_entry:
                data["observations"][gid] = group_entry

        return data


def temporal_fusion(frames_corners: List[List[CornerObservation]],
                     outlier_sigma: float = 3.0,
                     min_frames: int = 3) -> List[CornerObservation]:
    """Fuse multiple frames at the same target pose using robust statistics.

    For each unique corner (matched by face_name + marker_id + corner_idx):
      - Compute median img_pt across frames
      - Compute sigma_u, sigma_v (MAD-based robust std)
      - Flag outliers > outlier_sigma * MAD away
      - Return merged observation with temporal quality metadata

    Args:
        frames_corners: List of per-frame corner lists (same target pose)
        outlier_sigma: Number of MADs for outlier rejection
        min_frames: Minimum frames needed for fusion (fewer → keep raw)

    Returns:
        List of merged CornerObservations with temporal_sigma populated.
    """
    import math

    if len(frames_corners) < min_frames:
        # Not enough frames: return first frame as-is
        return frames_corners[0] if frames_corners else []

    # Group by (face_name, marker_id, corner_idx)
    from collections import defaultdict
    corner_groups = defaultdict(list)
    for frame_idx, corners in enumerate(frames_corners):
        for c in corners:
            key = (c.face_name, c.marker_id, c.corner_idx)
            corner_groups[key].append((frame_idx, c))

    merged = []
    for key, observations in corner_groups.items():
        face_name, marker_id, corner_idx = key

        n_obs = len(observations)
        if n_obs < min_frames:
            # Keep as-is without temporal sigma
            merged.append(observations[0][1])
            continue

        # Extract u, v coordinates
        us = np.array([obs[1].img_pt_raw[0] for obs in observations])
        vs = np.array([obs[1].img_pt_raw[1] for obs in observations])

        # Robust MAD-based statistics
        median_u = np.median(us)
        median_v = np.median(vs)
        mad_u = np.median(np.abs(us - median_u))
        mad_v = np.median(np.abs(vs - median_v))
        sigma_u = 1.4826 * mad_u  # MAD → sigma for Gaussian
        sigma_v = 1.4826 * mad_v

        # Filter outliers
        keep_indices = []
        for i, (u, v) in enumerate(zip(us, vs)):
            z_u = abs(u - median_u) / max(sigma_u, 1e-6)
            z_v = abs(v - median_v) / max(sigma_v, 1e-6)
            if z_u < outlier_sigma and z_v < outlier_sigma:
                keep_indices.append(i)

        if len(keep_indices) < min_frames:
            # Too many outliers, use median from all
            keep_indices = list(range(n_obs))

        # Take the observation closest to median
        best_idx = keep_indices[0]
        best_dist = float('inf')
        for i in keep_indices:
            dist = (us[i] - median_u)**2 + (vs[i] - median_v)**2
            if dist < best_dist:
                best_dist = dist
                best_idx = i

        merged_corner = observations[best_idx][1]

        # Update temporal quality stats
        merged_corner.temporal_sigma_u = float(sigma_u)
        merged_corner.temporal_sigma_v = float(sigma_v)

        # Set img_pt to the robust median
        merged_corner.img_pt_raw = (float(median_u), float(median_v))
        merged_corner.img_pt_undistorted = None  # Will need re-undistortion

        merged.append(merged_corner)

    return merged
