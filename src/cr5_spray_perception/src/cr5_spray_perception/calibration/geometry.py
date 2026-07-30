"""
CR5 Calibration — Unified SE(3) geometry utilities.

Consolidates all duplicated SE3/rotation/quaternion functions from:
  run_multi_frame_calibration.py
  rig_initialization.py
  run_final_calibration.py
  export_simulation_truth.py

All implementations are bit-exact reproductions of the proven originals.
"""
import math
import numpy as np
import cv2


def euler_matrix(ai, aj, ak, axes='sxyz'):
    """Return 4x4 SE(3) matrix for Euler angles. Default: static XYZ (Rz@Ry@Rx).

    This is the shared implementation from all 4 source files.
    """
    from math import cos, sin
    Rx = np.array([[1, 0, 0], [0, cos(ai), -sin(ai)], [0, sin(ai), cos(ai)]])
    Ry = np.array([[cos(aj), 0, sin(aj)], [0, 1, 0], [-sin(aj), 0, cos(aj)]])
    Rz = np.array([[cos(ak), -sin(ak), 0], [sin(ak), cos(ak), 0], [0, 0, 1]])
    R = Rz @ Ry @ Rx
    T = np.eye(4)
    T[:3, :3] = R
    return T


def rotation_matrix_3x3(ai, aj, ak):
    """Return 3x3 rotation matrix R = Rz(ak) @ Ry(aj) @ Rx(ai)."""
    from math import cos, sin
    Rx = np.array([[1, 0, 0], [0, cos(ai), -sin(ai)], [0, sin(ai), cos(ai)]])
    Ry = np.array([[cos(aj), 0, sin(aj)], [0, 1, 0], [-sin(aj), 0, cos(aj)]])
    Rz = np.array([[cos(ak), -sin(ak), 0], [sin(ak), cos(ak), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def quaternion_from_matrix(T):
    """Extract [x,y,z,w] quaternion from 4x4 SE(3) matrix.

    Matches tf.transformations.quaternion_from_matrix output.
    Copied verbatim from run_multi_frame_calibration.py L159-189.
    """
    R = np.asarray(T[:3, :3], dtype=np.float64)
    q = np.empty(4)
    t = R.trace()
    if t > 0:
        s = 0.5 / math.sqrt(t + 1.0)
        q[3] = 0.25 / s
        q[0] = (R[2, 1] - R[1, 2]) * s
        q[1] = (R[0, 2] - R[2, 0]) * s
        q[2] = (R[1, 0] - R[0, 1]) * s
    else:
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            q[3] = (R[2, 1] - R[1, 2]) / s
            q[0] = 0.25 * s
            q[1] = (R[0, 1] + R[1, 0]) / s
            q[2] = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            q[3] = (R[0, 2] - R[2, 0]) / s
            q[0] = (R[0, 1] + R[1, 0]) / s
            q[1] = 0.25 * s
            q[2] = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            q[3] = (R[1, 0] - R[0, 1]) / s
            q[0] = (R[0, 2] + R[2, 0]) / s
            q[1] = (R[1, 2] + R[2, 1]) / s
            q[2] = 0.25 * s
    return [float(v) for v in q]


def qt_to_T(qt):
    """[qw,qx,qy,qz, tx,ty,tz] → 4x4 SE(3) matrix.

    Copied verbatim from rig_initialization.py L78-89.
    """
    qw, qx, qy, qz = qt[0:4]
    tx, ty, tz = qt[4:7]
    T = np.eye(4)
    T[:3, :3] = np.array([
        [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
        [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
        [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2],
    ])
    T[:3, 3] = [tx, ty, tz]
    return T


def T_to_qt(T):
    """4x4 SE(3) matrix → [qw,qx,qy,qz, tx,ty,tz].

    Copied verbatim from rig_initialization.py L92-96.
    """
    q = quaternion_from_matrix(T)
    return [q[3], q[0], q[1], q[2],
            float(T[0, 3]), float(T[1, 3]), float(T[2, 3])]


def se3_log(T):
    """SE(3) log map: T → 6D tangent vector [rx,ry,rz, tx,ty,tz].

    Copied verbatim from rig_initialization.py L99-112.
    """
    R = T[:3, :3]
    t = T[:3, 3]
    tr = (np.trace(R) - 1.0) / 2.0
    tr = np.clip(tr, -1.0, 1.0)
    theta = math.acos(tr)
    if theta < 1e-12:
        return np.array([0.0, 0.0, 0.0, t[0], t[1], t[2]])
    s = theta / (2.0 * math.sin(theta))
    rx = s * (R[2, 1] - R[1, 2])
    ry = s * (R[0, 2] - R[2, 0])
    rz = s * (R[1, 0] - R[0, 1])
    return np.array([rx, ry, rz, t[0], t[1], t[2]])


def se3_exp(log):
    """SE(3) exponential map: 6D vector → 4x4 matrix.

    Input: [rx, ry, rz, tx, ty, tz] where [rx,ry,rz] = angle-axis rotation,
    [tx,ty,tz] = translation (simple copy, not proper Lie algebra velocity).

    Inverse of se3_log. For small rotations, this is equivalent to the
    proper Lie algebra exponential; for large rotations the translation
    is used directly (sufficient for optimization residual normalization).
    """
    rx, ry, rz = log[0], log[1], log[2]
    tx, ty, tz = log[3], log[4], log[5]
    theta = math.sqrt(rx*rx + ry*ry + rz*rz)
    T = np.eye(4)
    if theta < 1e-12:
        T[:3, 3] = [tx, ty, tz]
        return T
    axis = np.array([rx, ry, rz]) / theta
    K = np.array([[0, -axis[2], axis[1]],
                  [axis[2], 0, -axis[0]],
                  [-axis[1], axis[0], 0]])
    R = np.eye(3) + math.sin(theta) * K + (1 - math.cos(theta)) * K @ K
    # Translation: direct copy (consistent with se3_log)
    T[:3, :3] = R
    T[:3, 3] = [tx, ty, tz]
    return T


def se3_distance_mm_deg(T1, T2, sigma_t_mm=30.0, sigma_r_deg=3.0):
    """Weighted SE(3) distance between two transforms.

    Copied from rig_initialization.py L115-122.
    Returns (d_t_mm, d_r_deg, normalized_distance).
    """
    dT = np.linalg.inv(T1) @ T2
    log = se3_log(dT)
    d_t = np.linalg.norm(log[3:6]) * 1000.0  # mm
    d_r = np.linalg.norm(log[0:3])  # rad
    d_r_deg = math.degrees(d_r)
    return d_t, d_r_deg, math.sqrt((d_t / sigma_t_mm)**2 + (d_r_deg / sigma_r_deg)**2)


def rotation_distance_deg(R1, R2):
    """SO(3) geodesic distance in degrees.

    Copied verbatim from rig_initialization.py L125-129.
    """
    c = (np.trace(R2.T @ R1) - 1.0) / 2.0
    c = np.clip(c, -1.0, 1.0)
    return math.degrees(math.acos(c))


def rpy_from_rotation(R):
    """Extract (roll, pitch, yaw) in radians from 3x3 rotation matrix. ZYX convention.

    Copied from export_simulation_truth.py / camera_geometry.py.
    """
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0
    return roll, pitch, yaw


def rotation_from_rpy(roll, pitch, yaw):
    """Return 3x3 rotation matrix from ZYX Euler angles (radians)."""
    return rotation_matrix_3x3(roll, pitch, yaw)


def build_face_transform(xyz, rpy):
    """Build T_face_target 4x4 from xyz position + rpy Euler angles (radians).

    xyz: [x, y, z] translation
    rpy: [roll, pitch, yaw] in radians
    """
    T = euler_matrix(rpy[0], rpy[1], rpy[2])
    T[:3, 3] = xyz
    return T


def transform_points(points, T):
    """Transform Nx3 points by 4x4 SE(3) matrix. Returns Nx3."""
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim == 1:
        pts = pts.reshape(1, 3)
    N = pts.shape[0]
    pts_h = np.column_stack([pts, np.ones(N)])
    transformed = (T @ pts_h.T).T
    return transformed[:, :3]


def invert_transform(T):
    """Return inverse of 4x4 SE(3) matrix."""
    return np.linalg.inv(T)


def rvec_tvec_to_T(rvec, tvec):
    """OpenCV rvec (Rodrigues) + tvec → 4x4 SE(3) matrix.

    Copied from run_multi_frame_calibration.py L415-421.
    """
    R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(tvec, dtype=np.float64).flatten()
    return T


def T_to_rvec_tvec(T):
    """4x4 SE(3) matrix → (rvec, tvec).

    Copied from run_multi_frame_calibration.py L655-658.
    """
    rvec = cv2.Rodrigues(T[:3, :3])[0].flatten().tolist()
    tvec = T[:3, 3].flatten().tolist()
    return rvec, tvec


def quaternion_multiply(q1, q2):
    """Multiply two quaternions [w,x,y,z]. q_out = q1 ⊗ q2."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return [
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ]


def quaternion_conjugate(q):
    """Return conjugate of quaternion [w,x,y,z]."""
    return [q[0], -q[1], -q[2], -q[3]]


def quaternion_average(quaternions):
    """Average N quaternions [qw,qx,qy,qz] via eigenvector method.

    Builds Q = [q1, q2, ..., qN]^T (Nx4), then M = Q^T @ Q.
    Average = eigenvector of M with largest eigenvalue.

    Args:
        quaternions: Nx4 numpy array, each row is [qw, qx, qy, qz]

    Returns:
        4-element list [qw, qx, qy, qz] of average quaternion.
    """
    Q = np.asarray(quaternions, dtype=np.float64)
    if Q.ndim == 1:
        return Q.tolist()
    M = Q.T @ Q  # 4x4
    eigenvalues, eigenvectors = np.linalg.eigh(M)
    avg = eigenvectors[:, np.argmax(eigenvalues)]
    # Ensure w > 0
    if avg[0] < 0:
        avg = -avg
    return avg.tolist()


def look_at_rotation(cam_pos, target_pos):
    """Compute Gazebo camera link rotation: link +X = camera forward.

    Copied from camera_geometry.py / export_simulation_truth.py.

    Returns:
        R: 3x3 rotation matrix with columns [cam_x, cam_y, cam_z] in world frame
        direction: normalized view direction
        distance: distance from camera to target
    """
    cam = np.array(cam_pos, dtype=np.float64)
    tgt = np.array(target_pos, dtype=np.float64)
    d_raw = tgt - cam
    dist = float(np.linalg.norm(d_raw))
    if dist < 1e-9:
        raise ValueError(f"Camera at target position: {cam_pos}")
    d = d_raw / dist
    world_z = np.array([0.0, 0.0, 1.0])
    cam_x = d
    cam_y = np.cross(world_z, cam_x)
    cam_y_norm = float(np.linalg.norm(cam_y))
    if cam_y_norm < 1e-9:
        cam_y = np.array([0.0, 1.0, 0.0])
    else:
        cam_y = cam_y / cam_y_norm
    cam_z = np.cross(cam_x, cam_y)
    cam_z_norm = float(np.linalg.norm(cam_z))
    if cam_z_norm > 1e-9:
        cam_z = cam_z / cam_z_norm
    R = np.column_stack([cam_x, cam_y, cam_z])
    return R, d, dist


# ── SE3 Contract Self-Tests ──

def _run_se3_contract_tests():
    """Verify SE(3) mathematical contracts. Raises AssertionError on failure."""
    import numpy as np

    # Test 1: invert_transform
    T = euler_matrix(0.3, -0.5, 1.2)
    T[:3, 3] = [1.0, -2.0, 0.5]
    Tinv = invert_transform(T)
    assert np.allclose(T @ Tinv, np.eye(4), atol=1e-10), "T @ inv(T) ≠ I"
    assert np.allclose(Tinv @ T, np.eye(4), atol=1e-10), "inv(T) @ T ≠ I"

    # Test 2: se3_exp(se3_log(T)) ≈ T
    log = se3_log(T)
    T_recovered = se3_exp(log)
    assert np.allclose(T, T_recovered, atol=1e-10), \
        f"se3_exp(se3_log(T)) ≠ T:\n{T}\nvs\n{T_recovered}"

    # Test 3: se3_log order: [rx,ry,rz, tx,ty,tz]
    # Zero rotation should give zeros for rotation part
    T_pure_trans = np.eye(4)
    T_pure_trans[:3, 3] = [3.0, -1.0, 2.0]
    log_pt = se3_log(T_pure_trans)
    assert np.allclose(log_pt[0:3], [0, 0, 0], atol=1e-10), \
        f"pure translation log rotation part not zero: {log_pt[0:3]}"
    assert np.allclose(log_pt[3:6], [3.0, -1.0, 2.0], atol=1e-10), \
        f"pure translation log translation part wrong: {log_pt[3:6]}"

    # Test 4: Rotation geodesic distance
    R1 = np.eye(3)
    R2 = euler_matrix(0, 0, math.radians(90))[:3, :3]
    assert abs(rotation_distance_deg(R1, R2) - 90.0) < 1e-6, \
        f"90° rotation distance wrong: {rotation_distance_deg(R1, R2)}"

    # Test 5: qt_to_T / T_to_qt round-trip
    qt = T_to_qt(T)
    T_rt = qt_to_T(qt)
    assert np.allclose(T, T_rt, atol=1e-10), "qt_to_T(T_to_qt(T)) ≠ T"

    # Test 6: T_rig_camera contract: p_rig = T_rig_camera @ p_camera
    T_rc = euler_matrix(0.1, 0.2, 0.3)
    T_rc[:3, 3] = [5.0, 3.0, -1.0]
    p_camera = np.array([1.0, 2.0, 3.0, 1.0])
    p_rig = T_rc @ p_camera
    # Verify: this matches the contract
    assert np.allclose(p_rig[:3], T_rc[:3, :3] @ p_camera[:3] + T_rc[:3, 3], atol=1e-10)

    # Test 7: quaternion_average for identical quaternions
    q = np.array([[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]])
    avg = quaternion_average(q)
    assert np.allclose(avg, [1.0, 0.0, 0.0, 0.0], atol=1e-10), f"quat avg: {avg}"

    return True


if __name__ == "__main__":
    _run_se3_contract_tests()
    print("SE3 contract tests: ALL PASSED")
