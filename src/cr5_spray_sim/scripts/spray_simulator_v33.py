#!/usr/bin/env python3
"""
V3.3 Spray Simulator: 开关控制 + 工件命中计算 + 状态发布。

V3.2.1 修复:
- state publisher 使用 latch=True，后启动节点不会错过消息
- 每 2 Hz 周期重发当前状态
- world-frame TF 查询，避免树断开问题
- 新增 /spray_demo/reset_paint, /spray_demo/save_result 服务
"""
import math
import json
import os
import rospy
import tf2_ros
import numpy as np
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
from std_msgs.msg import Bool, String, Float32, Header, ColorRGBA
from geometry_msgs.msg import PoseStamped, PointStamped, Vector3Stamped, Pose, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray


# ===== Ray-Primitive Intersection (ported from V3.2, unchanged) =====

def ray_cylinder_intersect(origin, direction, cyl_radius, cyl_y_min, cyl_y_max):
    """Ray-infinite-cylinder (axis Y). Returns (t, point, normal) or None."""
    ox, oy, oz = origin
    dx, dy, dz = direction
    a = dx * dx + dz * dz
    if abs(a) < 1e-12:
        return None
    b = 2.0 * (ox * dx + oz * dz)
    c = ox * ox + oz * oz - cyl_radius * cyl_radius
    disc = b * b - 4.0 * a * c
    if disc < 0:
        return None
    sqrt_disc = math.sqrt(disc)
    for t in (( -b - sqrt_disc) / (2.0 * a), ( -b + sqrt_disc) / (2.0 * a)):
        if t <= 1e-9:
            continue
        py = oy + t * dy
        if cyl_y_min - 1e-6 <= py <= cyl_y_max + 1e-6:
            px = ox + t * dx
            pz = oz + t * dz
            return (t, np.array([px, py, pz]), np.array([px / cyl_radius, 0.0, pz / cyl_radius]))
    return None


def ray_disk_intersect(origin, direction, disk_y, disk_radius):
    """Ray-disk (Y=const). Returns (t, point, normal) or None."""
    if abs(direction[1]) < 1e-12:
        return None
    t = (disk_y - origin[1]) / direction[1]
    if t <= 1e-9:
        return None
    px = origin[0] + t * direction[0]
    pz = origin[2] + t * direction[2]
    if px * px + pz * pz <= disk_radius * disk_radius:
        ny = 1.0 if direction[1] < 0 else -1.0
        return (t, np.array([px, disk_y, pz]), np.array([0.0, ny, 0.0]))
    return None


def ray_box_intersect(origin, direction, box_min, box_max):
    """Ray-AABB slab method. Returns (t, point, normal) or None."""
    tmin, tmax = -float('inf'), float('inf')
    normal_axis, normal_sign = 0, 1.0
    for i in range(3):
        if abs(direction[i]) < 1e-12:
            if origin[i] < box_min[i] or origin[i] > box_max[i]:
                return None
            continue
        inv_d = 1.0 / direction[i]
        t0 = (box_min[i] - origin[i]) * inv_d
        t1 = (box_max[i] - origin[i]) * inv_d
        if t0 > t1:
            t0, t1 = t1, t0
        if t0 > tmin:
            tmin, normal_axis, normal_sign = t0, i, (-1.0 if direction[i] > 0 else 1.0)
        if t1 < tmax:
            tmax = t1
        if tmin > tmax:
            return None
    if tmin <= 1e-9:
        if tmax <= 1e-9:
            return None
        t = tmax
        for i in range(3):
            if abs(direction[i]) < 1e-12:
                continue
            inv_d = 1.0 / direction[i]
            t0 = (box_min[i] - origin[i]) * inv_d
            t1 = (box_max[i] - origin[i]) * inv_d
            if abs(t - t0) < 1e-9:
                normal_axis, normal_sign = i, (-1.0 if direction[i] > 0 else 1.0)
            elif abs(t - t1) < 1e-9:
                normal_axis, normal_sign = i, (1.0 if direction[i] > 0 else -1.0)
    else:
        t = tmin
    normal = np.zeros(3)
    normal[normal_axis] = normal_sign
    return (t, origin + t * direction, normal)


class SpraySimulatorV33:
    def __init__(self):
        rospy.init_node("spray_simulator_v33")

        # Parameters
        self.update_rate = rospy.get_param("~update_rate_hz", 20.0)
        self.nominal_standoff = rospy.get_param("~nominal_standoff_m", 0.18)
        self.min_standoff = rospy.get_param("~min_standoff_m", 0.10)
        self.max_standoff = rospy.get_param("~max_standoff_m", 0.25)
        self.cone_half_angle = math.radians(rospy.get_param("~cone_half_angle_deg", 15.0))
        self.max_incidence = math.radians(rospy.get_param("~max_incidence_angle_deg", 45.0))
        self.auto_off_timeout = rospy.get_param("~auto_off_timeout_s", 0.25)
        self.target_model = rospy.get_param("~target_model", "simple_hanging_workpiece")
        self.target_frame = rospy.get_param("~target_frame", "object_frame")
        self.object_type = rospy.get_param("~object_type", "motor_housing_cylinder")

        # State
        self.spray_enabled = False
        self.state = "OFF"
        self.last_no_hit_time = None
        self.nozzle_frame = "spray_nozzle_frame"
        self.latest_hit = None  # (t, hit_pt_world, normal_world) or None

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Publishers — V3.3 修复: state latched
        self.pub_enabled = rospy.Publisher("/spray_demo/enabled", Bool, queue_size=1, latch=True)
        self.pub_state = rospy.Publisher("/spray_demo/state", String, queue_size=1, latch=True)
        self.pub_nozzle_pose = rospy.Publisher("/spray_demo/nozzle_pose", PoseStamped, queue_size=1)
        self.pub_hit_point = rospy.Publisher("/spray_demo/hit_point", PointStamped, queue_size=1)
        self.pub_hit_normal = rospy.Publisher("/spray_demo/hit_normal", Vector3Stamped, queue_size=1)
        self.pub_hit_distance = rospy.Publisher("/spray_demo/hit_distance", Float32, queue_size=1)
        self.pub_spray_marker = rospy.Publisher("/spray_demo/spray_marker", Marker, queue_size=1)
        self.pub_paint_patches = rospy.Publisher("/spray_demo/paint_patches", MarkerArray, queue_size=1)

        # Services
        rospy.Service("/spray_demo/set_spray", SetBool, self.handle_set_spray)
        rospy.Service("/spray_demo/reset_paint", Trigger, self.handle_reset_paint)
        rospy.Service("/spray_demo/save_result", Trigger, self.handle_save_result)

        # Persistent patch state
        self.patches = []  # list of (position, normal, color) dicts
        self.patch_voxels = set()  # for dedup
        self.voxel_size = rospy.get_param("~paint/voxel_size_m", 0.010)
        self.max_patches = rospy.get_param("~paint/max_patches", 2000)
        self.patch_radius = rospy.get_param("~paint/patch_radius_m", 0.012)
        self.surface_offset = rospy.get_param("~paint/surface_offset_m", 0.0008)
        paint_color = rospy.get_param("~paint/paint_color", [0.10, 0.35, 0.75, 1.0])
        self.paint_color = paint_color
        self.patch_seq = 0

        # Rosbags / result saving
        self.result_dir = rospy.get_param("~result_dir",
                                          os.path.expanduser("~/cr5_spray_results"))

        # Set initial state (latched, 后启动节点可收到)
        self.pub_enabled.publish(Bool(data=False))
        self.pub_state.publish(String(data="OFF"))

        rospy.loginfo("Spray Simulator V3.3 ready")
        rospy.loginfo("  target: model=%s frame=%s type=%s",
                      self.target_model, self.target_frame, self.object_type)

    # ===== Service Handlers =====

    def handle_set_spray(self, req):
        if req.data:
            return self._try_enable()
        else:
            return self._disable()

    def handle_reset_paint(self, req):
        """清空所有漆层 patch"""
        count = len(self.patches)
        self.patches.clear()
        self.patch_voxels.clear()
        self.patch_seq = 0
        # 发布空 MarkerArray 清除 RViz 显示
        clear = MarkerArray()
        mk = Marker()
        mk.header.frame_id = "world"
        mk.ns = "paint_patch"
        mk.action = Marker.DELETEALL
        clear.markers.append(mk)
        self.pub_paint_patches.publish(clear)
        rospy.loginfo("Reset paint: removed %d patches", count)
        return TriggerResponse(success=True, message=f"Removed {count} patches")

    def handle_save_result(self, req):
        """保存当前喷涂结果"""
        try:
            os.makedirs(self.result_dir, exist_ok=True)
            stamp = rospy.Time.now()
            result = {
                "timestamp": {"secs": stamp.secs, "nsecs": stamp.nsecs},
                "object_type": self.object_type,
                "patch_count": len(self.patches),
                "patch_radius_m": self.patch_radius,
                "voxel_size_m": self.voxel_size,
                "paint_color": self.paint_color,
                "patches": [{"x": float(p[0][0]), "y": float(p[0][1]), "z": float(p[0][2]),
                             "nx": float(p[1][0]), "ny": float(p[1][1]), "nz": float(p[1][2])}
                            for p in self.patches]
            }
            fname = os.path.join(self.result_dir,
                                 f"spray_result_{stamp.secs}_{stamp.nsecs}.json")
            with open(fname, 'w') as f:
                json.dump(result, f, indent=2)
            rospy.loginfo("Saved result: %d patches → %s", len(self.patches), fname)
            return TriggerResponse(success=True, message=f"Saved: {fname}")
        except Exception as e:
            rospy.logerr("Save failed: %s", e)
            return TriggerResponse(success=False, message=str(e))

    def _try_enable(self):
        resp = SetBoolResponse()

        if not rospy.get_param("/use_sim_time", False):
            resp.success = False
            resp.message = "REJECTED: /use_sim_time != true"
            self._set_state("FAULT")
            return resp

        try:
            nozzle_world = self.tf_buffer.lookup_transform(
                "world", self.nozzle_frame, rospy.Time(0), rospy.Duration(1.0))
            target_world = self.tf_buffer.lookup_transform(
                "world", self.target_frame, rospy.Time(0), rospy.Duration(1.0))
        except Exception as e:
            resp.success = False
            resp.message = f"REJECTED: TF lookup failed: {e}"
            self._set_state("FAULT")
            return resp

        nozzle_pos_w = np.array([nozzle_world.transform.translation.x,
                                 nozzle_world.transform.translation.y,
                                 nozzle_world.transform.translation.z])
        nozzle_q_w = nozzle_world.transform.rotation
        target_pos_w = np.array([target_world.transform.translation.x,
                                 target_world.transform.translation.y,
                                 target_world.transform.translation.z])
        target_q_w = target_world.transform.rotation

        rel_pos = nozzle_pos_w - target_pos_w
        origin = self._inv_rotate(rel_pos, target_q_w)
        nozzle_z_w = self._rotate_z_axis(np.array([nozzle_q_w.x, nozzle_q_w.y,
                                                    nozzle_q_w.z, nozzle_q_w.w]))
        direction = self._inv_rotate_dir(nozzle_z_w, target_q_w)

        hit = self._compute_hit(origin, direction)
        if hit is None:
            resp.success = False
            resp.message = "REJECTED: no target hit"
            self._set_state("NO_TARGET")
            return resp

        t, hit_pt, normal = hit
        if t < self.min_standoff:
            resp.success = False
            resp.message = f"REJECTED: too close ({t:.3f}m < {self.min_standoff:.2f}m)"
            self._set_state("OUT_OF_RANGE")
            return resp
        if t > self.max_standoff:
            resp.success = False
            resp.message = f"REJECTED: too far ({t:.3f}m > {self.max_standoff:.2f}m)"
            self._set_state("OUT_OF_RANGE")
            return resp

        spray_dir = -direction
        cos_inc = abs(np.dot(spray_dir, normal))
        inc_angle = math.acos(min(cos_inc, 1.0))
        if inc_angle > self.max_incidence:
            resp.success = False
            resp.message = f"REJECTED: bad incidence ({math.degrees(inc_angle):.1f}deg)"
            self._set_state("BAD_INCIDENCE")
            return resp

        self.spray_enabled = True
        self._set_state("SPRAYING")
        self.last_no_hit_time = None
        self.pub_enabled.publish(Bool(data=True))
        resp.success = True
        resp.message = f"SPRAYING: dist={t:.3f}m inc={math.degrees(inc_angle):.1f}deg"
        rospy.loginfo("Spray ON: %s", resp.message)
        return resp

    def _disable(self):
        self.spray_enabled = False
        self._set_state("READY")
        self.last_no_hit_time = None
        self.pub_enabled.publish(Bool(data=False))
        rospy.loginfo("Spray OFF")
        return SetBoolResponse(success=True, message="OFF")

    def _set_state(self, new_state):
        """V3.3: 使用 latched publisher 设置状态，确保后启动节点可收到"""
        self.state = new_state
        self.pub_state.publish(String(data=new_state))

    # ===== Hit Testing =====

    def _rotate_z_axis(self, q_xyzw):
        x, y, z, w = q_xyzw
        return np.array([2*(x*z + w*y), 2*(y*z - w*x), 1 - 2*(x*x + y*y)])

    def _inv_rotate(self, v, q):
        x, y, z, w = -q.x, -q.y, -q.z, q.w
        return np.array([
            (1-2*(y*y+z*z))*v[0] + 2*(x*y-w*z)*v[1] + 2*(x*z+w*y)*v[2],
            2*(x*y+w*z)*v[0] + (1-2*(x*x+z*z))*v[1] + 2*(y*z-w*x)*v[2],
            2*(x*z-w*y)*v[0] + 2*(y*z+w*x)*v[1] + (1-2*(x*x+y*y))*v[2]])

    def _inv_rotate_dir(self, d, q):
        return self._inv_rotate(d, q)

    def _transform_point(self, pt, tf):
        q = tf.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        rx = (1-2*(y*y+z*z))*pt[0] + 2*(x*y-w*z)*pt[1] + 2*(x*z+w*y)*pt[2]
        ry = 2*(x*y+w*z)*pt[0] + (1-2*(x*x+z*z))*pt[1] + 2*(y*z-w*x)*pt[2]
        rz = 2*(x*z-w*y)*pt[0] + 2*(y*z+w*x)*pt[1] + (1-2*(x*x+y*y))*pt[2]
        return np.array([rx + tf.transform.translation.x,
                         ry + tf.transform.translation.y,
                         rz + tf.transform.translation.z])

    def _transform_direction(self, d, tf):
        q = tf.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        return np.array([
            (1-2*(y*y+z*z))*d[0] + 2*(x*y-w*z)*d[1] + 2*(x*z+w*y)*d[2],
            2*(x*y+w*z)*d[0] + (1-2*(x*x+z*z))*d[1] + 2*(y*z-w*x)*d[2],
            2*(x*z-w*y)*d[0] + 2*(y*z+w*x)*d[1] + (1-2*(x*x+y*y))*d[2]])

    def _compute_hit(self, origin, direction):
        if self.object_type == "motor_housing_cylinder":
            return self._hit_cylinder(origin, direction)
        elif self.object_type == "rectangular_housing":
            return self._hit_box(origin, direction)
        return None

    def _hit_cylinder(self, origin, direction):
        body_r, body_len = 0.105, 0.36
        body_ymin, body_ymax = -body_len/2, body_len/2
        lc_r, lc_len = 0.115, 0.035
        lc_ymin, lc_ymax = body_ymin - lc_len, body_ymin
        rc_r, rc_len = 0.115, 0.045
        rc_ymin, rc_ymax = body_ymax, body_ymax + rc_len
        sh_r, sh_len = 0.040, 0.055
        sh_ymin, sh_ymax = rc_ymax, rc_ymax + sh_len

        candidates = []
        for hit in [ray_cylinder_intersect(origin, direction, body_r, body_ymin, body_ymax),
                     ray_cylinder_intersect(origin, direction, lc_r, lc_ymin, lc_ymax),
                     ray_disk_intersect(origin, direction, lc_ymin, lc_r),
                     ray_cylinder_intersect(origin, direction, rc_r, rc_ymin, rc_ymax),
                     ray_disk_intersect(origin, direction, rc_ymax, rc_r),
                     ray_cylinder_intersect(origin, direction, sh_r, sh_ymin, sh_ymax),
                     ray_disk_intersect(origin, direction, sh_ymax, sh_r)]:
            if hit:
                candidates.append(hit)

        if not candidates:
            return None
        candidates.sort(key=lambda x: x[0])
        return candidates[0][:3]

    def _hit_box(self, origin, direction):
        hx, hy, hz = 0.09, 0.18, 0.13
        candidates = []
        hit = ray_box_intersect(origin, direction,
                                np.array([-hx, -hy, -hz]), np.array([hx, hy, hz]))
        if hit:
            candidates.append(hit)
        thx, thy, thz = 0.05, 0.06, 0.035
        toff_y, toff_z = 0.07, 0.165
        hit = ray_box_intersect(origin, direction,
                                np.array([-thx, toff_y-thy, toff_z-thz]),
                                np.array([thx, toff_y+thy, toff_z+thz]))
        if hit:
            candidates.append(hit)
        if not candidates:
            return None
        candidates.sort(key=lambda x: x[0])
        return candidates[0][:3]

    # ===== Paint Patch Management =====

    def _add_patch(self, hit_pt_world, normal_world):
        """添加一个 paint patch（带体素去重）"""
        if len(self.patches) >= self.max_patches:
            return

        # Voxel key for dedup
        vx = tuple((hit_pt_world / self.voxel_size).astype(int))
        if vx in self.patch_voxels:
            return
        self.patch_voxels.add(vx)

        # Offset patch slightly along normal to avoid z-fighting
        patch_pos = hit_pt_world + normal_world * self.surface_offset
        self.patches.append((patch_pos.copy(), normal_world.copy()))
        self.patch_seq += 1

    def _publish_patches(self, now):
        """发布所有 patch 为 MarkerArray (CYLINDER 贴合表面法向)"""
        ma = MarkerArray()
        for i, (pos, normal) in enumerate(self.patches):
            mk = Marker()
            mk.header.frame_id = "world"
            mk.header.stamp = now
            mk.ns = "paint_patch"
            mk.id = i
            mk.type = Marker.CYLINDER
            mk.action = Marker.ADD
            mk.pose.position.x = pos[0]
            mk.pose.position.y = pos[1]
            mk.pose.position.z = pos[2]

            # Orient cylinder (Z-axis) to surface normal
            z_axis = np.array([0.0, 0.0, 1.0])
            n = normal / (np.linalg.norm(normal) + 1e-12)
            # Rotation from Z to normal using quaternion
            v = np.cross(z_axis, n)
            s = np.linalg.norm(v)
            c = np.dot(z_axis, n)
            if s < 1e-12:
                mk.pose.orientation.w = 1.0
            else:
                v /= s
                half_angle = math.atan2(s, c) / 2.0
                sin_half = math.sin(half_angle)
                mk.pose.orientation.x = v[0] * sin_half
                mk.pose.orientation.y = v[1] * sin_half
                mk.pose.orientation.z = v[2] * sin_half
                mk.pose.orientation.w = math.cos(half_angle)

            mk.scale.x = self.patch_radius * 2
            mk.scale.y = self.patch_radius * 2
            mk.scale.z = 0.001  # very thin disk
            mk.color.r = self.paint_color[0]
            mk.color.g = self.paint_color[1]
            mk.color.b = self.paint_color[2]
            mk.color.a = self.paint_color[3]
            # No lifetime → persistent
            ma.markers.append(mk)
        self.pub_paint_patches.publish(ma)

    # ===== Main Loop =====

    def run(self):
        rate = rospy.Rate(self.update_rate)
        last_state_republish = rospy.Time.now()
        state_republish_interval = rospy.Duration(0.5)  # 2 Hz 周期重发

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            self._update(now)

            # V3.3: 周期重发状态，确保后启动节点不会永久等待
            if now - last_state_republish > state_republish_interval:
                self.pub_state.publish(String(data=self.state))
                last_state_republish = now

            rate.sleep()

    def _update(self, now):
        # Publish nozzle pose
        try:
            transform = self.tf_buffer.lookup_transform(
                "world", self.nozzle_frame, rospy.Time(0), rospy.Duration(0.1))
            nozzle_pose = PoseStamped(
                header=Header(stamp=now, frame_id="world"),
                pose=Pose(position=transform.transform.translation,
                          orientation=transform.transform.rotation))
            self.pub_nozzle_pose.publish(nozzle_pose)
        except Exception:
            nozzle_pose = None

        if not self.spray_enabled:
            self._publish_marker_off(now)
            return

        # Spraying: compute hit
        try:
            nozzle_world = self.tf_buffer.lookup_transform(
                "world", self.nozzle_frame, rospy.Time(0), rospy.Duration(0.1))
            target_world = self.tf_buffer.lookup_transform(
                "world", self.target_frame, rospy.Time(0), rospy.Duration(0.1))
        except Exception:
            self._check_auto_off(now, lost=True)
            return

        nozzle_pos_w = np.array([nozzle_world.transform.translation.x,
                                 nozzle_world.transform.translation.y,
                                 nozzle_world.transform.translation.z])
        nozzle_q_w = nozzle_world.transform.rotation
        target_pos_w = np.array([target_world.transform.translation.x,
                                 target_world.transform.translation.y,
                                 target_world.transform.translation.z])
        target_q_w = target_world.transform.rotation

        rel_pos = nozzle_pos_w - target_pos_w
        origin = self._inv_rotate(rel_pos, target_q_w)
        nozzle_z_w = self._rotate_z_axis(np.array([nozzle_q_w.x, nozzle_q_w.y,
                                                    nozzle_q_w.z, nozzle_q_w.w]))
        direction = self._inv_rotate_dir(nozzle_z_w, target_q_w)

        hit = self._compute_hit(origin, direction)
        if hit is None:
            self._check_auto_off(now, lost=True)
            self.pub_hit_distance.publish(Float32(data=-1.0))
            self._publish_marker_no_target(now)
            return

        t, hit_pt_obj, normal_obj = hit

        if not (self.min_standoff <= t <= self.max_standoff):
            self._check_auto_off(now, lost=True)
            self.pub_hit_distance.publish(Float32(data=t))
            self._publish_marker_out_of_range(now)
            return

        spray_dir = -direction
        cos_inc = abs(np.dot(spray_dir, normal_obj))
        inc_angle = math.acos(min(cos_inc, 1.0))
        if inc_angle > self.max_incidence:
            self._check_auto_off(now, lost=True)
            self.pub_hit_distance.publish(Float32(data=t))
            self._publish_marker_bad_incidence(now)
            return

        # Valid hit
        self.last_no_hit_time = None
        self.pub_hit_distance.publish(Float32(data=t))

        # Transform to world
        try:
            world_tf = self.tf_buffer.lookup_transform(
                "world", self.target_frame, rospy.Time(0), rospy.Duration(0.1))
        except Exception:
            return

        hit_pt_world = self._transform_point(hit_pt_obj, world_tf)
        normal_world = self._transform_direction(normal_obj, world_tf)
        # Normalize direction
        n = np.linalg.norm(normal_world)
        if n > 1e-12:
            normal_world /= n

        self.pub_hit_point.publish(PointStamped(
            header=Header(stamp=now, frame_id="world"),
            point=Point(x=hit_pt_world[0], y=hit_pt_world[1], z=hit_pt_world[2])))
        self.pub_hit_normal.publish(Vector3Stamped(
            header=Header(stamp=now, frame_id="world"),
            vector=Point(x=normal_world[0], y=normal_world[1], z=normal_world[2])))

        self._publish_marker_spraying(now, hit_pt_world, normal_world)

        # V3.3: 累积 paint patch
        self._add_patch(hit_pt_world, normal_world)
        self._publish_patches(now)

    def _check_auto_off(self, now, lost=False):
        if lost:
            if self.last_no_hit_time is None:
                self.last_no_hit_time = now
            elif (now - self.last_no_hit_time).to_sec() > self.auto_off_timeout:
                rospy.logwarn("Auto-off: target lost for %.2fs",
                              (now - self.last_no_hit_time).to_sec())
                self._disable()
        else:
            self.last_no_hit_time = None

    # ===== Marker Helpers =====

    def _publish_marker_off(self, now):
        mk = Marker(); mk.header.stamp = now; mk.header.frame_id = "world"
        mk.ns = "spray"; mk.id = 0; mk.action = Marker.DELETE
        self.pub_spray_marker.publish(mk)

    def _make_marker(self, now, mtype, color, scale, alpha=0.7):
        mk = Marker()
        mk.header.stamp = now; mk.header.frame_id = "world"
        mk.ns = "spray"; mk.id = 0; mk.type = mtype; mk.action = Marker.ADD
        mk.scale.x = scale[0]; mk.scale.y = scale[1]; mk.scale.z = scale[2]
        mk.color.r = color[0]; mk.color.g = color[1]; mk.color.b = color[2]; mk.color.a = alpha
        mk.lifetime = rospy.Duration(0.15)
        return mk

    def _publish_marker_spraying(self, now, hit_pt, normal):
        mk = self._make_marker(now, Marker.SPHERE, (0.0, 0.9, 0.0), (0.03, 0.03, 0.03))
        mk.pose.position.x = hit_pt[0]
        mk.pose.position.y = hit_pt[1]
        mk.pose.position.z = hit_pt[2]
        self.pub_spray_marker.publish(mk)

    def _publish_marker_no_target(self, now):
        mk = self._make_marker(now, Marker.ARROW, (1.0, 0.8, 0.0), (0.2, 0.01, 0.01), 0.6)
        self.pub_spray_marker.publish(mk)

    def _publish_marker_out_of_range(self, now):
        mk = self._make_marker(now, Marker.SPHERE, (1.0, 0.0, 0.0), (0.04, 0.04, 0.04))
        self.pub_spray_marker.publish(mk)

    def _publish_marker_bad_incidence(self, now):
        mk = self._make_marker(now, Marker.SPHERE, (1.0, 0.5, 0.0), (0.04, 0.04, 0.04))
        self.pub_spray_marker.publish(mk)


if __name__ == "__main__":
    SpraySimulatorV33().run()
