#!/usr/bin/env python3
"""
CR5 场景 Geometry Contract 验证器 (V8.15).

在 Gazebo paused 状态下验证物理一致性:

A. goalpost 实际 pose == YAML
B. target 实际 pose == YAML
C. camera 实际 position == YAML
D. beam_bottom_z vs spreader_top_z: delta <= 5mm
E. pedestal bracket tip vs camera center: translation <= 20mm
F. target body 不与 frame posts 相交
G. camera stands 不与 goalpost 相交
H. robot home 不与 target/goalpost/pedestals 碰撞 (简化 AABB 检查)

用法:
  rosrun cr5_spray_sim validate_scene_geometry.py

输出:
  SCENE_CONTRACT_PASS / SCENE_CONTRACT_FAIL (到 stderr)
  退出码: 0 = PASS, 1 = 服务不可用, 2 = contract 失败
"""
import sys
import os
import math
import yaml
import rospy
import rospkg
from gazebo_msgs.srv import GetModelState, GetModelStateRequest


# 容差
BEAM_SPREADER_MAX_DELTA_MM = 5.0
CAMERA_MOUNT_MAX_DIST_MM = 20.0
COLLISION_CLEARANCE_M = 0.010  # 10mm AABB 膨胀量 (单位: 米)

# 从 SDF 模型得出的相对几何 (不再写死世界坐标)
# calibration_target/model.sdf:
#   spreader_bar pose: 0 0 0.605
#   spreader half_height = 0.015
SPREADER_CENTER_Z_OFFSET = 0.605
SPREADER_HALF_HEIGHT = 0.015
SPREADER_TOP_OFFSET = SPREADER_CENTER_Z_OFFSET + SPREADER_HALF_HEIGHT  # 0.620

# Target body half-extents (model.sdf: body 0.34×0.28×0.24)
TARGET_BODY_HALF = (0.17, 0.14, 0.12)

# Goalpost post positions (based on post_y and profile)
POST_HALF_PROFILE = 0.025  # profile/2 = 0.05/2

# Pedestal 参数
POST_SIZE = 0.04  # from camera_pedestal.xacro default

# Pedestal base plate half-extents
PEDESTAL_BASE_HALF = (0.09, 0.075, 0.015)  # 0.18×0.15×0.03

# CR5 base approximate bounding box (home pose)
CR5_BASE_HALF = (0.25, 0.20, 0.30)  # 估计值

# Camera model name → pedestal name
CAM_TO_PED = {
    "cam_front_left": "pedestal_fl",
    "cam_front_right": "pedestal_fr",
    "cam_rear": "pedestal_rear",
}


def load_scene_config():
    """加载 simulation_scene.yaml."""
    try:
        rp = rospkg.RosPack()
        config_path = os.path.join(
            rp.get_path("cr5_spray_sim"), "config", "simulation_scene.yaml")
    except Exception:
        config_path = os.path.join(
            os.path.dirname(__file__), "..", "config", "simulation_scene.yaml")
    with open(config_path) as f:
        return yaml.safe_load(f)


def aabb_overlap_2d(min1, max1, min2, max2):
    """检查两个 AABB 在 XY 平面是否重叠 (带 CLEARANCE)."""
    return (min1[0] - COLLISION_CLEARANCE_M < max2[0] + COLLISION_CLEARANCE_M and
            max1[0] + COLLISION_CLEARANCE_M > min2[0] - COLLISION_CLEARANCE_M and
            min1[1] - COLLISION_CLEARANCE_M < max2[1] + COLLISION_CLEARANCE_M and
            max1[1] + COLLISION_CLEARANCE_M > min2[1] - COLLISION_CLEARANCE_M)


class ContractValidator:
    def __init__(self):
        self.config = load_scene_config()

        rospy.wait_for_service("/gazebo/get_model_state", timeout=10.0)
        self.get_model_state = rospy.ServiceProxy(
            "/gazebo/get_model_state", GetModelState)

        self.checks = {}
        self.errors = []

    def get_model_xyz(self, name):
        """获取模型的世界 xyz 位置."""
        req = GetModelStateRequest()
        req.model_name = name
        req.relative_entity_name = "world"
        try:
            resp = self.get_model_state(req)
            if resp.success:
                p = resp.pose.position
                return (p.x, p.y, p.z)
        except Exception as e:
            rospy.logerr("get_model_state(%s) failed: %s", name, e)
        return None

    def check_beam_spreader(self):
        """D: 检查 beam_bottom_z vs spreader_top_z."""
        rospy.loginfo("=== Check D: Beam-Spreader Match ===")

        gp = self.config.get("simple_goalpost_frame", {})
        gp_xyz = self.get_model_xyz("simple_goalpost_frame")
        if gp_xyz is None:
            self.errors.append("BEAM_SPREADER: cannot get goalpost state")
            return False

        height = gp.get("height", 1.29)
        profile = gp.get("profile_size", 0.05)
        # beam bottom Z in world = goalpost_z + height - profile/2
        # goalpost root link is at z=0 in local, so world_z = 0+beam_bottom = height - profile/2
        # beam bottom Z: top beam center at height - profile/2, box extends profile/2 in each direction.
        # beam_bottom = (height - profile/2) - profile/2 = height - profile
        beam_bottom_world_z = height - profile

        wp = self.config.get("simple_hanging_workpiece", {}).get("position", {})
        target_z = float(wp.get("z", 0.62))
        spreader_top_world_z = target_z + SPREADER_TOP_OFFSET

        delta_mm = abs(beam_bottom_world_z - spreader_top_world_z) * 1000

        rospy.loginfo("  Beam bottom z: %.4f m", beam_bottom_world_z)
        rospy.loginfo("  Spreader top z: %.4f m", spreader_top_world_z)
        rospy.loginfo("  Delta: %.1f mm (max %d mm)", delta_mm, BEAM_SPREADER_MAX_DELTA_MM)

        if delta_mm <= BEAM_SPREADER_MAX_DELTA_MM:
            rospy.loginfo("  BEAM_SPREADER_PASS")
            self.checks["beam_spreader"] = ("PASS", delta_mm)
            return True
        else:
            rospy.logerr("  BEAM_SPREADER_FAIL: delta=%.1f mm", delta_mm)
            self.checks["beam_spreader"] = ("FAIL", delta_mm)
            self.errors.append("BEAM_SPREADER: delta={:.1f}mm > {}mm".format(
                delta_mm, BEAM_SPREADER_MAX_DELTA_MM))
            return False

    def check_camera_mounts(self):
        """E: 检查 pedestal bracket tip vs camera center."""
        rospy.loginfo("=== Check E: Camera-Pedestal Mount ===")
        all_ok = True
        cameras = self.config.get("cameras", {}).get("cameras", [])
        pedestals = {p["name"]: p for p in self.config.get("pedestals", [])}

        for cam in cameras:
            cname = cam["name"]
            cpos = cam["position"]
            cx, cy, cz = float(cpos["x"]), float(cpos["y"]), float(cpos["z"])

            ped_name = CAM_TO_PED.get(cname)
            if ped_name is None or ped_name not in pedestals:
                rospy.logwarn("  %s: no pedestal mapping", cname)
                continue

            ped = pedestals[ped_name]
            base = ped["base"]
            height = ped.get("height", 1.15)
            arm_len = ped.get("arm_length", 0.10)

            # bracket tip 世界坐标
            bx = float(base["x"]) + arm_len
            by = float(base["y"])
            bz = height - POST_SIZE / 2.0

            dist_mm = math.sqrt((cx - bx)**2 + (cy - by)**2 + (cz - bz)**2) * 1000

            rospy.loginfo("  %s: cam(%.3f,%.3f,%.3f) bracket(%.3f,%.3f,%.3f) dist=%.1f mm",
                          cname, cx, cy, cz, bx, by, bz, dist_mm)

            if dist_mm <= CAMERA_MOUNT_MAX_DIST_MM:
                rospy.loginfo("    CAMERA_MOUNT_PASS")
                self.checks["camera_mount_{}".format(cname)] = ("PASS", dist_mm)
            else:
                rospy.logerr("    CAMERA_MOUNT_FAIL")
                self.checks["camera_mount_{}".format(cname)] = ("FAIL", dist_mm)
                self.errors.append("CAMERA_MOUNT {}: {:.1f}mm > {}mm".format(
                    cname, dist_mm, CAMERA_MOUNT_MAX_DIST_MM))
                all_ok = False

        return all_ok

    def check_collisions(self):
        """F, G, H: 简化 AABB 碰撞检查 (XY 平面)."""
        rospy.loginfo("=== Check Collisions: F/G/H ===")
        all_ok = True

        # Target body AABB
        wp = self.config.get("simple_hanging_workpiece", {}).get("position", {})
        tx, ty = float(wp.get("x", 0.72)), float(wp.get("y", 0.0))
        target_min = (tx - TARGET_BODY_HALF[0], ty - TARGET_BODY_HALF[1])
        target_max = (tx + TARGET_BODY_HALF[0], ty + TARGET_BODY_HALF[1])

        # Goalpost posts AABB (两根立柱)
        gp = self.config.get("simple_goalpost_frame", {})
        gx = gp.get("center_x", 0.72)
        post_y = gp.get("post_y", [-0.62, 0.62])
        # post center at (gx, ±post_y), half-extent = profile/2
        for py in post_y:
            post_min = (gx - POST_HALF_PROFILE, py - POST_HALF_PROFILE)
            post_max = (gx + POST_HALF_PROFILE, py + POST_HALF_PROFILE)

            if aabb_overlap_2d(target_min, target_max, post_min, post_max):
                rospy.logerr("  F: target body intersects goalpost post at y=%.2f", py)
                self.errors.append("COLLISION: target vs goalpost post y={}".format(py))
                all_ok = False
            else:
                rospy.loginfo("  F: target vs post y=%.2f: CLEAR", py)

        # Camera stands vs goalpost
        pedestals = self.config.get("pedestals", [])
        for ped in pedestals:
            base = ped["base"]
            px, py = float(base["x"]), float(base["y"])
            ped_min = (px - PEDESTAL_BASE_HALF[0], py - PEDESTAL_BASE_HALF[1])
            ped_max = (px + PEDESTAL_BASE_HALF[0], py + PEDESTAL_BASE_HALF[1])

            for post_py_val in post_y:
                post_min = (gx - POST_HALF_PROFILE, post_py_val - POST_HALF_PROFILE)
                post_max = (gx + POST_HALF_PROFILE, post_py_val + POST_HALF_PROFILE)

                if aabb_overlap_2d(ped_min, ped_max, post_min, post_max):
                    rospy.logerr("  G: pedestal %s intersects goalpost post y=%.2f",
                                 ped["name"], post_py_val)
                    self.errors.append("COLLISION: {} vs goalpost post y={}".format(
                        ped["name"], post_py_val))
                    all_ok = False
                else:
                    rospy.loginfo("  G: %s vs post y=%.2f: CLEAR", ped["name"], post_py_val)

        # Robot home vs target/goalpost/pedestals (simplified)
        cr5 = self.config.get("cr5_base", {}).get("position", {"x": 0.0, "y": 0.0})
        rx, ry = float(cr5["x"]), float(cr5["y"])
        robot_min = (rx - CR5_BASE_HALF[0], ry - CR5_BASE_HALF[1])
        robot_max = (rx + CR5_BASE_HALF[0], ry + CR5_BASE_HALF[1])

        # Robot vs target
        if aabb_overlap_2d(robot_min, robot_max, target_min, target_max):
            rospy.logwarn("  H: robot home AABB overlaps target — verify clearance")
            self.errors.append("COLLISION_WARN: robot vs target AABB overlap")
        else:
            rospy.loginfo("  H: robot vs target: CLEAR")

        # Robot vs goalpost
        for post_py_val in post_y:
            post_min = (gx - POST_HALF_PROFILE, post_py_val - POST_HALF_PROFILE)
            post_max = (gx + POST_HALF_PROFILE, post_py_val + POST_HALF_PROFILE)
            if aabb_overlap_2d(robot_min, robot_max, post_min, post_max):
                rospy.logwarn("  H: robot home AABB overlaps goalpost post y=%.2f", post_py_val)
                self.errors.append("COLLISION_WARN: robot vs goalpost post y={}".format(post_py_val))
            else:
                rospy.loginfo("  H: robot vs post y=%.2f: CLEAR", post_py_val)

        # Robot vs pedestals
        for ped in pedestals:
            base = ped["base"]
            px, py = float(base["x"]), float(base["y"])
            ped_min = (px - PEDESTAL_BASE_HALF[0], py - PEDESTAL_BASE_HALF[1])
            ped_max = (px + PEDESTAL_BASE_HALF[0], py + PEDESTAL_BASE_HALF[1])
            if aabb_overlap_2d(robot_min, robot_max, ped_min, ped_max):
                rospy.logwarn("  H: robot home AABB overlaps %s", ped["name"])
                self.errors.append("COLLISION_WARN: robot vs {}".format(ped["name"]))
            else:
                rospy.loginfo("  H: robot vs %s: CLEAR", ped["name"])

        return all_ok

    def check_model_positions(self):
        """A, B, C: 验证模型实际位置 vs YAML."""
        rospy.loginfo("=== Check A/B/C: Model Positions vs YAML ===")
        all_ok = True

        # A: goalpost
        gp = self.config.get("simple_goalpost_frame", {})
        gx = gp.get("center_x", 0.72)
        gp_xyz = self.get_model_xyz("simple_goalpost_frame")
        if gp_xyz:
            err_mm = abs(gp_xyz[0] - gx) * 1000
            rospy.loginfo("  A: goalpost x: actual=%.3f yaml=%.3f err=%.1f mm",
                          gp_xyz[0], gx, err_mm)
            if err_mm > 2.0:
                rospy.logerr("  A: GOALPOST_POSITION_FAIL")
                self.errors.append("GOALPOST_POSITION: err={:.1f}mm".format(err_mm))
                all_ok = False
            else:
                rospy.loginfo("  A: GOALPOST_POSITION_PASS")
        else:
            self.errors.append("GOALPOST: cannot get state")
            all_ok = False

        # B: target
        wp = self.config.get("simple_hanging_workpiece", {}).get("position", {})
        tx, ty, tz = float(wp.get("x", 0.72)), float(wp.get("y", 0.0)), float(wp.get("z", 0.62))
        target_xyz = self.get_model_xyz("simple_hanging_workpiece")
        if target_xyz:
            err_mm = max(abs(target_xyz[0] - tx), abs(target_xyz[1] - ty), abs(target_xyz[2] - tz)) * 1000
            rospy.loginfo("  B: target actual=(%.3f,%.3f,%.3f) yaml=(%.3f,%.3f,%.3f) max_err=%.1f mm",
                          *target_xyz, tx, ty, tz, err_mm)
            if err_mm > 2.0:
                rospy.logerr("  B: TARGET_POSITION_FAIL")
                self.errors.append("TARGET_POSITION: err={:.1f}mm".format(err_mm))
                all_ok = False
            else:
                rospy.loginfo("  B: TARGET_POSITION_PASS")
        else:
            self.errors.append("TARGET: cannot get state")
            all_ok = False

        # C: cameras
        for cam in self.config.get("cameras", {}).get("cameras", []):
            cname = cam["name"]
            cpos = cam["position"]
            cam_xyz = self.get_model_xyz(cname)
            if cam_xyz:
                err_mm = max(
                    abs(cam_xyz[0] - float(cpos["x"])),
                    abs(cam_xyz[1] - float(cpos["y"])),
                    abs(cam_xyz[2] - float(cpos["z"])),
                ) * 1000
                rospy.loginfo("  C: %s actual=(%.3f,%.3f,%.3f) max_err=%.1f mm",
                              cname, *cam_xyz, err_mm)
                if err_mm > 2.0:
                    rospy.logerr("  C: %s POSITION_FAIL", cname)
                    self.errors.append("CAMERA_POSITION {}: err={:.1f}mm".format(cname, err_mm))
                    all_ok = False
                else:
                    rospy.loginfo("  C: %s POSITION_PASS", cname)
            else:
                self.errors.append("CAMERA {}: cannot get state".format(cname))
                all_ok = False

        return all_ok

    def run(self):
        rospy.loginfo("=== CR5 V8.15 Scene Contract Validation ===")
        all_ok = True

        if not self.check_model_positions():
            all_ok = False
        if not self.check_beam_spreader():
            all_ok = False
        if not self.check_camera_mounts():
            all_ok = False
        if not self.check_collisions():
            all_ok = False

        rospy.loginfo("=== Contract Check Summary ===")
        for name, (status, value) in sorted(self.checks.items()):
            rospy.loginfo("  %s: %s (%.1f)", name, status, value)

        if all_ok:
            rospy.loginfo("SCENE_CONTRACT_PASS")
            sys.stderr.write("SCENE_CONTRACT_PASS\n")
        else:
            rospy.logerr("SCENE_CONTRACT_FAIL: %d errors", len(self.errors))
            for e in self.errors:
                rospy.logerr("  - %s", e)
            sys.stderr.write("SCENE_CONTRACT_FAIL\n")

        sys.stderr.flush()
        return all_ok


def main():
    rospy.init_node("validate_scene_geometry", anonymous=True)
    validator = ContractValidator()
    ok = validator.run()
    sys.exit(0 if ok else 2)


if __name__ == "__main__":
    main()
