#!/usr/bin/env python3
"""
V3.3.3 Gazebo Spray Plume Visualizer.

在 Gazebo Classic 中显示半透明喷雾锥:
- 由 5 段同心锥台组成，叠加成完整喷雾锥效果
- 无 collision、无重力 (static model)
- OFF → 隐藏到地下 (z=-100)
- SPRAYING → 根据 spray_nozzle_frame 更新模型姿态
- NO_TARGET / OUT_OF_RANGE / BAD_INCIDENCE → 不显示
- 使用 /gazebo/set_model_state 更新姿态 (不重新生成模型)

注意:
- 本节点只负责 Gazebo 视觉，RViz 中的 Marker 仍由 spray_simulator_v33.py 发布
- Gazebo 中不显示漆层累积 (paint patches)，只显示当前喷雾锥
"""
import sys
import math
import time
import rospy
import tf2_ros
import numpy as np
from std_msgs.msg import String
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SpawnModel, DeleteModel, SetModelState, SetModelStateRequest
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped

# 喷雾锥 SDF 模板
PLUME_SDF = """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{model_name}">
    <static>true</static>
    {links}
  </model>
</sdf>"""

# 单段锥台的 visual link
LINK_SDF = """
    <link name="plume_seg_{i}">
      <visual name="plume_vis_{i}">
        <pose>0 0 {z_center} 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>{length}</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>{r} {g} {b} {a}</ambient>
          <diffuse>{r} {g} {b} {a}</diffuse>
          <emissive>{er} {eg} {eb} 0.0</emissive>
        </material>
      </visual>
    </link>"""

MODEL_NAME = "spray_plume_visual"
HIDE_Z = -100.0


class GazeboSprayVisual:
    def __init__(self):
        rospy.init_node("gazebo_spray_visual_v333")

        self.nominal_standoff = rospy.get_param("~nominal_standoff_m", 0.18)
        self.cone_half_angle = math.radians(rospy.get_param("~cone_half_angle_deg", 15.0))
        self.paint_color = rospy.get_param("~paint_color", [0.10, 0.35, 0.75])
        self.num_segments = rospy.get_param("~num_segments", 5)
        self.update_rate = rospy.get_param("~update_rate_hz", 20.0)
        self.alpha = rospy.get_param("~alpha", 0.22)

        self.nozzle_frame = "spray_nozzle_frame"
        self.spawned = False
        self.current_spray_state = "OFF"
        self.latest_hit_distance = self.nominal_standoff

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # 订阅喷涂状态
        self._state_sub = rospy.Subscriber(
            "/spray_demo/state", String, self._state_callback, queue_size=5)

        # V3.3.4: 订阅诊断 test plume pose (用于 Gazebo 视觉验证)
        self.test_plume_active = False
        self.test_plume_pose = None
        self._test_plume_sub = rospy.Subscriber(
            "/spray_demo/test_plume_pose", PoseStamped,
            self._test_plume_callback, queue_size=5)

        rospy.loginfo("Gazebo Spray Visual V3.3.3 ready")
        rospy.loginfo("  model=%s segments=%d nominal_dist=%.2fm alpha=%.2f",
                      MODEL_NAME, self.num_segments, self.nominal_standoff, self.alpha)

    def _state_callback(self, msg):
        self.current_spray_state = msg.data

    def _test_plume_callback(self, msg):
        """V3.3.4: 接收诊断 test plume 指令."""
        # 如果 pose.z 被设为 -100，表示隐藏
        if msg.pose.position.z < -50.0:
            self.test_plume_active = False
            self.test_plume_pose = None
        else:
            self.test_plume_active = True
            self.test_plume_pose = msg

    def _build_sdf(self):
        """Build the multi-segment spray plume SDF."""
        r, g, b = self.paint_color
        a = self.alpha
        er, eg, eb = r * 0.3, g * 0.3, b * 0.3

        # 计算锥体几何
        total_length = self.nominal_standoff
        base_radius = total_length * math.tan(self.cone_half_angle)
        seg_length = total_length / self.num_segments

        links = []
        for i in range(self.num_segments):
            # 每段的半径: 从 nozzle 向外线性增加
            t_start = i / self.num_segments
            t_end = (i + 1) / self.num_segments
            avg_t = (t_start + t_end) / 2.0
            radius = base_radius * avg_t
            z_center = seg_length * (i + 0.5)

            # alpha 从 nozzle 向外递减
            seg_alpha = a * (1.0 - avg_t * 0.5)

            links.append(LINK_SDF.format(
                i=i,
                z_center=z_center,
                radius=radius,
                length=seg_length,
                r=r, g=g, b=b,
                a=seg_alpha,
                er=er, eg=eg, eb=eb))

        return PLUME_SDF.format(model_name=MODEL_NAME, links="".join(links))

    def spawn_model(self):
        """Spawn the plume model in Gazebo (hidden underground)."""
        rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=15.0)
        spawn = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

        sdf = self._build_sdf()
        pose = Pose(position=Point(x=0, y=0, z=HIDE_Z),
                    orientation=Quaternion(w=1.0))

        try:
            resp = spawn(MODEL_NAME, sdf, "spray_plume_ns", pose, "world")
            if resp.success:
                rospy.loginfo("Spray plume model spawned (hidden)")
                self.spawned = True
                return True
            else:
                rospy.logerr("Failed to spawn plume: %s", resp.status_message)
                return False
        except rospy.ServiceException as e:
            # 可能已经存在，尝试删除再创建
            rospy.logwarn("Spawn service call failed: %s (model may already exist)", e)
            return False

    def delete_model(self):
        """Remove the plume model from Gazebo."""
        try:
            rospy.wait_for_service("/gazebo/delete_model", timeout=3.0)
            delete = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
            delete(MODEL_NAME)
        except Exception:
            pass
        self.spawned = False

    def update_pose(self):
        """Update plume pose based on current spray state."""
        if not self.spawned:
            return

        srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        req = SetModelStateRequest()

        # V3.3.4: 优先处理诊断 test plume
        if self.test_plume_active and self.test_plume_pose is not None:
            req.model_state.pose = self.test_plume_pose.pose
        elif self.current_spray_state == "SPRAYING":
            # 获取 nozzle 姿态
            try:
                t = self.tf_buffer.lookup_transform(
                    "world", self.nozzle_frame, rospy.Time(0), rospy.Duration(0.0))
                req.model_state.pose.position = t.transform.translation
                req.model_state.pose.orientation = t.transform.rotation
            except Exception:
                # TF 不可用，隐藏
                req.model_state.pose.position.z = HIDE_Z
                req.model_state.pose.orientation.w = 1.0
        else:
            # 所有非 SPRAYING 状态都隐藏
            req.model_state.pose.position.z = HIDE_Z
            req.model_state.pose.orientation.w = 1.0

        req.model_state.model_name = MODEL_NAME
        req.model_state.reference_frame = "world"
        try:
            srv(req)
        except rospy.ServiceException:
            pass

    def run(self):
        # 等待 Gazebo 就绪
        rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=30.0)

        # 清理可能残留的旧模型
        self.delete_model()
        rospy.sleep(1.0)

        # Spawn 模型
        if not self.spawn_model():
            rospy.logerr("Cannot spawn spray plume model, exiting")
            sys.exit(1)

        rate = rospy.Rate(self.update_rate)
        last_state = "OFF"

        while not rospy.is_shutdown():
            if self.current_spray_state != last_state:
                rospy.loginfo("Spray state: %s → %s", last_state, self.current_spray_state)
                last_state = self.current_spray_state

            self.update_pose()
            rate.sleep()

        # Cleanup
        self.delete_model()


if __name__ == "__main__":
    GazeboSprayVisual().run()
