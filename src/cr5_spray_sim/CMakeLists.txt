cmake_minimum_required(VERSION 3.5)
project(cr5_spray_sim)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Python scripts
catkin_install_python(PROGRAMS
  scripts/spawn_scene_cameras.py
  scripts/object_yaw_controller.py
  scripts/validate_scene.py
  scripts/execution_safety_check.py
  scripts/spawn_cameras_v2.py
  scripts/object_yaw_v2.py
  scripts/scene_tf_v2.py
  scripts/compute_look_at.py
  scripts/scene_v2_preflight.py
  scripts/object_pose_tf_v2.py
  # V3.1
  scripts/spawn_cameras_v31.py
  scripts/object_yaw_v31.py
  scripts/object_pose_tf_v31.py
  # V3.3
  scripts/spray_simulator_v33.py
  # V3.3.1
  scripts/check_tf_once_v331.py
  scripts/check_scene_v331.py
  # V3.3.2
  scripts/initialize_cr5_pose_v332.py
  scripts/check_scene_v332.py
  # V3.3.3
  scripts/check_cr5_runtime_ready_v333.py
  scripts/unpause_and_verify_clock_v333.py
  scripts/check_camera_streams_v333.py
  scripts/gazebo_spray_visual_v333.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Shell launchers and tools
install(PROGRAMS
  scripts/run_scene_v2.sh
  scripts/run_scene_v31_simple.sh
  scripts/run_scene_v33_spray.sh
  scripts/use_spray_session_v33.sh
  # V3.3.3
  scripts/show_camera_status_v333.sh
  scripts/open_spray_rviz_v333.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
