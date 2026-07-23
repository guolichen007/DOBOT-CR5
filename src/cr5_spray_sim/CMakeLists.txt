cmake_minimum_required(VERSION 3.5)
project(cr5_spray_sim)

find_package(catkin REQUIRED)

catkin_package()

# Python package — cr5_spray_sim.aruco_compat
catkin_python_setup()

# Resource directories
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

install(DIRECTORY models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Python scripts — 标定与仿真核心
catkin_install_python(PROGRAMS
  # 相机与对象
  scripts/spawn_fixed_cameras.py
  scripts/control_object_yaw.py
  scripts/publish_object_pose.py
  # 场景引导
  scripts/audit_sim_processes.py
  scripts/bootstrap_controllers.py
  scripts/wait_scene_models.py
  scripts/capture_scene_snapshot.py
  # 场景检查
  scripts/check_scene_geometry.py
  scripts/check_scene_runtime.py
  scripts/check_runtime_signals.py
  # 喷涂
  scripts/spray_simulator.py
  # 标定目标 (仿真模型职责)
  scripts/publish_calibration_target_frames.py
  scripts/validate_calibration_target_geometry.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Shell tools
install(PROGRAMS
  scripts/run_simulation.sh
  scripts/use_simulation_session.sh
  scripts/run_in_simulation_session.sh
  scripts/validate_calibration_texture.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
