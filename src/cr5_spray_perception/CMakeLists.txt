cmake_minimum_required(VERSION 3.5)
project(cr5_spray_perception)

find_package(catkin REQUIRED)

catkin_package()

# Python package — cr5_spray_perception.aruco_compat
catkin_python_setup()

# ── C++ Ceres Bundle Adjustment ──
find_package(Ceres REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

add_executable(ceres_ba_optimizer
  src/ceres_ba_optimizer.cpp
)
target_link_libraries(ceres_ba_optimizer
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
)

# ── Install ──
install(TARGETS ceres_ba_optimizer
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  # 标定感知 — 可见性验证与外参估计
  scripts/validate_calibration_visibility.py
  scripts/estimate_camera_extrinsics.py
  # 采集 — 单相机 + 跨相机联合
  scripts/capture_manager.py
  scripts/joint_capture_manager.py
  # Bundle Adjustment + 多帧标定工作流
  scripts/bundle_adjustment.py
  scripts/run_multi_frame_calibration.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# TSDF 重建与评价 (正式启用)
install(PROGRAMS
  scripts/reconstruct_tsdf.py
  scripts/evaluate_reconstruction.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
