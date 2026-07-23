cmake_minimum_required(VERSION 3.5)
project(cr5_spray_perception)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  # 标定感知 — 可见性验证与外参估计
  scripts/validate_calibration_visibility.py
  scripts/estimate_camera_extrinsics.py
  # 下一阶段：三相机同步采集入口
  scripts/capture_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Experimental — 下一阶段 TSDF 重建与评价 (not production ready)
# install(PROGRAMS
#   scripts/reconstruct_tsdf.py
#   scripts/evaluate_reconstruction.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
