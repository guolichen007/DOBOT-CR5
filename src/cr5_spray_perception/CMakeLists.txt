cmake_minimum_required(VERSION 3.5)
project(cr5_spray_perception)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/capture_manager.py
  scripts/reconstruct_tsdf.py
  scripts/evaluate_reconstruction.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
