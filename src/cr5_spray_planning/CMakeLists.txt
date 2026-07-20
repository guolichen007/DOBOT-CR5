cmake_minimum_required(VERSION 3.5)
project(cr5_spray_planning)
find_package(catkin REQUIRED)
catkin_package()
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(PROGRAMS scripts/spray_path_generator.py
  scripts/spray_coverage_eval.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
