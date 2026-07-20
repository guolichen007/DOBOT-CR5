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
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
