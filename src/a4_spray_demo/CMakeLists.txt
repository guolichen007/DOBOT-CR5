cmake_minimum_required(VERSION 3.0.2)
project(a4_spray_demo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  moveit_commander
  moveit_msgs
  tf
  dobot_bringup
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    visualization_msgs
    moveit_commander
    moveit_msgs
    tf
    dobot_bringup
)

catkin_install_python(
  PROGRAMS
    scripts/a4_raster_demo.py
    scripts/quick_check.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
