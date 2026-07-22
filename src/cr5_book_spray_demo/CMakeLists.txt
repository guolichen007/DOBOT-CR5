cmake_minimum_required(VERSION 3.5)
project(cr5_book_spray_demo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  std_msgs
  std_srvs
  visualization_msgs
  cv_bridge
  image_transport
  message_filters
  tf
  tf2_ros
  tf2_geometry_msgs
  moveit_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/book_pose_estimator.py
  scripts/book_spray_planner.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config docs rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
