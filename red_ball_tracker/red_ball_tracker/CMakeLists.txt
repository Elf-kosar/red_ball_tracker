cmake_minimum_required(VERSION 3.0.2)
project(red_ball_tracker)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Python scriptlerin executable olarak kurulum için:
catkin_install_python(PROGRAMS
  scripts/red_ball_tracker.py
  scripts/camera_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
