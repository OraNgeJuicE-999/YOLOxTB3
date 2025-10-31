cmake_minimum_required(VERSION 3.0.2)
project(led_control_pkg)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/led_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
