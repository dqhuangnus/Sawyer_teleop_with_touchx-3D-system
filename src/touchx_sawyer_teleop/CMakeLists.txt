cmake_minimum_required(VERSION 3.0.2)
project(touchx_sawyer_teleop)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  intera_interface
  intera_core_msgs
  robotiq_2f_gripper_msgs
  actionlib
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/test_touchx_teleop_viz.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
