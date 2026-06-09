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

# Only the node entry-point is an executable. sensors.py / data_recorder.py /
# realsense_camera.py are import-only helper modules — installing them as
# PROGRAMS makes catkin generate relay wrappers that can't be imported as
# modules (the relay execs the source into a throwaway dict, so symbols like
# TactileReader never attach to the module). The node imports them from its own
# source dir via sys.path, so they must NOT be listed here.
catkin_install_python(PROGRAMS
  scripts/test_touchx_teleop_viz.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
