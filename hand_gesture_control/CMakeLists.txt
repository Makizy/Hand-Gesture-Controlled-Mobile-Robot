cmake_minimum_required(VERSION 3.0.2)
project(hand_gesture_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
)

catkin_package()

catkin_install_python(PROGRAMS src/hand_gesture.py
src/control.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)




