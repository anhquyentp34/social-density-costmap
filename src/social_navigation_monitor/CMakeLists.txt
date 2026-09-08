cmake_minimum_required(VERSION 3.0.2)
project(social_navigation_monitor)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  actionlib_msgs
  social_msgs
  message_generation
)

add_message_files(
  FILES
  SII.msg
  SGI.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  social_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime rospy std_msgs geometry_msgs nav_msgs actionlib_msgs social_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/sgi_calculator.py
  scripts/sii_calculator.py
  scripts/move_base_status_timeline.py
  scripts/path_planning_monitor.py
  scripts/social_navigation_dashboard.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  OPTIONAL
)
