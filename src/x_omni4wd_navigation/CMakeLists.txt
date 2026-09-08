cmake_minimum_required(VERSION 3.0.2)
project(x_omni4wd_navigation)

find_package(catkin REQUIRED COMPONENTS
  amcl
  map_server
  move_base
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY config launch maps rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
