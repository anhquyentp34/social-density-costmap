project(srf_laser_odometry)

CMAKE_MINIMUM_REQUIRED(VERSION 3.3)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nav_msgs
  sensor_msgs
  std_msgs
  tf
)

## System dependencies
find_package(Boost REQUIRED COMPONENTS system)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)

# MRPT 2.x (ros-noetic-mrpt2). Imported targets via ${MRPT_LIBRARIES}/${MRPT_LIBS}.
find_package(MRPT REQUIRED COMPONENTS poses obs maps slam math)
message(STATUS "MRPT_VERSION=${MRPT_VERSION}")
message(STATUS "MRPT_LIBRARIES=${MRPT_LIBRARIES}")
message(STATUS "MRPT_LIBS=${MRPT_LIBS}")
message(STATUS "MRPT_INCLUDE_DIRS=${MRPT_INCLUDE_DIRS}")

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS nav_msgs roscpp sensor_msgs std_msgs tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${MRPT_INCLUDE_DIRS}
)

## Main node (required)
add_executable(srf_laser_odometry_node
  src/srf_node.cpp
  src/srf_node.h
  src/laser_odometry_refscans.cpp
  src/laser_odometry_refscans.h
)

if(MRPT_LIBRARIES)
  set(_SRF_MRPT_LIBS ${MRPT_LIBRARIES})
elseif(MRPT_LIBS)
  set(_SRF_MRPT_LIBS ${MRPT_LIBS})
endif()

target_link_libraries(srf_laser_odometry_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${_SRF_MRPT_LIBS}
)

## Optional helper node (disabled by default; source kept for research)
option(BUILD_SRF_TO_TXT "Build the optional SRF text export node" OFF)
if(BUILD_SRF_TO_TXT AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/srf_to_txt_node.cpp")
  add_executable(srf_to_txt_node src/srf_to_txt_node.cpp)
  target_link_libraries(srf_to_txt_node
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${_SRF_MRPT_LIBS}
  )
endif()
