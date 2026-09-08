cmake_minimum_required(VERSION 3.5)

project(zed_sdk_bridge)

## Generate symbols for IDE indexer
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# if CMAKE_BUILD_TYPE is not specified, take 'Release' as default
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    geometry_msgs
    nav_msgs
    std_msgs
    message_generation
    cv_bridge
    image_transport
    visualization_msgs
    pcl_ros
    pcl_conversions
    tf2
    tf2_ros
    tf2_geometry_msgs
    social_msgs
)

## Find CUDA (required by ZED SDK)
find_package(CUDA)
if(NOT CUDA_FOUND)
    message(FATAL_ERROR "CUDA not found. Please install CUDA from: https://developer.nvidia.com/cuda-downloads")
endif()

## Find ZED SDK
find_package(ZED 4)
if(NOT ZED_FOUND)
    # Fallback: try to find manually
    if(DEFINED ENV{ZED_SDK_ROOT})
        set(ZED_SDK_ROOT $ENV{ZED_SDK_ROOT})
    else()
        set(ZED_SDK_ROOT "/usr/local/zed")
    endif()

    find_path(ZED_INCLUDE_DIR
        NAMES sl/Camera.hpp
        PATHS ${ZED_SDK_ROOT}/include
        /usr/local/zed/include
        /opt/zed/include
    )

    find_library(ZED_LIBRARY
        NAMES sl_zed sl_zed64
        PATHS ${ZED_SDK_ROOT}/lib
        /usr/local/zed/lib
        /opt/zed/lib
    )

    if(NOT ZED_INCLUDE_DIR OR NOT ZED_LIBRARY)
        message(FATAL_ERROR "ZED SDK not found. Please install ZED SDK or set ZED_SDK_ROOT environment variable.")
    endif()
    
    set(ZED_INCLUDE_DIRS ${ZED_INCLUDE_DIR})
    set(ZED_LIBRARIES ${ZED_LIBRARY})
else()
    set(ZED_INCLUDE_DIRS ${ZED_INCLUDE_DIRS})
    set(ZED_LIBRARIES ${ZED_LIBRARIES})
endif()

message(STATUS "Found CUDA: ${CUDA_VERSION}")
message(STATUS "Found ZED SDK")
message(STATUS "ZED Include: ${ZED_INCLUDE_DIRS}")
message(STATUS "ZED Libraries: ${ZED_LIBRARIES}")

## Generate messages
add_message_files(
    FILES
    Body.msg
    BodiesStamped.msg
    Object.msg
    ObjectsStamped.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs
    geometry_msgs
)

## Declare a catkin package
catkin_package(
    CATKIN_DEPENDS
        roscpp
        sensor_msgs
        geometry_msgs
        nav_msgs
        std_msgs
        message_runtime
        cv_bridge
        image_transport
        visualization_msgs
        pcl_ros
        pcl_conversions
        tf2
        tf2_ros
        tf2_geometry_msgs
        social_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${ZED_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

## Build executables
add_executable(zed_sdk_bridge_node
    src/zed_sdk_bridge_node.cpp
)

add_executable(skeleton_visualizer
    src/skeleton_visualizer.cpp
)

add_executable(proxemic_pointcloud_node
    src/proxemic_pointcloud_node.cpp
)

add_executable(skeleton_costmap_3d_node
    src/skeleton_costmap_3d_node.cpp
)

target_link_libraries(zed_sdk_bridge_node
    ${catkin_LIBRARIES}
    ${ZED_LIBRARIES}
    ${CUDA_LIBRARIES}
    ${OpenCV_LIBS}
)

target_link_libraries(skeleton_visualizer
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)

target_link_libraries(proxemic_pointcloud_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

target_link_libraries(skeleton_costmap_3d_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

add_dependencies(zed_sdk_bridge_node
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_generate_messages_cpp
)

add_dependencies(skeleton_visualizer
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_generate_messages_cpp
)

add_dependencies(proxemic_pointcloud_node
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_generate_messages_cpp
)

add_dependencies(skeleton_costmap_3d_node
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_generate_messages_cpp
)

## Install
install(TARGETS zed_sdk_bridge_node skeleton_visualizer proxemic_pointcloud_node skeleton_costmap_3d_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

