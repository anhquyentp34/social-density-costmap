cmake_minimum_required(VERSION 3.0.2)
project(x_omni4wd_slam)

## Launch/config/scripts package for SLAM + odom fusion.
## RF2O algorithm lives in rf2o_laser_odometry (research package).

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros
)

###########
## Install ##
###########

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY docs/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/docs
  PATTERN ".svn" EXCLUDE
)

catkin_install_python(PROGRAMS
  scripts/wheel_odometry_node.py
  scripts/publish_cmd_vel_odom_test.py
  scripts/scan_deskew_node.py
  scripts/scan_static_filter_node.py
  scripts/reliability_node.py
  scripts/odom_livo_relay.py
  scripts/gtsam_livo_node.py
  scripts/imu_yaw_scale_node.py
  scripts/odom_xy_scale_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/record_odom_mode_bag.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
