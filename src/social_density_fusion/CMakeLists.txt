cmake_minimum_required(VERSION 3.0.2)
project(social_density_fusion)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  gazebo_msgs
  visualization_msgs
  social_msgs
  tf2_ros
  std_srvs
  message_generation
)

add_message_files(
  FILES
  DensityInfo.msg
  PedestrianMeasurement.msg
  PedestrianMeasurementArray.msg
  PersonVelocityDebug.msg
  PersonVelocityDebugArray.msg
  VelocityCompensationDebug.msg
  VelocityCompensationDebugArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs nav_msgs tf2_ros social_msgs
)

catkin_install_python(PROGRAMS
  scripts/social_density_fusion_node.py
  scripts/social_density_fusion_node_65.py
  scripts/social_density_fusion_node_14_15.py
  scripts/social_density_fusion_node_zed.py
  scripts/social_density_fusion_node_zed_pos.py
  scripts/scenario_gt_social_state_publisher.py
  scripts/velocity_compensation.py
  scripts/velocity_compensation_pos.py
  scripts/lidar_map_polygon_node.py
  scripts/social_map_annotation_editor.py
  scripts/export_annotation_to_canonical.py
  scripts/layer_weights_logger.py
  scripts/analyze_zed_velocity_bag.py
  scripts/analyze_zed_person_velocity_stationary_robot.py
  scripts/auto_verify_velocity_compensation.py
  scripts/record_zed_velocity_test.sh
  scripts/record_zed_person_velocity_stationary_robot.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
