cmake_minimum_required(VERSION 3.0.2)
project(fer_emotion)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  social_msgs
  cv_bridge
  message_filters
  zed_sdk_bridge
)

catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs social_msgs cv_bridge message_filters zed_sdk_bridge
)

catkin_install_python(PROGRAMS
  scripts/emotion_fer_backend.py
  scripts/emotion_detection_node.py
  scripts/test_fer_webcam.py
  scripts/download_fer_model.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
  FILES_MATCHING PATTERN "*.onnx"
)
