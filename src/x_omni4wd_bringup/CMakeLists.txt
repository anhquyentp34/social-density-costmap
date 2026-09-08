cmake_minimum_required(VERSION 3.0.2)
project(x_omni4wd_bringup)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/arduino_interface_omni.py
  scripts/arduino_interface_omni_wheel_odom.py
  scripts/system_health_monitor.py
  scripts/hardware_shutdown_guard.py
  scripts/stop_lidar_motor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config udev rviz firmware
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
