#!/usr/bin/env bash
# Dừng HẲN stack Track D real (exp1–7): stop LiDAR motor + kill ROS/bringup.
# Không bắt buộc source devel (ổ đầy vẫn dừng được).
#
#   ./tracks/D-density-costmap/stop_real_track_d_all.sh
set +e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${WORKSPACE_PATH:-/home/quyenanh/omni4wd_ws}"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
export PATH="/opt/ros/noetic/bin:${PATH}"

echo "🛑 Dừng toàn bộ Track D real (LiDAR + ZED + ROS)..."

# 1) Zero cmd + cancel goal (nếu master còn)
timeout 0.8 rostopic pub -r 25 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' \
  >/dev/null 2>&1
timeout 1 rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID '{}' >/dev/null 2>&1

# 2) Tắt motor LiDAR TRƯỚC khi kill node
timeout 2 rosservice call /stop_motor >/dev/null 2>&1
timeout 2 rosservice call /rplidarNode/stop_motor >/dev/null 2>&1
python3 "$WS/src/x_omni4wd_bringup/scripts/stop_lidar_motor.py" 2>/dev/null
python3 "$WS/src/x_omni4wd_bringup/scripts/stop_lidar_motor.py" --serial-only 2>/dev/null

# 3) SIGINT graceful cho roslaunch
pkill -INT -f 'x_omni4wd_navigation_real_track_d|real_D_exp[0-9]|social_map_annotation' 2>/dev/null
sleep 3

# 4) TERM các node phần cứng / nav
for pat in \
  'rplidar_ros/rplidarNode' 'rplidarNode' \
  'zed_sdk_bridge' 'skeleton_visualizer' \
  'arduino_interface_omni' \
  'move_base' 'amcl' 'map_server' \
  'social_density_fusion' 'goal_sender' \
  'rf2o_laser_odometry' 'srf_laser_odometry' \
  'hardware_shutdown_guard' \
  'social_navigation_dashboard' 'sgi_calculator' 'sii_calculator' \
  'path_density_weight_controller' \
  '/opt/ros/noetic/lib/rviz/rviz' \
  'rosmaster --core' 'roslaunch'
do
  pkill -TERM -f "$pat" 2>/dev/null
done
sleep 2

# 5) Serial DTR lần nữa + KILL sót
python3 "$WS/src/x_omni4wd_bringup/scripts/stop_lidar_motor.py" --serial-only 2>/dev/null
for pat in 'rplidarNode' 'zed_sdk_bridge' 'arduino_interface_omni' 'rosmaster --core' 'roslaunch'; do
  pkill -KILL -f "$pat" 2>/dev/null
done

# 6) Arduino STOP qua serial
python3 - <<'PY' 2>/dev/null
import os
try:
    import serial
except ImportError:
    raise SystemExit(0)
for port in ("/dev/robot_arduino", "/dev/ttyACM0", "/dev/ttyUSB0"):
    if not os.path.exists(port):
        continue
    try:
        s = serial.Serial(port, 19200, timeout=0.3, write_timeout=0.3)
        s.write(b"STOP\nVXYW 0 0 0\nSTOP\n")
        s.flush()
        s.close()
    except Exception:
        pass
PY

echo "✅ Đã gửi lệnh dừng. Kiểm tra LiDAR không còn quay."
pgrep -af 'rplidar|roslaunch|rosmaster --core' 2>/dev/null | grep -vE 'pgrep|stop_real' || echo "   (không còn process ROS/LiDAR)"
