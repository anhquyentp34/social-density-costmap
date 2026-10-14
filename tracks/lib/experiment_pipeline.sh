#!/usr/bin/env bash
# Thư viện pipeline thí nghiệm — mirror run_simulation_experiment_ICR2026.sh
# Mỗi script track gọi: cấu hình biến PIPE_* → pipeline_run
#
# Output chuẩn báo cáo:
#   bags/<TRACK>/<EXP>.bag
#   videos/<TRACK>/<EXP>_screen.mp4
#   csv/<TRACK>/<EXP>/  (+ data_config.m, timeline CSV)
#   runs/<TRACK>/<EXP>.log

[[ -n "${_EXPERIMENT_PIPELINE_LOADED:-}" ]] && return 0
_EXPERIMENT_PIPELINE_LOADED=1

PIPE_WS="${PIPE_WS:-${WORKSPACE_PATH:-/home/quyenanh/omni4wd_ws}}"
PIPE_TRACK="${PIPE_TRACK:-unknown-track}"
PIPE_EXPERIMENT_NAME="${PIPE_EXPERIMENT_NAME:-exp_$(date +%Y%m%d_%H%M%S)}"
PIPE_DURATION_MINUTES="${PIPE_DURATION_MINUTES:-0}"
PIPE_MODE="${PIPE_MODE:-real}"          # real | sim
PIPE_READY_TOPIC="${PIPE_READY_TOPIC:-/move_base/status}"
PIPE_READY_TIMEOUT_SEC="${PIPE_READY_TIMEOUT_SEC:-120}"
PIPE_WALL_SEGMENTS="${PIPE_WALL_SEGMENTS:-}"

# Đường dẫn output — gọi lại pipeline_refresh_paths sau khi set PIPE_TRACK / PIPE_EXPERIMENT_NAME
pipeline_refresh_paths() {
  PIPE_BAG_DIR="$PIPE_WS/bags/$PIPE_TRACK"
  PIPE_VIDEO_DIR="$PIPE_WS/videos/$PIPE_TRACK"
  PIPE_RUN_DIR="$PIPE_WS/runs/$PIPE_TRACK"
  PIPE_CSV_DIR="$PIPE_WS/csv/$PIPE_TRACK/$PIPE_EXPERIMENT_NAME"
  PIPE_ENV_DIR="$PIPE_WS/environment/$PIPE_TRACK"
  PIPE_BAG_FILE="$PIPE_BAG_DIR/${PIPE_EXPERIMENT_NAME}.bag"
  PIPE_VIDEO_FILE="$PIPE_VIDEO_DIR/${PIPE_EXPERIMENT_NAME}_screen.mp4"
  PIPE_LOG_FILE="$PIPE_RUN_DIR/${PIPE_EXPERIMENT_NAME}.log"
}
pipeline_refresh_paths

LAUNCH_PID=""
ROSBAG_PID=""
VIDEO_PID=""
_PIPELINE_SELF=$$
_PIPELINE_CLEANUP_DONE=0
_PIPELINE_EXTRACT_DONE=0

pipeline_log() {
  echo "$@" | tee -a "$PIPE_LOG_FILE"
}

pipeline_setup_ros() {
  cd "$PIPE_WS" || true
  # Ổ đầy có thể làm source setup.bash crash — không để sập cả pipeline
  set +e
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash >/dev/null 2>&1
  # shellcheck disable=SC1091
  source "$PIPE_WS/devel/setup.bash" >/dev/null 2>&1
  export PATH="/opt/ros/noetic/bin:${PATH}"
  export PYTHONPATH="${HOME}/.local/lib/python3.8/site-packages:${PYTHONPATH:-}"
  set -e
  return 0
}

pipeline_setup_display() {
  if [ -z "${DISPLAY:-}" ]; then
    for d in 1 0; do
      if [ -S "/tmp/.X11-unix/X${d}" ]; then
        export DISPLAY=":${d}"
        export XAUTHORITY="${XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority}"
        break
      fi
    done
  fi
  if [ -n "${DISPLAY:-}" ] && [ "${USE_RTX3050:-1}" = "1" ] && command -v nvidia-smi >/dev/null 2>&1; then
    export __NV_PRIME_RENDER_OFFLOAD=1
    export __GLX_VENDOR_LIBRARY_NAME=nvidia
  fi
}

pipeline_kill_matching() {
  local pat="$1" pid
  for pid in $(pgrep -f "$pat" 2>/dev/null || true); do
    [[ "$pid" -eq "$_PIPELINE_SELF" || "$pid" -eq "$$" ]] && continue
    kill -TERM "$pid" 2>/dev/null || true
  done
}

pipeline_kill_process_tree() {
  local pid=$1 sig=${2:-TERM}
  local children
  children=$(pgrep -P "$pid" 2>/dev/null || true)
  for c in $children; do pipeline_kill_process_tree "$c" "$sig"; done
  kill -0 "$pid" 2>/dev/null && kill -"$sig" "$pid" 2>/dev/null || true
}

pipeline_stop_stale_gazebo() {
  if ! pgrep -x gzserver >/dev/null 2>&1 && ! pgrep -x gzclient >/dev/null 2>&1; then
    return 0
  fi
  pipeline_log "⚠️ Phát hiện Gazebo cũ — đang dừng..."
  if command -v killall >/dev/null 2>&1; then
    killall -TERM gzserver gzclient 2>/dev/null || true
  else
    pkill -TERM -x gzserver 2>/dev/null || true
    pkill -TERM -x gzclient 2>/dev/null || true
  fi
  sleep 2
  if command -v killall >/dev/null 2>&1; then
    killall -KILL gzserver gzclient 2>/dev/null || true
  else
    pkill -KILL -x gzserver 2>/dev/null || true
    pkill -KILL -x gzclient 2>/dev/null || true
  fi
  sleep 1
  pipeline_log "✅ Đã dừng Gazebo cũ."
}

_pipeline_ros_bin() {
  # Ổ đầy / source hỏng → vẫn gọi tuyệt đối từ /opt/ros
  local name="$1"
  if [ -x "/opt/ros/noetic/bin/$name" ]; then
    echo "/opt/ros/noetic/bin/$name"
  else
    echo "$name"
  fi
}

pipeline_publish_zero_cmd_vel() {
  local rostopic
  rostopic="$(_pipeline_ros_bin rostopic)"
  command -v "$rostopic" >/dev/null 2>&1 || [ -x "$rostopic" ] || return 0
  timeout 0.8 "$rostopic" pub -r 25 /cmd_vel geometry_msgs/Twist \
    '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' \
    >/dev/null 2>&1 || true
  timeout 1 "$rostopic" pub -1 /move_base/cancel actionlib_msgs/GoalID '{}' >/dev/null 2>&1 || true
}

pipeline_send_arduino_stop() {
  local port="${ARDUINO_PORT:-/dev/robot_arduino}"
  [ -e "$port" ] || return 0
  python3 - "$port" <<'PY' 2>/dev/null || true
import sys, time
try:
    import serial
except ImportError:
    sys.exit(0)
s = serial.Serial(port=sys.argv[1], baudrate=19200, timeout=0.3, write_timeout=0.3)
try:
    s.open()
    time.sleep(0.1)
    s.write(b"STOP\nVXYW 0 0 0\nSTOP\n")
    s.flush()
finally:
    s.close()
PY
  for p in /dev/ttyACM0 /dev/ttyUSB0; do
    [ -e "$p" ] && [ "$p" != "$port" ] && python3 -c "import serial,time; s=serial.Serial('$p',19200,timeout=0.3); s.write(b'STOP\n'); s.close()" 2>/dev/null || true
  done
}

pipeline_stop_lidar_motor() {
  # 1) ROS service (node còn sống)  2) serial DTR fallback (node đã chết nhưng motor vẫn quay)
  local helper="$PIPE_WS/src/x_omni4wd_bringup/scripts/stop_lidar_motor.py"
  local rosservice rostopic
  rosservice="$(_pipeline_ros_bin rosservice)"
  rostopic="$(_pipeline_ros_bin rostopic)"

  if timeout 1 "$rostopic" list >/dev/null 2>&1; then
    for svc in /stop_motor /rplidarNode/stop_motor; do
      if timeout 0.8 "$rosservice" list 2>/dev/null | grep -qE "(^| )${svc}($| )"; then
        pipeline_log "  → stop_motor: $svc"
        timeout 2 "$rosservice" call "$svc" >/dev/null 2>&1 || true
        sleep 0.4
        break
      fi
    done
    # Gọi trực tiếp luôn (rosservice list có thể fail khi ổ đầy)
    timeout 2 "$rosservice" call /stop_motor >/dev/null 2>&1 || true
    timeout 2 "$rosservice" call /rplidarNode/stop_motor >/dev/null 2>&1 || true
  fi

  if [ -f "$helper" ]; then
    pipeline_log "  → stop_lidar_motor.py (ROS + serial DTR)"
    python3 "$helper" >>"$PIPE_LOG_FILE" 2>&1 || python3 "$helper" --serial-only >>"$PIPE_LOG_FILE" 2>&1 || true
  else
    python3 - <<'PY' 2>/dev/null || true
import os, time
try:
    import serial
except ImportError:
    raise SystemExit(0)
port = os.environ.get("LIDAR_PORT", "/dev/robot_lidar")
if not os.path.exists(port):
    raise SystemExit(0)
s = serial.Serial(port, int(os.environ.get("LIDAR_BAUD", "115200")), timeout=0.3)
s.setDTR(False); s.setRTS(False); time.sleep(0.25)
try:
    s.write(bytes([0xA5, 0x25])); s.flush()
except Exception:
    pass
s.close()
PY
  fi
  sleep 0.3
}

pipeline_force_kill_real_hardware() {
  # Tắt motor TRƯỚC khi giết process — nếu kill trước thì LiDAR vẫn quay
  pipeline_stop_lidar_motor

  # TERM trước để rplidar / guard chạy shutdown hook
  pipeline_kill_matching 'rplidar_ros/rplidarNode|/rplidarNode'
  pipeline_kill_matching 'zed_sdk_bridge|zed2i_core|skeleton_visualizer|zed_wrapper|zed_node'
  pipeline_kill_matching 'srf_laser_odometry|ekf_localization_node|rf2o_laser_odometry|imu_yaw_scale_node|gtsam_livo_node'
  pipeline_kill_matching 'arduino_interface_omni|/lib/move_base/move_base|/lib/amcl/amcl|map_server'
  pipeline_kill_matching 'social_density_fusion|emotion_detection_node|path_density_weight_controller|scan_static_filter'
  pipeline_kill_matching 'ultralytics|yolo|yolo_objects_to_obstacle_cloud|topic_tools/relay'
  pipeline_kill_matching 'goal_sender|scenario_gt_social|person_visualization|sgi_calculator|sii_calculator|social_navigation_dashboard'
  pipeline_kill_matching 'hardware_shutdown_guard'
  pipeline_kill_matching '/opt/ros/noetic/lib/rviz/rviz| rviz '
  pipeline_kill_matching 'rosmaster --core|roscore|rosout'
  sleep 2

  # Serial DTR lần nữa sau khi node chết
  pipeline_stop_lidar_motor

  # Còn sống → KILL
  local pid
  for pid in $(pgrep -f 'rplidar_ros/rplidarNode|/rplidarNode|zed_sdk_bridge_node|arduino_interface_omni|rosmaster --core|roslaunch' 2>/dev/null || true); do
    [[ "$pid" -eq "$_PIPELINE_SELF" || "$pid" -eq "$$" ]] && continue
    kill -KILL "$pid" 2>/dev/null || true
  done
  sleep 0.3
}

pipeline_start_screen_recording() {
  if ! command -v ffmpeg >/dev/null 2>&1; then
    pipeline_log "⚠️ Không có ffmpeg — bỏ qua ghi video."
    return 0
  fi
  if [ -z "${DISPLAY:-}" ]; then
    pipeline_log "⚠️ Không có DISPLAY — bỏ qua ghi video."
    return 0
  fi
  mkdir -p "$PIPE_VIDEO_DIR"
  pipeline_log "🎥 Ghi màn hình: $PIPE_VIDEO_FILE"
  local grab_input="${DISPLAY}.0"
  [[ "$grab_input" == *"+"* ]] && grab_input="${DISPLAY}+0,0"
  ffmpeg -y -f x11grab -r 30 -s 1920x1080 -i "$grab_input" \
    -vcodec libx264 -preset ultrafast -crf 18 "$PIPE_VIDEO_FILE" \
    >/dev/null 2>&1 &
  VIDEO_PID=$!
  pipeline_log "   screen record PID: $VIDEO_PID"
}

pipeline_stop_recording() {
  if [ -n "${ROSBAG_PID:-}" ] && kill -0 "$ROSBAG_PID" 2>/dev/null; then
    pipeline_log "  → Dừng rosbag PID=$ROSBAG_PID"
    kill -INT "$ROSBAG_PID" 2>/dev/null || true
    wait "$ROSBAG_PID" 2>/dev/null || true
  fi
  ROSBAG_PID=""
  if [ -n "${VIDEO_PID:-}" ] && kill -0 "$VIDEO_PID" 2>/dev/null; then
    pipeline_log "  → Dừng ghi màn hình PID=$VIDEO_PID"
    kill -INT "$VIDEO_PID" 2>/dev/null || true
    wait "$VIDEO_PID" 2>/dev/null || true
  fi
  VIDEO_PID=""
}

pipeline_stop_launch() {
  # INT trước (graceful) để hardware_shutdown_guard + rplidar kịp stop_motor
  if [ -n "${LAUNCH_PID:-}" ] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    pipeline_log "  → Dừng launch PID=$LAUNCH_PID (SIGINT graceful)"
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    # Gửi INT cả process group nếu launch chạy dưới setsid
    kill -INT "-$LAUNCH_PID" 2>/dev/null || true
    local i
    for i in 1 2 3 4 5 6 7 8; do
      kill -0 "$LAUNCH_PID" 2>/dev/null || break
      sleep 1
    done
    if kill -0 "$LAUNCH_PID" 2>/dev/null; then
      pipeline_log "  → Launch còn sống — TERM process tree"
      pipeline_kill_process_tree "$LAUNCH_PID" TERM
      sleep 2
    fi
    if kill -0 "$LAUNCH_PID" 2>/dev/null; then
      pipeline_log "  → Launch còn sống — KILL"
      kill -KILL "$LAUNCH_PID" 2>/dev/null || true
      kill -KILL "-$LAUNCH_PID" 2>/dev/null || true
    fi
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
  LAUNCH_PID=""
  if [ -n "${PIPE_LAUNCH_KILL_PATTERN:-}" ]; then
    pipeline_kill_matching "$PIPE_LAUNCH_KILL_PATTERN"
  fi
}

pipeline_post_process() {
  [ "${_PIPELINE_EXTRACT_DONE:-0}" -eq 1 ] && return 0
  _PIPELINE_EXTRACT_DONE=1

  if [ ! -f "$PIPE_BAG_FILE" ]; then
    pipeline_log "❌ Không có bag file — bỏ qua trích xuất."
    return 1
  fi

  pipeline_log ""
  pipeline_log "📊 Trích xuất bag → CSV + data_config.m ..."
  mkdir -p "$PIPE_CSV_DIR"
  local extract_script="$PIPE_WS/scripts/extract_experiment_optimized.py"
  if [ ! -f "$extract_script" ]; then
    pipeline_log "❌ Không tìm thấy $extract_script"
    return 1
  fi
  if ! python3 "$extract_script" "$PIPE_BAG_FILE" --output-dir "$PIPE_CSV_DIR" --timeline; then
    pipeline_log "❌ Trích xuất thất bại."
    return 1
  fi
  if [ ! -f "$PIPE_CSV_DIR/data_config.m" ]; then
    pipeline_log "❌ Không thấy data_config.m sau trích xuất."
    return 1
  fi
  pipeline_log "✅ CSV + data_config.m: $PIPE_CSV_DIR/"
  if [ -n "$PIPE_WALL_SEGMENTS" ] && [ -f "$PIPE_WALL_SEGMENTS" ]; then
    pipeline_log "   → File tường MATLAB: $PIPE_WALL_SEGMENTS"
  fi
  return 0
}

pipeline_print_summary() {
  pipeline_log ""
  pipeline_log "🎉 Hoàn tất thí nghiệm: $PIPE_EXPERIMENT_NAME"
  pipeline_log "Track         : $PIPE_TRACK"
  pipeline_log "Bag           : $PIPE_BAG_FILE"
  if [ -f "$PIPE_VIDEO_FILE" ]; then
    pipeline_log "Video         : $PIPE_VIDEO_FILE"
  else
    pipeline_log "Video         : (không ghi — thiếu ffmpeg/DISPLAY)"
  fi
  pipeline_log "CSV/MATLAB    : $PIPE_CSV_DIR/"
  pipeline_log "Log           : $PIPE_LOG_FILE"
  pipeline_log ""
  pipeline_log "MATLAB (chỉnh data_path nếu cần):"
  pipeline_log "  cd $PIPE_CSV_DIR && matlab -r \"run('data_config.m')\""
}

pipeline_cleanup() {
  if [ "${_PIPELINE_CLEANUP_DONE:-0}" -eq 1 ]; then return 0; fi
  _PIPELINE_CLEANUP_DONE=1

  pipeline_log ""
  pipeline_log "🛑 Cleanup (dừng hẳn phần cứng + ROS)..."

  if [ "$PIPE_MODE" = "real" ]; then
    # Quan trọng: stop motor khi ROS còn sống
    pipeline_publish_zero_cmd_vel
    pipeline_stop_lidar_motor
  fi

  pipeline_stop_recording
  pipeline_stop_launch

  if [ "$PIPE_MODE" = "sim" ]; then
    pgrep -x gzserver >/dev/null 2>&1 && pipeline_stop_stale_gazebo || true
  fi

  if [ "$PIPE_MODE" = "real" ]; then
    pipeline_force_kill_real_hardware
    pipeline_send_arduino_stop
    # Đảm bảo LiDAR tắt dù kill đột ngột
    pipeline_stop_lidar_motor
  fi

  local rosnode rostopic
  rosnode="$(_pipeline_ros_bin rosnode)"
  rostopic="$(_pipeline_ros_bin rostopic)"
  if [ -x "$rosnode" ] || command -v "$rosnode" >/dev/null 2>&1; then
    if timeout 2 "$rostopic" list >/dev/null 2>&1; then
      yes | timeout 10 "$rosnode" cleanup >/dev/null 2>&1 || true
    fi
  fi

  pipeline_log "✅ Cleanup xong — LiDAR/Arduino/ROS đã dừng."
}

# Helper: chạy roslaunch nền trong session riêng (Ctrl+C/cleanup không lẫn process group shell)
pipeline_roslaunch_bg() {
  setsid roslaunch "$@" >>"$PIPE_LOG_FILE" 2>&1 &
  LAUNCH_PID=$!
  pipeline_log "   roslaunch PID: $LAUNCH_PID (setsid)"
}

# Gọi sau khi set PIPE_* và PIPE_TOPICS (array)
pipeline_run() {
  # Luôn refresh — tránh bags/unknown-track khi script set PIPE_TRACK sau khi source lib
  pipeline_refresh_paths
  mkdir -p "$PIPE_BAG_DIR" "$PIPE_VIDEO_DIR" "$PIPE_RUN_DIR" "$PIPE_CSV_DIR" "$PIPE_ENV_DIR"
  : > "$PIPE_LOG_FILE"

  pipeline_setup_ros
  pipeline_setup_display

  if [ -z "${PIPE_WALL_SEGMENTS:-}" ] && [ -f "$PIPE_ENV_DIR/wall_segments.txt" ]; then
    PIPE_WALL_SEGMENTS="$PIPE_ENV_DIR/wall_segments.txt"
  fi

  trap pipeline_cleanup INT TERM EXIT

  pipeline_log "=== EXPERIMENT PIPELINE — $PIPE_TRACK ==="
  pipeline_log "Workspace     : $PIPE_WS"
  pipeline_log "Experiment    : $PIPE_EXPERIMENT_NAME"
  pipeline_log "Mode          : $PIPE_MODE"
  pipeline_log "Ready topic   : $PIPE_READY_TOPIC"
  pipeline_log "Bag           : $PIPE_BAG_FILE"
  pipeline_log "Video         : $PIPE_VIDEO_FILE"
  pipeline_log "Results (CSV) : $PIPE_CSV_DIR"
  pipeline_log "Duration (min): $PIPE_DURATION_MINUTES"
  pipeline_log ""

  if [ "$PIPE_MODE" = "sim" ]; then
    pipeline_stop_stale_gazebo
  fi

  # Hook: script gọi phải định nghĩa pipeline_start_launch()
  if ! declare -F pipeline_start_launch >/dev/null 2>&1; then
    pipeline_log "❌ Thiếu hàm pipeline_start_launch()"
    exit 1
  fi
  pipeline_start_launch

  pipeline_log "⏳ Chờ $PIPE_READY_TOPIC sẵn sàng (tối đa ${PIPE_READY_TIMEOUT_SEC}s)..."
  local ready=0 i max_iter
  max_iter=$(( PIPE_READY_TIMEOUT_SEC / 2 ))
  for i in $(seq 1 "$max_iter"); do
    if timeout 2s rostopic echo -n1 "$PIPE_READY_TOPIC" >/dev/null 2>&1; then
      ready=1
      break
    fi
    sleep 2
  done
  if [ "$ready" -ne 1 ]; then
    pipeline_log "❌ Timeout — hệ thống chưa sẵn sàng."
    exit 1
  fi
  pipeline_log "✅ Hệ thống sẵn sàng."

  pipeline_start_screen_recording

  pipeline_log "📡 Bắt đầu ghi rosbag..."
  if [ "${#PIPE_TOPICS[@]}" -eq 0 ]; then
    pipeline_log "❌ PIPE_TOPICS rỗng"
    exit 1
  fi
  if [ "$PIPE_DURATION_MINUTES" -gt 0 ] 2>/dev/null; then
    rosbag record --duration="$(( PIPE_DURATION_MINUTES * 60 ))" \
      -O "$PIPE_BAG_FILE" "${PIPE_TOPICS[@]}" >>"$PIPE_LOG_FILE" 2>&1 &
  else
    rosbag record -O "$PIPE_BAG_FILE" "${PIPE_TOPICS[@]}" >>"$PIPE_LOG_FILE" 2>&1 &
  fi
  ROSBAG_PID=$!
  pipeline_log "   rosbag PID: $ROSBAG_PID"

  if [ "$PIPE_DURATION_MINUTES" -gt 0 ] 2>/dev/null; then
    pipeline_log "⏱️ Đang ghi ${PIPE_DURATION_MINUTES} phút..."
    wait "$ROSBAG_PID" 2>/dev/null || true
    ROSBAG_PID=""
    pipeline_stop_recording
  else
    pipeline_log "Nhấn Enter để dừng ghi và trích xuất kết quả..."
    read -r _
    pipeline_stop_recording
  fi

  if [ ! -f "$PIPE_BAG_FILE" ]; then
    pipeline_log "❌ Không tạo được bag file."
    exit 1
  fi
  pipeline_log "✅ Kết thúc ghi bag."

  pipeline_post_process || exit 1
  pipeline_print_summary

  # Thoát bình thường — cleanup không trích xuất lại
  trap - INT TERM EXIT
  pipeline_cleanup
}
