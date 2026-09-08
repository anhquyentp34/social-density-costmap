# Move Base State Monitor

Package này cung cấp các công cụ để theo dõi và phân tích sự biến đổi trạng thái của move_base theo thời gian.

## Tính năng

- **Real-time monitoring**: Theo dõi trạng thái move_base theo thời gian thực
- **Data logging**: Lưu trữ dữ liệu vào file CSV với timestamp
- **Visualization**: Hiển thị trạng thái theo thời gian với matplotlib
- **Analysis tools**: Phân tích và tạo báo cáo từ dữ liệu log

## Cài đặt

1. Build workspace:
```bash
cd ~/costmapex_unified_ws
catkin_make
source devel/setup.bash
```

2. Cài đặt dependencies:
```bash
sudo apt-get install python3-matplotlib python3-pandas python3-numpy
```

## Sử dụng

### 1. Chạy monitor trong quá trình navigation

Có 2 cách để chạy monitor:

**Cách 1: Chạy riêng biệt**
```bash
# Terminal 1: Chạy navigation
roslaunch diffbot_navigation test_layer_costmap_proxemic.launch

# Terminal 2: Chạy monitor
roslaunch move_base_monitor move_base_monitor.launch
```

**Cách 2: Tích hợp vào launch file chính**
```bash
# Monitor đã được tích hợp sẵn trong test_layer_costmap_proxemic.launch
roslaunch diffbot_navigation test_layer_costmap_proxemic.launch
```

### 2. Các tùy chọn monitor

```bash
# Chỉ chạy monitor (không có visualization)
roslaunch move_base_monitor move_base_monitor.launch enable_visualizer:=false

# Chạy với rqt_plot thay vì matplotlib
roslaunch move_base_monitor move_base_monitor.launch enable_visualizer:=false enable_rqt_plot:=true

# Chạy với rqt_console để xem logs
roslaunch move_base_monitor move_base_monitor.launch enable_rqt_console:=true
```

### 3. Phân tích dữ liệu sau khi chạy

```bash
# Phân tích file log mới nhất
rosrun move_base_monitor analyze_move_base_logs.py

# Phân tích file log cụ thể
rosrun move_base_monitor analyze_move_base_logs.py --log_file ~/move_base_logs/move_base_state_20241201_143022.csv

# Chỉ vẽ timeline
rosrun move_base_monitor analyze_move_base_logs.py --timeline

# Chỉ vẽ phân phối
rosrun move_base_monitor analyze_move_base_logs.py --distribution

# Chỉ vẽ quỹ đạo robot
rosrun move_base_monitor analyze_move_base_logs.py --trajectory

# Tạo báo cáo
rosrun move_base_monitor analyze_move_base_logs.py --report

# Thực hiện tất cả phân tích
rosrun move_base_monitor analyze_move_base_logs.py --all
```

## Dữ liệu được thu thập

Monitor sẽ thu thập các thông tin sau:

- **Timestamp**: Thời gian chính xác
- **Time_Elapsed**: Thời gian đã trôi qua từ lúc bắt đầu
- **State**: Trạng thái hiện tại của move_base
- **Goal_Status**: Mã trạng thái goal
- **Robot_X, Robot_Y, Robot_Yaw**: Vị trí và hướng robot
- **Linear_Vel, Angular_Vel**: Vận tốc robot
- **Goal_X, Goal_Y, Goal_Yaw**: Vị trí và hướng goal

## Các trạng thái move_base

- **PENDING**: Chờ xử lý
- **ACTIVE**: Đang thực hiện
- **PREEMPTED**: Bị gián đoạn
- **SUCCEEDED**: Thành công
- **ABORTED**: Bị hủy
- **REJECTED**: Bị từ chối
- **PREEMPTING**: Đang gián đoạn
- **RECALLING**: Đang thu hồi
- **RECALLED**: Đã thu hồi
- **LOST**: Bị mất

## File output

### Log files
- Vị trí: `~/move_base_logs/`
- Format: `move_base_state_YYYYMMDD_HHMMSS.csv`

### Plot files
- Timeline: `move_base_timeline_YYYYMMDD_HHMMSS.png`
- Distribution: `move_base_distribution_YYYYMMDD_HHMMSS.png`
- Trajectory: `robot_trajectory_YYYYMMDD_HHMMSS.png`

### Report files
- Format: `move_base_report_YYYYMMDD_HHMMSS.txt`

## Topics

### Subscribed
- `/move_base/status` (actionlib_msgs/GoalStatusArray)
- `/move_base/current_goal` (geometry_msgs/PoseStamped)
- `/odom` (nav_msgs/Odometry)
- `/cmd_vel` (geometry_msgs/Twist)

### Published
- `/move_base_monitor/current_state` (std_msgs/String)

## Troubleshooting

### Lỗi "No module named 'matplotlib'"
```bash
sudo apt-get install python3-matplotlib
```

### Lỗi "No module named 'pandas'"
```bash
sudo apt-get install python3-pandas
```

### Không thấy dữ liệu trong log
- Kiểm tra xem move_base có đang chạy không
- Kiểm tra các topic có được publish không: `rostopic list | grep move_base`

### Plot không hiển thị
- Đảm bảo có display (X11 forwarding nếu chạy từ xa)
- Thử chạy với `enable_visualizer:=false` và sử dụng `rqt_plot` thay thế

## Ví dụ sử dụng

1. **Chạy navigation với monitor**:
```bash
roslaunch diffbot_navigation test_layer_costmap_proxemic.launch
```

2. **Gửi goal để test**:
```bash
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: 'map'
pose:
  position: {x: 5.0, y: 0.0, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}" -r 1
```

3. **Phân tích sau khi hoàn thành**:
```bash
rosrun move_base_monitor analyze_move_base_logs.py --all
```

## Tùy chỉnh

### Thay đổi tần số log
Trong file `move_base_state_monitor.py`, thay đổi:
```python
self.log_timer = rospy.Timer(rospy.Duration(0.1), self.log_state)  # 0.1 giây
```

### Thay đổi thư mục log
```python
self.log_dir = os.path.expanduser("~/custom_logs")  # Thay đổi đường dẫn
```

### Thêm thông tin monitor
Thêm subscriber mới trong `__init__`:
```python
rospy.Subscriber("/custom_topic", CustomMsg, self.custom_callback)
``` 