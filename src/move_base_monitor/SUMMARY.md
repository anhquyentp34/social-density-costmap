# Tóm Tắt: Move Base State Monitor

## Tổng Quan
Đã tạo thành công một hệ thống hoàn chỉnh để theo dõi và phân tích sự biến đổi trạng thái của move_base theo thời gian.

## Các Thành Phần Đã Tạo

### 1. Package Structure
```
move_base_monitor/
├── src/
│   ├── move_base_state_monitor.py      # Node monitor chính
│   ├── move_base_state_visualizer.py   # Node visualization
│   └── analyze_move_base_logs.py       # Tool phân tích
├── launch/
│   └── move_base_monitor.launch        # Launch file
├── config/
│   └── monitor_params.yaml             # File cấu hình
├── scripts/
│   └── demo_monitor.sh                 # Script demo
├── package.xml                         # Package dependencies
├── CMakeLists.txt                      # Build configuration
├── README.md                           # Hướng dẫn sử dụng
└── SUMMARY.md                          # File này
```

### 2. Các Node Chính

#### move_base_state_monitor.py
- **Chức năng**: Monitor trạng thái move_base theo thời gian thực
- **Input**: 
  - `/move_base/status` (GoalStatusArray)
  - `/move_base/current_goal` (PoseStamped)
  - `/odom` (Odometry)
  - `/cmd_vel` (Twist)
- **Output**: 
  - File CSV log với timestamp
  - `/move_base_monitor/current_state` (String)
- **Tần số**: 10Hz (có thể tùy chỉnh)

#### move_base_state_visualizer.py
- **Chức năng**: Hiển thị trạng thái theo thời gian với matplotlib
- **Features**:
  - Timeline plot real-time
  - State duration distribution
  - Auto-save plots
- **Tần số update**: 1Hz

#### analyze_move_base_logs.py
- **Chức năng**: Phân tích dữ liệu log sau khi chạy
- **Features**:
  - Timeline analysis
  - State distribution analysis
  - Robot trajectory visualization
  - Report generation
- **Output**: Plots và reports

### 3. Dữ Liệu Được Thu Thập

| Field | Type | Description |
|-------|------|-------------|
| Timestamp | DateTime | Thời gian chính xác |
| Time_Elapsed | Float | Thời gian đã trôi qua (s) |
| State | String | Trạng thái move_base |
| Goal_Status | Integer | Mã trạng thái goal |
| Robot_X, Robot_Y | Float | Vị trí robot (m) |
| Robot_Yaw | Float | Hướng robot (rad) |
| Linear_Vel, Angular_Vel | Float | Vận tốc robot |
| Goal_X, Goal_Y, Goal_Yaw | Float | Vị trí và hướng goal |

### 4. Các Trạng Thái Move Base

| State | Code | Description |
|-------|------|-------------|
| PENDING | 0 | Chờ xử lý |
| ACTIVE | 1 | Đang thực hiện |
| PREEMPTED | 2 | Bị gián đoạn |
| SUCCEEDED | 3 | Thành công |
| ABORTED | 4 | Bị hủy |
| REJECTED | 5 | Bị từ chối |
| PREEMPTING | 6 | Đang gián đoạn |
| RECALLING | 7 | Đang thu hồi |
| RECALLED | 8 | Đã thu hồi |
| LOST | 9 | Bị mất |

## Cách Sử Dụng

### 1. Chạy Monitor
```bash
# Cách 1: Tích hợp vào launch chính
roslaunch diffbot_navigation test_layer_costmap_proxemic.launch

# Cách 2: Chạy riêng biệt
roslaunch move_base_monitor move_base_monitor.launch
```

### 2. Phân Tích Dữ Liệu
```bash
# Phân tích file log mới nhất
rosrun move_base_monitor analyze_move_base_logs.py --all

# Phân tích file cụ thể
rosrun move_base_monitor analyze_move_base_logs.py --log_file ~/move_base_logs/file.csv
```

### 3. Demo Script
```bash
cd ~/costmapex_unified_ws/src/move_base_monitor/scripts
./demo_monitor.sh
```

## Output Files

### Log Files
- **Location**: `~/move_base_logs/`
- **Format**: `move_base_state_YYYYMMDD_HHMMSS.csv`

### Plot Files
- **Timeline**: `move_base_timeline_YYYYMMDD_HHMMSS.png`
- **Distribution**: `move_base_distribution_YYYYMMDD_HHMMSS.png`
- **Trajectory**: `robot_trajectory_YYYYMMDD_HHMMSS.png`

### Report Files
- **Format**: `move_base_report_YYYYMMDD_HHMMSS.txt`

## Tích Hợp

### Launch File Integration
Đã tích hợp vào `test_layer_costmap_proxemic.launch`:
```xml
<include file="$(find move_base_monitor)/launch/move_base_monitor.launch">
    <arg name="enable_visualizer" value="true" />
    <arg name="enable_rqt_plot" value="false" />
    <arg name="enable_rqt_console" value="false" />
</include>
```

### Dependencies
- **ROS**: rospy, std_msgs, actionlib_msgs, geometry_msgs, nav_msgs
- **Python**: matplotlib, pandas, numpy
- **Tools**: rqt_plot, rqt_console

## Tính Năng Nâng Cao

### 1. Real-time Monitoring
- Theo dõi trạng thái theo thời gian thực
- Log dữ liệu với tần số cao (10Hz)
- Publish trạng thái hiện tại

### 2. Visualization
- Timeline plot real-time
- State duration distribution
- Robot trajectory tracking
- Auto-save functionality

### 3. Analysis Tools
- Comprehensive data analysis
- Multiple output formats
- Statistical reporting
- Performance metrics

### 4. Configuration
- Flexible parameter system
- Multiple launch options
- Customizable thresholds
- Alert system

## Troubleshooting

### Common Issues
1. **No matplotlib**: `sudo apt-get install python3-matplotlib`
2. **No pandas**: `sudo apt-get install python3-pandas`
3. **No display**: Use `enable_visualizer:=false` and `rqt_plot`
4. **No data**: Check if move_base is running

### Debug Commands
```bash
# Check topics
rostopic list | grep move_base

# Check log files
ls -la ~/move_base_logs/

# Monitor state changes
rostopic echo /move_base_monitor/current_state
```

## Kết Luận

Hệ thống move_base monitor đã được tạo hoàn chỉnh với:
- ✅ Real-time monitoring
- ✅ Data logging
- ✅ Visualization
- ✅ Analysis tools
- ✅ Integration ready
- ✅ Documentation
- ✅ Demo scripts

Hệ thống sẵn sàng sử dụng để theo dõi và phân tích trạng thái move_base theo thời gian. 