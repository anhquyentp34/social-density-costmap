# Goal Sender Node

Node gửi mục tiêu tự động cho robot navigation với hỗ trợ nhiều waypoints và chế độ di chuyển khác nhau.

## Tính năng

- **Nhiều waypoints**: Hỗ trợ nhiều điểm đến thay vì chỉ 1 START + 1 GOAL
- **Chế độ di chuyển**: cycle, sequence, random
- **Cấu hình linh hoạt**: Load từ file YAML
- **Visualization**: Hiển thị waypoints trên map với màu sắc khác nhau
- **Thống kê chi tiết**: Theo dõi hiệu suất navigation

## Chế độ di chuyển

### 1. Cycle Mode (Mặc định)
- **Mô tả**: Đi qua tất cả waypoints theo thứ tự, lặp lại `num_cycles` lần
- **Ví dụ**: START → GOAL_1 → GOAL_2 → GOAL_3 → START → GOAL_1 → ...
- **Số goals**: `num_cycles × (len(waypoints) - 1)`

### 2. Sequence Mode
- **Mô tả**: Đi qua waypoints một lần duy nhất
- **Ví dụ**: START → GOAL_1 → GOAL_2 → GOAL_3 (kết thúc)
- **Số goals**: `len(waypoints) - 1`

### 3. Random Mode (Chưa implement đầy đủ)
- **Mô tả**: Chọn waypoint ngẫu nhiên
- **Trạng thái**: Đang phát triển

## Cấu hình Waypoints

### File cấu hình: `config/waypoints_config.yaml`

```yaml
# Số lần lặp lại chu trình
num_cycles: 3

# Chế độ di chuyển: 'cycle', 'sequence', 'random'
movement_mode: 'cycle'

# Định nghĩa các waypoints
waypoints:
  - name: 'START'
    x: -11.0
    y: 0.0
    yaw: 0.0
    color: [0.0, 1.0, 0.0]  # Xanh lá
    
  - name: 'GOAL_1'
    x: 11.0
    y: 0.0
    yaw: 3.14
    color: [1.0, 0.0, 0.0]  # Đỏ
    
  - name: 'GOAL_2'
    x: 0.0
    y: 5.0
    yaw: 1.57
    color: [0.0, 0.0, 1.0]  # Xanh dương
```

### Cấu trúc Waypoint

Mỗi waypoint cần có:
- **name**: Tên waypoint (hiển thị trên map)
- **x, y**: Tọa độ vị trí
- **yaw**: Góc quay (radian)
- **color**: Màu sắc [r, g, b] (0.0-1.0)

## Cách sử dụng

### 1. Sử dụng với launch file chính
```bash
roslaunch diffbot_navigation test_layer_costmap_groups.launch
```

### 2. Sử dụng launch file riêng
```bash
roslaunch goal_sender goal_sender_with_config.launch
```

### 3. Chạy node trực tiếp với parameters
```bash
# Sử dụng cấu hình mặc định
rosrun goal_sender goal_sender_node_one.py

# Tùy chỉnh parameters
rosrun goal_sender goal_sender_node_one.py _num_cycles:=5 _movement_mode:=sequence

# Load từ file config khác
rosrun goal_sender goal_sender_node_one.py _config_file:=/path/to/custom_config.yaml
```

## Tùy chỉnh

### Thay đổi waypoints
1. Chỉnh sửa file `config/waypoints_config.yaml`
2. Thay đổi tọa độ, tên, màu sắc
3. Restart node

### Thêm waypoints mới
```yaml
waypoints:
  - name: 'START'
    x: -11.0
    y: 0.0
    yaw: 0.0
    color: [0.0, 1.0, 0.0]
    
  - name: 'NEW_GOAL'
    x: 15.0
    y: 10.0
    yaw: 0.785
    color: [0.5, 0.5, 1.0]
```

### Thay đổi chế độ di chuyển
```yaml
movement_mode: 'sequence'  # Thay vì 'cycle'
```

## Output

### Log messages
```
[INFO] Waypoints configuration:
[INFO]   0: START at (-11.00, 0.00, 0.00)
[INFO]   1: GOAL_1 at (11.00, 0.00, 3.14)
[INFO]   2: GOAL_2 at (0.00, 5.00, 1.57)

[INFO] Moving to next goal. Mode: cycle, Cycle: 1, Waypoint: 1/4
```

### Thống kê
- Tổng số goals
- Số lần thành công/thất bại
- Tỷ lệ ABORTED
- Thời gian thực hiện

### Visualization
- Markers màu sắc cho mỗi waypoint
- Text hiển thị tên waypoint
- Marker xanh dương cho mục tiêu hiện tại

## Troubleshooting

### Lỗi thường gặp
1. **Config file không tìm thấy**: Sử dụng waypoints mặc định
2. **Waypoints không load**: Kiểm tra cú pháp YAML
3. **Robot không di chuyển**: Kiểm tra move_base status

### Debug
```bash
# Kiểm tra parameters
rosparam get /goal_sender_cycle_node/waypoints

# Kiểm tra topics
rostopic list | grep move_base

# Kiểm tra services
rosservice list | grep move_base
``` 