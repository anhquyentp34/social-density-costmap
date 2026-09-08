# Path Planning Monitor

## Mô tả
Node `path_planning_monitor` được thiết kế để theo dõi và phân tích hiệu suất của global planner (NavfnROS) trong hệ thống navigation. Node này monitor topic `/move_base/NavfnROS/plan` để theo dõi khi nào planner tìm được đường đi và khi nào không.

## Tính năng

### 1. Theo dõi thời gian thực
- **Path Status Timeline**: Hiển thị timeline khi nào có đường đi được tìm thấy (1) và khi nào không (0)

### 2. Thống kê chi tiết
- Tổng số lần thử lập kế hoạch
- Số lần thành công/thất bại
- Tỷ lệ thành công (%)
- Thời gian lần cuối thành công/thất bại

### 3. Logging
- Log thông tin khi tìm thấy đường đi: thời gian, số poses
- Log cảnh báo khi không tìm thấy đường đi
- In thống kê mỗi 30 giây

## Cách sử dụng

### Chạy độc lập
```bash
rosrun move_base_monitor path_planning_monitor.py
```

### Chạy với launch file
Node đã được thêm vào `test_layer_costmap_extention.launch` và sẽ tự động chạy khi launch file được thực thi.

## Giao diện

### Plot: Path Status Timeline
- **Trục Y**: 0 = No Path, 1 = Path Found
- **Trục X**: Thời gian (giây)
- **Màu sắc**: 
  - Xanh lá = Path Found
  - Đỏ = No Path

## Thông số kỹ thuật

### Input Topic
- **Topic**: `/move_base/NavfnROS/plan`
- **Type**: `nav_msgs/Path`

### Output
- **Console Logs**: Thông tin chi tiết về mỗi lần lập kế hoạch
- **Real-time Plot**: Giao diện matplotlib với timeline
- **Statistics**: In thống kê định kỳ và khi kết thúc

### Performance
- Giữ tối đa 2000 điểm dữ liệu để tối ưu hiệu suất
- Cập nhật plot mỗi 100ms
- Tự động scale trục thời gian dựa trên độ dài thí nghiệm

## Ví dụ Output

```
[15.23s] Path FOUND - Poses: 42
[16.78s] NO PATH FOUND
[18.12s] Path FOUND - Poses: 61

==================================================
PATH PLANNING STATISTICS
==================================================
Total planning attempts: 45
Successful plans: 38
Failed plans: 7
Success rate: 84.44%
Last successful plan: 18.12s
Last failed plan: 16.78s
==================================================
```

## Dependencies
- `rospy`
- `matplotlib`
- `numpy`
- `nav_msgs`

## Troubleshooting

### Lỗi thường gặp
1. **Không hiển thị plot**: Đảm bảo có display X11 hoặc set `DISPLAY=:0`
2. **Không nhận được dữ liệu**: Kiểm tra topic `/move_base/NavfnROS/plan` có đang publish không
3. **Performance chậm**: Giảm số điểm dữ liệu lưu trữ trong code

### Debug
- Sử dụng `rostopic echo /move_base/NavfnROS/plan` để kiểm tra dữ liệu
- Kiểm tra log ROS để xem thông tin chi tiết 