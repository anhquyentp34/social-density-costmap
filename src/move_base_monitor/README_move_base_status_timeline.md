# Move Base Status Timeline

## Mô tả
Node `move_base_status_timeline` được thiết kế để theo dõi và hiển thị timeline trạng thái của move_base action server. Node này monitor topic `/move_base/status` để theo dõi các trạng thái khác nhau của move_base theo thời gian thực.

## Tính năng

### 1. Theo dõi thời gian thực
- **Status Timeline**: Hiển thị timeline các trạng thái của move_base theo thời gian
- **Status Codes**: Hiển thị tất cả các trạng thái có thể của move_base

### 2. Trạng thái được theo dõi
- **0: PENDING** - Mục tiêu chưa được xử lý
- **1: ACTIVE** - Mục tiêu đang được xử lý
- **2: PREEMPTED** - Mục tiêu bị hủy sau khi bắt đầu thực thi
- **3: SUCCEEDED** - Mục tiêu hoàn thành thành công
- **4: ABORTED** - Mục tiêu bị hủy do lỗi
- **5: REJECTED** - Mục tiêu bị từ chối
- **6: PREEMPTING** - Mục tiêu đang bị hủy
- **7: RECALLING** - Mục tiêu đang được thu hồi
- **8: RECALLED** - Mục tiêu đã được thu hồi
- **9: LOST** - Mục tiêu bị mất

### 3. Giao diện trực quan
- Timeline với các điểm đánh dấu trạng thái
- Đường ngang cho trạng thái hiện tại
- Annotation cho các thay đổi trạng thái
- Tự động scale theo thời gian

## Cách sử dụng

### Chạy độc lập
```bash
rosrun move_base_monitor move_base_status_timeline.py
```

### Chạy với launch file
Node đã được thêm vào `test_layer_costmap_extention.launch` và sẽ tự động chạy khi launch file được thực thi.

## Giao diện

### Plot: Move Base Status Timeline
- **Trục Y**: Các trạng thái từ 0-9
- **Trục X**: Thời gian (giây)
- **Đường chính**: Timeline trạng thái
- **Đường đỏ**: Trạng thái hiện tại
- **Đường xám**: Trạng thái terminal

## Thông số kỹ thuật

### Input Topic
- **Topic**: `/move_base/status`
- **Type**: `actionlib_msgs/GoalStatusArray`

### Output
- **Console Logs**: Thông tin khởi động và trạng thái
- **Real-time Plot**: Giao diện matplotlib với timeline
- **Window Title**: "Move Base Status Timeline"

### Performance
- Giữ tối đa 2000 điểm dữ liệu để tối ưu hiệu suất
- Cập nhật plot mỗi 100ms
- Tự động scale trục thời gian dựa trên độ dài thí nghiệm

## Ví dụ Output

```
[INFO] [move_base_status_timeline]: Move Base Status Timeline đã khởi động
[INFO] [move_base_status_timeline]: Status codes:
[INFO] [move_base_status_timeline]:   0: PENDING
[INFO] [move_base_status_timeline]:   1: ACTIVE
[INFO] [move_base_status_timeline]:   2: PREEMPTED
[INFO] [move_base_status_timeline]:   3: SUCCEEDED
[INFO] [move_base_status_timeline]:   4: ABORTED
[INFO] [move_base_status_timeline]:   5: REJECTED
[INFO] [move_base_status_timeline]:   6: PREEMPTING
[INFO] [move_base_status_timeline]:   7: RECALLING
[INFO] [move_base_status_timeline]:   8: RECALLED
[INFO] [move_base_status_timeline]:   9: LOST
```

## Dependencies
- `rospy`
- `matplotlib`
- `numpy`
- `actionlib_msgs`

## Troubleshooting

### Lỗi thường gặp
1. **Không hiển thị plot**: Đảm bảo có display X11 hoặc set `DISPLAY=:0`
2. **Không nhận được dữ liệu**: Kiểm tra topic `/move_base/status` có đang publish không
3. **Performance chậm**: Giảm số điểm dữ liệu lưu trữ trong code

### Debug
- Sử dụng `rostopic echo /move_base/status` để kiểm tra dữ liệu
- Kiểm tra log ROS để xem thông tin chi tiết 