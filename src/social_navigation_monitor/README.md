# social_navigation_monitor

Gói hợp nhất cho theo dõi và đánh giá social navigation, gom từ 2 gói cũ:

- `social_metrics_calculator`
- `move_base_monitor`

Hiện tại gói này chứa 5 node chính:

1. `sgi_calculator.py`
2. `sii_calculator.py`
3. `move_base_status_timeline.py`
4. `path_planning_monitor.py`
5. `social_navigation_dashboard.py`

---

## 1) Yêu cầu

- ROS Noetic
- Python 3
- `matplotlib`, `numpy` (qua apt hoặc môi trường ROS)

Build:

```bash
cd /home/quyenanhpt/ICR2026
catkin_make --pkg social_navigation_monitor
source devel/setup.bash
```

---

## 2) Messages

Gói định nghĩa 2 message:

- `social_navigation_monitor/SII`
- `social_navigation_monitor/SGI`

Các topic publish mặc định:

- `/sii` (`social_navigation_monitor/SII`)
- `/sgi` (`social_navigation_monitor/SGI`)

---

## 3) Mô tả node

### `sii_calculator.py`

- Input:
  - `/amcl_pose` (`geometry_msgs/PoseWithCovarianceStamped`)
  - `/social_state` (`social_msgs/SocialState`)
- Output:
  - `/sii` (`social_navigation_monitor/SII`)
- Chức năng:
  - Tính Social Individual Index (SII) theo vị trí robot và con người.

### `sgi_calculator.py`

- Input:
  - `/amcl_pose` (`geometry_msgs/PoseWithCovarianceStamped`)
  - `/social_state` (`social_msgs/SocialState`)
- Output:
  - `/sgi` (`social_navigation_monitor/SGI`)
- Chức năng:
  - Tính Social Grouping Index (SGI) cho nhóm người và tương tác người-vật.

### `move_base_status_timeline.py`

- Input:
  - `/move_base/status` (`actionlib_msgs/GoalStatusArray`)
- Chức năng:
  - Vẽ timeline trạng thái `move_base` theo thời gian thực.
  - Lưu ảnh PNG khi node shutdown.

### `path_planning_monitor.py`

- Input mặc định:
  - `/move_base/NavfnROS/plan` (`nav_msgs/Path`)
- Chức năng:
  - Theo dõi trạng thái tìm đường (Path Found / No Path).
  - Tính thống kê tỉ lệ thành công và lưu PNG khi shutdown.
- Ghi chú:
  - Có thể remap sang `/move_base/GlobalPlanner/plan` trong file launch.

### `social_navigation_dashboard.py`

- Input:
  - `/sii`
  - `/sgi`
  - `/move_base/status`
  - `~plan_topic` (mặc định `/move_base/GlobalPlanner/plan`)
- Chức năng:
  - Hiển thị dashboard 4 đồ thị:
    - SII
    - SGI
    - Move Base Status
    - Path Planning
  - Thời gian SII/SGI đã đồng bộ về gốc 0 (relative time).

---

## 4) Background and References

Trong ngữ cảnh social navigation, SII (Social Individual Index) thường được dùng để phản ánh mức độ robot xâm phạm không gian cá nhân của từng người, trong khi SGI (Social Group Index) thường được dùng để phản ánh mức độ robot xâm phạm không gian tương tác của nhóm người.

Mục này được đưa vào như tài liệu nền tảng theo literature; các chỉ số SII/SGI có thể được dùng như tham chiếu khái niệm và chỉ số đánh giá liên quan trong các nghiên cứu trước đó.

## References

1. Xuan-Tung Truong and Trung Dung Ngo, “Dynamic Social Zone based Mobile Robot Navigation for Human Comfortable Safety in Social Environments,” International Journal of Social Robotics, vol. 8, no. 5, pp. 663–684, 2016.
2. Xuan-Tung Truong and Trung Dung Ngo, “Toward Socially Aware Robot Navigation in Dynamic and Crowded Environments: A Proactive Social Motion Model,” IEEE Transactions on Automation Science and Engineering, vol. 14, no. 4, 2017.
3. Xuan-Tung Truong and Trung Dung Ngo, “To Approach Humans?: A Unified Framework for Approaching Pose Prediction and Socially Aware Robot Navigation,” IEEE Transactions on Cognitive and Developmental Systems, vol. 10, no. 3, pp. 557–572, 2018.

---

## 5) Chạy nhanh từng node

```bash
rosrun social_navigation_monitor sii_calculator.py
rosrun social_navigation_monitor sgi_calculator.py
rosrun social_navigation_monitor move_base_status_timeline.py
rosrun social_navigation_monitor path_planning_monitor.py
rosrun social_navigation_monitor social_navigation_dashboard.py
```

Ví dụ remap topic plan cho `path_planning_monitor.py`:

```bash
rosrun social_navigation_monitor path_planning_monitor.py /move_base/NavfnROS/plan:=/move_base/GlobalPlanner/plan
```

---

## 6) Tích hợp launch hiện có

Các launch chính trong repo đã được cập nhật gọi package mới `social_navigation_monitor`:

- `src/x_omni4wd_simulation/launch/ICR2026_simulation.launch`
- `src/x_omni4wd_navigation/launch/x_omni4wd_navigation.launch`
- `src/x_omni4wd_simulation/launch/x_omni4wd_simulation.launch` (comment examples)

---

## 7) Ghi chú bảo trì

- Hai package cũ đã được loại bỏ hoàn toàn khỏi `src/`.
- Khi thêm node mới liên quan social metrics/navigation monitoring, ưu tiên đặt trong gói này để giữ cấu trúc gọn.
