# Odometry selectable + ZED VIO (Body Tracking đồng thời)

## Kiến trúc trước / sau

### Trước

```text
wheel → /odom_wheel ─┐
                     ├─ EKF (ekf_x_omni4wd.yaml) → /odometry/filtered → /odom
RF2O  → /odom_rf2o  ─┘                         + TF odom→base_footprint

ZED body tracking → /bodies → social_density_fusion → /social_state
(Positional Tracking bật trong SDK nhưng KHÔNG xuất ROS odom)
```

### Sau

```text
ZED 2i (zed_sdk_bridge_node)
├── Body Tracking → /bodies → social costmap (giữ nguyên)
└── Positional Tracking (VIO) → /odom_zed  (publish_tf=false)

wheel → /odom_wheel \ 
RF2O  → /odom_rf2o   }  chọn theo odom_mode → EKF → /odometry/filtered → /odom
ZED   → /odom_zed   /                              + TF odom→base_footprint (chỉ EKF)
```

`baseline` vẫn gọi `ekf_odom_fusion.launch` + `ekf_x_omni4wd.yaml` — hành vi cũ không đổi.

## File đã tạo / sửa

### Tạo mới

| File | Vai trò |
|------|---------|
| `x_omni4wd_slam/config/odometry/ekf_*.yaml` (7 file) | EKF theo từng mode |
| `x_omni4wd_slam/launch/odometry_modes.launch` | Chọn nguồn + EKF theo `odom_mode` |
| `x_omni4wd_navigation/launch/navigation_with_selectable_odom.launch` | Navigation + social + odom_mode |
| `x_omni4wd_bringup/launch/zed_vio.launch` | Body Tracking + publish `/odom_zed` |
| `x_omni4wd_bringup/launch/zed_sdk_bridge.launch` | Alias body tracking |
| `x_omni4wd_slam/scripts/record_odom_mode_bag.sh` | Ghi rosbag theo mode/test |
| `x_omni4wd_slam/docs/SELECTABLE_ODOM_ZED_VIO.md` | Tài liệu này |

### Đã sửa

| File | Thay đổi |
|------|----------|
| `zed_sdk_bridge/src/zed_sdk_bridge_node.cpp` | Phục hồi + publish ZED VIO `/odom_zed` (+ optional IMU/TF) |
| `zed_sdk_bridge/CMakeLists.txt`, `package.xml` | Depend nav_msgs, tf2* |
| `x_omni4wd_bringup/launch/zed2i_body_tracking_robot.launch` | Args VIO |
| `x_omni4wd_bringup/launch/x_omni4wd_bringup_fusion.launch` | Truyền VIO xuống body node |
| `ODOMETRY_MODES.md` | Thêm mục selectable |

**Không sửa:** `ekf_x_omni4wd.yaml`, `ekf_odom_fusion.launch`, `x_omni4wd_navigation_real_ekf.launch` (baseline).

## Bảng tám `odom_mode`

| `odom_mode` | Nguồn | YAML / launch | Node xuất TF `odom→base_footprint` |
|-------------|-------|---------------|-------------------------------------|
| `baseline` | wheel + RF2O (cũ) | `ekf_x_omni4wd.yaml` qua `ekf_odom_fusion.launch` | `ekf_localization` |
| `wheel_only` | wheel twist | `ekf_wheel_only.yaml` | `ekf_localization` |
| `rf2o_only` | RF2O pose | `ekf_rf2o_only.yaml` | `ekf_localization` |
| `zed_vio_only` | ZED VIO pose | `ekf_zed_vio_only.yaml` | `ekf_localization` |
| `wheel_rf2o` | wheel + RF2O (YAML thống nhất) | `ekf_wheel_rf2o.yaml` | `ekf_localization` |
| `wheel_zed` | wheel + ZED VIO | `ekf_wheel_zed.yaml` | `ekf_localization` |
| `rf2o_zed` | RF2O + ZED VIO | `ekf_rf2o_zed.yaml` | `ekf_localization` |
| `wheel_rf2o_zed` | cả ba | `ekf_wheel_rf2o_zed.yaml` | `ekf_localization` |

## Topic

| Vai trò | Topic |
|---------|-------|
| Wheel | `/odom_wheel` |
| RF2O | `/odom_rf2o` |
| ZED VIO | `/odom_zed` |
| ZED IMU (raw) | `/zed_imu` |
| EKF filtered | `/odometry/filtered` |
| Navigation (relay) | `/odom` |
| Body Tracking | `/bodies` |
| Social costmap | `/social_state` |

## Mode `srf_zed_imu` (SRF + IMU thô)

Khác `srf_zed` (SRF + VIO):

| | `srf_zed` | `srf_zed_imu` |
|--|-----------|----------------|
| LiDAR | `/odom_srf` pose | `/odom_srf` pose |
| ZED | `/odom_zed` (VIO stereo+IMU SDK) | `/zed_imu` (gyro wz vào EKF) |
| `publish_vio_odom` | true | **false** |
| `publish_imu` | false | **true** |
| EKF YAML | `ekf_srf_zed.yaml` | `ekf_srf_zed_imu.yaml` |

```bash
roslaunch x_omni4wd_navigation navigation_with_selectable_odom.launch odom_mode:=srf_zed_imu
rostopic hz /odom_srf /zed_imu /odometry/filtered
```

## Tham số ZED (body node)

| Param | Mặc định thí nghiệm VIO | Ý nghĩa |
|-------|-------------------------|---------|
| `publish_vio_odom` | true khi mode cần ZED | Xuất `/odom_zed` từ Positional Tracking |
| `publish_vio_tf` | **false** | Không cạnh tranh TF với EKF |
| `enable_area_memory` | **true** (bắt buộc khi body+VIO) | `false` gây SIGSEGV với Body Tracking trên setup này |
| `enable_imu_fusion` | true | IMU fusion nội bộ SDK (không fuse raw IMU vào EKF) |
| `vio_two_d_mode` | true | Ép z=0, roll=pitch=0 |
| `vio_publish_hz` | 15 | Giới hạn Hz VIO để giảm tải GPU |
| `measure3d_reference_frame` | camera | Body data vẫn camera_link |

## Lệnh chạy

```bash
# Body + VIO + YOLO (tương tác người–đối tượng): YOLO mặc định chạy CPU
roslaunch x_omni4wd_navigation navigation_with_selectable_odom.launch odom_mode:=wheel_rf2o_zed

# Ép YOLO GPU (có thể làm skeleton CUDA fail):
roslaunch x_omni4wd_navigation navigation_with_selectable_odom.launch \
  odom_mode:=wheel_rf2o_zed yolo_device:=0

# Tắt YOLO nếu chỉ cần /bodies:
roslaunch x_omni4wd_navigation navigation_with_selectable_odom.launch \
  odom_mode:=wheel_rf2o_zed use_yolo:=false
```

Launch cũ vẫn dùng được:

```bash
roslaunch x_omni4wd_navigation x_omni4wd_navigation_real_ekf.launch
```

## Kiểm tra TF / topic

```bash
rostopic list | grep -Ei "zed|odom|imu|rf2o|body"
rostopic hz /odom_wheel
rostopic hz /odom_rf2o
rostopic hz /odom_zed          # khi publish_vio_odom
rostopic hz /odometry/filtered
rostopic hz /odom
rostopic echo -n 1 /odom_zed
rosrun tf tf_echo odom base_footprint
rosrun tf tf_monitor
# Chỉ một publisher odom→base_footprint (ekf_localization)
```

## Ghi chú kỹ thuật

1. **Không** chạy `zed_wrapper` cùng body tracking (cùng camera). VIO lấy từ SDK trong `zed_sdk_bridge_node`.
2. **Không** fuse raw `/zed_imu` với `/odom_zed` trong 8 mode chính.
3. Wheel omni: EKF giữ **vx và vy**.
4. `wheel_rf2o` ≠ `baseline`: cùng ý tưởng fusion nhưng YAML/topic tuyệt đối trong kiến trúc mới; baseline giữ file cũ.
5. **YOLO + Body Tracking:** một GPU — YOLO trên CUDA làm ZED skeleton fail. Mode có ZED VIO mặc định `yolo_device:=cpu` (chậm hơn nhưng `/objects` + `/bodies` cùng sống).
6. Chưa xác nhận trên robot thật: covariance ZED thực tế, drift, đồng bộ timestamp body↔TF khi đổi mode.

## Rủi ro còn lại

- Camera USB / ZED NOT DETECTED (đã thấy trên log trước) → không có `/bodies` cũng như `/odom_zed`.
- Hai nguồn pose (RF2O + ZED) có thể “kéo” nhau nếu covariance SDK kém — cần tune rejection trên dữ liệu thật.
- `zed_sdk_bridge_node.cpp` trước đây từng bị 0 byte; cần rebuild package trước khi chạy.
