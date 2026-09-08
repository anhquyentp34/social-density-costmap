# EKF odom fusion (wheel + RF2O)

Gộp hai nguồn định vị tương đối cho `x_omni4wd` bằng `robot_localization` (`ekf_localization_node`).

Xem bảng đầy đủ launch + thí nghiệm: [`ODOMETRY_MODES.md`](../../../ODOMETRY_MODES.md) (gốc workspace).

## Tệp trong `x_omni4wd_slam`

| Tệp | Mô tả |
|-----|--------|
| `config/ekf_x_omni4wd.yaml` | Tham số EKF (odom0=wheel twist, odom1=RF2O pose) |
| `launch/ekf_odom_fusion.launch` | Wheel + RF2O + EKF + relay `/odom` |
| `config/wheel_odom_x_omni4wd.yaml` | Tham số wheel odometry |
| `config/rf2o_x_omni4wd.yaml` | Tham số RF2O |
| `scripts/wheel_odometry_node.py` | Odom từ encoder / Gazebo joint_states |
| `launch/rf2o_laser_odometry.launch` | Chỉ RF2O (một nguồn) |
| `launch/wheel_odometry.launch` | Chỉ wheel odom (một nguồn) |

## Luồng topic

```text
joint_states -> wheel_odometry -> /odom_wheel
/scan        -> RF2O          -> /odom_rf2o
                    |
              ekf_localization
                    |
         /odometry/filtered -> relay -> /odom
                    |
              TF odom -> base_footprint
```

**Không** chạy đồng thời `wheel_odometry` và RF2O cùng publish `/odom` + TF — chỉ EKF xuất ra.

## Launch đầy đủ

| Môi trường | Launch |
|------------|--------|
| Robot thật | `roslaunch x_omni4wd_navigation x_omni4wd_navigation_real_ekf.launch` |
| Mô phỏng | `roslaunch x_omni4wd_simulation x_omni4wd_simulation_ekf.launch` |

## Chạy độc lập (chỉ fusion, không navigation)

```bash
roslaunch x_omni4wd_slam ekf_odom_fusion.launch
# Robot thật:
roslaunch x_omni4wd_slam ekf_odom_fusion.launch joint_states_topic:=/wheel_encoder_joint_states
```

## Script thí nghiệm

| Script | Môi trường | Odom |
|--------|------------|------|
| `run_x_omni4wd_navigation_real_ekf_experiment_VCCA2026.sh` | Robot thật | EKF |
| `run_x_omni4wd_simulation_ekf_experiment_VCCA2026.sh` | Gazebo | EKF |

## Robot tự dừng khi đang chạy?

Nếu terminal báo `REQUIRED process [zed2i/zed_node] has died! exit code -11` → node ZED segfault, roslaunch **tắt cả stack** (Arduino ngừng nhận `cmd_vel`).

Đã xử lý trong `x_omni4wd_bringup/launch/zed2i_robot.launch` (`required:=false`, `respawn:=true`, tắt spatial mapping).

Tạm thời không dùng ZED:

```bash
roslaunch x_omni4wd_navigation x_omni4wd_navigation_real_ekf.launch use_zed:=false
```

## Tune

Chỉnh `config/ekf_x_omni4wd.yaml`:

- `odom0_*`: wheel (twist) — tăng `twist_rejection_threshold` nếu encoder nhiễu
- `odom1_*`: RF2O (pose) — tăng `pose_rejection_threshold` nếu scan kém
- `process_noise_covariance`: độ “mềm” của mô hình dự đoán
