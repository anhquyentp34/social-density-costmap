# Wheel odometry (thay RF2O)

Phiên bản odom từ **4 bánh** (encoder / Gazebo `joint_states.velocity`), không dùng laser RF2O.

## File

| File | Mô tả |
|------|--------|
| `launch/wheel_odometry.launch` | Tương đương `rf2o_laser_odometry.launch` |
| `config/wheel_odom_x_omni4wd.yaml` | Tham số Jacobian, frame, topic |
| `scripts/wheel_odometry_node.py` | Node tích phân `/odom` + TF |

## Chạy

**Mô phỏng Gazebo:**

```bash
roslaunch x_omni4wd_slam wheel_odometry.launch simulation:=true
```

**Robot thật** (cần `joint_states` có `velocity`; tùy chọn RPLidar cho `/scan`):

```bash
roslaunch x_omni4wd_slam wheel_odometry.launch simulation:=false
```

Nếu encoder ở trục motor (gear 19.2 như firmware MSD):

```bash
roslaunch x_omni4wd_slam wheel_odometry.launch simulation:=false gear_ratio:=19.2
```

## Dấu bánh (`sign_fl` … `sign_fr`)

Theo URDF hiện tại, cả 4 khớp bánh chủ dùng **axis `+X`** (đã đảo từ `-X` SolidWorks để khớp Jacobian / firmware). `cmd_vel_to_omni_wheels.py` không nhân dấu hiệu chỉnh — wheel odom mặc định `sign_*: 1.0`.

Chỉ chỉnh `±1` khi chạy robot thật và tốc độ encoder MSD ngược chiều so với URDF.

## So với RF2O

| | RF2O | Wheel odom |
|---|------|------------|
| Nguồn | `/scan` | `joint_states.velocity` |
| Trượt bánh omni | Ít ảnh hưởng | Dễ lệch khi trượt |
| Launch | `rf2o_laser_odometry.launch` | `wheel_odometry.launch` |

## Bringup / Cartographer

Thay RF2O trong bringup: không include node `rf2o_laser_odometry`; include `wheel_odometry.launch` hoặc chạy song song với `use_lidar:=false` và node wheel odom riêng.

Cartographer: vẫn remap `/odom` như cũ; có thể `publish_tf:=false` trên wheel odom khi Cartographer cấp TF `map->odom`.

Kết hợp wheel + RF2O (EKF): xem `docs/EKF_ODOM_FUSION.md` và `launch/ekf_odom_fusion.launch`.
