# B-srf-imu — EKF overlay (Track B)

Track B dùng EKF yaml gốc trong `config/odometry/`:
- `ekf_wheel_only.yaml` → variant **W**
- `ekf_srf_only.yaml` → variant **S**
- `ekf_srf_zed_imu.yaml` → variant **SI**

Copy vào đây nếu cần tune riêng cho thí nghiệm Track B (không ảnh hưởng track khác).

Launch: `launch/B-srf-imu/x_omni4wd_navigation_real_track_b.launch`  
Script: `tracks/B-srf-imu/run_real_track_b.sh`
