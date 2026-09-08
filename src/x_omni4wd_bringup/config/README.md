# Config `x_omni4wd_bringup`

- Tham số điều khiển omni (Jacobian X-drive: `R`, `wheel_radius`, …) được truyền vào node `cmd_vel_to_omni_wheels` (package `x_omni4wd_control`) từ `launch/x_omni4wd_bringup.launch`.
- Cấu hình `ros_control` (velocity controllers) dùng chung file `$(find x_omni4wd_control)/config/controllers.yaml`.
- File bổ sung (EKF, navigation) có thể đặt tại đây khi tích hợp với stack navigation trong workspace.
