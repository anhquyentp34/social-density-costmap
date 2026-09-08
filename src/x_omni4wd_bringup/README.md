# x_omni4wd_bringup

Gói ROS bringup cho robot **x_omni4wd** (4 bánh omni X-drive).
Gói này dùng cho 2 kịch bản chính:
- Chế độ 1: Mô phỏng Gazebo (không dùng Arduino/firmware)
  - Thường chạy `roslaunch x_omni4wd_simulation x_omni4wd_simulation_rf2o.launch` (xem `ODOMETRY_MODES.md`).
  - Nếu dùng thêm `x_omni4wd_bringup/launch/x_omni4wd_bringup.launch` thì đảm bảo `use_serial_firmware:=false`.
  - Nên bật `use_sim_time:=true` khi chạy Gazebo.
  - Nếu bạn muốn dùng `ros_control` trong mô phỏng: bật `use_ros_control:=true`.
  - Lưu ý: nếu bạn đã chạy launch Gazebo (ví dụ `x_omni4wd_simulation`), tránh chạy trùng node điều khiển bánh (`cmd_vel_to_omni_wheels.py`).
- Chế độ 2: Điều hướng robot thật (dùng Arduino/firmware)
  - Bật `use_serial_firmware:=true` để `cmd_vel` được bridge qua `arduino_interface_omni.py` và gửi về firmware `MSD_H20_OMNI_Bridge`.
  - Navigation stack chỉ cần publish `cmd_vel` (thường là `/cmd_vel`; hoặc đổi bằng `cmd_vel_topic` trong launch).

`x_omni4wd_bringup/launch/x_omni4wd_bringup.launch` điều khiển việc bật/tắt chế độ qua `use_serial_firmware` và `use_ros_control`.

## UART / Firmware (MSD_H20_OMNI_Bridge)
Phần này chỉ áp dụng cho chế độ robot thật khi `use_serial_firmware:=true`.

### Giao thức PC -> Arduino
- **DEPRECATED**: `VXYW <vx> <vy> <w>` (m/s, m/s, rad/s) qua USB, baud **19200**.
- **Khuyến nghị**: `VXYW <vx_mm_s> <vy_mm_s> <w_mrad_s>` (scaled int: nhân 1000; chia 1000 để ra m/s, rad/s) qua USB, baud **19200**.

### Arduino -> 4 driver MSD
- Lệnh `N<addr> v<rad/s> a<accel>` qua SoftwareSerial (chân 12 TX, 13 RX), baud 19200.
- Trong firmware này, trường `v` được làm tròn sang int để giảm thời gian truyền UART.

### Jacobian chuẩn (khớp `cmd_vel_to_omni_wheels.py`)

Với \(K = 1/\sqrt{2} \approx 0{,}7071\), \(R =\) `ROBOT_R`, \(r =\) `WHEEL_R` (rad/s tại bánh trước `GEAR_RATIO`):

- \( \omega_{FL} = (K v_x - K v_y - R\,\omega) / r \)
- \( \omega_{RL} = (K v_x + K v_y - R\,\omega) / r \)
- \( \omega_{RR} = (-K v_x + K v_y - R\,\omega) / r \)
- \( \omega_{FR} = (-K v_x - K v_y - R\,\omega) / r \)

REP-103: X tiến, Y trái, ω CCW.

### Tham số mặc định (trùng Python / `rosparam`)

| Tham số     | Mặc định | Ghi chú |
|------------|----------|--------|
| WHEEL_R    | **0.0635** m | `~wheel_radius` |
| ROBOT_R    | **0.2505** m | `~R` |
| K_LIN      | **0.7071** | \(1/\sqrt{2}\) |
| GEAR_RATIO | 19.2 | Tỷ số Planet (catalog) |
| MOTOR (catalog) | 24V / 90W / **9000 rpm** / enc **13 ppr** / sau hộp số **468 rpm** | Xem [Thế Giới Chip — Động cơ Planet 24V](https://thegioichip.com.vn/products/dong-co-planet-24v) và `docs/MOTOR_PLANET_24V.md` |
| MOTOR_MAX_RPM (ghi chú cũ) | **14000** | Biến thể khác cùng shop; sau hộp số ≈ **729 rpm** ≈ **76,37 rad/s** tại bánh |
| WHEEL_MAX_RAD_S / `max_wheel_speed` | **50.0** | ≈ **468 rpm** (listing Planet 24V) hoặc ~65% trần 76.37 nếu motor 14k rpm |
| `max_linear` / `max_angular` | **0.7** m/s / **1.2** rad/s | Clamp Arduino; khớp TEB profile mượt Track D |
| ADDR_FL / ADDR_RL / ADDR_RR / ADDR_FR | **10 / 20 / 30 / 40** | Khớp `w_fl`, `w_rl`, `w_rr`, `w_fr` |

Chi tiết datasheet + quy đổi SI: **`docs/MOTOR_PLANET_24V.md`**.

### Nạp Arduino (chỉ điều khiển)
1. Mở thư mục `x_omni4wd_bringup/firmware/MSD_H20_OMNI_Bridge` trong Arduino IDE.
2. Chọn board (Arduino Uno/Nano) và cổng COM.
3. Upload. Serial Monitor **19200**.

### Firmware wheel feedback (`MSD_H20_OMNI_Bridge_WheelOdom`)

Dùng cho bringup odom từ bánh (bước tiếp theo):

| | `MSD_H20_OMNI_Bridge` | `MSD_H20_OMNI_Bridge_WheelOdom` |
|---|----------------------|----------------------------------|
| PC → Arduino | `VXYW ...` | Giống |
| Arduino → MSD | `N<addr> v... a...` | Giống |
| MSD → Arduino | — | `N<addr> O G1` (đọc `v` motor) |
| Arduino → PC | — | `WVEL w_fl w_rl w_rr w_fr` (mrad/s×1000, tại bánh) |

Chi tiết: `firmware/MSD_H20_OMNI_Bridge_WheelOdom/README.md`.

Bringup + navigation wheel odom:

```bash
roslaunch x_omni4wd_bringup x_omni4wd_bringup_wheel_odom.launch
roslaunch x_omni4wd_navigation x_omni4wd_navigation_real_wheel_odom.launch
```

### ROS
`arduino_interface_omni.py` đóng vai trò bridge (chế độ robot thật): lấy `Twist` (`linear.x`, `linear.y`, `angular.z`) từ `cmd_vel` (thường `/cmd_vel`; hoặc theo `cmd_vel_topic`) và phát về Arduino theo lệnh `VXYW ...`.
