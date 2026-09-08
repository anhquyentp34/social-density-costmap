# MSD_H20_OMNI_Bridge_WheelOdom

Firmware Arduino (Uno/Nano) cho robot thật **x_omni4wd**: điều khiển 4 driver MSD_H20 **và** đọc vận tốc encoder để ROS tính wheel odom.

Dựa trên:
- `../MSD_H20_OMNI_Bridge/MSD_H20_OMNI_Bridge.ino` — gửi lệnh `VXYW` → `N<addr> v... a...`
- `../MSD_XX_Uart_Example.ino` — `N<addr> O G1` / `O G3` đọc phản hồi driver

## Kết nối

| Bus | Baud | Ghi chú |
|-----|------|---------|
| USB Serial (PC) | 19200 | `/dev/robot_arduino` |
| SoftwareSerial 13(RX), 12(TX) | 19200 | Mạng UART nhiều MSD_H20 |

Cấu hình driver MSD (nút / DcTunerPro): **C4=1** velocity, **C5=1** UART network, **C7=2** → 19200, địa chỉ **10/20/30/40**.

## Giao thức USB

### PC → Arduino (giống bridge cũ)

```
VXYW <vx_mm_s> <vy_mm_s> <w_mrad_s>
STOP
```

Scaled int: mm/s và mrad/s nhân **1000** (khớp `arduino_interface_omni.py`).

### Arduino → PC (mới)

```
WVEL <w_fl> <w_rl> <w_rr> <w_fr>
```

- Đơn vị: **mrad/s × 1000** tại **trục bánh** (đã chia `GEAR_RATIO` từ vận tốc motor).
- Thứ tự: **FL, RL, RR, FR** (khớp `wheel_odometry_node.py`).
- Tần số ~**20 Hz** (`WVEL_USB_PERIOD_MS`).

Ví dụ: bánh FL quay `0.5 rad/s` → `WVEL 500 ...`.

## Đọc MSD (chu kỳ 4 bánh)

Mỗi **50 ms** (~20 Hz), firmware hỏi **đủ 4** driver rồi mới gửi một dòng `WVEL`:

```
N10 O G1  →  N20 O G1  →  N30 O G1  →  N40 O G1  →  WVEL ...
```

- Parse địa chỉ `N<addr>` trong dòng trả về (tránh gán nhầm bánh).
- Parse trường `v` (rad/s tại motor) → `ω_bánh = v_motor / GEAR_RATIO`.
- Đo G1 cũ hơn `WHEEL_MEAS_STALE_MS` (250 ms) → dùng ω từ lệnh `VXYW` cho bánh đó.

**Lỗi đã sửa (bản mới):** trước đây `pollIdx` tăng trước khi nhận phản hồi → vận tốc gán sai bánh; `WVEL` có thể chỉ đúng 1/4 encoder.

## Tham số chỉnh trong .ino

| Biến | Mặc định | Ý nghĩa |
|------|----------|---------|
| `WHEEL_R` | 0.0635 | Bán kính bánh (m) |
| `ROBOT_R` | 0.2505 | Tâm robot → bánh (m) |
| `GEAR_RATIO` | 19.2 | Motor / bánh |
| `WHEEL_MAX_RAD_S` | 50.0 | ≈ 468 rpm bánh ([Planet 24V](https://thegioichip.com.vn/products/dong-co-planet-24v)); hoặc ~65% trần 76.37 nếu motor 14k rpm — xem `docs/MOTOR_PLANET_24V.md` |
| `ADDR_FL/RL/RR/FR` | 10/20/30/40 | Địa chỉ MSD |

## ROS (bước sau)

Node Python (bringup v2) sẽ:
1. Gửi `VXYW` như `arduino_interface_omni.py`
2. Đọc `WVEL` → publish `/joint_states` hoặc trực tiếp feed `wheel_odometry_node.py`

## Nạp firmware

1. Arduino IDE → mở `MSD_H20_OMNI_Bridge_WheelOdom.ino`
2. Board Uno/Nano, cổng COM, upload
3. Serial Monitor **19200** — thấy `READY ...`, khi quay bánh thấy dòng `WVEL ...`
