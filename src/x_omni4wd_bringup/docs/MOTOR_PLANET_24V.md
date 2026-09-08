# Động cơ Planet 24V (x_omni4wd)

**Nguồn / catalog:** [Động cơ Planet 24V — Thế Giới Chip](https://thegioichip.com.vn/products/dong-co-planet-24v)  
**Ghi chú:** 25/08/2026 — lưu từ trang sản phẩm để đối chiếu ROS / firmware.

## Thông số catalog (trích trang)

| Hạng mục | Giá trị |
|----------|---------|
| Điện áp / công suất | DC **24 V**, **90 W** |
| Tốc độ motor (trước hộp số) | **9000 rpm** |
| Encoder | **13 ppr** |
| Hộp số Planet | tỷ số **1 : 19.2**, chiều dài L = **45 mm** |
| Mô-men định mức (sau hộp số) | **15 kgf·cm** |
| Tốc độ sau hộp số (datasheet) | **468 rpm** |

## Đổi sang SI / bánh (dùng trong bringup)

Giả sử trục ra hộp số = trục bánh (không bánh răng trung gian thêm):

| Đại lượng | Công thức / giá trị |
|-----------|---------------------|
| ω bánh định mức | \(468 \times 2\pi / 60 \approx\) **49.0 rad/s** |
| `max_wheel_speed` ROS / firmware | **50.0 rad/s** ≈ trần datasheet 468 rpm (không còn là “65% của 76.37”) |
| `GEAR_RATIO` | **19.2** (khớp catalog) |
| `WHEEL_R` | **0.0635 m** (URDF / bringup) |
| `ROBOT_R` | **0.2505 m** |
| Vận tốc tuyến tính bánh (nếu ω=49 rad/s) | \(v = \omega r \approx 49 \times 0.0635 \approx\) **3.1 m/s** tại chu vi bánh — **không** phải tốc độ robot omni (Jacobian + 4 bánh) |

Envelope omni (comment TEB):  
\(k(v_x+v_y) + R\,\omega \le \omega_{\max}\,r\) với \(k=0.7071\), \(\omega_{\max}=50\), \(r=0.0635\) → vế phải ≈ **3.175**.

## Clamp cmd_vel hiện tại (an toàn lab)

Trong `x_omni4wd_bringup_*.launch` (Arduino bridge):

| Param | Mặc định | Ý nghĩa |
|-------|----------|---------|
| `max_linear` | **0.7** m/s | Khớp TEB `max_vel_x` profile mượt |
| `max_angular` | **1.2** rad/s | Khớp TEB `max_vel_theta` profile mượt |

## Lưu ý biến thể catalog cùng shop

Trên cùng cửa hàng còn biến thể **SERVO 14 000 rpm**, hộp số 19.2, encoder 13/500 ppr (trần bánh ≈ 729 rpm ≈ 76.4 rad/s).  
Ghi chú cũ trong README/firmware dùng **14 000 rpm → 76.37 rad/s → 65% = 50 rad/s**.  

Trang [dong-co-planet-24v](https://thegioichip.com.vn/products/dong-co-planet-24v) ghi **9000 rpm → 468 rpm** sau hộp số → **50 rad/s ≈ 100%** tốc độ định mức bánh theo listing này.  
Nếu robot lab thực tế là biến thể 14 000 rpm, giữ diễn giải “65% trần”; nếu đúng listing 9000 rpm / 468 rpm, diễn giải “≈ định mức bánh”.

## Liên kết file trong repo

- Launch: `launch/x_omni4wd_bringup.launch`, `x_omni4wd_bringup_fusion.launch`, `x_omni4wd_bringup_wheel_odom.launch`
- Firmware: `firmware/MSD_H20_OMNI_Bridge*`, `GEAR_RATIO`, `WHEEL_MAX_RAD_S`
- Tổng quan: `README.md` (mục tham số động cơ)
