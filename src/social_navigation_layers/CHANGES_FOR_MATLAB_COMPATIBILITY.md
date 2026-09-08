# Thay đổi để tương thích với MATLAB motion_simulation_2d.m

> **Lưu ý:** `AngularProxemicLayer` đã được gỡ khỏi dự án. Tài liệu này chỉ giữ cho mục đích tham khảo lịch sử.

## Tổng quan
Đã cập nhật `AngularProxemicLayer` để tính toán chi phí giống hệt như trong MATLAB `motion_simulation_2d.m`.

## Những thay đổi chính

### 1. Hàm `calculateAngularProxemicCost()`

**Trước:**
```cpp
// Tính vận tốc tổng hợp (tuyến tính + góc)
double mag = sqrt(pow(person.velocity.linear.x, 2) + pow(person.velocity.linear.y, 2));
double angular_vel = calculateAngularVelocity(person.velocity);
double factor = 1.0 + mag * factor_ + fabs(angular_vel) * angular_factor_;

// Tính hướng ảnh hưởng
double influence_direction = calculateInfluenceDirection(person);
double elongation_factor = calculateElongationFactor(person);
```

**Sau (giống MATLAB):**
```cpp
// Tính vận tốc và factor - giống MATLAB
double velocity_mag = sqrt(pow(person.velocity.linear.x, 2) + pow(person.velocity.linear.y, 2));
double velocity_angle = atan2(person.velocity.linear.y, person.velocity.linear.x);
double factor = 1.0 + velocity_mag * factor_;
```

### 2. Logic tính toán góc chênh lệch

**Trước:**
```cpp
double angle_diff = angles::shortest_angular_distance(influence_direction, person_to_point_angle);
```

**Sau (giống MATLAB):**
```cpp
// Tính góc chênh lệch so với hướng vận tốc - giống MATLAB wrapToPi
double angle_diff = person_to_point_angle - velocity_angle;
// Chuẩn hóa góc về [-π, π] - giống MATLAB wrapToPi
while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
while (angle_diff < -M_PI) angle_diff += 2 * M_PI;
```

### 3. Logic tính toán chi phí

**Trước:**
```cpp
if (fabs(angle_diff) <= M_PI / 2) {
    // Phía trước: kéo dài theo hướng vận tốc và góc
    double sigma_x = covar_ * factor * elongation_factor;
    double sigma_y = covar_;
    cost = gaussian2D_oriented(x, y, person.position.position.x, person.position.position.y, 
                             amplitude_, sigma_x, sigma_y, influence_direction);
}
```

**Sau (giống MATLAB):**
```cpp
if (fabs(angle_diff) <= M_PI / 2) {
    // Phía trước: kéo dài theo hướng vận tốc - giống MATLAB
    double sigma_x = covar_ * factor;
    double sigma_y = covar_;
    cost = gaussian2D_oriented(x, y, person.position.position.x, person.position.position.y, 
                             amplitude_, sigma_x, sigma_y, velocity_angle);
}
```

### 4. Áp dụng cutoff

**Trước:**
```cpp
if (cost < cutoff_)
    continue;
```

**Sau (giống MATLAB):**
```cpp
// Áp dụng cutoff - giống MATLAB
if (cost < cutoff_) {
    cost = 0.0;
}
```

### 5. Logic lấy giá trị tối đa

**Trước:**
```cpp
// Tính toán chi phí dựa trên mật độ
double density_cost = calculateDensityAdjustedCost(cost);
unsigned char cvalue = (unsigned char) density_cost;
costmap->setCost(i + dx, j + dy, std::max(cvalue, old_cost));
```

**Sau (giống MATLAB):**
```cpp
// Lấy giá trị tối đa thay vì cộng dồn - giống MATLAB
unsigned char cvalue = (unsigned char) cost;
costmap->setCost(i + dx, j + dy, std::max(cvalue, old_cost));
```

## Các hàm đã loại bỏ

1. `calculateInfluenceDirection()` - không cần thiết
2. `calculateElongationFactor()` - không cần thiết
3. `calculateDensityAdjustedCost()` - không sử dụng

## Các hàm giữ lại

1. `calculateAngularVelocity()` - giữ lại để tương thích
2. `gaussian2D_oriented()` - giống hệt MATLAB
3. `get_radius()` - giống hệt MATLAB

## Tham số cấu hình

Các tham số vận tốc góc và mật độ được giữ lại để tương thích với cấu hình hiện tại, nhưng không được sử dụng trong tính toán chính.

## Kết quả

- ✅ Tính toán chi phí giống hệt MATLAB
- ✅ Logic góc chênh lệch giống MATLAB
- ✅ Áp dụng cutoff giống MATLAB
- ✅ Lấy giá trị tối đa giống MATLAB
- ✅ Tương thích với cấu hình hiện tại

## Script so sánh

Đã tạo script `compare_with_matlab.py` để so sánh kết quả C++ với MATLAB.

## Cách sử dụng

1. Chạy MATLAB `motion_simulation_2d.m`
2. Chạy C++ `angular_proxemic_layer`
3. Chạy script so sánh: `python3 compare_with_matlab.py` 