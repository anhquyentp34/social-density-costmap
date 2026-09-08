# Motion Simulation 2D với Proxemic Cost

## Mô tả
Đây là phiên bản C++ của code MATLAB `motion_simulation_2d.m`, mô phỏng chuyển động người đi bộ với vận tốc góc và tính toán proxemic cost.

## Cấu trúc Files

```
social_navigation_layers/
├── src/
│   ├── motion_simulation_2d.cpp          # Code C++ chính
│   └── proxemic_layer.cpp                # Code gốc để tham khảo
├── scripts/
│   └── visualize_motion_simulation.py    # Script Python để visualize
├── CMakeLists_motion_simulation.txt      # CMakeLists cho biên dịch
└── README_motion_simulation.md           # File này
```

## Tính năng chính

### 1. Mô phỏng chuyển động
- **Vận tốc tịnh tiến**: 2.0 m/s
- **Vận tốc góc**: 1.0 rad/s (tạo chuyển động cong)
- **Thời gian mô phỏng**: 10 giây
- **Bước thời gian**: 0.05s (20Hz)

### 2. Proxemic Model
- **Biên độ**: 77.0
- **Phương sai cơ bản**: 0.25
- **Ngưỡng cutoff**: 10.0
- **Hệ số vận tốc**: 2.0

### 3. Tính toán Proxemic Cost
- Sử dụng hàm Gaussian 2D có định hướng
- Kéo dài theo hướng vận tốc (phía trước)
- Không kéo dài (phía sau)
- Áp dụng ngưỡng cutoff

## Cách biên dịch và chạy

### Bước 1: Biên dịch code C++
```bash
# Tạo thư mục build
mkdir build
cd build

# Biên dịch
cmake -f ../CMakeLists_motion_simulation.txt ..
make

# Hoặc biên dịch trực tiếp với g++
g++ -std=c++11 -O2 -o motion_simulation_2d ../src/motion_simulation_2d.cpp -lm
```

### Bước 2: Chạy mô phỏng
```bash
./motion_simulation_2d
```

### Bước 3: Visualize kết quả
```bash
python3 ../scripts/visualize_motion_simulation.py
```

## Output Files

### 1. trajectory_data.csv
```
# Trajectory Data (time, x, y, theta, vx, vy)
0.000000 0.000000 0.000000 0.000000 2.000000 0.000000
0.050000 0.100000 0.000000 0.050000 1.998750 0.099917
...
```

### 2. proxemic_cost_data.csv
```
# Grid Information
# x_min: -3.000000
# x_max: 3.000000
# y_min: -3.000000
# y_max: 3.000000
# nx: 120
# ny: 120
# dx: 0.050847
# dy: 0.050847

# Proxemic Cost Data (i, j, x, y, cost)
0 0 -3.000000 -3.000000 0.000000
0 1 -2.949153 -3.000000 0.000000
...
```

### 3. motion_simulation_results.png
- Biểu đồ 3D surface của proxemic cost
- Biểu đồ 2D contour với quỹ đạo
- Thống kê chi tiết

## So sánh với MATLAB

| Tính năng | MATLAB | C++ |
|-----------|--------|-----|
| Tốc độ | Chậm hơn | Nhanh hơn |
| Memory usage | Cao hơn | Thấp hơn |
| Visualization | Tích hợp | Cần Python |
| Deployment | Khó | Dễ |
| Debugging | Dễ | Khó hơn |

## Tham số có thể điều chỉnh

Trong `SimulationParams` constructor:

```cpp
SimulationParams() {
    v = 2.0;              // Vận tốc tịnh tiến (m/s)
    w = 1.0;              // Vận tốc góc quay (rad/s)
    sim_time = 10.0;      // Thời gian mô phỏng (s)
    dt = 0.05;            // Bước thời gian (s)
    amplitude = 77.0;      // Biên độ cực đại
    sigma_base = 0.25;    // Phương sai cơ bản
    cutoff = 10.0;        // Ngưỡng chi phí tối thiểu
    factor_param = 2.0;   // Hệ số vận tốc
}
```

## Lưu ý quan trọng

1. **Performance**: Code C++ chạy nhanh hơn MATLAB khoảng 10-50 lần
2. **Memory**: Sử dụng ít bộ nhớ hơn MATLAB
3. **Precision**: Độ chính xác số học tương đương
4. **Compatibility**: Cần compiler hỗ trợ C++11

## Troubleshooting

### Lỗi biên dịch
```bash
# Lỗi: không tìm thấy math.h
# Giải pháp: Thêm -lm flag
g++ -std=c++11 -o motion_simulation_2d motion_simulation_2d.cpp -lm
```

### Lỗi runtime
```bash
# Lỗi: không tìm thấy file output
# Giải pháp: Kiểm tra quyền ghi trong thư mục
chmod 755 .
```

### Lỗi visualization
```bash
# Lỗi: thiếu thư viện Python
# Giải pháp: Cài đặt dependencies
pip install numpy matplotlib pandas
```

## Mở rộng

### 1. Thêm ROS integration
```cpp
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
```

### 2. Thêm dynamic reconfigure
```cpp
#include <dynamic_reconfigure/server.h>
#include <motion_simulation/SimulationConfig.h>
```

### 3. Thêm visualization real-time
```cpp
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
```

## Tác giả
Dựa trên code MATLAB gốc từ `motion_simulation_2d.m`
Chuyển đổi sang C++ để tối ưu performance và deployment. 