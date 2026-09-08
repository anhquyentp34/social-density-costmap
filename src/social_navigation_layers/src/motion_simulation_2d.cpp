// motion_simulation_2d.cpp
// Mô phỏng mô hình proxemic có tính đến vận tốc góc của người đi bộ
// Tích hợp social cost visualization

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <string>
#include <chrono>

// Cấu trúc để lưu trữ điểm
struct Point2D {
    double x, y;
    Point2D(double x = 0.0, double y = 0.0) : x(x), y(y) {}
};

// Cấu trúc để lưu trữ vận tốc
struct Velocity2D {
    double vx, vy;
    Velocity2D(double vx = 0.0, double vy = 0.0) : vx(vx), vy(vy) {}
};

// Cấu trúc để lưu trữ tham số mô phỏng
struct SimulationParams {
    double v;              // Vận tốc tịnh tiến (m/s)
    double w;              // Vận tốc góc quay (rad/s)
    double sim_time;       // Thời gian mô phỏng (s)
    double dt;             // Bước thời gian (s)
    
    // Tham số Social Cost (Proxemic)
    double amplitude;      // Biên độ cực đại
    double sigma_base;     // Phương sai cơ bản
    double cutoff;         // Ngưỡng chi phí tối thiểu
    double factor_param;   // Hệ số vận tốc
    
    SimulationParams() {
        v = 2.0;
        w = 1.0;
        sim_time = 10.0;
        dt = 0.05;
        amplitude = 77.0;
        sigma_base = 0.25;
        cutoff = 10.0;
        factor_param = 2.0;
    }
};

// Cấu trúc để lưu trữ quỹ đạo
struct Trajectory {
    std::vector<double> x, y, theta;
    std::vector<double> velocity_x, velocity_y;
    std::vector<double> time;
    
    void resize(size_t size) {
        x.resize(size);
        y.resize(size);
        theta.resize(size);
        velocity_x.resize(size);
        velocity_y.resize(size);
        time.resize(size);
    }
    
    size_t size() const {
        return x.size();
    }
};

// Cấu trúc để lưu trữ lưới
struct Grid {
    std::vector<std::vector<double>> data;
    double x_min, x_max, y_min, y_max;
    int nx, ny;
    double dx, dy;
    
    Grid(double x_min, double x_max, double y_min, double y_max, int nx, int ny) 
        : x_min(x_min), x_max(x_max), y_min(y_min), y_max(y_max), nx(nx), ny(ny) {
        
        dx = (x_max - x_min) / (nx - 1);
        dy = (y_max - y_min) / (ny - 1);
        
        data.resize(ny, std::vector<double>(nx, 0.0));
    }
    
    double& operator()(int i, int j) {
        return data[i][j];
    }
    
    const double& operator()(int i, int j) const {
        return data[i][j];
    }
    
    Point2D getWorldCoords(int i, int j) const {
        return Point2D(x_min + j * dx, y_min + i * dy);
    }
};

class MotionSimulation2D {
private:
    SimulationParams params_;
    Trajectory trajectory_;
    Grid* grid_;
    
public:
    MotionSimulation2D(const SimulationParams& params) : params_(params), grid_(nullptr) {}
    
    ~MotionSimulation2D() {
        if (grid_) delete grid_;
    }
    
    // Hàm Gaussian 2D có định hướng cho proxemic model
    double gaussian2D_oriented(double x, double y, double x0, double y0, 
                               double A, double sigma_x, double sigma_y, double theta_bearing) {
        // Tính khoảng cách và góc từ tâm đến điểm
        double dx = x - x0;
        double dy = y - y0;
        double h = sqrt(dx * dx + dy * dy);
        double angle = atan2(dy, dx);
        
        // Chuyển đổi sang hệ tọa độ định hướng
        double mx = h * cos(angle - theta_bearing);
        double my = h * sin(angle - theta_bearing);
        
        // Tính giá trị Gaussian
        double f1 = (mx * mx) / (2.0 * sigma_x * sigma_x);
        double f2 = (my * my) / (2.0 * sigma_y * sigma_y);
        return A * exp(-(f1 + f2));
    }
    
    // Tính toán quỹ đạo người đi bộ
    void calculateTrajectory() {
        std::cout << "🔄 Đang tính toán quỹ đạo người đi bộ..." << std::endl;
        
        int N = static_cast<int>(params_.sim_time / params_.dt) + 1;
        trajectory_.resize(N);
        
        // Khởi tạo vị trí ban đầu
        trajectory_.x[0] = 0.0;
        trajectory_.y[0] = 0.0;
        trajectory_.theta[0] = 0.0;
        trajectory_.time[0] = 0.0;
        
        // Tính vận tốc ban đầu
        trajectory_.velocity_x[0] = params_.v * cos(trajectory_.theta[0]);
        trajectory_.velocity_y[0] = params_.v * sin(trajectory_.theta[0]);
        
        // Tính toán quỹ đạo theo thời gian
        for (int i = 1; i < N; i++) {
            trajectory_.time[i] = i * params_.dt;
            trajectory_.theta[i] = trajectory_.theta[i-1] + params_.w * params_.dt;
            
            double vx = params_.v * cos(trajectory_.theta[i]);
            double vy = params_.v * sin(trajectory_.theta[i]);
            
            trajectory_.x[i] = trajectory_.x[i-1] + vx * params_.dt;
            trajectory_.y[i] = trajectory_.y[i-1] + vy * params_.dt;
            trajectory_.velocity_x[i] = vx;
            trajectory_.velocity_y[i] = vy;
        }
        
        std::cout << "✅ Hoàn thành tính toán quỹ đạo!" << std::endl;
    }
    
    // Tạo lưới cho proxemic cost
    void createGrid() {
        std::cout << "🔄 Đang tạo lưới cho proxemic cost..." << std::endl;
        
        // Tìm phạm vi của quỹ đạo
        double x_min = *std::min_element(trajectory_.x.begin(), trajectory_.x.end()) - 3.0;
        double x_max = *std::max_element(trajectory_.x.begin(), trajectory_.x.end()) + 3.0;
        double y_min = *std::min_element(trajectory_.y.begin(), trajectory_.y.end()) - 3.0;
        double y_max = *std::max_element(trajectory_.y.begin(), trajectory_.y.end()) + 3.0;
        
        // Tạo lưới 120x120 điểm
        grid_ = new Grid(x_min, x_max, y_min, y_max, 120, 120);
        
        std::cout << "✅ Hoàn thành tạo lưới!" << std::endl;
    }
    
    // Tính toán proxemic cost
    void calculateProxemicCost() {
        std::cout << "🔄 Đang tính toán proxemic cost..." << std::endl;
        
        int num_prediction_points = std::min(40, static_cast<int>(trajectory_.size()));
        std::cout << "   Số điểm dự đoán: " << num_prediction_points << std::endl;
        std::cout << "   Khoảng thời gian giữa các điểm: " << params_.dt << " s" << std::endl;
        
        // Tính proxemic cost cho các điểm dự đoán
        for (int k = 0; k < num_prediction_points; k++) {
            Point2D person_pos(trajectory_.x[k], trajectory_.y[k]);
            Velocity2D velocity(trajectory_.velocity_x[k], trajectory_.velocity_y[k]);
            
            // Tính vận tốc và factor
            double velocity_mag = sqrt(velocity.vx * velocity.vx + velocity.vy * velocity.vy);
            double velocity_angle = atan2(velocity.vy, velocity.vx);
            double factor = 1.0 + velocity_mag * params_.factor_param;
            
            // Tính proxemic cost cho từng điểm trong lưới
            for (int i = 0; i < grid_->ny; i++) {
                for (int j = 0; j < grid_->nx; j++) {
                    Point2D point = grid_->getWorldCoords(i, j);
                    
                    // Tính khoảng cách và góc từ người đến điểm
                    double dx = point.x - person_pos.x;
                    double dy = point.y - person_pos.y;
                    double person_to_point_angle = atan2(dy, dx);
                    
                    // Tính góc chênh lệch so với hướng vận tốc
                    double angle_diff = person_to_point_angle - velocity_angle;
                    // Chuẩn hóa góc về [-π, π]
                    while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
                    while (angle_diff < -M_PI) angle_diff += 2 * M_PI;
                    
                    double cost = 0.0;
                    
                    // Tính chi phí dựa trên vận tốc (proxemic model)
                    if (fabs(angle_diff) <= M_PI / 2) {
                        // Phía trước: kéo dài theo hướng vận tốc
                        double sigma_x = params_.sigma_base * factor;
                        double sigma_y = params_.sigma_base;
                        cost = gaussian2D_oriented(point.x, point.y, person_pos.x, person_pos.y, 
                                                 params_.amplitude, sigma_x, sigma_y, velocity_angle);
                    } else {
                        // Phía sau: không kéo dài
                        cost = gaussian2D_oriented(point.x, point.y, person_pos.x, person_pos.y, 
                                                 params_.amplitude, params_.sigma_base, params_.sigma_base, velocity_angle);
                    }
                    
                    // Áp dụng cutoff
                    if (cost < params_.cutoff) {
                        cost = 0.0;
                    }
                    
                    // Lấy giá trị tối đa thay vì cộng dồn
                    grid_->operator()(i, j) = std::max(grid_->operator()(i, j), cost);
                }
            }
        }
        
        std::cout << "✅ Hoàn thành tính toán proxemic cost!" << std::endl;
    }
    
    // Xuất kết quả ra file
    void exportResults(const std::string& filename) {
        std::cout << "🔄 Đang xuất kết quả ra file: " << filename << std::endl;
        
        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cerr << "❌ Không thể mở file: " << filename << std::endl;
            return;
        }
        
        // Xuất thông tin lưới
        file << "# Grid Information" << std::endl;
        file << "# x_min: " << grid_->x_min << std::endl;
        file << "# x_max: " << grid_->x_max << std::endl;
        file << "# y_min: " << grid_->y_min << std::endl;
        file << "# y_max: " << grid_->y_max << std::endl;
        file << "# nx: " << grid_->nx << std::endl;
        file << "# ny: " << grid_->ny << std::endl;
        file << "# dx: " << grid_->dx << std::endl;
        file << "# dy: " << grid_->dy << std::endl;
        file << std::endl;
        
        // Xuất dữ liệu proxemic cost
        file << "# Proxemic Cost Data (i, j, x, y, cost)" << std::endl;
        for (int i = 0; i < grid_->ny; i++) {
            for (int j = 0; j < grid_->nx; j++) {
                Point2D world_coords = grid_->getWorldCoords(i, j);
                file << i << " " << j << " " 
                     << std::fixed << std::setprecision(6)
                     << world_coords.x << " " << world_coords.y << " " 
                     << grid_->operator()(i, j) << std::endl;
            }
        }
        
        file.close();
        std::cout << "✅ Hoàn thành xuất kết quả!" << std::endl;
    }
    
    // Xuất quỹ đạo ra file
    void exportTrajectory(const std::string& filename) {
        std::cout << "🔄 Đang xuất quỹ đạo ra file: " << filename << std::endl;
        
        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cerr << "❌ Không thể mở file: " << filename << std::endl;
            return;
        }
        
        file << "# Trajectory Data (time, x, y, theta, vx, vy)" << std::endl;
        for (size_t i = 0; i < trajectory_.size(); i++) {
            file << std::fixed << std::setprecision(6)
                 << trajectory_.time[i] << " " 
                 << trajectory_.x[i] << " " << trajectory_.y[i] << " "
                 << trajectory_.theta[i] << " "
                 << trajectory_.velocity_x[i] << " " << trajectory_.velocity_y[i] << std::endl;
        }
        
        file.close();
        std::cout << "✅ Hoàn thành xuất quỹ đạo!" << std::endl;
    }
    
    // Tính thống kê
    void printStatistics() {
        std::cout << "\n📈 THỐNG KÊ:" << std::endl;
        
        // Tính tổng quãng đường
        double total_distance = 0.0;
        for (size_t i = 1; i < trajectory_.size(); i++) {
            double dx = trajectory_.x[i] - trajectory_.x[i-1];
            double dy = trajectory_.y[i] - trajectory_.y[i-1];
            total_distance += sqrt(dx * dx + dy * dy);
        }
        
        double total_rotation = trajectory_.theta.back() - trajectory_.theta.front();
        
        std::cout << "   Quãng đường: " << std::fixed << std::setprecision(2) << total_distance << " m" << std::endl;
        std::cout << "   Góc quay: " << std::fixed << std::setprecision(2) << total_rotation 
                  << " rad (" << std::fixed << std::setprecision(1) << (total_rotation * 180.0 / M_PI) << "°)" << std::endl;
        std::cout << "   Vị trí cuối: [" << std::fixed << std::setprecision(2) 
                  << trajectory_.x.back() << ", " << trajectory_.y.back() << "]" << std::endl;
        
        // Thống kê proxemic cost
        double max_cost = 0.0;
        double sum_cost = 0.0;
        int nonzero_points = 0;
        
        for (int i = 0; i < grid_->ny; i++) {
            for (int j = 0; j < grid_->nx; j++) {
                double cost = grid_->operator()(i, j);
                max_cost = std::max(max_cost, cost);
                sum_cost += cost;
                if (cost > 0) nonzero_points++;
            }
        }
        
        double mean_cost = sum_cost / (grid_->nx * grid_->ny);
        double ratio = 100.0 * nonzero_points / (grid_->nx * grid_->ny);
        
        std::cout << "\n📊 THỐNG KÊ PROXEMIC COST:" << std::endl;
        std::cout << "   Chi phí tối đa: " << std::fixed << std::setprecision(2) << max_cost << std::endl;
        std::cout << "   Chi phí trung bình: " << std::fixed << std::setprecision(2) << mean_cost << std::endl;
        std::cout << "   Số điểm có chi phí > 0: " << nonzero_points << std::endl;
        std::cout << "   Tỷ lệ điểm có chi phí: " << std::fixed << std::setprecision(1) << ratio << "%" << std::endl;
        
        if (params_.w != 0) {
            double radius_curvature = params_.v / fabs(params_.w);
            std::cout << "   Bán kính cong: " << std::fixed << std::setprecision(2) << radius_curvature << " m" << std::endl;
        }
    }
    
    // Chạy mô phỏng hoàn chỉnh
    void runSimulation() {
        std::cout << "🎯 MÔ PHỎNG PROXEMIC - VẬN TỐC GÓC NGƯỜI ĐI BỘ" << std::endl;
        std::cout << "==================================================" << std::endl;
        
        std::cout << "📊 Tham số: v=" << params_.v << " m/s, w=" << params_.w 
                  << " rad/s, t=" << params_.sim_time << " s" << std::endl;
        std::cout << "📊 Tham số Proxemic: amplitude=" << params_.amplitude 
                  << ", sigma=" << params_.sigma_base << ", cutoff=" << params_.cutoff << std::endl;
        
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // Thực hiện các bước mô phỏng
        calculateTrajectory();
        createGrid();
        calculateProxemicCost();
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        // Xuất kết quả
        exportTrajectory("trajectory_data.csv");
        exportResults("proxemic_cost_data.csv");
        
        // In thống kê
        printStatistics();
        
        std::cout << "\n⏱️  Thời gian thực thi: " << duration.count() << " ms" << std::endl;
        std::cout << "\n✅ Hoàn thành mô phỏng proxemic với vận tốc góc!" << std::endl;
    }
};

int main() {
    // Tạo tham số mô phỏng
    SimulationParams params;
    
    // Tạo và chạy mô phỏng
    MotionSimulation2D simulation(params);
    simulation.runSimulation();
    
    return 0;
} 