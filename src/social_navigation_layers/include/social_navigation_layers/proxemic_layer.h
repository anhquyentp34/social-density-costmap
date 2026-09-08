#ifndef PROXEMIC_LAYER_H_ // Nếu chưa định nghĩa PROXEMIC_LAYER_H_
#define PROXEMIC_LAYER_H_ // Định nghĩa PROXEMIC_LAYER_H_

#include <ros/ros.h> // Thư viện ROS
#include <social_navigation_layers/social_layer.h> // Thư viện lớp social_layer
#include <dynamic_reconfigure/server.h> // Thư viện server cấu hình động
#include <social_navigation_layers/ProxemicLayerConfig.h> // Thư viện cấu hình lớp ProxemicLayer
#include <visualization_msgs/Marker.h> // Thư viện thông điệp đánh dấu
#include <visualization_msgs/MarkerArray.h> // Thư viện mảng thông điệp đánh dấu

double gaussian(double x, double y, double x0, double y0, double A, double varx, double vary, double skew); // Hàm tính Gaussian
double get_radius(double cutoff, double A, double var); // Hàm tính bán kính

namespace social_navigation_layers // Không gian tên social_navigation_layers
{
  class ProxemicLayer : public SocialLayer // Định nghĩa lớp ProxemicLayer kế thừa từ lớp SocialLayer
  {
    public:
      ProxemicLayer() { layered_costmap_ = NULL; } // Hàm khởi tạo, khởi tạo layered_costmap_ bằng NULL

      virtual void onInitialize(); // Hàm khởi tạo
      virtual void updateBoundsFromPeople(double* min_x, double* min_y, double* max_x, double* max_y); // Hàm cập nhật giới hạn từ thông tin người
      virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j); // Hàm cập nhật chi phí

    protected:
      void configure(ProxemicLayerConfig &config, uint32_t level); // Hàm cấu hình lớp ProxemicLayer
      double cutoff_, amplitude_, covar_, factor_, min_radius_, density_weight_; // Các biến cấu hình
      dynamic_reconfigure::Server<ProxemicLayerConfig>* server_; // Server cấu hình động
      dynamic_reconfigure::Server<ProxemicLayerConfig>::CallbackType f_; // Kiểu callback cho server cấu hình động
      double calculateDensityAdjustedCost(double base_cost);
      
  };
};

#endif // Kết thúc định nghĩa PROXEMIC_LAYER_H_
