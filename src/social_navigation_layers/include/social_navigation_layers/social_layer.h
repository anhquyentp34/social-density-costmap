#ifndef SOCIAL_LAYER_H_ // Nếu chưa định nghĩa SOCIAL_LAYER_H_
#define SOCIAL_LAYER_H_ // Định nghĩa SOCIAL_LAYER_H_

#include <ros/ros.h> // Thư viện ROS
#include <costmap_2d/layer.h> // Thư viện lớp chi phí 2D
#include <costmap_2d/layered_costmap.h> // Thư viện bản đồ chi phí nhiều lớp
#include <people_msgs/People.h> // Thư viện thông điệp người
#include <social_msgs/SocialState.h> 
#include <social_msgs/SocialPeople.h>
#include <social_msgs/SocialPerson.h>
#include <boost/thread.hpp> // Thư viện luồng Boost
#include <list> // Thư viện danh sách

namespace social_navigation_layers // Tạo không gian tên cho các lớp điều hướng xã hội
{
  class SocialLayer : public costmap_2d::Layer // Định nghĩa lớp SocialLayer kế thừa từ lớp Layer của costmap_2d
  {
    public:
      SocialLayer()
      : people_keep_time_(0.75)
      , last_social_update_(0.0)
      , last_people_nonempty_(0.0)
      , last_groups_nonempty_(0.0)
      , last_interactions_nonempty_(0.0)
      , density_(0.0)
      {
        layered_costmap_ = NULL;
      } // Hàm khởi tạo, khởi tạo layered_costmap_ bằng NULL

      virtual void onInitialize(); // Hàm khởi tạo
      virtual void updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y, double* max_x, double* max_y); // Hàm cập nhật giới hạn
      virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) = 0; // Hàm cập nhật chi phí (thuần ảo)
      
      virtual void updateBoundsFromPeople(double* min_x, double* min_y, double* max_x, double* max_y) {}

      bool isDiscretized() { return false; } // Hàm kiểm tra xem có rời rạc hóa không, trả về false

    protected:
      void socialCallback(const social_msgs::SocialState& socialstate_msgs); // Hàm callback khi nhận thông tin người
      /** Xóa people/groups/interactions đã giữ quá people_keep_time_ (mất track). */
      void expireStaleSocialCaches();
      ros::Subscriber social_sub_; // Biến đăng ký nhận thông tin người
      
      social_msgs::SocialPeople social_people_list_; // Danh sách thông tin người
      social_msgs::SocialGroups social_groups_list_; // Danh sách thông tin nhóm người
      social_msgs::SocialInteractions social_interactions_list_; // Danh sách thông tin tương tác

      std::list<social_msgs::SocialPerson> transformed_people_;// Danh sách thông tin người đã biến đổi
      std::list<social_msgs::SocialGroup> transformed_group_; // Danh sách thông tin nhóm người đã biến đổi
      std::list<social_msgs::SocialInteraction> transformed_interactions_; // Danh sách thông tin tương tác đã biến đổi
      
      ros::Duration people_keep_time_; // Thời gian giữ thông tin khi mất track / mất message
      ros::Time last_social_update_; // Thời điểm nhận social state gần nhất
      ros::Time last_people_nonempty_; // Lần cuối còn ít nhất 1 người
      ros::Time last_groups_nonempty_; // Lần cuối còn ít nhất 1 nhóm
      ros::Time last_interactions_nonempty_; // Lần cuối còn ít nhất 1 tương tác
      boost::recursive_mutex lock_; // Khóa để đảm bảo an toàn khi truy cập đa luồng
      bool first_time_; // Biến kiểm tra lần đầu khởi tạo
      double last_min_x_, last_min_y_, last_max_x_, last_max_y_; // Các biến giữ giới hạn cuối cùng

      double density_; // Mật độ hiệu lực dùng để tác động lên layer
  };
};

#endif // Kết thúc định nghĩa SOCIAL_LAYER_H_
