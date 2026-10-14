                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       #include <social_navigation_layers/emotion_layer.h>
#include <math.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <algorithm>
#include <string>
#include <dynamic_reconfigure/server.h>
#include <social_navigation_layers/EmotionLayerConfig.h>
#include <cmath>

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace social_navigation_layers
{
    namespace
    {
        double computeDensityGeometryScale(double density, double density_weight)
        {
            const double safe_density = std::max(0.0, density);
            return std::exp(-density_weight * safe_density);
        }   
    }  // namespace

    namespace emotion_layer_internal // Namespace nội bộ cho các hàm tiện ích
    {
        // Utility functions
        double gaussian(double x, double y, double x0, double y0, double A, double varx, double vary, double skew)
        {
            double dx = x - x0, dy = y - y0;
            double h = sqrt(dx * dx + dy * dy);
            double angle = atan2(dy, dx);
            double mx = cos(angle - skew) * h;
            double my = sin(angle - skew) * h;
            double f1 = pow(mx, 2.0) / (2.0 * varx),
                   f2 = pow(my, 2.0) / (2.0 * vary);
            return A * exp(-(f1 + f2));
        }

        // Hàm tính bán kính dựa trên cảm xúc
        double get_radius_by_emotion(const std::string& emotion)
        {
            // In ra thông tin debug về cảm xúc nhận được
            // ROS_DEBUG("EmotionLayer: Processing emotion: '%s'", emotion.c_str());
            
            // Bán kính cố định cho từng loại cảm xúc
            if (emotion == "happy" || emotion == "Happy")
                return 1.3;  // 1.3m cho người vui vẻ
            else if (emotion == "angry" || emotion == "Angry")
                return 1.35;  // 1.35m cho người tức giận (was 2.0)
            else if (emotion == "neutral" || emotion == "Neutral")
                return 0.0; // 1.65m cho người trung tính
            else if (emotion.empty()) {
                ROS_WARN_ONCE("EmotionLayer: Empty emotion field detected, using default neutral radius (1.65m)");
                return 0.0; // 1.65m cho trường hợp không có thông tin cảm xúc
            } else {
                // Nếu không phải happy/angry/neutral hoặc trường hợp khác, sử dụng neutral
                // ROS_DEBUG("EmotionLayer: Using neutral radius for unknown emotion: '%s'", emotion.c_str());
                return 1.35; // 1.35m cho trường hợp không xác định (was 1.65)
            }
        }
    } // namespace emotion_layer_internal

    // Implementation of EmotionLayer methods
    void EmotionLayer::onInitialize()
    {
        SocialLayer::onInitialize();
        ros::NodeHandle nh("~/" + name_), g_nh;
        server_ = new dynamic_reconfigure::Server<social_navigation_layers::EmotionLayerConfig>(nh);
        f_ = boost::bind(&EmotionLayer::configure, this, _1, _2);
        server_->setCallback(f_);
        
        // enabled_ = true;
        // cutoff_ = 200.0;
        // amplitude_ = 254.0;
        // covar_ = 25.0;
        // people_keep_time_ = ros::Duration(0.75);
        // ROS_INFO("EmotionLayer: Initialized with dynamic radius calculation");
    }

    void EmotionLayer::updateBoundsFromPeople(double* min_x, double* min_y, double* max_x, double* max_y)
    {
        std::list<social_msgs::SocialPerson>::iterator p_it;
        // ROS_INFO("EmotionLayer: updateBoundsFromPeople with %zu people", transformed_people_.size());

        for (p_it = transformed_people_.begin(); p_it != transformed_people_.end(); ++p_it)
        {
            social_msgs::SocialPerson person = *p_it;
            
            // Lấy bán kính giới hạn dựa trên cảm xúc
            double radius = emotion_layer_internal::get_radius_by_emotion(person.emotion);
            
            // ROS_INFO("EmotionLayer: Setting bounds for %s with emotion %s, radius %.2f at (%.2f, %.2f)", 
                    // person.name.c_str(), person.emotion.c_str(), radius,
                    // person.position.position.x, person.position.position.y);

            *min_x = std::min(*min_x, person.position.position.x - radius);
            *min_y = std::min(*min_y, person.position.position.y - radius);
            *max_x = std::max(*max_x, person.position.position.x + radius);
            *max_y = std::max(*max_y, person.position.position.y + radius);
        }
        
        // ROS_INFO("EmotionLayer: Final bounds min_x: %.2f, min_y: %.2f, max_x: %.2f, max_y: %.2f", 
                // *min_x, *min_y, *max_x, *max_y);
    }

    void EmotionLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
    {
        boost::recursive_mutex::scoped_lock lock(lock_);
        if (!enabled_) {
            ROS_WARN("EmotionLayer: Layer is disabled!");
            return;
        }
        
        if (social_people_list_.people.size() == 0) {
            // Bình thường khi ZED mất track; annotation dùng /social_state_gt relay.
            ROS_WARN_THROTTLE(5.0, "EmotionLayer: No people in social_people_list_!");
            return;
        }
        
        if (cutoff_ >= amplitude_) {
            ROS_WARN("EmotionLayer: Invalid parameters: cutoff_ >= amplitude_");
            return;
        }
        
        if (transformed_people_.size() == 0) {
            ROS_WARN("EmotionLayer: No people in transformed_people_!");
            return;
        }
            
        // ROS_INFO("EmotionLayer: updateCosts with %zu people", transformed_people_.size());
        
        std::list<social_msgs::SocialPerson>::iterator p_it;
        costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
        double res = costmap->getResolution();
        
        for (p_it = transformed_people_.begin(); p_it != transformed_people_.end(); ++p_it)
        {
            social_msgs::SocialPerson person = *p_it;
            
            // 1. Lấy bán kính giới hạn dựa trên cảm xúc
            const double density_scale = computeDensityGeometryScale(density_, density_weight_);
            double radius = emotion_layer_internal::get_radius_by_emotion(person.emotion) * density_scale;
            if (radius <= 0.0)
                continue;
            
            // 2. Tính covar_ động dựa trên radius
            // Sử dụng công thức: covar = (r^2) / (-2 * ln(cutoff/amplitude))
            // trong đó r là radius từ cảm xúc
            double dynamic_covar = (radius * radius) / (-2 * log(cutoff_/amplitude_));
            
            unsigned int width = std::max(1, static_cast<int>((radius * 2) / res)),
                         height = std::max(1, static_cast<int>((radius * 2) / res));
                         
            double cx = person.position.position.x, cy = person.position.position.y;

            double ox = cx - radius;
            double oy = cy - radius;

            int dx, dy;
            costmap->worldToMapNoBounds(ox, oy, dx, dy);

            int start_x = 0, start_y = 0, end_x = width, end_y = height;
            if (dx < 0)
                start_x = -dx;
            else if (dx + width > costmap->getSizeInCellsX())
                end_x = std::max(0, static_cast<int>(costmap->getSizeInCellsX()) - dx);

            if (static_cast<int>(start_x + dx) < min_i)
                start_x = min_i - dx;
            if (static_cast<int>(end_x + dx) > max_i)
                end_x = max_i - dx;

            if (dy < 0)
                start_y = -dy;
            else if (dy + height > costmap->getSizeInCellsY())
                end_y = std::max(0, static_cast<int>(costmap->getSizeInCellsY()) - dy);

            if (static_cast<int>(start_y + dy) < min_j)
                start_y = min_j - dy;
            if (static_cast<int>(end_y + dy) > max_j)
                end_y = max_j - dy;

            double bx = ox + res / 2,
                   by = oy + res / 2;
                   
            int cells_updated = 0;
            for (int i = start_x; i < end_x; i++)
            {
                for (int j = start_y; j < end_y; j++)
                {
                    unsigned char old_cost = costmap->getCost(i + dx, j + dy);
                    if (old_cost == costmap_2d::NO_INFORMATION)
                        continue;

                    double x = bx + i * res, y = by + j * res;
                    double dist = sqrt(pow(x - cx, 2) + pow(y - cy, 2));
                    
                    // Kiểm tra xem điểm có nằm trong bán kính giới hạn của cảm xúc không
                    if (dist > radius)
                        continue;
                        
                    // 3. Tính giá trị Gaussian với covar_ động
                    double base_cost = amplitude_ * exp(-(dist * dist) / (2 * dynamic_covar));

                    if (base_cost < cutoff_)
                        continue;
                        
                    unsigned char cvalue = (unsigned char)base_cost;
                    costmap->setCost(i + dx, j + dy, std::max(cvalue, old_cost));
                    cells_updated++;
                }
            }
            // ROS_INFO("EmotionLayer: Updated %d cells for person %s", cells_updated, person.name.c_str());
        }
    }

    void EmotionLayer::configure(social_navigation_layers::EmotionLayerConfig &config, uint32_t level)
    {
        enabled_ = config.enabled;
        cutoff_ = config.cutoff;
        amplitude_ = config.amplitude;
        covar_ = config.covariance;
        density_weight_ = config.emotion_density_weight;
        people_keep_time_ = ros::Duration(config.keep_time);
        // ROS_INFO("EmotionLayer: Reconfigured with enabled=%d, cutoff=%.2f, amplitude=%.2f, covariance=%.2f", 
                // enabled_, cutoff_, amplitude_, covar_);
    }
}

PLUGINLIB_EXPORT_CLASS(social_navigation_layers::EmotionLayer, costmap_2d::Layer)
