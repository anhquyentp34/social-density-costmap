///////////////////////////////////////////////////////////////////////////
//
// Skeleton 3D Costmap Node
// Tạo voxel costmap 3D từ skeleton data (tương tự costmap 2D trong ROS navigation)
// Mỗi voxel có cost value (0-255) dựa trên khoảng cách đến skeleton
// Có thể chọn chỉ tạo costmap quanh bones (không tính keypoints) hoặc cả hai
//
///////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Point.h>

// Custom messages
#include "zed_sdk_bridge/Body.h"
#include "zed_sdk_bridge/BodiesStamped.h"

#include <cmath>
#include <vector>
#include <algorithm>
#include <limits>
#include <sstream>
#include <set>

class SkeletonCostmap3DNode {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    
    // Subscriber
    ros::Subscriber bodies_sub_;
    
    // Publishers
    ros::Publisher costmap_pub_;  // Publish as PointCloud2 với intensity = cost
    ros::Publisher costmap_info_pub_;  // Publish metadata
    
    // Parameters
    float voxel_resolution_;        // Độ phân giải voxel (m) - giống costmap resolution
    float inflation_radius_;        // Bán kính inflation (m) - giống costmap inflation
    float lethal_cost_threshold_;   // Ngưỡng cost để coi là lethal (giống costmap)
    float inscribed_cost_;          // Cost cho inscribed radius
    float cost_scaling_factor_;     // Hệ số scaling cho cost
    float max_cost_;                // Cost tối đa (255)
    float min_cost_;                // Cost tối thiểu (0)
    float map_size_x_;              // Kích thước map theo X (m)
    float map_size_y_;              // Kích thước map theo Y (m)
    float map_size_z_;              // Kích thước map theo Z (m)
    float map_origin_x_;            // Origin X (m)
    float map_origin_y_;            // Origin Y (m)
    float map_origin_z_;            // Origin Z (m)
    bool use_body_volume_;          // Sử dụng toàn bộ body volume hay chỉ keypoints
    bool use_bones_only_;           // Chỉ sử dụng bones, không tính keypoints
    std::string selected_bones_;    // Danh sách bones được chọn (format: "idx1,idx2;idx3,idx4" hoặc rỗng = tất cả)
    std::set<std::pair<int, int>> selected_bones_set_;  // Set các bones được chọn để filter
    float body_radius_;             // Bán kính body (m) - cho body volume
    float joint_radius_;            // Bán kính joint (m) - để costmap bám sát joints
    float bone_radius_;             // Bán kính bone (m) - để costmap bám sát bones
    std::string frame_id_;
    
    // Parse selected_bones string thành set
    void parseSelectedBones(const std::string& bones_str) {
        selected_bones_set_.clear();
        if (bones_str.empty()) {
            return;  // Rỗng = chọn tất cả
        }
        
        std::istringstream iss(bones_str);
        std::string pair_str;
        
        while (std::getline(iss, pair_str, ';')) {
            // Parse từng cặp "idx1,idx2"
            size_t comma_pos = pair_str.find(',');
            if (comma_pos != std::string::npos) {
                try {
                    int idx1 = std::stoi(pair_str.substr(0, comma_pos));
                    int idx2 = std::stoi(pair_str.substr(comma_pos + 1));
                    selected_bones_set_.insert({idx1, idx2});
                    // Thêm cả chiều ngược lại để dễ so sánh
                    selected_bones_set_.insert({idx2, idx1});
                } catch (...) {
                    ROS_WARN("Invalid bone pair format: %s", pair_str.c_str());
                }
            }
        }
    }
    
    // Kiểm tra bone có được chọn không
    bool isBoneSelected(int idx1, int idx2) {
        if (selected_bones_set_.empty()) {
            return true;  // Không filter = chọn tất cả
        }
        return selected_bones_set_.count({idx1, idx2}) > 0 || 
               selected_bones_set_.count({idx2, idx1}) > 0;
    }
    
    // Tính cost dựa trên khoảng cách
    // Cost = 0 (free) -> 255 (lethal) dựa trên khoảng cách
    // Sử dụng exponential function để cost cao hơn ở gần skeleton
    uint8_t computeCost(float distance) {
        if (distance < 0.0f) {
            return static_cast<uint8_t>(max_cost_);  // Lethal
        }
        
        if (distance > inflation_radius_) {
            return static_cast<uint8_t>(min_cost_);  // Free
        }
        
        // Exponential cost function để cost cao hơn ở gần skeleton
        // normalized_dist = 0 (trên skeleton) -> 1 (xa skeleton)
        float normalized_dist = distance / inflation_radius_;
        
        // Exponential decay: cost cao ở gần, giảm nhanh khi xa
        // exp(-3 * normalized_dist) cho curve tốt hơn
        float exp_factor = std::exp(-3.0f * normalized_dist);
        float cost = max_cost_ * exp_factor;
        
        // Đảm bảo cost tối thiểu ở gần skeleton
        if (distance < joint_radius_ || distance < bone_radius_) {
            cost = max_cost_;  // Lethal ở ngay trên joints/bones
        }
        
        // Apply cost scaling
        cost = cost * cost_scaling_factor_;
        
        // Clamp to [0, 255]
        cost = std::max(min_cost_, std::min(max_cost_, cost));
        
        return static_cast<uint8_t>(cost);
    }
    
    // Tính khoảng cách từ điểm đến skeleton (keypoints + bones hoặc chỉ bones)
    float distanceToSkeleton(float x, float y, float z, const zed_sdk_bridge::Body& body) {
        float min_dist = std::numeric_limits<float>::max();
        
        if (use_body_volume_) {
            // Tính khoảng cách đến body center (sphere)
            float dx = x - body.position[0];
            float dy = y - body.position[1];
            float dz = z - body.position[2];
            float dist_to_center = std::sqrt(dx*dx + dy*dy + dz*dz);
            float dist_to_surface = dist_to_center - body_radius_;
            min_dist = std::min(min_dist, dist_to_surface);
        } else {
            // Tính khoảng cách đến keypoints (chỉ khi không dùng bones_only)
            if (!use_bones_only_) {
                for (int i = 0; i < body.num_keypoints; i++) {
                    if (i * 3 + 2 >= body.keypoints_3d.size()) continue;
                    
                    float kx = body.keypoints_3d[i * 3];
                    float ky = body.keypoints_3d[i * 3 + 1];
                    float kz = body.keypoints_3d[i * 3 + 2];
                    
                    // Kiểm tra keypoint hợp lệ
                    if (std::abs(kx) < 0.0001f && std::abs(ky) < 0.0001f && std::abs(kz) < 0.0001f) {
                        continue;  // Keypoint không được detect
                    }
                    
                    float dx = x - kx;
                    float dy = y - ky;
                    float dz = z - kz;
                    float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
                    dist = std::max(0.0f, dist - joint_radius_);  // Trừ joint radius để costmap bám sát joints
                    min_dist = std::min(min_dist, dist);
                }
            }
            
            // Tính khoảng cách đến bones (line segments) - LUÔN tính
            std::vector<std::pair<int, int>> bones;
            
            if (body.body_format == 18) {
                // BODY_18 bones
                bones = {
                    {0, 1}, {1, 2}, {2, 3}, {3, 4}, {1, 5}, {5, 6}, {6, 7},
                    {8, 9}, {9, 10}, {11, 12}, {12, 13}, {2, 5}, {8, 11},
                    {0, 14}, {14, 16}, {0, 15}, {15, 17}, {2, 8}, {5, 11}
                };
            } else if (body.body_format == 34) {
                // BODY_34 bones
                bones = {
                    // Spine
                    {0, 1}, {1, 2}, {2, 3}, {3, 26}, {26, 27}, {27, 28}, {28, 29}, {27, 30}, {30, 31},
                    // Left arm
                    {2, 4}, {4, 5}, {5, 6}, {6, 7}, {7, 8}, {8, 9}, {7, 10},
                    // Right arm
                    {2, 11}, {11, 12}, {12, 13}, {13, 14}, {14, 15}, {15, 16}, {14, 17},
                    // Left leg
                    {0, 18}, {18, 19}, {19, 20}, {20, 21}, {20, 32},
                    // Right leg
                    {0, 22}, {22, 23}, {23, 24}, {24, 25}, {24, 33}
                };
            }
            
            for (const auto& bone : bones) {
                int idx1 = bone.first;
                int idx2 = bone.second;
                
                // Kiểm tra bone có được chọn không
                if (!isBoneSelected(idx1, idx2)) {
                    continue;  // Bỏ qua bone không được chọn
                }
                
                if (idx1 * 3 + 2 >= body.keypoints_3d.size() || 
                    idx2 * 3 + 2 >= body.keypoints_3d.size()) {
                    continue;
                }
                
                float x1 = body.keypoints_3d[idx1 * 3];
                float y1 = body.keypoints_3d[idx1 * 3 + 1];
                float z1 = body.keypoints_3d[idx1 * 3 + 2];
                float x2 = body.keypoints_3d[idx2 * 3];
                float y2 = body.keypoints_3d[idx2 * 3 + 1];
                float z2 = body.keypoints_3d[idx2 * 3 + 2];
                
                // Kiểm tra keypoints hợp lệ
                if ((std::abs(x1) < 0.0001f && std::abs(y1) < 0.0001f && std::abs(z1) < 0.0001f) ||
                    (std::abs(x2) < 0.0001f && std::abs(y2) < 0.0001f && std::abs(z2) < 0.0001f)) {
                    continue;
                }
                
                // Khoảng cách từ điểm đến line segment (trừ bone_radius để bám sát hơn)
                float dist = distanceToLineSegment(x, y, z, x1, y1, z1, x2, y2, z2);
                dist = std::max(0.0f, dist - bone_radius_);  // Trừ bone radius để costmap bám sát bones
                min_dist = std::min(min_dist, dist);
            }
        }
        
        return min_dist;
    }
    
    // Khoảng cách từ điểm đến line segment
    float distanceToLineSegment(float px, float py, float pz,
                                float x1, float y1, float z1,
                                float x2, float y2, float z2) {
        float dx = x2 - x1;
        float dy = y2 - y1;
        float dz = z2 - z1;
        float len2 = dx*dx + dy*dy + dz*dz;
        
        if (len2 < 0.0001f) {
            // Line segment là một điểm
            float ddx = px - x1;
            float ddy = py - y1;
            float ddz = pz - z1;
            return std::sqrt(ddx*ddx + ddy*ddy + ddz*ddz);
        }
        
        float t = ((px - x1) * dx + (py - y1) * dy + (pz - z1) * dz) / len2;
        t = std::max(0.0f, std::min(1.0f, t));  // Clamp to [0, 1]
        
        float proj_x = x1 + t * dx;
        float proj_y = y1 + t * dy;
        float proj_z = z1 + t * dz;
        
        float ddx = px - proj_x;
        float ddy = py - proj_y;
        float ddz = pz - proj_z;
        
        return std::sqrt(ddx*ddx + ddy*ddy + ddz*ddz);
    }
    
    // Tính bounding box từ tất cả bodies
    void computeBoundingBox(const std::vector<zed_sdk_bridge::Body>& bodies,
                           float& min_x, float& max_x,
                           float& min_y, float& max_y,
                           float& min_z, float& max_z) {
        if (bodies.empty()) {
            min_x = max_x = min_y = max_y = min_z = max_z = 0.0f;
            return;
        }
        
        min_x = max_x = bodies[0].position[0];
        min_y = max_y = bodies[0].position[1];
        min_z = max_z = bodies[0].position[2];
        
        for (const auto& body : bodies) {
            // Expand với inflation radius
            min_x = std::min(min_x, body.position[0] - inflation_radius_);
            max_x = std::max(max_x, body.position[0] + inflation_radius_);
            min_y = std::min(min_y, body.position[1] - inflation_radius_);
            max_y = std::max(max_y, body.position[1] + inflation_radius_);
            min_z = std::min(min_z, body.position[2] - inflation_radius_);
            max_z = std::max(max_z, body.position[2] + inflation_radius_);
        }
        
        // Expand thêm một chút để đảm bảo đủ không gian
        float margin = 0.5f;
        min_x -= margin;
        max_x += margin;
        min_y -= margin;
        max_y += margin;
        min_z -= margin;
        max_z += margin;
    }
    
    void bodiesCallback(const zed_sdk_bridge::BodiesStampedConstPtr& msg) {
        if (msg->bodies.empty()) {
            // Publish empty costmap
            sensor_msgs::PointCloud2 empty_cloud;
            empty_cloud.header = msg->header;
            empty_cloud.width = 0;
            empty_cloud.height = 1;
            empty_cloud.is_dense = true;
            costmap_pub_.publish(empty_cloud);
            return;
        }
        
        // Tính bounding box
        float min_x, max_x, min_y, max_y, min_z, max_z;
        computeBoundingBox(msg->bodies, min_x, max_x, min_y, max_y, min_z, max_z);
        
        // Sử dụng map size nếu được set, nếu không dùng bounding box
        float map_min_x = (map_size_x_ > 0) ? map_origin_x_ : min_x;
        float map_max_x = (map_size_x_ > 0) ? (map_origin_x_ + map_size_x_) : max_x;
        float map_min_y = (map_size_y_ > 0) ? map_origin_y_ : min_y;
        float map_max_y = (map_size_y_ > 0) ? (map_origin_y_ + map_size_y_) : max_y;
        float map_min_z = (map_size_z_ > 0) ? map_origin_z_ : min_z;
        float map_max_z = (map_size_z_ > 0) ? (map_origin_z_ + map_size_z_) : max_z;
        
        // Tạo lưới voxel
        int nx = static_cast<int>((map_max_x - map_min_x) / voxel_resolution_) + 1;
        int ny = static_cast<int>((map_max_y - map_min_y) / voxel_resolution_) + 1;
        int nz = static_cast<int>((map_max_z - map_min_z) / voxel_resolution_) + 1;
        
        // Giới hạn số điểm để tránh quá tải
        const int MAX_VOXELS = 500000;
        if (nx * ny * nz > MAX_VOXELS) {
            ROS_WARN_THROTTLE(1.0, "Too many voxels (%d), increasing resolution", nx * ny * nz);
            voxel_resolution_ = std::max(voxel_resolution_, 0.1f);
            nx = static_cast<int>((map_max_x - map_min_x) / voxel_resolution_) + 1;
            ny = static_cast<int>((map_max_y - map_min_y) / voxel_resolution_) + 1;
            nz = static_cast<int>((map_max_z - map_min_z) / voxel_resolution_) + 1;
        }
        
        // Tạo point cloud với intensity = cost
        pcl::PointCloud<pcl::PointXYZI> cloud;
        cloud.reserve(nx * ny * nz);
        
        int lethal_count = 0;
        int free_count = 0;
        int inflated_count = 0;
        
        for (int ix = 0; ix < nx; ix++) {
            float x = map_min_x + ix * voxel_resolution_;
            for (int iy = 0; iy < ny; iy++) {
                float y = map_min_y + iy * voxel_resolution_;
                for (int iz = 0; iz < nz; iz++) {
                    float z = map_min_z + iz * voxel_resolution_;
                    
                    // Tính khoảng cách tối thiểu đến skeleton
                    float min_dist = std::numeric_limits<float>::max();
                    for (const auto& body : msg->bodies) {
                        float dist = distanceToSkeleton(x, y, z, body);
                        min_dist = std::min(min_dist, dist);
                    }
                    
                    // Tính cost
                    uint8_t cost = computeCost(min_dist);
                    
                    // Chỉ thêm voxel nếu có cost (không phải free space hoàn toàn)
                    // Hoặc có thể thêm tất cả để có full costmap
                    if (cost > min_cost_ || true) {  // Publish tất cả để có full costmap
                        pcl::PointXYZI point;
                        point.x = x;
                        point.y = y;
                        point.z = z;
                        point.intensity = static_cast<float>(cost);
                        cloud.points.push_back(point);
                        
                        if (cost >= lethal_cost_threshold_) {
                            lethal_count++;
                        } else if (cost == min_cost_) {
                            free_count++;
                        } else {
                            inflated_count++;
                        }
                    }
                }
            }
        }
        
        // Convert to ROS message
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(cloud, cloud_msg);
        cloud_msg.header = msg->header;
        cloud_msg.header.frame_id = frame_id_.empty() ? msg->header.frame_id : frame_id_;
        
        costmap_pub_.publish(cloud_msg);
        
        ROS_INFO_THROTTLE(2.0, "Published 3D costmap: %zu voxels (lethal: %d, inflated: %d, free: %d)",
                         cloud.points.size(), lethal_count, inflated_count, free_count);
    }
    
public:
    SkeletonCostmap3DNode() : nh_private_("~") {
        // Get parameters
        nh_private_.param<float>("voxel_resolution", voxel_resolution_, 0.05f);  // 5cm - giống costmap resolution
        nh_private_.param<float>("inflation_radius", inflation_radius_, 0.5f);   // 50cm - giống costmap inflation
        nh_private_.param<float>("lethal_cost_threshold", lethal_cost_threshold_, 253.0f);  // Giống costmap
        nh_private_.param<float>("inscribed_cost", inscribed_cost_, 128.0f);
        nh_private_.param<float>("cost_scaling_factor", cost_scaling_factor_, 1.0f);
        nh_private_.param<float>("max_cost", max_cost_, 255.0f);
        nh_private_.param<float>("min_cost", min_cost_, 0.0f);
        nh_private_.param<float>("map_size_x", map_size_x_, 0.0f);  // 0 = auto (dùng bounding box)
        nh_private_.param<float>("map_size_y", map_size_y_, 0.0f);
        nh_private_.param<float>("map_size_z", map_size_z_, 0.0f);
        nh_private_.param<float>("map_origin_x", map_origin_x_, 0.0f);
        nh_private_.param<float>("map_origin_y", map_origin_y_, 0.0f);
        nh_private_.param<float>("map_origin_z", map_origin_z_, 0.0f);
        nh_private_.param<bool>("use_body_volume", use_body_volume_, false);  // false = dùng keypoints
        nh_private_.param<bool>("use_bones_only", use_bones_only_, true);  // true = chỉ dùng bones, không tính keypoints
        nh_private_.param<std::string>("selected_bones", selected_bones_, "");  // Format: "idx1,idx2;idx3,idx4" hoặc rỗng = tất cả
        nh_private_.param<float>("body_radius", body_radius_, 0.3f);  // 30cm
        nh_private_.param<float>("joint_radius", joint_radius_, 0.02f);  // 2cm - bán kính joint
        nh_private_.param<float>("bone_radius", bone_radius_, 0.015f);  // 1.5cm - bán kính bone
        nh_private_.param<std::string>("frame_id", frame_id_, "");
        
        // Parse selected bones
        parseSelectedBones(selected_bones_);
        
        // Subscriber
        bodies_sub_ = nh_.subscribe("bodies", 1, &SkeletonCostmap3DNode::bodiesCallback, this);
        
        // Publishers
        costmap_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("costmap_3d", 1);
        
        ROS_INFO("Skeleton 3D Costmap Node initialized");
        ROS_INFO("  - Voxel resolution: %.3f m", voxel_resolution_);
        ROS_INFO("  - Inflation radius: %.3f m", inflation_radius_);
        ROS_INFO("  - Lethal cost threshold: %.0f", lethal_cost_threshold_);
        ROS_INFO("  - Joint radius: %.3f m (để bám sát joints)", joint_radius_);
        ROS_INFO("  - Bone radius: %.3f m (để bám sát bones)", bone_radius_);
        ROS_INFO("  - Use body volume: %s", use_body_volume_ ? "true" : "false");
        ROS_INFO("  - Use bones only: %s (chỉ tính costmap quanh bones, không tính keypoints)", use_bones_only_ ? "true" : "false");
        if (!selected_bones_.empty()) {
            ROS_INFO("  - Selected bones: %s (chỉ tạo costmap cho bones được chọn)", selected_bones_.c_str());
        } else {
            ROS_INFO("  - Selected bones: all (tạo costmap cho tất cả bones)");
        }
        ROS_INFO("  - Frame ID: %s", frame_id_.empty() ? "auto" : frame_id_.c_str());
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "skeleton_costmap_3d_node");
    
    try {
        SkeletonCostmap3DNode node;
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
        return -1;
    }
    
    return 0;
}

