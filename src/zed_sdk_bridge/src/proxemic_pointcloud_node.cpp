///////////////////////////////////////////////////////////////////////////
//
// Proxemic 3D PointCloud Node
// Tạo PointCloud2 biểu diễn vùng proxemic (social space) xung quanh người
// Sử dụng Gaussian 3D volume với intensity field
//
///////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Custom messages
#include "zed_sdk_bridge/Body.h"
#include "zed_sdk_bridge/BodiesStamped.h"

#include <cmath>
#include <vector>

class ProxemicPointCloudNode {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    
    // Subscriber
    ros::Subscriber bodies_sub_;
    
    // Publisher
    ros::Publisher cloud_pub_;
    
    // Parameters
    float sigma_body_;           // Sigma cho Gaussian của body
    float sigma_arm_;            // Sigma cho Gaussian của cánh tay
    float voxel_resolution_;     // Độ phân giải voxel (m)
    float volume_size_;          // Kích thước volume xung quanh body (m)
    float intensity_threshold_;  // Ngưỡng intensity để bỏ bớt điểm
    bool enable_arm_proxemic_;   // Bật proxemic cho cánh tay
    std::string frame_id_;
    
    // Gaussian 3D function
    float gaussian3D(float x, float y, float z, float cx, float cy, float cz, float sigma) {
        float dx = x - cx;
        float dy = y - cy;
        float dz = z - cz;
        float d2 = dx*dx + dy*dy + dz*dz;
        return std::exp(-d2 / (sigma * sigma));
    }
    
    // Tính toán intensity cho một điểm dựa trên tất cả bodies
    float computeIntensity(float x, float y, float z, const std::vector<zed_sdk_bridge::Body>& bodies) {
        float max_intensity = 0.0f;
        
        for (const auto& body : bodies) {
            // Body center position
            float cx = body.position[0];
            float cy = body.position[1];
            float cz = body.position[2];
            
            // Intensity từ body center
            float body_intensity = gaussian3D(x, y, z, cx, cy, cz, sigma_body_);
            max_intensity = std::max(max_intensity, body_intensity);
            
            // Intensity từ cánh tay (nếu bật)
            if (enable_arm_proxemic_ && body.num_keypoints >= 18) {
                // Left arm: keypoints 5, 7, 9 (shoulder, elbow, wrist)
                // Right arm: keypoints 6, 8, 10
                
                // Left arm
                if (body.keypoints_3d.size() >= 10 * 3) {
                    // Shoulder (5)
                    float lx = body.keypoints_3d[5 * 3];
                    float ly = body.keypoints_3d[5 * 3 + 1];
                    float lz = body.keypoints_3d[5 * 3 + 2];
                    if (lx != 0 || ly != 0 || lz != 0) {
                        float arm_intensity = gaussian3D(x, y, z, lx, ly, lz, sigma_arm_);
                        max_intensity = std::max(max_intensity, arm_intensity * 0.7f);
                    }
                    
                    // Elbow (7)
                    lx = body.keypoints_3d[7 * 3];
                    ly = body.keypoints_3d[7 * 3 + 1];
                    lz = body.keypoints_3d[7 * 3 + 2];
                    if (lx != 0 || ly != 0 || lz != 0) {
                        float arm_intensity = gaussian3D(x, y, z, lx, ly, lz, sigma_arm_);
                        max_intensity = std::max(max_intensity, arm_intensity * 0.6f);
                    }
                    
                    // Wrist (9)
                    lx = body.keypoints_3d[9 * 3];
                    ly = body.keypoints_3d[9 * 3 + 1];
                    lz = body.keypoints_3d[9 * 3 + 2];
                    if (lx != 0 || ly != 0 || lz != 0) {
                        float arm_intensity = gaussian3D(x, y, z, lx, ly, lz, sigma_arm_);
                        max_intensity = std::max(max_intensity, arm_intensity * 0.5f);
                    }
                }
                
                // Right arm
                if (body.keypoints_3d.size() >= 11 * 3) {
                    // Shoulder (6)
                    float rx = body.keypoints_3d[6 * 3];
                    float ry = body.keypoints_3d[6 * 3 + 1];
                    float rz = body.keypoints_3d[6 * 3 + 2];
                    if (rx != 0 || ry != 0 || rz != 0) {
                        float arm_intensity = gaussian3D(x, y, z, rx, ry, rz, sigma_arm_);
                        max_intensity = std::max(max_intensity, arm_intensity * 0.7f);
                    }
                    
                    // Elbow (8)
                    rx = body.keypoints_3d[8 * 3];
                    ry = body.keypoints_3d[8 * 3 + 1];
                    rz = body.keypoints_3d[8 * 3 + 2];
                    if (rx != 0 || ry != 0 || rz != 0) {
                        float arm_intensity = gaussian3D(x, y, z, rx, ry, rz, sigma_arm_);
                        max_intensity = std::max(max_intensity, arm_intensity * 0.6f);
                    }
                    
                    // Wrist (10)
                    rx = body.keypoints_3d[10 * 3];
                    ry = body.keypoints_3d[10 * 3 + 1];
                    rz = body.keypoints_3d[10 * 3 + 2];
                    if (rx != 0 || ry != 0 || rz != 0) {
                        float arm_intensity = gaussian3D(x, y, z, rx, ry, rz, sigma_arm_);
                        max_intensity = std::max(max_intensity, arm_intensity * 0.5f);
                    }
                }
            }
        }
        
        return max_intensity;
    }
    
    // Tạo bounding box xung quanh tất cả bodies
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
            float cx = body.position[0];
            float cy = body.position[1];
            float cz = body.position[2];
            
            min_x = std::min(min_x, cx - volume_size_);
            max_x = std::max(max_x, cx + volume_size_);
            min_y = std::min(min_y, cy - volume_size_);
            max_y = std::max(max_y, cy + volume_size_);
            min_z = std::min(min_z, cz - volume_size_);
            max_z = std::max(max_z, cz + volume_size_);
        }
    }
    
    void bodiesCallback(const zed_sdk_bridge::BodiesStampedConstPtr& msg) {
        if (msg->bodies.empty()) {
            // Publish empty cloud
            sensor_msgs::PointCloud2 empty_cloud;
            empty_cloud.header = msg->header;
            empty_cloud.width = 0;
            empty_cloud.height = 1;
            empty_cloud.is_dense = true;
            cloud_pub_.publish(empty_cloud);
            return;
        }
        
        // Tính bounding box
        float min_x, max_x, min_y, max_y, min_z, max_z;
        computeBoundingBox(msg->bodies, min_x, max_x, min_y, max_y, min_z, max_z);
        
        // Tạo lưới voxel
        int nx = static_cast<int>((max_x - min_x) / voxel_resolution_) + 1;
        int ny = static_cast<int>((max_y - min_y) / voxel_resolution_) + 1;
        int nz = static_cast<int>((max_z - min_z) / voxel_resolution_) + 1;
        
        // Giới hạn số điểm để tránh quá tải
        if (nx * ny * nz > 100000) {
            voxel_resolution_ = std::max(voxel_resolution_, 0.1f);
            nx = static_cast<int>((max_x - min_x) / voxel_resolution_) + 1;
            ny = static_cast<int>((max_y - min_y) / voxel_resolution_) + 1;
            nz = static_cast<int>((max_z - min_z) / voxel_resolution_) + 1;
        }
        
        // Tạo point cloud
        pcl::PointCloud<pcl::PointXYZI> cloud;
        cloud.reserve(nx * ny * nz);
        
        for (int ix = 0; ix < nx; ix++) {
            float x = min_x + ix * voxel_resolution_;
            for (int iy = 0; iy < ny; iy++) {
                float y = min_y + iy * voxel_resolution_;
                for (int iz = 0; iz < nz; iz++) {
                    float z = min_z + iz * voxel_resolution_;
                    
                    // Tính intensity
                    float intensity = computeIntensity(x, y, z, msg->bodies);
                    
                    // Chỉ thêm điểm nếu intensity > threshold
                    if (intensity > intensity_threshold_) {
                        pcl::PointXYZI point;
                        point.x = x;
                        point.y = y;
                        point.z = z;
                        point.intensity = intensity;
                        cloud.points.push_back(point);
                    }
                }
            }
        }
        
        // Convert to ROS message
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(cloud, cloud_msg);
        cloud_msg.header = msg->header;
        cloud_msg.header.frame_id = frame_id_.empty() ? msg->header.frame_id : frame_id_;
        
        cloud_pub_.publish(cloud_msg);
        
        ROS_DEBUG_THROTTLE(1.0, "Published proxemic cloud with %zu points", cloud.points.size());
    }
    
public:
    ProxemicPointCloudNode() : nh_private_("~") {
        // Get parameters
        nh_private_.param<float>("sigma_body", sigma_body_, 0.5f);
        nh_private_.param<float>("sigma_arm", sigma_arm_, 0.3f);
        nh_private_.param<float>("voxel_resolution", voxel_resolution_, 0.05f);
        nh_private_.param<float>("volume_size", volume_size_, 1.0f);
        nh_private_.param<float>("intensity_threshold", intensity_threshold_, 0.1f);
        nh_private_.param<bool>("enable_arm_proxemic", enable_arm_proxemic_, true);
        nh_private_.param<std::string>("frame_id", frame_id_, "");
        
        // Subscriber
        bodies_sub_ = nh_.subscribe("bodies", 1, &ProxemicPointCloudNode::bodiesCallback, this);
        
        // Publisher
        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("proxemic_cloud", 1);
        
        ROS_INFO("Proxemic PointCloud Node initialized");
        ROS_INFO("  - Sigma body: %.2f m", sigma_body_);
        ROS_INFO("  - Sigma arm: %.2f m", sigma_arm_);
        ROS_INFO("  - Voxel resolution: %.3f m", voxel_resolution_);
        ROS_INFO("  - Volume size: %.2f m", volume_size_);
        ROS_INFO("  - Intensity threshold: %.2f", intensity_threshold_);
        ROS_INFO("  - Arm proxemic: %s", enable_arm_proxemic_ ? "enabled" : "disabled");
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "proxemic_pointcloud_node");
    
    try {
        ProxemicPointCloudNode node;
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
        return -1;
    }
    
    return 0;
}

