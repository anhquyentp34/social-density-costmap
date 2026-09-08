///////////////////////////////////////////////////////////////////////////
//
// Skeleton Visualizer Node
// Subscribe bodies và visualize skeleton trên ảnh và publish markers cho RViz
//
///////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Header.h>
#include <cmath>
#include <limits>
#include <sstream>
#include <iomanip>
#include <map>
#include <set>
#include <string>

// Custom messages
#include "zed_sdk_bridge/Body.h"
#include "zed_sdk_bridge/BodiesStamped.h"
#include "zed_sdk_bridge/Object.h"
#include "zed_sdk_bridge/ObjectsStamped.h"
#include <social_msgs/SocialPeople.h>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

class SkeletonVisualizer {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    
    // Subscribers
    ros::Subscriber bodies_sub_;
    ros::Subscriber objects_sub_;
    ros::Subscriber emotions_sub_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    
    // Publishers
    image_transport::Publisher image_pub_;
    ros::Publisher marker_pub_;
    
    // Latest data
    sensor_msgs::ImageConstPtr latest_image_;
    zed_sdk_bridge::BodiesStampedConstPtr latest_bodies_;
    zed_sdk_bridge::ObjectsStampedConstPtr latest_objects_;
    
    // Parameters
    bool show_on_image_;
    bool publish_markers_;
    bool show_objects_;
    bool show_skeleton_overlay_;  // false: chỉ box + nhãn (Track D)
    float joint_radius_;
    float link_width_;
    float joint_radius_2d_;
    float link_width_2d_;
    float velocity_smoothing_;
    float velocity_min_display_;
    float marker_lifetime_sec_;
    float object_marker_size_;
    float image_display_scale_;
    float max_image_hz_;
    float max_marker_hz_;
    float label_font_scale_;
    int label_font_thickness_;
    bool use_latest_tf_;
    ros::Time last_image_pub_;
    ros::Time last_marker_pub_;
    std::string camera_frame_id_;

    struct SmoothedVelocity {
        float vx = 0.0f;
        float vy = 0.0f;
        float vz = 0.0f;
        float speed = 0.0f;
        bool initialized = false;
    };
    std::map<int, SmoothedVelocity> smoothed_velocity_;
    std::map<int, std::string> last_emotion_;
    std::set<int> last_body_ids_;
    std::set<int> last_object_ids_;
    int max_joint_marker_id_ = 0;
    int max_bone_marker_id_ = 0;
    
    // Color palette từ ZED SDK (8 màu đẹp, dễ phân biệt)
    // Khai báo ngoài class để tránh lỗi C++11
    static const float id_colors[8][3];
    
    // Skeleton connections for BODY_18 (theo ZED SDK - COCO18 format)
    // Index mapping: 0=NOSE, 1=NECK, 2=RIGHT_SHOULDER, 3=RIGHT_ELBOW, 4=RIGHT_WRIST,
    //                5=LEFT_SHOULDER, 6=LEFT_ELBOW, 7=LEFT_WRIST, 8=RIGHT_HIP, 9=RIGHT_KNEE,
    //                10=RIGHT_ANKLE, 11=LEFT_HIP, 12=LEFT_KNEE, 13=LEFT_ANKLE,
    //                14=RIGHT_EYE, 15=LEFT_EYE, 16=RIGHT_EAR, 17=LEFT_EAR
    std::vector<std::pair<int, int>> body_18_bones = {
        // Head & Neck
        {0, 1},      // NOSE -> NECK
        {0, 14},     // NOSE -> RIGHT_EYE
        {0, 15},     // NOSE -> LEFT_EYE
        {14, 16},    // RIGHT_EYE -> RIGHT_EAR
        {15, 17},    // LEFT_EYE -> LEFT_EAR
        
        // Upper Body
        {1, 2},      // NECK -> RIGHT_SHOULDER
        {1, 5},      // NECK -> LEFT_SHOULDER
        {2, 5},      // RIGHT_SHOULDER -> LEFT_SHOULDER (shoulders)
        
        // Right Arm
        {2, 3},      // RIGHT_SHOULDER -> RIGHT_ELBOW
        {3, 4},      // RIGHT_ELBOW -> RIGHT_WRIST
        
        // Left Arm
        {5, 6},      // LEFT_SHOULDER -> LEFT_ELBOW
        {6, 7},      // LEFT_ELBOW -> LEFT_WRIST
        
        // Torso (shoulders to hips - giống ZED ROS examples)
        {2, 8},      // RIGHT_SHOULDER -> RIGHT_HIP
        {5, 11},     // LEFT_SHOULDER -> LEFT_HIP
        
        // Lower Body
        {8, 11},     // RIGHT_HIP -> LEFT_HIP (hips)
        
        // Right Leg
        {8, 9},      // RIGHT_HIP -> RIGHT_KNEE
        {9, 10},     // RIGHT_KNEE -> RIGHT_ANKLE
        
        // Left Leg
        {11, 12},    // LEFT_HIP -> LEFT_KNEE
        {12, 13}     // LEFT_KNEE -> LEFT_ANKLE
    };
    
    // Skeleton connections for BODY_34 (theo ZED SDK)
    // Index mapping: 0=PELVIS, 1=NAVAL_SPINE, 2=CHEST_SPINE, 3=NECK, 4=LEFT_CLAVICLE,
    //                5=LEFT_SHOULDER, 6=LEFT_ELBOW, 7=LEFT_WRIST, 8=LEFT_HAND, 9=LEFT_HANDTIP,
    //                10=LEFT_THUMB, 11=RIGHT_CLAVICLE, 12=RIGHT_SHOULDER, 13=RIGHT_ELBOW,
    //                14=RIGHT_WRIST, 15=RIGHT_HAND, 16=RIGHT_HANDTIP, 17=RIGHT_THUMB,
    //                18=LEFT_HIP, 19=LEFT_KNEE, 20=LEFT_ANKLE, 21=LEFT_FOOT,
    //                22=RIGHT_HIP, 23=RIGHT_KNEE, 24=RIGHT_ANKLE, 25=RIGHT_FOOT,
    //                26=HEAD, 27=NOSE, 28=LEFT_EYE, 29=LEFT_EAR, 30=RIGHT_EYE, 31=RIGHT_EAR,
    //                32=LEFT_HEEL, 33=RIGHT_HEEL
    std::vector<std::pair<int, int>> body_34_bones = {
        // Spine
        {0, 1},      // PELVIS -> NAVAL_SPINE
        {1, 2},      // NAVAL_SPINE -> CHEST_SPINE
        {2, 3},      // CHEST_SPINE -> NECK
        {3, 26},     // NECK -> HEAD
        {26, 27},    // HEAD -> NOSE
        {27, 28},    // NOSE -> LEFT_EYE
        {28, 29},    // LEFT_EYE -> LEFT_EAR
        {27, 30},    // NOSE -> RIGHT_EYE
        {30, 31},    // RIGHT_EYE -> RIGHT_EAR
        // Left arm
        {2, 4},      // CHEST_SPINE -> LEFT_CLAVICLE
        {4, 5},      // LEFT_CLAVICLE -> LEFT_SHOULDER
        {5, 6},      // LEFT_SHOULDER -> LEFT_ELBOW
        {6, 7},      // LEFT_ELBOW -> LEFT_WRIST
        {7, 8},      // LEFT_WRIST -> LEFT_HAND
        {8, 9},      // LEFT_HAND -> LEFT_HANDTIP
        {7, 10},     // LEFT_WRIST -> LEFT_THUMB
        // Right arm
        {2, 11},     // CHEST_SPINE -> RIGHT_CLAVICLE
        {11, 12},    // RIGHT_CLAVICLE -> RIGHT_SHOULDER
        {12, 13},    // RIGHT_SHOULDER -> RIGHT_ELBOW
        {13, 14},    // RIGHT_ELBOW -> RIGHT_WRIST
        {14, 15},    // RIGHT_WRIST -> RIGHT_HAND
        {15, 16},    // RIGHT_HAND -> RIGHT_HANDTIP
        {14, 17},    // RIGHT_WRIST -> RIGHT_THUMB
        // Left leg
        {0, 18},     // PELVIS -> LEFT_HIP
        {18, 19},    // LEFT_HIP -> LEFT_KNEE
        {19, 20},    // LEFT_KNEE -> LEFT_ANKLE
        {20, 21},    // LEFT_ANKLE -> LEFT_FOOT
        {20, 32},    // LEFT_ANKLE -> LEFT_HEEL
        // Right leg
        {0, 22},     // PELVIS -> RIGHT_HIP
        {22, 23},    // RIGHT_HIP -> RIGHT_KNEE
        {23, 24},    // RIGHT_KNEE -> RIGHT_ANKLE
        {24, 25},    // RIGHT_ANKLE -> RIGHT_FOOT
        {24, 33}     // RIGHT_ANKLE -> RIGHT_HEEL
    };
    
    static int parsePersonId(const std::string& name) {
        if (name.empty()) return -1;
        const std::string prefix = "person_";
        try {
            if (name.compare(0, prefix.size(), prefix) == 0) {
                return std::stoi(name.substr(prefix.size()));
            }
            return std::stoi(name);
        } catch (...) {
            return -1;
        }
    }

    std::string emotionForId(int id) const {
        auto it = last_emotion_.find(id);
        if (it == last_emotion_.end()) return "";
        return it->second;
    }

    // Box người: Angry = đỏ, còn lại (Happy/Neutral/chưa có nhãn) = xanh lá
    cv::Scalar boxColorForEmotion(const std::string& emo) const {
        if (emo == "Angry") {
            return cv::Scalar(0, 0, 255);  // BGR red
        }
        return cv::Scalar(0, 255, 0);  // BGR green
    }

    std_msgs::Header makeMarkerHeader(const std_msgs::Header& src) const {
        std_msgs::Header header = src;
        if (header.frame_id.empty()) {
            header.frame_id = camera_frame_id_;
        }
        // RViz Fixed Frame=map: dùng TF mới nhất tránh extrapolation đỏ
        if (use_latest_tf_) {
            header.stamp = ros::Time(0);
        }
        return header;
    }

    // Generate color từ bảng màu cố định (giống ZED SDK)
    cv::Scalar getColorForId(int id) {
        if (id < 0) {
            return cv::Scalar(236, 184, 36); // Màu vàng cho ID không hợp lệ
        }
        int color_idx = id % 8;
        // BGR format cho OpenCV
        return cv::Scalar(
            static_cast<int>(id_colors[color_idx][2]),  // B
            static_cast<int>(id_colors[color_idx][1]),  // G
            static_cast<int>(id_colors[color_idx][0])   // R
        );
    }
    
    // Kiểm tra giá trị hợp lệ (không phải NaN, inf, hoặc quá lớn)
    bool isValidValue(float val) {
        // Cho phép giá trị 0 và giá trị nhỏ (có thể là keypoint không được detect)
        // Nhưng vẫn filter NaN và inf
        return std::isfinite(val) && 
               std::abs(val) < 100.0f &&  // Giới hạn trong 100m
               !std::isnan(val) && 
               !std::isinf(val);
    }
    
    // Kiểm tra keypoint 3D hợp lệ và có giá trị thực (không phải 0,0,0)
    bool isValidKeypoint3D(const std::vector<float>& keypoints_3d, int idx) {
        if (idx * 3 + 2 >= keypoints_3d.size()) return false;
        float x = keypoints_3d[idx * 3];
        float y = keypoints_3d[idx * 3 + 1];
        float z = keypoints_3d[idx * 3 + 2];
        
        // Kiểm tra giá trị hợp lệ (NaN, inf)
        if (!isValidValue(x) || !isValidValue(y) || !isValidValue(z)) {
            return false;
        }
        
        // Kiểm tra keypoint có giá trị thực (không phải 0,0,0 - có thể là keypoint không được detect)
        // Nới lỏng threshold để cho phép nhiều keypoints hơn
        float threshold = 0.0001f;  // 0.1mm - rất nhỏ để cho phép nhiều keypoints
        return (std::abs(x) > threshold || std::abs(y) > threshold || std::abs(z) > threshold);
    }
    
    SmoothedVelocity smoothVelocity(int body_id, float vx, float vy, float vz) {
        const float speed = std::sqrt(vx * vx + vy * vy + vz * vz);
        auto& s = smoothed_velocity_[body_id];
        if (!s.initialized) {
            s.vx = vx;
            s.vy = vy;
            s.vz = vz;
            s.speed = speed;
            s.initialized = true;
            return s;
        }
        const float a = velocity_smoothing_;
        s.vx = a * vx + (1.0f - a) * s.vx;
        s.vy = a * vy + (1.0f - a) * s.vy;
        s.vz = a * vz + (1.0f - a) * s.vz;
        s.speed = std::sqrt(s.vx * s.vx + s.vy * s.vy + s.vz * s.vz);
        return s;
    }

    void drawSkeletonOnImage(cv::Mat& image, const zed_sdk_bridge::Body& body) {
        cv::Scalar color = getColorForId(body.id);

        if (show_skeleton_overlay_ && body.num_keypoints > 0) {
        std::vector<std::pair<int, int>>* bones = nullptr;
        
        // Chọn bones dựa trên body format
        if (body.body_format == 18) {
            bones = &body_18_bones;
        } else if (body.body_format == 34) {
            bones = &body_34_bones;
        }
        
        // Vẽ các bones (đường nối giữa các keypoints)
        // Sử dụng LINE_AA để có đường thẳng mịn hơn (giống ZED SDK)
        if (bones) {
        for (const auto& bone : *bones) {
            int idx1 = bone.first;
            int idx2 = bone.second;
            
            if (idx1 * 2 + 1 < body.keypoints_2d.size() && 
                idx2 * 2 + 1 < body.keypoints_2d.size()) {
                
                float x1 = body.keypoints_2d[idx1 * 2];
                float y1 = body.keypoints_2d[idx1 * 2 + 1];
                float x2 = body.keypoints_2d[idx2 * 2];
                float y2 = body.keypoints_2d[idx2 * 2 + 1];
                
                // Kiểm tra keypoints hợp lệ và trong bounds của ảnh
                int img_w = image.cols;
                int img_h = image.rows;
                if (x1 > 0 && y1 > 0 && x2 > 0 && y2 > 0 &&
                    x1 < img_w && y1 < img_h && x2 < img_w && y2 < img_h) {
                    cv::Point pt1(static_cast<int>(x1), static_cast<int>(y1));
                    cv::Point pt2(static_cast<int>(x2), static_cast<int>(y2));
                    // Sử dụng LINE_AA cho đường thẳng mịn hơn (giống ZED SDK)
                    cv::line(image, pt1, pt2, color, 
                            std::max(1, static_cast<int>(link_width_2d_)), 
                            cv::LINE_AA);
                }
            }
        }
        
        // Vẽ các joints (keypoints)
        // Joint size nhỏ hơn (3 pixels) giống ZED SDK examples
        int joint_size = std::max(2, std::min(5, static_cast<int>(joint_radius_2d_)));
        for (int i = 0; i < body.num_keypoints; i++) {
            if (i * 2 + 1 < body.keypoints_2d.size()) {
                float x = body.keypoints_2d[i * 2];
                float y = body.keypoints_2d[i * 2 + 1];
                
                // Kiểm tra trong bounds của ảnh
                int img_w = image.cols;
                int img_h = image.rows;
                if (x > 0 && y > 0 && x < img_w && y < img_h) {
                    cv::Point pt(static_cast<int>(x), static_cast<int>(y));
                    // Vẽ joint với size nhỏ hơn, mịn hơn
                    cv::circle(image, pt, joint_size, color, -1, cv::LINE_AA);
                    // Vẽ border trắng mỏng để nổi bật
                    cv::circle(image, pt, joint_size + 1, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
                }
            }
        }
        }
        }  // show_skeleton_overlay_
        
        // Vẽ bounding box
        if (body.bounding_box_2d_tl[0] > 0 && body.bounding_box_2d_tl[1] > 0 &&
            body.bounding_box_2d_br[0] > 0 && body.bounding_box_2d_br[1] > 0) {
            cv::Point tl(static_cast<int>(body.bounding_box_2d_tl[0]), 
                        static_cast<int>(body.bounding_box_2d_tl[1]));
            cv::Point br(static_cast<int>(body.bounding_box_2d_br[0]), 
                        static_cast<int>(body.bounding_box_2d_br[1]));
            const std::string emo = emotionForId(body.id);
            const cv::Scalar box_color = boxColorForEmotion(emo);
            cv::rectangle(image, tl, br, box_color, 2);
            
            // Vẽ ID, confidence và tốc độ (đã làm mượt)
            float speed = 0.0f;
            if (body.velocity.size() >= 3) {
                speed = smoothVelocity(body.id, body.velocity[0], body.velocity[1], body.velocity[2]).speed;
            }
            std::ostringstream oss;
            oss << "ID:" << body.id
                << " (" << static_cast<int>(body.confidence) << "%)"
                << " v=" << std::fixed << std::setprecision(2) << speed << " m/s";
            if (!emo.empty()) {
                oss << " | " << emo;
            }
            const std::string label = oss.str();
            int baseline = 0;
            const int pad = std::max(2, label_font_thickness_);
            cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX,
                                                 label_font_scale_, label_font_thickness_, &baseline);
            cv::Point text_org(tl.x, std::max(tl.y - pad * 2, text_size.height + pad));
            cv::rectangle(image,
                          cv::Point(text_org.x - pad, text_org.y - text_size.height - pad),
                          cv::Point(text_org.x + text_size.width + pad, text_org.y + baseline + pad),
                          cv::Scalar(0, 0, 0), -1);
            cv::putText(image, label, text_org, cv::FONT_HERSHEY_SIMPLEX,
                        label_font_scale_, box_color, label_font_thickness_, cv::LINE_AA);
        }
    }

    void drawObjectOnImage(cv::Mat& image, const zed_sdk_bridge::Object& obj) {
        float tlx = obj.bounding_box_2d_tl[0];
        float tly = obj.bounding_box_2d_tl[1];
        float brx = obj.bounding_box_2d_br[0];
        float bry = obj.bounding_box_2d_br[1];
        if (!std::isfinite(tlx) || !std::isfinite(tly) || !std::isfinite(brx) || !std::isfinite(bry)) {
            return;
        }
        if (brx <= tlx || bry <= tly) return;

        cv::Scalar color(40, 220, 40);  // BGR green
        cv::Point tl(static_cast<int>(tlx), static_cast<int>(tly));
        cv::Point br(static_cast<int>(brx), static_cast<int>(bry));
        cv::rectangle(image, tl, br, color, 2, cv::LINE_AA);

        std::ostringstream oss;
        oss << obj.label << " #" << obj.id << " " << std::fixed << std::setprecision(0) << obj.confidence;
        int baseline = 0;
        const int pad = std::max(2, label_font_thickness_);
        cv::Size text_size = cv::getTextSize(oss.str(), cv::FONT_HERSHEY_SIMPLEX,
                                             label_font_scale_, label_font_thickness_, &baseline);
        cv::Point text_org(tl.x, std::max(tl.y - pad * 2, text_size.height + pad));
        cv::rectangle(image,
                      cv::Point(text_org.x - pad, text_org.y - text_size.height - pad),
                      cv::Point(text_org.x + text_size.width + pad, text_org.y + baseline + pad),
                      cv::Scalar(0, 0, 0), -1);
        cv::putText(image, oss.str(), text_org, cv::FONT_HERSHEY_SIMPLEX,
                    label_font_scale_, color, label_font_thickness_, cv::LINE_AA);
    }

    void publishObjectMarkers(const zed_sdk_bridge::ObjectsStampedConstPtr& objects_msg) {
        visualization_msgs::MarkerArray marker_array;
        std::set<int> active_ids;
        ros::Duration lifetime(marker_lifetime_sec_);

        for (const auto& obj : objects_msg->objects) {
            active_ids.insert(obj.id);

            visualization_msgs::Marker cube;
            cube.header = makeMarkerHeader(objects_msg->header);
            cube.ns = "od_objects";
            cube.id = obj.id;
            cube.type = visualization_msgs::Marker::CUBE;
            cube.action = visualization_msgs::Marker::ADD;
            cube.pose.position.x = obj.position[0];
            cube.pose.position.y = obj.position[1];
            cube.pose.position.z = obj.position[2];
            cube.pose.orientation.w = 1.0;
            float sx = std::max(0.15f, obj.dimensions[0] > 0.01f ? obj.dimensions[0] : object_marker_size_);
            float sy = std::max(0.15f, obj.dimensions[1] > 0.01f ? obj.dimensions[1] : object_marker_size_);
            float sz = std::max(0.15f, obj.dimensions[2] > 0.01f ? obj.dimensions[2] : object_marker_size_);
            cube.scale.x = sx;
            cube.scale.y = sy;
            cube.scale.z = sz;
            cube.color.r = 0.15f;
            cube.color.g = 0.9f;
            cube.color.b = 0.2f;
            cube.color.a = 0.55f;
            cube.lifetime = lifetime;
            marker_array.markers.push_back(cube);

            visualization_msgs::Marker text;
            text.header = makeMarkerHeader(objects_msg->header);
            text.ns = "od_objects_text";
            text.id = obj.id;
            text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text.action = visualization_msgs::Marker::ADD;
            text.pose.position.x = obj.position[0];
            text.pose.position.y = obj.position[1];
            text.pose.position.z = obj.position[2] + sz * 0.5f + 0.15f;
            text.pose.orientation.w = 1.0;
            text.scale.z = 0.18;
            text.color.r = 0.9f;
            text.color.g = 1.0f;
            text.color.b = 0.9f;
            text.color.a = 1.0f;
            text.lifetime = lifetime;
            std::ostringstream oss;
            oss << obj.label << " #" << obj.id;
            text.text = oss.str();
            marker_array.markers.push_back(text);
        }

        for (int old_id : last_object_ids_) {
            if (active_ids.count(old_id)) continue;
            visualization_msgs::Marker del;
            del.header = makeMarkerHeader(objects_msg->header);
            del.action = visualization_msgs::Marker::DELETE;
            del.id = old_id;
            del.ns = "od_objects";
            marker_array.markers.push_back(del);
            del.ns = "od_objects_text";
            marker_array.markers.push_back(del);
        }
        last_object_ids_ = active_ids;

        if (!marker_array.markers.empty()) {
            marker_pub_.publish(marker_array);
        }
        ROS_INFO_THROTTLE(2.0, "OD markers: %zu objects", active_ids.size());
    }
    
    void publishMarkers(const zed_sdk_bridge::BodiesStampedConstPtr& bodies_msg) {
        visualization_msgs::MarkerArray marker_array;
        const ros::Duration marker_lifetime(marker_lifetime_sec_);

        std::set<int> active_body_ids;
        int joint_marker_id = 0;
        int bone_marker_id = 0;
        int total_joints = 0;
        int total_bones = 0;

        for (const auto& body : bodies_msg->bodies) {
            if (body.num_keypoints == 0) continue;
            active_body_ids.insert(body.id);

            cv::Scalar color = getColorForId(body.id);
            std::vector<std::pair<int, int>>* bones = nullptr;

            if (body.body_format == 18) {
                bones = &body_18_bones;
            } else if (body.body_format == 34) {
                bones = &body_34_bones;
            } else {
                continue;
            }

            for (int i = 0; i < body.num_keypoints; i++) {
                if (!isValidKeypoint3D(body.keypoints_3d, i)) continue;
                total_joints++;

                visualization_msgs::Marker joint_marker;
                joint_marker.header = makeMarkerHeader(bodies_msg->header);
                joint_marker.ns = "skeleton_joints";
                joint_marker.id = body.id * 100 + i;
                joint_marker.type = visualization_msgs::Marker::SPHERE;
                joint_marker.action = visualization_msgs::Marker::ADD;
                joint_marker.pose.position.x = body.keypoints_3d[i * 3];
                joint_marker.pose.position.y = body.keypoints_3d[i * 3 + 1];
                joint_marker.pose.position.z = body.keypoints_3d[i * 3 + 2];
                joint_marker.pose.orientation.w = 1.0;
                joint_marker.scale.x = joint_marker.scale.y = joint_marker.scale.z = joint_radius_;
                joint_marker.color.r = color[2] / 255.0;
                joint_marker.color.g = color[1] / 255.0;
                joint_marker.color.b = color[0] / 255.0;
                joint_marker.color.a = 0.95;
                joint_marker.lifetime = marker_lifetime;
                marker_array.markers.push_back(joint_marker);
                joint_marker_id = std::max(joint_marker_id, joint_marker.id);
            }

            int bone_idx = 0;
            for (const auto& bone : *bones) {
                const int idx1 = bone.first;
                const int idx2 = bone.second;
                if (!isValidKeypoint3D(body.keypoints_3d, idx1) ||
                    !isValidKeypoint3D(body.keypoints_3d, idx2)) {
                    continue;
                }
                total_bones++;
                bone_idx++;

                visualization_msgs::Marker line_marker;
                line_marker.header = makeMarkerHeader(bodies_msg->header);
                line_marker.ns = "skeleton_bones";
                line_marker.id = body.id * 100 + bone_idx;
                line_marker.type = visualization_msgs::Marker::LINE_LIST;
                line_marker.action = visualization_msgs::Marker::ADD;

                geometry_msgs::Point p1, p2;
                p1.x = body.keypoints_3d[idx1 * 3];
                p1.y = body.keypoints_3d[idx1 * 3 + 1];
                p1.z = body.keypoints_3d[idx1 * 3 + 2];
                p2.x = body.keypoints_3d[idx2 * 3];
                p2.y = body.keypoints_3d[idx2 * 3 + 1];
                p2.z = body.keypoints_3d[idx2 * 3 + 2];
                line_marker.points.push_back(p1);
                line_marker.points.push_back(p2);
                line_marker.pose.orientation.w = 1.0;
                line_marker.scale.x = link_width_;
                line_marker.color.r = std::min(1.0f, static_cast<float>(color[2] / 255.0f));
                line_marker.color.g = std::min(1.0f, static_cast<float>(color[1] / 255.0f));
                line_marker.color.b = std::min(1.0f, static_cast<float>(color[0] / 255.0f));
                line_marker.color.a = 0.95;
                line_marker.lifetime = marker_lifetime;
                marker_array.markers.push_back(line_marker);
                bone_marker_id = std::max(bone_marker_id, line_marker.id);
            }

            if (body.position.size() >= 3 && body.velocity.size() >= 3 &&
                isValidValue(body.position[0]) && isValidValue(body.position[1]) &&
                isValidValue(body.position[2])) {
                const SmoothedVelocity vel = smoothVelocity(
                    body.id, body.velocity[0], body.velocity[1], body.velocity[2]);

                geometry_msgs::Point start;
                start.x = body.position[0];
                start.y = body.position[1];
                start.z = body.position[2];

                if (vel.speed >= velocity_min_display_) {
                    geometry_msgs::Point end;
                    const float arrow_scale = 2.0f;
                    const float len = std::max(0.08f, vel.speed * arrow_scale);
                    end.x = start.x + (vel.vx / vel.speed) * len;
                    end.y = start.y + (vel.vy / vel.speed) * len;
                    end.z = start.z + (vel.vz / vel.speed) * len;

                    visualization_msgs::Marker vel_arrow;
                    vel_arrow.header = makeMarkerHeader(bodies_msg->header);
                    vel_arrow.ns = "skeleton_velocity";
                    vel_arrow.id = body.id;
                    vel_arrow.type = visualization_msgs::Marker::ARROW;
                    vel_arrow.action = visualization_msgs::Marker::ADD;
                    vel_arrow.points.push_back(start);
                    vel_arrow.points.push_back(end);
                    vel_arrow.scale.x = 0.03;
                    vel_arrow.scale.y = 0.06;
                    vel_arrow.scale.z = 0.10;
                    vel_arrow.color.r = 1.0;
                    vel_arrow.color.g = 0.75;
                    vel_arrow.color.b = 0.0;
                    vel_arrow.color.a = 0.9;
                    vel_arrow.lifetime = marker_lifetime;
                    vel_arrow.pose.orientation.w = 1.0;
                    marker_array.markers.push_back(vel_arrow);
                }

                visualization_msgs::Marker vel_text;
                vel_text.header = makeMarkerHeader(bodies_msg->header);
                vel_text.ns = "skeleton_velocity_text";
                vel_text.id = body.id;
                vel_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                vel_text.action = visualization_msgs::Marker::ADD;
                vel_text.pose.position = start;
                vel_text.pose.position.z += 0.25;
                vel_text.pose.orientation.w = 1.0;
                vel_text.scale.z = 0.10;
                vel_text.color.r = 1.0;
                vel_text.color.g = 1.0;
                vel_text.color.b = 1.0;
                vel_text.color.a = 0.95;
                vel_text.lifetime = marker_lifetime;
                std::ostringstream oss;
                oss << "ID" << body.id;
                const std::string emo = emotionForId(body.id);
                if (!emo.empty()) {
                    oss << " " << emo;
                }
                oss << "  " << std::fixed << std::setprecision(2) << vel.speed << " m/s";
                vel_text.text = oss.str();
                marker_array.markers.push_back(vel_text);
            }
        }

        // Xóa marker của body không còn track
        for (int old_id : last_body_ids_) {
            if (active_body_ids.count(old_id)) continue;
            smoothed_velocity_.erase(old_id);
            last_emotion_.erase(old_id);

            visualization_msgs::Marker delete_marker;
            delete_marker.header = makeMarkerHeader(bodies_msg->header);
            delete_marker.action = visualization_msgs::Marker::DELETE;
            delete_marker.lifetime = ros::Duration(0);
            delete_marker.pose.orientation.w = 1.0;

            delete_marker.ns = "skeleton_velocity";
            delete_marker.id = old_id;
            marker_array.markers.push_back(delete_marker);
            delete_marker.ns = "skeleton_velocity_text";
            marker_array.markers.push_back(delete_marker);

            for (int i = 0; i < 100; ++i) {
                delete_marker.ns = "skeleton_joints";
                delete_marker.id = old_id * 100 + i;
                marker_array.markers.push_back(delete_marker);
                delete_marker.ns = "skeleton_bones";
                delete_marker.id = old_id * 100 + i;
                marker_array.markers.push_back(delete_marker);
            }
        }
        last_body_ids_ = active_body_ids;
        max_joint_marker_id_ = joint_marker_id;
        max_bone_marker_id_ = bone_marker_id;

        ROS_INFO_THROTTLE(2.0, "Skeleton markers: %d joints, %d bones, %zu bodies",
                          total_joints, total_bones, active_body_ids.size());

        marker_pub_.publish(marker_array);
    }
    
    void bodiesCallback(const zed_sdk_bridge::BodiesStampedConstPtr& msg) {
        latest_bodies_ = msg;
        
        if (publish_markers_ && marker_pub_.getNumSubscribers() > 0) {
            const ros::Time now = ros::Time::now();
            if (max_marker_hz_ <= 0.f || last_marker_pub_.isZero() ||
                (now - last_marker_pub_).toSec() >= (1.0 / max_marker_hz_)) {
                publishMarkers(msg);
                last_marker_pub_ = now;
            }
        }
        
        if (latest_image_) {
            processAndPublishImage();
        }
    }

    void emotionsCallback(const social_msgs::SocialPeople::ConstPtr& msg) {
        for (const auto& person : msg->people) {
            const int id = parsePersonId(person.name);
            if (id < 0 || person.emotion.empty()) continue;
            last_emotion_[id] = person.emotion;
        }
    }

    void objectsCallback(const zed_sdk_bridge::ObjectsStampedConstPtr& msg) {
        latest_objects_ = msg;
        if (show_objects_ && publish_markers_ && marker_pub_.getNumSubscribers() > 0) {
            const ros::Time now = ros::Time::now();
            if (max_marker_hz_ <= 0.f || last_marker_pub_.isZero() ||
                (now - last_marker_pub_).toSec() >= (1.0 / max_marker_hz_)) {
                publishObjectMarkers(msg);
                last_marker_pub_ = now;
            }
        }
        if (latest_image_) {
            processAndPublishImage();
        }
    }
    
    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        latest_image_ = msg;
        if (latest_bodies_ || latest_objects_) {
            processAndPublishImage();
        }
    }
    
    void processAndPublishImage() {
        if (!latest_image_) return;
        if (image_pub_.getNumSubscribers() == 0) return;

        const ros::Time now = ros::Time::now();
        if (max_image_hz_ > 0.f && !last_image_pub_.isZero() &&
            (now - last_image_pub_).toSec() < (1.0 / max_image_hz_)) {
            return;
        }
        
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(latest_image_, sensor_msgs::image_encodings::BGR8);
            cv::Mat image = cv_ptr->image;

            if (latest_bodies_) {
                for (const auto& body : latest_bodies_->bodies) {
                    drawSkeletonOnImage(image, body);
                }
            }
            if (show_objects_ && latest_objects_) {
                for (const auto& obj : latest_objects_->objects) {
                    drawObjectOnImage(image, obj);
                }
                std::ostringstream status;
                status << "OD: " << latest_objects_->objects.size() << " obj";
                if (latest_bodies_) {
                    status << " | BT: " << latest_bodies_->bodies.size() << " body";
                }
                cv::putText(image, status.str(), cv::Point(12, 28),
                            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(40, 220, 40), 2, cv::LINE_AA);
            }

            // scale=1.0 mặc định (phóng 2x làm RViz rất nặng)
            if (image_display_scale_ > 1.01f) {
                cv::Mat scaled;
                cv::resize(image, scaled, cv::Size(),
                           static_cast<double>(image_display_scale_),
                           static_cast<double>(image_display_scale_),
                           cv::INTER_LINEAR);
                image = scaled;
            }

            sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(latest_image_->header, "bgr8", image).toImageMsg();
            image_pub_.publish(img_msg);
            last_image_pub_ = now;
            
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }
    
public:
    SkeletonVisualizer() : nh_private_("~"), it_(nh_) {
        // Get parameters
        nh_private_.param<bool>("show_on_image", show_on_image_, true);
        nh_private_.param<bool>("publish_markers", publish_markers_, true);
        nh_private_.param<bool>("show_objects", show_objects_, true);
        nh_private_.param<bool>("show_skeleton_overlay", show_skeleton_overlay_, true);
        // 3D RViz markers (meters)
        nh_private_.param<float>("joint_radius", joint_radius_, 0.025f);
        nh_private_.param<float>("link_width", link_width_, 0.012f);
        // 2D overlay (pixels)
        nh_private_.param<float>("joint_radius_2d", joint_radius_2d_, 3.0f);
        nh_private_.param<float>("link_width_2d", link_width_2d_, 2.0f);
        nh_private_.param<float>("velocity_smoothing", velocity_smoothing_, 0.85f);
        nh_private_.param<float>("velocity_min_display", velocity_min_display_, 0.05f);
        nh_private_.param<float>("marker_lifetime_sec", marker_lifetime_sec_, 0.25f);
        nh_private_.param<float>("object_marker_size", object_marker_size_, 0.3f);
        nh_private_.param<float>("image_display_scale", image_display_scale_, 1.0f);
        nh_private_.param<float>("max_image_hz", max_image_hz_, 8.0f);
        nh_private_.param<float>("max_marker_hz", max_marker_hz_, 8.0f);
        nh_private_.param<float>("label_font_scale", label_font_scale_, 0.9f);
        nh_private_.param<int>("label_font_thickness", label_font_thickness_, 3);
        nh_private_.param<bool>("use_latest_tf", use_latest_tf_, true);
        nh_private_.param<std::string>("camera_frame_id", camera_frame_id_, "camera_link");
        std::string emotions_topic;
        nh_private_.param<std::string>("emotions_topic", emotions_topic, "/person_emotions");
        
        // Subscribers
        bodies_sub_ = nh_.subscribe("bodies", 1, &SkeletonVisualizer::bodiesCallback, this);
        objects_sub_ = nh_.subscribe("objects", 1, &SkeletonVisualizer::objectsCallback, this);
        emotions_sub_ = nh_.subscribe(emotions_topic, 5, &SkeletonVisualizer::emotionsCallback, this);
        image_sub_ = it_.subscribe("image_raw", 1, &SkeletonVisualizer::imageCallback, this);
        
        // Publishers
        image_pub_ = it_.advertise("image_skeleton", 1);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("skeleton_markers", 1);
        
        ROS_INFO("Skeleton Visualizer initialized");
        ROS_INFO("  - Show on image: %s", show_on_image_ ? "true" : "false");
        ROS_INFO("  - Publish markers: %s", publish_markers_ ? "true" : "false");
        ROS_INFO("  - Show objects: %s", show_objects_ ? "true" : "false");
        ROS_INFO("  - image_display_scale=%.2f max_image_hz=%.1f max_marker_hz=%.1f use_latest_tf=%s",
                 image_display_scale_, max_image_hz_, max_marker_hz_, use_latest_tf_ ? "true" : "false");
        ROS_INFO("  - label_font_scale=%.2f label_font_thickness=%d",
                 label_font_scale_, label_font_thickness_);
    }
};

// Define static member outside class (C++11 requirement)
const float SkeletonVisualizer::id_colors[8][3] = {
    {232.0f, 176.0f, 59.0f},   // Vàng cam
    {175.0f, 208.0f, 25.0f},   // Xanh lá
    {102.0f, 205.0f, 105.0f},  // Xanh lá nhạt
    {185.0f, 0.0f,   255.0f},  // Tím
    {99.0f,  107.0f, 252.0f},  // Xanh dương
    {252.0f, 225.0f, 8.0f},    // Vàng
    {167.0f, 130.0f, 141.0f},  // Hồng xám
    {194.0f, 72.0f,  113.0f}   // Hồng đậm
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "skeleton_visualizer");
    
    try {
        SkeletonVisualizer visualizer;
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
        return -1;
    }
    
    return 0;
}

