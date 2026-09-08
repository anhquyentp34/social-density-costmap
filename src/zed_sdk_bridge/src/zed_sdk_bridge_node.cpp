///////////////////////////////////////////////////////////////////////////
//
// ZED Body Tracking ROS Node (+ optional ZED Object Detection)
// Object Detection mặc định tắt — dùng Ultralytics YOLO → /objects thay thế.
// Publish thêm /depth + /camera_info cho YOLO bridge.
//
///////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <cmath>
#include <memory>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include <sl/Camera.hpp>

#include "zed_sdk_bridge/Body.h"
#include "zed_sdk_bridge/BodiesStamped.h"
#include "zed_sdk_bridge/Object.h"
#include "zed_sdk_bridge/ObjectsStamped.h"

using namespace sl;

class ZedSdkBridgeNode {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Publisher bodies_pub_;
    ros::Publisher objects_pub_;
    ros::Publisher camera_info_pub_;
    ros::Publisher vio_odom_pub_;
    ros::Publisher imu_pub_;
    ros::Publisher vio_state_pub_;
    ros::Publisher vio_reliability_pub_;
    int last_vio_state_code_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;
    image_transport::Publisher depth_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    Camera zed_;
    Bodies bodies_;
    Objects objects_;
    BodyTrackingRuntimeParameters body_rt_;
    ObjectDetectionRuntimeParameters od_rt_;
    RuntimeParameters runtime_params_;

    // Body tracking
    std::string camera_frame_id_;
    std::string bodies_frame_id_;
    std::string measure3d_reference_frame_;
    std::string detection_model_str_;
    bool enable_tracking_;
    bool enable_positional_tracking_;
    bool positional_tracking_enabled_;
    bool enable_body_fitting_;
    bool enable_segmentation_;
    int body_format_;
    float detection_confidence_threshold_;
    float skeleton_smoothing_;
    int camera_resolution_;
    int depth_mode_;
    unsigned int body_instance_id_;
    bool publish_depth_;

    // ZED Visual–Inertial Odometry (Positional Tracking → ROS odom)
    // Không tích phân accelerometer thô; dùng getPosition() từ SDK VIO.
    bool publish_vio_odom_;
    bool publish_vio_tf_;
    bool publish_imu_;
    bool vio_two_d_mode_;
    bool enable_area_memory_;
    bool enable_imu_fusion_;
    std::string vio_odom_topic_;
    std::string imu_topic_;
    std::string odom_frame_id_;
    std::string base_frame_id_;
    double fixed_z_value_;
    double vio_publish_hz_;
    ros::Time last_vio_pub_;
    // T_base_camera từ URDF (base_footprint≈base_link → camera_link)
    double cam_pos_x_, cam_pos_y_, cam_pos_z_;
    double cam_roll_, cam_pitch_, cam_yaw_;
    tf2::Transform T_base_camera_;
    tf2::Transform T_odom_base_;
    bool vio_origin_set_;
    tf2::Transform T_world_camera_origin_inv_;
    ros::Time last_vio_stamp_;
    tf2::Vector3 last_base_origin_;
    double last_base_yaw_;
    bool have_last_base_pose_;
    uint64_t vio_ok_count_;
    uint64_t vio_lost_count_;

    // Object detection
    bool enable_object_detection_;
    std::string od_model_str_;
    float od_confidence_threshold_;
    float od_max_range_;
    bool od_enable_tracking_;
    bool od_filter_person_;  // bỏ PERSON (đã có body tracking)
    bool od_mc_people_;
    bool od_mc_vehicle_;
    bool od_mc_bag_;
    bool od_mc_animal_;
    bool od_mc_electronics_;
    bool od_mc_fruit_vegetable_;
    bool od_mc_sport_;
    unsigned int od_instance_id_;
    bool object_detection_enabled_;
    double rgb_publish_hz_;
    double depth_publish_hz_;
    ros::Time last_rgb_pub_;
    ros::Time last_depth_pub_;
    int camera_serial_number_;
    int camera_fps_;
    double open_retry_delay_sec_;
    float open_timeout_sec_;

    Mat image_left_;
    Mat depth_map_;
    sensor_msgs::CameraInfo camera_info_msg_;
    bool camera_info_ready_;
    cv::Mat image_left_ocv_;
    bool camera_initialized_;

    static std::string objectClassToString(OBJECT_CLASS cls) {
        switch (cls) {
            case OBJECT_CLASS::PERSON: return "Person";
            case OBJECT_CLASS::VEHICLE: return "Vehicle";
            case OBJECT_CLASS::BAG: return "Bag";
            case OBJECT_CLASS::ANIMAL: return "Animal";
            case OBJECT_CLASS::ELECTRONICS: return "Electronics";
            case OBJECT_CLASS::FRUIT_VEGETABLE: return "Fruit_Vegetable";
            case OBJECT_CLASS::SPORT: return "Sport";
            default: return "Unknown";
        }
    }

public:
    ZedSdkBridgeNode()
        : nh_private_("~"),
          it_(nh_),
          camera_initialized_(false),
          positional_tracking_enabled_(false),
          object_detection_enabled_(false),
          camera_info_ready_(false),
          vio_origin_set_(false),
          have_last_base_pose_(false),
          vio_ok_count_(0),
          vio_lost_count_(0),
          last_vio_pub_(0) {
        bodies_pub_ = nh_.advertise<zed_sdk_bridge::BodiesStamped>("bodies", 1);
        objects_pub_ = nh_.advertise<zed_sdk_bridge::ObjectsStamped>("objects", 1);
        camera_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("camera_info", 1);
        image_pub_ = it_.advertise("image_raw", 1);
        depth_pub_ = it_.advertise("depth", 1);

        nh_private_.param<std::string>("camera_frame_id", camera_frame_id_, "camera_link");
        // CAMERA (default): BodyData position/velocity relative to camera — ROS frame = camera_link.
        // WORLD: BodyData in ZED positional-tracking world — NOT ROS map; use separate frame_id.
        nh_private_.param<std::string>("measure3d_reference_frame", measure3d_reference_frame_, "camera");
        nh_private_.param<std::string>("bodies_frame_id", bodies_frame_id_, "");
        nh_private_.param<std::string>("detection_model", detection_model_str_, "HUMAN_BODY_FAST");
        nh_private_.param<bool>("enable_tracking", enable_tracking_, true);
        nh_private_.param<bool>("enable_positional_tracking", enable_positional_tracking_, false);
        nh_private_.param<bool>("enable_body_fitting", enable_body_fitting_, false);
        nh_private_.param<bool>("enable_segmentation", enable_segmentation_, false);

        // VIO (Positional Tracking) → /odom_zed — mặc định tắt TF để tránh xung đột EKF
        nh_private_.param<bool>("publish_vio_odom", publish_vio_odom_, false);
        nh_private_.param<bool>("publish_vio_tf", publish_vio_tf_, false);
        nh_private_.param<bool>("publish_imu", publish_imu_, false);
        nh_private_.param<bool>("vio_two_d_mode", vio_two_d_mode_, true);
        // Mặc định true: area_memory=false + Body Tracking đồng thời gây SIGSEGV trên setup này.
        nh_private_.param<bool>("enable_area_memory", enable_area_memory_, true);
        nh_private_.param<bool>("enable_imu_fusion", enable_imu_fusion_, true);
        nh_private_.param<std::string>("vio_odom_topic", vio_odom_topic_, "/odom_zed");
        nh_private_.param<std::string>("imu_topic", imu_topic_, "/zed_imu");
        nh_private_.param<std::string>("odom_frame_id", odom_frame_id_, "odom");
        nh_private_.param<std::string>("base_frame_id", base_frame_id_, "base_footprint");
        nh_private_.param<double>("fixed_z_value", fixed_z_value_, 0.0);
        // Giới hạn Hz VIO để không tranh GPU với Body Tracking + YOLO
        nh_private_.param<double>("vio_publish_hz", vio_publish_hz_, 15.0);
        // Khớp URDF x_omni4wd_description: base_link → camera_link
        nh_private_.param<double>("cam_pos_x", cam_pos_x_, 0.052484109778863);
        nh_private_.param<double>("cam_pos_y", cam_pos_y_, 8.47956709802977e-05);
        nh_private_.param<double>("cam_pos_z", cam_pos_z_, 1.05312524052101);
        nh_private_.param<double>("cam_roll", cam_roll_, 0.0);
        nh_private_.param<double>("cam_pitch", cam_pitch_, 0.0);
        nh_private_.param<double>("cam_yaw", cam_yaw_, 0.0);

        tf2::Quaternion q_bc;
        q_bc.setRPY(cam_roll_, cam_pitch_, cam_yaw_);
        T_base_camera_.setOrigin(tf2::Vector3(cam_pos_x_, cam_pos_y_, cam_pos_z_));
        T_base_camera_.setRotation(q_bc);
        T_odom_base_.setIdentity();

        if (publish_vio_odom_) {
            vio_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(vio_odom_topic_, 10);
            vio_state_pub_ = nh_.advertise<std_msgs::Int8>("/vio_tracking_state", 10);
            vio_reliability_pub_ = nh_.advertise<std_msgs::Float32>("/vio_reliability", 10);
        }
        if (publish_imu_) {
            imu_pub_ = nh_.advertise<sensor_msgs::Imu>(imu_topic_, 50);
        }
        if (publish_vio_tf_) {
            tf_broadcaster_.reset(new tf2_ros::TransformBroadcaster());
            ROS_WARN(
                "publish_vio_tf=true: node sẽ xuất TF %s→%s. "
                "Tắt khi dùng làm đầu vào robot_localization.",
                odom_frame_id_.c_str(), base_frame_id_.c_str());
        }
        nh_private_.param<int>("body_format", body_format_, 34);
        nh_private_.param<float>("detection_confidence_threshold", detection_confidence_threshold_, 40.0f);
        nh_private_.param<float>("skeleton_smoothing", skeleton_smoothing_, 0.7f);
        nh_private_.param<int>("camera_resolution", camera_resolution_, 0);
        nh_private_.param<int>("depth_mode", depth_mode_, 1);
        nh_private_.param<bool>("publish_depth", publish_depth_, true);
        nh_private_.param<double>("rgb_publish_hz", rgb_publish_hz_, 10.0);
        nh_private_.param<double>("depth_publish_hz", depth_publish_hz_, 8.0);
        nh_private_.param<int>("camera_serial_number", camera_serial_number_, 0);
        nh_private_.param<int>("camera_fps", camera_fps_, 30);
        nh_private_.param<double>("open_retry_delay_sec", open_retry_delay_sec_, 15.0);
        nh_private_.param<float>("open_timeout_sec", open_timeout_sec_, 15.0f);
        int body_id = 0;
        nh_private_.param<int>("body_instance_module_id", body_id, 0);
        body_instance_id_ = static_cast<unsigned int>(body_id);

        // Mặc định tắt ZED OD — dùng YOLO → /objects thay thế
        nh_private_.param<bool>("enable_object_detection", enable_object_detection_, false);
        nh_private_.param<std::string>("od_model", od_model_str_, "MULTI_CLASS_BOX_FAST");
        nh_private_.param<float>("od_confidence_threshold", od_confidence_threshold_, 40.0f);
        nh_private_.param<float>("od_max_range", od_max_range_, 15.0f);
        nh_private_.param<bool>("od_enable_tracking", od_enable_tracking_, true);
        nh_private_.param<bool>("od_filter_person", od_filter_person_, true);
        nh_private_.param<bool>("od_mc_people", od_mc_people_, false);
        nh_private_.param<bool>("od_mc_vehicle", od_mc_vehicle_, true);
        nh_private_.param<bool>("od_mc_bag", od_mc_bag_, true);
        nh_private_.param<bool>("od_mc_animal", od_mc_animal_, true);
        nh_private_.param<bool>("od_mc_electronics", od_mc_electronics_, true);
        nh_private_.param<bool>("od_mc_fruit_vegetable", od_mc_fruit_vegetable_, false);
        nh_private_.param<bool>("od_mc_sport", od_mc_sport_, false);
        int od_id = 1;
        nh_private_.param<int>("od_instance_module_id", od_id, 1);
        od_instance_id_ = static_cast<unsigned int>(od_id);

        configureRuntimeParameters();

        ROS_INFO("=== ZED Body Tracking + Object Detection ===");
        ROS_INFO("Frame: %s | Body model: %s (instance_id=%u)",
                 camera_frame_id_.c_str(), detection_model_str_.c_str(), body_instance_id_);
        ROS_INFO("measure3D_reference_frame=%s | bodies header.frame_id=%s",
                 measure3d_reference_frame_.c_str(),
                 bodies_frame_id_.empty() ? "(auto)" : bodies_frame_id_.c_str());
        ROS_INFO("Object detection: %s | model=%s (instance_id=%u)",
                 enable_object_detection_ ? "ON" : "OFF", od_model_str_.c_str(), od_instance_id_);
        ROS_INFO("VIO odom: %s topic=%s TF=%s 2D=%s area_memory=%s imu_fusion=%s",
                 publish_vio_odom_ ? "ON" : "OFF",
                 vio_odom_topic_.c_str(),
                 publish_vio_tf_ ? "ON" : "OFF",
                 vio_two_d_mode_ ? "ON" : "OFF",
                 enable_area_memory_ ? "ON" : "OFF",
                 enable_imu_fusion_ ? "ON" : "OFF");

        ROS_INFO("RGB publish <= %.1f Hz | depth publish <= %.1f Hz",
                 rgb_publish_hz_, depth_publish_hz_);
        if (camera_serial_number_ > 0) {
            ROS_INFO("Camera serial: %d | open_timeout=%.1fs | retry_delay=%.1fs",
                     camera_serial_number_, open_timeout_sec_, open_retry_delay_sec_);
        }
    }

    bool setupCamera() {
        if (camera_initialized_) {
            return true;
        }

        if (!initializeZED()) {
            return false;
        }

        // ZED SDK: enable_tracking trên Body Tracking bắt buộc Positional Tracking bật trước.
        // VIO publish cũng cần Positional Tracking.
        const bool need_pt =
            enable_tracking_ ||
            (measure3d_reference_frame_ == "world") ||
            enable_positional_tracking_ ||
            publish_vio_odom_;
        if (measure3d_reference_frame_ == "world" && !enable_tracking_) {
            ROS_ERROR("measure3d_reference_frame=world requires enable_tracking:=true");
            shutdownCamera();
            return false;
        }
        if (enable_tracking_ && !enable_positional_tracking_ && !publish_vio_odom_) {
            ROS_WARN(
                "enable_tracking=true requires Positional Tracking (ZED SDK); enabling PT.");
        }
        if (need_pt) {
            if (!enablePositionalTracking()) {
                shutdownCamera();
                return false;
            }
            positional_tracking_enabled_ = true;
        } else {
            positional_tracking_enabled_ = false;
            ROS_INFO("Positional tracking OFF (enable_tracking=false, CAMERA mode, VIO off)");
        }

        if (!enableBodyTracking()) {
            shutdownCamera();
            return false;
        }

        if (enable_object_detection_) {
            if (!enableObjectDetection()) {
                ROS_WARN("Object detection failed to enable — tiếp tục chỉ body tracking");
            }
        }

        camera_initialized_ = true;
        ROS_INFO("ZED node initialized (body + OD concurrent=%s)",
                 object_detection_enabled_ ? "yes" : "no");
        return true;
    }

    void shutdownCamera() {
        if (!zed_.isOpened()) {
            camera_initialized_ = false;
            object_detection_enabled_ = false;
            camera_info_ready_ = false;
            return;
        }
        if (object_detection_enabled_) {
            zed_.disableObjectDetection(od_instance_id_);
        }
        zed_.disableBodyTracking(body_instance_id_);
        if (positional_tracking_enabled_) {
            zed_.disablePositionalTracking();
            positional_tracking_enabled_ = false;
        }
        zed_.close();
        ROS_INFO("ZED camera closed");
        camera_initialized_ = false;
        object_detection_enabled_ = false;
        camera_info_ready_ = false;
        vio_origin_set_ = false;
        have_last_base_pose_ = false;
    }

    ~ZedSdkBridgeNode() {
        shutdownCamera();
    }

    bool initializeZED() {
        InitParameters init_params;
        switch (camera_resolution_) {
            case 0: init_params.camera_resolution = RESOLUTION::AUTO; break;
            case 1: init_params.camera_resolution = RESOLUTION::HD2K; break;
            case 2: init_params.camera_resolution = RESOLUTION::HD1080; break;
            case 3: init_params.camera_resolution = RESOLUTION::HD720; break;
            case 4: init_params.camera_resolution = RESOLUTION::SVGA; break;
            case 5: init_params.camera_resolution = RESOLUTION::VGA; break;
            default:
                ROS_WARN("Unknown camera_resolution=%d, using AUTO", camera_resolution_);
                init_params.camera_resolution = RESOLUTION::AUTO;
                break;
        }
        switch (depth_mode_) {
            case 0: init_params.depth_mode = DEPTH_MODE::PERFORMANCE; break;
            case 1: init_params.depth_mode = DEPTH_MODE::QUALITY; break;
            case 2: init_params.depth_mode = DEPTH_MODE::ULTRA; break;
            case 3: init_params.depth_mode = DEPTH_MODE::NEURAL; break;
            default:
                ROS_WARN("Unknown depth_mode=%d, using PERFORMANCE", depth_mode_);
                init_params.depth_mode = DEPTH_MODE::PERFORMANCE;
                break;
        }
        init_params.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
        init_params.coordinate_units = UNIT::METER;
        init_params.open_timeout_sec = open_timeout_sec_;
        if (camera_fps_ > 0) {
            init_params.camera_fps = camera_fps_;
        }
        if (camera_serial_number_ > 0) {
            init_params.input.setFromSerialNumber(static_cast<unsigned>(camera_serial_number_));
        }

        ROS_INFO("Opening ZED: resolution=%s depth=%s",
                 toString(init_params.camera_resolution).c_str(),
                 toString(init_params.depth_mode).c_str());

        ERROR_CODE err = zed_.open(init_params);
        if (err != ERROR_CODE::SUCCESS) {
            ROS_ERROR("Failed to open ZED camera: %s", toString(err).c_str());
            return false;
        }

        auto camera_info = zed_.getCameraInformation();
        ROS_INFO("ZED opened: %s SN=%d",
                 toString(camera_info.camera_model).c_str(), camera_info.serial_number);

        auto camera_config = camera_info.camera_configuration;
        image_left_ = Mat(camera_config.resolution.width, camera_config.resolution.height, MAT_TYPE::U8_C4);
        image_left_ocv_ = cv::Mat(camera_config.resolution.height, camera_config.resolution.width, CV_8UC4);
        depth_map_ = Mat(camera_config.resolution.width, camera_config.resolution.height, MAT_TYPE::F32_C1);

        // CameraInfo (left) — dùng YOLO bridge back-project depth → 3D
        const auto& cal = camera_config.calibration_parameters.left_cam;
        camera_info_msg_.header.frame_id = camera_frame_id_;
        camera_info_msg_.height = camera_config.resolution.height;
        camera_info_msg_.width = camera_config.resolution.width;
        camera_info_msg_.distortion_model = "plumb_bob";
        camera_info_msg_.D = {cal.disto[0], cal.disto[1], cal.disto[2], cal.disto[3], cal.disto[4]};
        camera_info_msg_.K = {cal.fx, 0.0, cal.cx, 0.0, cal.fy, cal.cy, 0.0, 0.0, 1.0};
        camera_info_msg_.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
        camera_info_msg_.P = {cal.fx, 0.0, cal.cx, 0.0, 0.0, cal.fy, cal.cy, 0.0, 0.0, 0.0, 1.0, 0.0};
        camera_info_ready_ = true;
        ROS_INFO("CameraInfo: %ux%u fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
                 camera_info_msg_.width, camera_info_msg_.height, cal.fx, cal.fy, cal.cx, cal.cy);
        return true;
    }

    void configureRuntimeParameters() {
        // Explicit — do not rely on silent SDK default (CAMERA).
        if (measure3d_reference_frame_ == "world") {
            runtime_params_.measure3D_reference_frame = REFERENCE_FRAME::WORLD;
            if (bodies_frame_id_.empty()) {
                bodies_frame_id_ = "zed_world";
            }
            ROS_WARN(
                "BodyData in ZED WORLD frame (header.frame_id=%s). "
                "This is NOT ROS map; fusion must NOT treat it as camera_link.",
                bodies_frame_id_.c_str());
        } else {
            if (measure3d_reference_frame_ != "camera") {
                ROS_WARN("Unknown measure3d_reference_frame='%s', using camera",
                         measure3d_reference_frame_.c_str());
                measure3d_reference_frame_ = "camera";
            }
            runtime_params_.measure3D_reference_frame = REFERENCE_FRAME::CAMERA;
            if (bodies_frame_id_.empty()) {
                bodies_frame_id_ = camera_frame_id_;
            }
            ROS_INFO(
                "BodyData in CAMERA reference (header.frame_id=%s). "
                "Velocity is camera-relative; robot-motion compensation required for map-frame speed.",
                bodies_frame_id_.c_str());
        }
    }

    bool enablePositionalTracking() {
        PositionalTrackingParameters tracking_params;
        tracking_params.set_floor_as_origin = true;
        // Body Tracking + VIO đồng thời: giữ area_memory theo SDK/param an toàn.
        // Ép area_memory=false đã gây SIGSEGV ngay sau getPosition() trên setup này.
        if (publish_vio_odom_) {
            if (!enable_area_memory_) {
                ROS_WARN(
                    "enable_area_memory:=false không ổn định với Body Tracking trên máy này; "
                    "ép true để tránh crash. Tắt area_memory chỉ khi chạy VIO không body.");
                enable_area_memory_ = true;
            }
            tracking_params.enable_area_memory = enable_area_memory_;
            tracking_params.enable_imu_fusion = enable_imu_fusion_;
        }
        ERROR_CODE err = zed_.enablePositionalTracking(tracking_params);
        if (err != ERROR_CODE::SUCCESS) {
            ROS_ERROR("Failed to enable positional tracking: %s", toString(err).c_str());
            return false;
        }
        if (publish_vio_odom_) {
            ROS_INFO(
                "Positional tracking (ZED VIO) enabled: floor_origin=true area_memory=%s imu_fusion=%s vio_hz=%.1f",
                enable_area_memory_ ? "true" : "false",
                enable_imu_fusion_ ? "true" : "false",
                vio_publish_hz_);
        } else {
            ROS_INFO(
                "Positional tracking enabled (before body tracking; does not change measure3D frame alone)");
        }
        return true;
    }

    bool enableBodyTracking() {
        BodyTrackingParameters body_params;
        if (detection_model_str_ == "HUMAN_BODY_ACCURATE") {
            body_params.detection_model = BODY_TRACKING_MODEL::HUMAN_BODY_ACCURATE;
        } else if (detection_model_str_ == "HUMAN_BODY_MEDIUM") {
            body_params.detection_model = BODY_TRACKING_MODEL::HUMAN_BODY_MEDIUM;
        } else {
            body_params.detection_model = BODY_TRACKING_MODEL::HUMAN_BODY_FAST;
        }

        body_params.enable_tracking = enable_tracking_;
        body_params.enable_body_fitting = enable_body_fitting_;
        body_params.enable_segmentation = enable_segmentation_;
        body_params.body_format = (body_format_ == 18) ? BODY_FORMAT::BODY_18 : BODY_FORMAT::BODY_34;
        body_params.instance_module_id = body_instance_id_;

        ERROR_CODE err = zed_.enableBodyTracking(body_params);
        if (err != ERROR_CODE::SUCCESS) {
            ROS_ERROR("Failed to enable body tracking: %s", toString(err).c_str());
            return false;
        }

        body_rt_.detection_confidence_threshold = detection_confidence_threshold_;
        body_rt_.skeleton_smoothing = skeleton_smoothing_;

        ROS_INFO("Body tracking ON: model=%s format=%d instance_id=%u",
                 detection_model_str_.c_str(), body_format_, body_instance_id_);
        return true;
    }

    bool enableObjectDetection() {
        ObjectDetectionParameters od_params;
        if (od_model_str_ == "MULTI_CLASS_BOX_MEDIUM") {
            od_params.detection_model = OBJECT_DETECTION_MODEL::MULTI_CLASS_BOX_MEDIUM;
        } else if (od_model_str_ == "MULTI_CLASS_BOX_ACCURATE") {
            od_params.detection_model = OBJECT_DETECTION_MODEL::MULTI_CLASS_BOX_ACCURATE;
        } else {
            od_params.detection_model = OBJECT_DETECTION_MODEL::MULTI_CLASS_BOX_FAST;
        }

        od_params.enable_tracking = od_enable_tracking_ && enable_tracking_;
        od_params.enable_segmentation = false;
        od_params.max_range = od_max_range_;
        od_params.instance_module_id = od_instance_id_;

        ERROR_CODE err = zed_.enableObjectDetection(od_params);
        if (err != ERROR_CODE::SUCCESS) {
            ROS_ERROR("Failed to enable object detection: %s", toString(err).c_str());
            return false;
        }

        od_rt_.detection_confidence_threshold = od_confidence_threshold_;
        // Optional per-class filtering via object_class_filter if API available
        tryConfigureOdClassFilter();

        object_detection_enabled_ = true;
        ROS_INFO("Object detection ON: model=%s instance_id=%u conf=%.1f max_range=%.1f filter_person=%s",
                 od_model_str_.c_str(), od_instance_id_, od_confidence_threshold_, od_max_range_,
                 od_filter_person_ ? "true" : "false");
        return true;
    }

    void tryConfigureOdClassFilter() {
        // Build allowed class list from params
        std::vector<OBJECT_CLASS> allowed;
        if (od_mc_people_ && !od_filter_person_) {
            allowed.push_back(OBJECT_CLASS::PERSON);
        }
        if (od_mc_vehicle_) allowed.push_back(OBJECT_CLASS::VEHICLE);
        if (od_mc_bag_) allowed.push_back(OBJECT_CLASS::BAG);
        if (od_mc_animal_) allowed.push_back(OBJECT_CLASS::ANIMAL);
        if (od_mc_electronics_) allowed.push_back(OBJECT_CLASS::ELECTRONICS);
        if (od_mc_fruit_vegetable_) allowed.push_back(OBJECT_CLASS::FRUIT_VEGETABLE);
        if (od_mc_sport_) allowed.push_back(OBJECT_CLASS::SPORT);

        if (!allowed.empty()) {
            od_rt_.object_class_filter = allowed;
            ROS_INFO("OD class filter: %zu classes enabled", allowed.size());
        }
    }

    bool isOdClassEnabled(OBJECT_CLASS cls) const {
        switch (cls) {
            case OBJECT_CLASS::PERSON:
                return od_mc_people_ && !od_filter_person_;
            case OBJECT_CLASS::VEHICLE: return od_mc_vehicle_;
            case OBJECT_CLASS::BAG: return od_mc_bag_;
            case OBJECT_CLASS::ANIMAL: return od_mc_animal_;
            case OBJECT_CLASS::ELECTRONICS: return od_mc_electronics_;
            case OBJECT_CLASS::FRUIT_VEGETABLE: return od_mc_fruit_vegetable_;
            case OBJECT_CLASS::SPORT: return od_mc_sport_;
            default: return false;
        }
    }

    void publishImu(const ros::Time& stamp) {
        if (!publish_imu_) {
            return;
        }
        SensorsData sensors;
        // CURRENT: gyro/accel đầy đủ. IMAGE chỉ chắc quaternion (SDK warning: các giá trị khác có thể = 0).
        const ERROR_CODE err = zed_.getSensorsData(sensors, TIME_REFERENCE::CURRENT);
        if (err != ERROR_CODE::SUCCESS) {
            return;
        }
        sensor_msgs::Imu imu_msg;
        // Dùng stamp cảm biến nếu có; fallback stamp ảnh/body.
        imu_msg.header.stamp = stamp;
        if (sensors.imu.timestamp.data_ns > 0) {
            imu_msg.header.stamp = ros::Time().fromNSec(static_cast<uint64_t>(sensors.imu.timestamp.data_ns));
        }
        imu_msg.header.frame_id = camera_frame_id_;
        const auto& imu = sensors.imu;
        imu_msg.orientation.x = imu.pose.getOrientation().x;
        imu_msg.orientation.y = imu.pose.getOrientation().y;
        imu_msg.orientation.z = imu.pose.getOrientation().z;
        imu_msg.orientation.w = imu.pose.getOrientation().w;
        // ZED SDK: angular_velocity = deg/s → ROS sensor_msgs/Imu = rad/s
        constexpr double kDegToRad = M_PI / 180.0;
        imu_msg.angular_velocity.x = imu.angular_velocity.x * kDegToRad;
        imu_msg.angular_velocity.y = imu.angular_velocity.y * kDegToRad;
        imu_msg.angular_velocity.z = imu.angular_velocity.z * kDegToRad;
        // linear_acceleration đã là m/s²
        imu_msg.linear_acceleration.x = imu.linear_acceleration.x;
        imu_msg.linear_acceleration.y = imu.linear_acceleration.y;
        imu_msg.linear_acceleration.z = imu.linear_acceleration.z;
        // Covariance: tin wx/wy/wz + ax/ay; roll/pitch orientation ổn (gravity); yaw kém (không mag).
        for (size_t i = 0; i < imu_msg.orientation_covariance.size(); ++i) {
            imu_msg.orientation_covariance[i] = 0.0;
            imu_msg.angular_velocity_covariance[i] = 0.0;
            imu_msg.linear_acceleration_covariance[i] = 0.0;
        }
        imu_msg.orientation_covariance[0] = 0.04;   // roll
        imu_msg.orientation_covariance[4] = 0.04;   // pitch
        imu_msg.orientation_covariance[8] = 0.25;   // yaw — không fuse absolute
        imu_msg.angular_velocity_covariance[0] = 0.02;   // wx (không fuse 2D)
        imu_msg.angular_velocity_covariance[4] = 0.02;   // wy
        // wz: R cao hơn để bias ~0.02 rad/s không kéo EKF xoay khi đứng yên
        imu_msg.angular_velocity_covariance[8] = 0.06;
        imu_msg.linear_acceleration_covariance[0] = 0.1984; // ax
        imu_msg.linear_acceleration_covariance[4] = 0.1984; // ay
        imu_msg.linear_acceleration_covariance[8] = 0.30; // az (không fuse)
        imu_pub_.publish(imu_msg);
    }

    void publishVioOdom(const ros::Time& stamp) {
        if (!publish_vio_odom_ || !positional_tracking_enabled_) {
            return;
        }
        if (vio_publish_hz_ > 0.0 && !last_vio_pub_.isZero() &&
            (stamp - last_vio_pub_).toSec() < (1.0 / vio_publish_hz_)) {
            return;
        }

        Pose camera_pose_world;
        const POSITIONAL_TRACKING_STATE state =
            zed_.getPosition(camera_pose_world, REFERENCE_FRAME::WORLD);

        if (state == POSITIONAL_TRACKING_STATE::OK ||
            state == POSITIONAL_TRACKING_STATE::SEARCHING ||
            state == POSITIONAL_TRACKING_STATE::FPS_TOO_LOW) {
            ++vio_ok_count_;
        } else {
            ++vio_lost_count_;
            ROS_WARN_THROTTLE(
                2.0,
                "ZED VIO tracking state=%s (ok=%llu lost=%llu)",
                toString(state).c_str(),
                static_cast<unsigned long long>(vio_ok_count_),
                static_cast<unsigned long long>(vio_lost_count_));
            // P7: still publish reliability=0 when lost
            {
                std_msgs::Int8 st;
                st.data = static_cast<int8_t>(static_cast<int>(state));
                vio_state_pub_.publish(st);
                std_msgs::Float32 rel;
                rel.data = 0.0f;
                vio_reliability_pub_.publish(rel);
            }
            return;
        }

        // P7 reliability from tracking state
        {
            float reliab = 0.0f;
            if (state == POSITIONAL_TRACKING_STATE::OK)
                reliab = 1.0f;
            else if (state == POSITIONAL_TRACKING_STATE::SEARCHING)
                reliab = 0.5f;
            else if (state == POSITIONAL_TRACKING_STATE::FPS_TOO_LOW)
                reliab = 0.3f;
            std_msgs::Int8 st;
            st.data = static_cast<int8_t>(static_cast<int>(state));
            vio_state_pub_.publish(st);
            std_msgs::Float32 rel;
            rel.data = reliab;
            vio_reliability_pub_.publish(rel);
        }

        const sl::Translation t = camera_pose_world.getTranslation();
        const sl::Orientation q = camera_pose_world.getOrientation();
        const double qx = static_cast<double>(q.x);
        const double qy = static_cast<double>(q.y);
        const double qz = static_cast<double>(q.z);
        const double qw = static_cast<double>(q.w);
        if (qx == 0.0 && qy == 0.0 && qz == 0.0 && qw == 0.0) {
            return;
        }

        tf2::Transform T_world_camera;
        T_world_camera.setOrigin(tf2::Vector3(t.x, t.y, t.z));
        T_world_camera.setRotation(tf2::Quaternion(qx, qy, qz, qw));

        if (!vio_origin_set_) {
            T_world_camera_origin_inv_ = T_world_camera.inverse();
            vio_origin_set_ = true;
            ROS_INFO("ZED VIO origin set (relative odom from first valid pose)");
        }

        const tf2::Transform T_odom_camera = T_world_camera_origin_inv_ * T_world_camera;
        T_odom_base_ = T_odom_camera * T_base_camera_.inverse();

        double yaw = 0.0;
        if (vio_two_d_mode_) {
            tf2::Vector3 origin = T_odom_base_.getOrigin();
            origin.setZ(fixed_z_value_);
            T_odom_base_.setOrigin(origin);
            double roll, pitch;
            tf2::Matrix3x3(T_odom_base_.getRotation()).getRPY(roll, pitch, yaw);
            tf2::Quaternion q2d;
            q2d.setRPY(0.0, 0.0, yaw);
            T_odom_base_.setRotation(q2d);
        } else {
            double roll, pitch;
            tf2::Matrix3x3(T_odom_base_.getRotation()).getRPY(roll, pitch, yaw);
        }

        nav_msgs::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = odom_frame_id_;
        odom.child_frame_id = base_frame_id_;

        odom.pose.pose.position.x = T_odom_base_.getOrigin().x();
        odom.pose.pose.position.y = T_odom_base_.getOrigin().y();
        odom.pose.pose.position.z = T_odom_base_.getOrigin().z();
        odom.pose.pose.orientation.x = T_odom_base_.getRotation().x();
        odom.pose.pose.orientation.y = T_odom_base_.getRotation().y();
        odom.pose.pose.orientation.z = T_odom_base_.getRotation().z();
        odom.pose.pose.orientation.w = T_odom_base_.getRotation().w();

        // Covariance 2D — P6 (2026-08-05): nới R_xy VIO để EKF tin SRF hơn trên T1
        // (P5: VIO T1 return biến thiên mạnh; R cũ 0.0467 < SRF 0.081 → over-trust VIO).
        for (size_t i = 0; i < odom.pose.covariance.size(); ++i) {
            odom.pose.covariance[i] = 0.0;
        }
        odom.pose.covariance[0] = 0.20;     // x
        odom.pose.covariance[7] = 0.20;     // y
        odom.pose.covariance[14] = 1e-3;  // z
        odom.pose.covariance[21] = 1e-3;  // roll
        odom.pose.covariance[28] = 1e-3;  // pitch
        odom.pose.covariance[35] = 0.18;    // yaw (hơi nới vs 0.15)

        double vx = 0.0, vy = 0.0, wz = 0.0;
        if (have_last_base_pose_ && !last_vio_stamp_.isZero()) {
            const double dt = (stamp - last_vio_stamp_).toSec();
            if (dt > 1e-4 && dt < 1.0) {
                const tf2::Vector3 cur = T_odom_base_.getOrigin();
                const double dx = cur.x() - last_base_origin_.x();
                const double dy = cur.y() - last_base_origin_.y();
                const double c = std::cos(last_base_yaw_);
                const double s = std::sin(last_base_yaw_);
                vx = (c * dx + s * dy) / dt;
                vy = (-s * dx + c * dy) / dt;
                double dyaw = yaw - last_base_yaw_;
                while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
                while (dyaw < -M_PI) dyaw += 2.0 * M_PI;
                wz = dyaw / dt;
                last_base_yaw_ = yaw;
                last_base_origin_ = cur;
            }
        } else {
            last_base_origin_ = T_odom_base_.getOrigin();
            last_base_yaw_ = yaw;
            have_last_base_pose_ = true;
        }
        last_vio_stamp_ = stamp;
        last_vio_pub_ = stamp;

        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.linear.z = 0.0;
        odom.twist.twist.angular.z = wz;
        for (size_t i = 0; i < odom.twist.covariance.size(); ++i) {
            odom.twist.covariance[i] = 0.0;
        }
        odom.twist.covariance[0] = 0.5;
        odom.twist.covariance[7] = 0.5;
        odom.twist.covariance[35] = 0.5;

        vio_odom_pub_.publish(odom);

        if (publish_vio_tf_ && tf_broadcaster_) {
            geometry_msgs::TransformStamped tf_msg;
            tf_msg.header.stamp = stamp;
            tf_msg.header.frame_id = odom_frame_id_;
            tf_msg.child_frame_id = base_frame_id_;
            tf_msg.transform.translation.x = odom.pose.pose.position.x;
            tf_msg.transform.translation.y = odom.pose.pose.position.y;
            tf_msg.transform.translation.z = odom.pose.pose.position.z;
            tf_msg.transform.rotation = odom.pose.pose.orientation;
            tf_broadcaster_->sendTransform(tf_msg);
        }

        ROS_INFO_THROTTLE(
            5.0,
            "ZED VIO: x=%.3f y=%.3f yaw=%.3f state=%s",
            odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            last_base_yaw_,
            toString(state).c_str());
    }

    void publishBodies(const Bodies& bodies, const std_msgs::Header& header) {
        zed_sdk_bridge::BodiesStamped bodies_msg;
        bodies_msg.header = header;
        bodies_msg.is_tracked = bodies.is_tracked;

        for (const auto& body : bodies.body_list) {
            zed_sdk_bridge::Body body_msg;
            body_msg.id = body.id;
            body_msg.confidence = body.confidence;
            body_msg.position[0] = body.position.x;
            body_msg.position[1] = body.position.y;
            body_msg.position[2] = body.position.z;
            body_msg.velocity[0] = body.velocity.x;
            body_msg.velocity[1] = body.velocity.y;
            body_msg.velocity[2] = body.velocity.z;
            body_msg.dimensions[0] = body.dimensions.x;
            body_msg.dimensions[1] = body.dimensions.y;
            body_msg.dimensions[2] = body.dimensions.z;
            body_msg.tracking_state = static_cast<int8_t>(body.tracking_state);
            body_msg.action_state = static_cast<int8_t>(body.action_state);

            if (body.bounding_box_2d.size() >= 4) {
                body_msg.bounding_box_2d_tl[0] = body.bounding_box_2d[0].x;
                body_msg.bounding_box_2d_tl[1] = body.bounding_box_2d[0].y;
                body_msg.bounding_box_2d_br[0] = body.bounding_box_2d[2].x;
                body_msg.bounding_box_2d_br[1] = body.bounding_box_2d[2].y;
            }

            body_msg.num_keypoints = body.keypoint.size();
            body_msg.body_format = body_format_;
            body_msg.keypoints_2d.clear();
            for (const auto& kp : body.keypoint_2d) {
                body_msg.keypoints_2d.push_back(kp.x);
                body_msg.keypoints_2d.push_back(kp.y);
            }
            body_msg.keypoints_3d.clear();
            for (const auto& kp : body.keypoint) {
                body_msg.keypoints_3d.push_back(kp.x);
                body_msg.keypoints_3d.push_back(kp.y);
                body_msg.keypoints_3d.push_back(kp.z);
            }
            bodies_msg.bodies.push_back(body_msg);
        }
        bodies_pub_.publish(bodies_msg);
    }

    void publishObjects(const Objects& objects, const std_msgs::Header& header) {
        zed_sdk_bridge::ObjectsStamped objects_msg;
        objects_msg.header = header;
        objects_msg.is_tracked = objects.is_tracked;

        for (const auto& obj : objects.object_list) {
            if (!isOdClassEnabled(obj.label)) {
                continue;
            }

            zed_sdk_bridge::Object obj_msg;
            obj_msg.id = obj.id;
            obj_msg.label = objectClassToString(obj.label);
            obj_msg.label_id = static_cast<int8_t>(obj.label);
            obj_msg.confidence = obj.confidence;
            obj_msg.position[0] = obj.position.x;
            obj_msg.position[1] = obj.position.y;
            obj_msg.position[2] = obj.position.z;
            obj_msg.velocity[0] = obj.velocity.x;
            obj_msg.velocity[1] = obj.velocity.y;
            obj_msg.velocity[2] = obj.velocity.z;
            obj_msg.dimensions[0] = obj.dimensions.x;
            obj_msg.dimensions[1] = obj.dimensions.y;
            obj_msg.dimensions[2] = obj.dimensions.z;
            obj_msg.tracking_state = static_cast<int8_t>(obj.tracking_state);
            obj_msg.action_state = static_cast<int8_t>(obj.action_state);

            if (obj.bounding_box_2d.size() >= 4) {
                obj_msg.bounding_box_2d_tl[0] = obj.bounding_box_2d[0].x;
                obj_msg.bounding_box_2d_tl[1] = obj.bounding_box_2d[0].y;
                obj_msg.bounding_box_2d_br[0] = obj.bounding_box_2d[2].x;
                obj_msg.bounding_box_2d_br[1] = obj.bounding_box_2d[2].y;
            }
            objects_msg.objects.push_back(obj_msg);
        }
        objects_pub_.publish(objects_msg);
    }

    void publishImage(const Mat& image, const std_msgs::Header& header) {
        cv::Mat cv_image(image.getHeight(), image.getWidth(), CV_8UC4, image.getPtr<sl::uchar1>(MEM::CPU));
        sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(header, "bgra8", cv_image).toImageMsg();
        image_pub_.publish(img_msg);
    }

    void publishDepth(const Mat& depth, const std_msgs::Header& header) {
        cv::Mat cv_depth(depth.getHeight(), depth.getWidth(), CV_32FC1, depth.getPtr<float>(MEM::CPU));
        sensor_msgs::ImagePtr depth_msg = cv_bridge::CvImage(header, "32FC1", cv_depth).toImageMsg();
        depth_pub_.publish(depth_msg);
    }

    void publishCameraInfo(const std_msgs::Header& header) {
        if (!camera_info_ready_) return;
        camera_info_msg_.header = header;
        camera_info_pub_.publish(camera_info_msg_);
    }

    void run() {
        ros::Rate rate(30);

        while (ros::ok()) {
            if (!camera_initialized_) {
                if (!setupCamera()) {
                    ROS_WARN_THROTTLE(
                        30.0,
                        "ZED chưa mở được (CAMERA NOT DETECTED?). "
                        "Thử: sudo bash $(rospack find x_omni4wd_bringup)/scripts/fix_zed2i_usb.sh "
                        "| retry sau %.0fs",
                        open_retry_delay_sec_);
                    ros::Duration(open_retry_delay_sec_).sleep();
                    ros::spinOnce();
                    continue;
                }
                ROS_INFO("Starting grab loop (body instance=%u, od=%s)...",
                         body_instance_id_, object_detection_enabled_ ? "ON" : "OFF");
            }

            if (!zed_.isOpened()) {
                ROS_ERROR("Camera closed. Sẽ thử mở lại...");
                shutdownCamera();
                continue;
            }

            ERROR_CODE err = zed_.grab(runtime_params_);
            if (err == ERROR_CODE::SUCCESS) {
                std_msgs::Header header;
                header.frame_id = bodies_frame_id_;
                header.stamp = ros::Time::now();  // temporary; overwritten after retrieveBodies

                err = zed_.retrieveBodies(bodies_, body_rt_, body_instance_id_);
                if (err == ERROR_CODE::SUCCESS && bodies_.is_new) {
                    const uint64_t ns = bodies_.timestamp.getNanoseconds();
                    if (ns > 0) {
                        header.stamp.sec = static_cast<uint32_t>(ns / 1000000000ULL);
                        header.stamp.nsec = static_cast<uint32_t>(ns % 1000000000ULL);
                    } else {
                        header.stamp = ros::Time::now();
                    }
                    publishBodies(bodies_, header);
                    if (!bodies_.body_list.empty()) {
                        ROS_INFO_THROTTLE(
                            1.0, "Bodies: %zu | ref=%s frame=%s",
                            bodies_.body_list.size(),
                            measure3d_reference_frame_.c_str(),
                            header.frame_id.c_str());
                    }
                } else if (err != ERROR_CODE::SUCCESS) {
                    ROS_WARN_THROTTLE(5.0, "retrieveBodies failed: %s", toString(err).c_str());
                }

                // VIO/IMU chỉ khi bật — không xen vào đường body baseline
                if (publish_vio_odom_ || publish_imu_) {
                    publishVioOdom(header.stamp);
                    publishImu(header.stamp);
                }

                if (object_detection_enabled_) {
                    err = zed_.retrieveObjects(objects_, od_rt_, od_instance_id_);
                    if (err == ERROR_CODE::SUCCESS && objects_.is_new) {
                        publishObjects(objects_, header);
                        if (!objects_.object_list.empty()) {
                            ROS_INFO_THROTTLE(1.0, "Objects raw: %zu", objects_.object_list.size());
                        }
                    } else if (err != ERROR_CODE::SUCCESS) {
                        ROS_WARN_THROTTLE(5.0, "retrieveObjects failed: %s", toString(err).c_str());
                    }
                }

                // Giảm băng thông: không publish RGB/depth mỗi frame grab (~30Hz)
                const ros::Time now = header.stamp;
                const bool want_rgb =
                    (rgb_publish_hz_ <= 0.0) ||
                    last_rgb_pub_.isZero() ||
                    (now - last_rgb_pub_).toSec() >= (1.0 / rgb_publish_hz_);
                const bool want_depth =
                    publish_depth_ &&
                    ((depth_publish_hz_ <= 0.0) ||
                     last_depth_pub_.isZero() ||
                     (now - last_depth_pub_).toSec() >= (1.0 / depth_publish_hz_));

                if (want_rgb || want_depth) {
                    if (want_rgb) {
                        zed_.retrieveImage(image_left_, VIEW::LEFT, MEM::CPU);
                        publishImage(image_left_, header);
                        publishCameraInfo(header);
                        last_rgb_pub_ = now;
                    }
                    if (want_depth) {
                        err = zed_.retrieveMeasure(depth_map_, MEASURE::DEPTH, MEM::CPU);
                        if (err == ERROR_CODE::SUCCESS) {
                            publishDepth(depth_map_, header);
                            last_depth_pub_ = now;
                        } else {
                            ROS_WARN_THROTTLE(5.0, "retrieveMeasure DEPTH failed: %s", toString(err).c_str());
                        }
                    }
                }
            } else if (err == ERROR_CODE::END_OF_SVOFILE_REACHED) {
                zed_.setSVOPosition(0);
            } else {
                ROS_WARN_THROTTLE(5.0, "grab failed: %s", toString(err).c_str());
            }

            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "zed_sdk_bridge_node");
    try {
        ZedSdkBridgeNode node;
        node.run();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
        return -1;
    }
    return 0;
}
