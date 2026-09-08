///////////////////////////////////////////////////////////////////////////
//
// ZED Body Tracking ROS Node (+ optional ZED Object Detection)
// Object Detection mặc định tắt — dùng Ultralytics YOLO → /objects thay thế.
// Publish thêm /depth + /camera_info cho YOLO bridge.
//
///////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Header.h>

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
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;
    image_transport::Publisher depth_pub_;

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
          camera_info_ready_(false) {
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
        const bool need_pt =
            enable_tracking_ ||
            (measure3d_reference_frame_ == "world") ||
            enable_positional_tracking_;
        if (measure3d_reference_frame_ == "world" && !enable_tracking_) {
            ROS_ERROR("measure3d_reference_frame=world requires enable_tracking:=true");
            shutdownCamera();
            return false;
        }
        if (enable_tracking_ && !enable_positional_tracking_) {
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
            ROS_INFO("Positional tracking OFF (enable_tracking=false, CAMERA mode)");
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
        ERROR_CODE err = zed_.enablePositionalTracking(tracking_params);
        if (err != ERROR_CODE::SUCCESS) {
            ROS_ERROR("Failed to enable positional tracking: %s", toString(err).c_str());
            return false;
        }
        ROS_INFO("Positional tracking enabled (before body tracking; does not change measure3D frame alone)");
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
