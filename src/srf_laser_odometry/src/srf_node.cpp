/*********************************************************************
*
* Software License Agreement (GPLv3 License)
*
*  Authors: Mariano Jaimez Tarifa and Javier Monroy
*           MAPIR group, University of Malaga, Spain
*           http://mapir.uma.es
*
*  Date: January 2016
*
* This pkgs offers a fast and reliable estimation of 2D odometry based on planar laser scans.
* SRF is a fast and precise method to estimate the planar motion of a lidar from consecutive range scans.
* SRF presents a dense method for estimating planar motion with a laser scanner. Starting from a symmetric
* representation of geometric consistency between scans, we derive a precise range flow constraint and
* express the motion of the scan observations as a function of the rigid motion of the scanner.
* In contrast to existing techniques, which align the incoming scan with either the previous one or the last
* selected keyscan, we propose a combined and efficient formulation to jointly align all these three scans at
* every iteration. This new formulation preserves the advantages of keyscan-based strategies but is more robust
* against suboptimal selection of keyscans and the presence of moving objects.
*
*  More Info: http://mapir.isa.uma.es/work/SRF-Odometry
*
*  VCCA2026: ROS Noetic / MRPT 2.x port (mrpt_bridge removed).
*********************************************************************/

#include "srf_node.h"

#include <algorithm>
#include <cmath>

using namespace mrpt::poses;
using namespace std;
using namespace Eigen;

namespace
{
mrpt::poses::CPose3D rosPoseToMrpt(const geometry_msgs::Pose& pose)
{
    tf::Quaternion quaternion;
    tf::quaternionMsgToTF(pose.orientation, quaternion);

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

    // MRPT CPose3D(x,y,z, yaw,pitch,roll)
    return mrpt::poses::CPose3D(
        pose.position.x,
        pose.position.y,
        pose.position.z,
        yaw,
        pitch,
        roll);
}

bool tfToMrptPose3D(const tf::StampedTransform& transform, mrpt::poses::CPose3D& out)
{
    const tf::Vector3& t = transform.getOrigin();
    const tf::Matrix3x3& basis = transform.getBasis();
    mrpt::math::CMatrixDouble33 R;
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            R(r, c) = basis[r][c];

    mrpt::math::CVectorFixedDouble<3> xyz;
    xyz[0] = t[0];
    xyz[1] = t[1];
    xyz[2] = t[2];
    out = mrpt::poses::CPose3D(R, xyz);
    return true;
}
}  // namespace


// --------------------------
// CLaserOdometry2D Wrapper
//---------------------------
CLaserOdometry2D::CLaserOdometry2D()
    : laser_x_on_base_(0.0),
      laser_y_on_base_(0.0),
      laser_yaw_on_base_(0.0),
      laser_extrinsics_valid_(false),
      pose_cov_x_(0.04),
      pose_cov_y_(0.04),
      pose_cov_yaw_(0.05),
      twist_cov_vx_(0.08),
      twist_cov_vy_(0.08),
      twist_cov_wz_(0.06)
{
    ROS_INFO("[SRF] Initializing SRF_node...");

    //Read Parameters
    //----------------
    ros::NodeHandle pn("~");
    pn.param<std::string>("laser_scan_topic", laser_scan_topic, "/laser_scan");
    pn.param<bool>("publish_tf", publish_tf, true);
    pn.param<bool>("publish_diagnostics", publish_diagnostics_, true);
    pn.param<double>("rel_cov_scale", rel_cov_scale_, 5.0);   // trace(cov) scale → reliability
    pn.param<double>("rel_cond_scale", rel_cond_scale_, 50.0); // cond number scale
    pn.param<std::string>("base_frame_id", base_frame_id, "/base_link");
    pn.param<std::string>("odom_topic", odom_topic, "/odom");
    pn.param<std::string>("odom_frame_id", odom_frame_id, "/odom");
    pn.param<std::string>("init_pose_from_topic", init_pose_from_topic, "");
    pn.param<int>("laser_decimation", laser_decimation, 1);
    pn.param<double>("laser_min_range", laser_min_range, -1.0);
    pn.param<double>("laser_max_range", laser_max_range, -1.0);

    pn.param<std::string>("operation_mode", operation_mode, "HYBRID");  //CS=consecutiveScans, KS=keyScans, HYBRID=threeScansWithKeyScan

    // Covariance defaults (avoid all-zero → EKF over-trust)
    pn.param<double>("pose_cov_x", pose_cov_x_, pose_cov_x_);
    pn.param<double>("pose_cov_y", pose_cov_y_, pose_cov_y_);
    pn.param<double>("pose_cov_yaw", pose_cov_yaw_, pose_cov_yaw_);
    pn.param<double>("twist_cov_vx", twist_cov_vx_, twist_cov_vx_);
    pn.param<double>("twist_cov_vy", twist_cov_vy_, twist_cov_vy_);
    pn.param<double>("twist_cov_wz", twist_cov_wz_, twist_cov_wz_);

    //Publishers and Subscribers
    //--------------------------
    laser_sub = n.subscribe<sensor_msgs::LaserScan>(laser_scan_topic, 1, &CLaserOdometry2D::LaserCallBack, this);
    odom_pub = pn.advertise<nav_msgs::Odometry>(odom_topic, 5);
    laser_pub = pn.advertise<sensor_msgs::LaserScan>("srf_laser_truncated", 5);
    if (publish_diagnostics_) {
        srf_diag_pub_ = n.advertise<std_msgs::Float32MultiArray>("/srf_diagnostics", 5);
        srf_reliability_pub_ = n.advertise<std_msgs::Float32>("/srf_reliability", 5);
    }

    //init pose
    //----------
    if (init_pose_from_topic != "")
    {
        initPose_sub = n.subscribe<nav_msgs::Odometry>(init_pose_from_topic, 1, &CLaserOdometry2D::initPoseCallBack, this);
        GT_pose_initialized = false;
    }
    else
    {
        GT_pose_initialized = true;
        initial_robot_pose.pose.pose.position.x = 0.0;
        initial_robot_pose.pose.pose.position.y = 0.0;
        initial_robot_pose.pose.pose.position.z = 0.0;
        initial_robot_pose.pose.pose.orientation.x = 0.0;
        initial_robot_pose.pose.pose.orientation.y = 0.0;
        initial_robot_pose.pose.pose.orientation.z = 0.0;
        initial_robot_pose.pose.pose.orientation.w = 1.0;
    }

    //Init variables
    //----------------
    module_initialized = false;
    first_laser_scan = true;
    new_scan_available = false;
    laser_counter = 0;
}


CLaserOdometry2D::~CLaserOdometry2D()
{
}


bool CLaserOdometry2D::is_initialized()
{
    return module_initialized;
}


bool CLaserOdometry2D::scan_available()
{
    return new_scan_available;
}


bool CLaserOdometry2D::Init()
{
    ROS_INFO("[SRF] Got first Laser Scan .... Configuring node");
    const unsigned int scan_size = last_scan.ranges.size();             // Num of samples (size) of the scan laser
    const float fov = fabs(last_scan.angle_max - last_scan.angle_min);  // Horizontal Laser's FOV
    const float angle_min = static_cast<float>(last_scan.angle_min);

    //Init core class with Laser specs and corresponding operation_mode
    // Pass angle_min so RPLidar 0..2pi scans are not assumed symmetric about 0
    if (!strcmp(operation_mode.c_str(), "CS"))
        srf_obj.initialize(scan_size, fov, 0, angle_min);
    else if (!strcmp(operation_mode.c_str(), "KS"))
        srf_obj.initialize(scan_size, fov, 1, angle_min);
    else if (!strcmp(operation_mode.c_str(), "HYBRID"))
        srf_obj.initialize(scan_size, fov, 2, angle_min);
    else
    {
        ROS_ERROR("[srf] Operation mode not implemented. Using default HYBRID");
        srf_obj.initialize(scan_size, fov, 2, angle_min);
    }
    ROS_INFO("[SRF] scan_size=%u fov=%.4f angle_min=%.4f mode=%s",
             scan_size, fov, angle_min, operation_mode.c_str());

    //Set laser pose on the robot (through tF)
    // This allow estimation of the odometry with respect to the robot base reference system.
    // Patch A: prefer scan stamp; fall back to Time(0) for static extrinsics.
    tf::StampedTransform transform;
    try
    {
        tf_listener.waitForTransform(base_frame_id, last_scan.header.frame_id,
                                     last_scan.header.stamp, ros::Duration(0.2));
        tf_listener.lookupTransform(base_frame_id, last_scan.header.frame_id,
                                    last_scan.header.stamp, transform);
    }
    catch (tf::TransformException&)
    {
        try
        {
            tf_listener.lookupTransform(base_frame_id, last_scan.header.frame_id, ros::Time(0), transform);
        }
        catch (tf::TransformException& ex)
        {
            ROS_WARN_THROTTLE(1.0, "[SRF] TF lookup failed (%s -> %s): %s",
                              base_frame_id.c_str(), last_scan.header.frame_id.c_str(), ex.what());
            return false;
        }
    }

    mrpt::poses::CPose3D LaserPoseOnTheRobot;
    tfToMrptPose3D(transform, LaserPoseOnTheRobot);

    // Cache extrinsics for twist rotate + lever-arm (base ← laser)
    laser_x_on_base_ = LaserPoseOnTheRobot.x();
    laser_y_on_base_ = LaserPoseOnTheRobot.y();
    laser_yaw_on_base_ = LaserPoseOnTheRobot.yaw();
    laser_extrinsics_valid_ = true;
    ROS_INFO("[SRF] Laser on base: x=%.4f y=%.4f yaw=%.4f rad (twist→base + lever-arm enabled)",
             laser_x_on_base_, laser_y_on_base_, laser_yaw_on_base_);

    //Robot initial pose (local conversion, no mrpt_bridge)
    const mrpt::poses::CPose3D robotInitialPose =
        rosPoseToMrpt(initial_robot_pose.pose.pose);

    //Set the Laser initial pose = Robot_initial_pose + LaserPoseOnTheRobot
    srf_obj.laser_pose = CPose2D(robotInitialPose + LaserPoseOnTheRobot);
    srf_obj.laser_oldpose = CPose2D(robotInitialPose + LaserPoseOnTheRobot);

    module_initialized = true;
    last_odom_time = last_scan.header.stamp;
    ROS_INFO("[SRF] Configuration Done.");
    return true;
}


void CLaserOdometry2D::laserTwistToBase(double v_lx, double v_ly, double wz,
                                        double& v_bx, double& v_by, double& wz_b) const
{
    // kai_loc / dt is scanner velocity in the laser body frame.
    // 1) Rotate into base_footprint axes (laser yaw on base).
    // 2) Lever-arm: v_L = v_B + ω × r_BL  ⇒
    //    v_Bx = v_Lx + wz * y_L ;  v_By = v_Ly - wz * x_L
    //    (v_Lx,v_Ly after rotation into base-aligned axes).
    wz_b = wz;
    if (!laser_extrinsics_valid_)
    {
        v_bx = v_lx;
        v_by = v_ly;
        return;
    }

    const double c = std::cos(laser_yaw_on_base_);
    const double s = std::sin(laser_yaw_on_base_);
    const double v_lx_b = c * v_lx - s * v_ly;
    const double v_ly_b = s * v_lx + c * v_ly;

    v_bx = v_lx_b + wz * laser_y_on_base_;
    v_by = v_ly_b - wz * laser_x_on_base_;
}



void CLaserOdometry2D::publishPoseFromSRF()
{
    ROS_DEBUG_THROTTLE(1.0, "[SRF] LASERodom = [%f %f %f]",
                       srf_obj.laser_pose.x(), srf_obj.laser_pose.y(), srf_obj.laser_pose.phi());

    // GET ROBOT POSE from LASER POSE
    //--------------------------------
    tf::StampedTransform transform;
    try
    {
        // Patch A: prefer measurement stamp for consistency with odom header
        try
        {
            tf_listener.lookupTransform(last_scan.header.frame_id, base_frame_id,
                                        last_scan.header.stamp, transform);
        }
        catch (tf::TransformException&)
        {
            tf_listener.lookupTransform(last_scan.header.frame_id, base_frame_id, ros::Time(0), transform);
        }
    }
    catch (tf::TransformException& ex)
    {
        ROS_WARN_THROTTLE(1.0, "[SRF] TF lookup failed while publishing (%s -> %s): %s",
                          last_scan.header.frame_id.c_str(), base_frame_id.c_str(), ex.what());
        new_scan_available = false;
        return;
    }

    mrpt::poses::CPose3D LaserPoseOnTheRobot_inv;
    tfToMrptPose3D(transform, LaserPoseOnTheRobot_inv);

    //Compose Transformations
    robot_pose = CPose3D(srf_obj.laser_pose) + LaserPoseOnTheRobot_inv;
    ROS_DEBUG_THROTTLE(1.0, "[SRF] BASEodom = [%f %f %f]", robot_pose.x(), robot_pose.y(), robot_pose.yaw());


    // Estimate linear/angular speeds (mandatory for base_local_planner)
    // last_scan -> the last scan received (practically now)
    // last_odom_time -> The time of the previous scan lasser used to estimate the pose
    //-------------------------------------------------------------------------------------
    double time_inc_sec = (last_scan.header.stamp - last_odom_time).toSec();
    last_odom_time = last_scan.header.stamp;
    if (time_inc_sec <= 0)
        ROS_WARN_THROTTLE(1.0, "[SRF] Time increment between Odom estimation is: %.6f sec", time_inc_sec);
    else
    {
        // Patch B+C: kai_loc is scanner motion → rotate to base + lever-arm compensation
        const double v_lx = static_cast<double>(srf_obj.kai_loc(0)) / time_inc_sec;
        const double v_ly = static_cast<double>(srf_obj.kai_loc(1)) / time_inc_sec;
        const double wz_l = static_cast<double>(srf_obj.kai_loc(2)) / time_inc_sec;
        double lin_speed_x = 0.0;
        double lin_speed_y = 0.0;
        double ang_speed = 0.0;
        laserTwistToBase(v_lx, v_ly, wz_l, lin_speed_x, lin_speed_y, ang_speed);
        robot_oldpose = robot_pose;

        //first, we'll publish the odometry over tf
        //---------------------------------------
        if (publish_tf)
        {
            geometry_msgs::TransformStamped odom_trans;
            odom_trans.header.stamp = last_scan.header.stamp;
            odom_trans.header.frame_id = odom_frame_id;
            odom_trans.child_frame_id = base_frame_id;
            odom_trans.transform.translation.x = robot_pose.x();
            odom_trans.transform.translation.y = robot_pose.y();
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(robot_pose.yaw());
            //send the transform
            odom_broadcaster.sendTransform(odom_trans);
        }

        //next, we'll publish the odometry message over ROS topic
        //-------------------------------------------------------
        nav_msgs::Odometry odom;
        odom.header.stamp = last_scan.header.stamp;
        odom.header.frame_id = odom_frame_id;
        //set the position
        odom.pose.pose.position.x = robot_pose.x();
        odom.pose.pose.position.y = robot_pose.y();
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(robot_pose.yaw());
        //set the velocity (now expressed in base_frame_id)
        odom.child_frame_id = base_frame_id;
        odom.twist.twist.linear.x = lin_speed_x;
        odom.twist.twist.linear.y = lin_speed_y;
        odom.twist.twist.angular.z = ang_speed;

        // Patch D: non-zero covariances (twist after frame change uses base-frame R)
        for (size_t i = 0; i < odom.pose.covariance.size(); ++i)
            odom.pose.covariance[i] = 0.0;
        odom.pose.covariance[0] = pose_cov_x_;
        odom.pose.covariance[7] = pose_cov_y_;
        odom.pose.covariance[14] = 1e-3;
        odom.pose.covariance[21] = 1e-3;
        odom.pose.covariance[28] = 1e-3;
        odom.pose.covariance[35] = pose_cov_yaw_;

        for (size_t i = 0; i < odom.twist.covariance.size(); ++i)
            odom.twist.covariance[i] = 0.0;
        // Approximate: if laser yaw≈0, J≈identity + lever terms; use diagonal R in base
        odom.twist.covariance[0] = twist_cov_vx_;
        odom.twist.covariance[7] = twist_cov_vy_;
        odom.twist.covariance[35] = twist_cov_wz_;

        //publish the message
        odom_pub.publish(odom);

        // P7: expose internal cov_odo → reliability [0,1]
        if (publish_diagnostics_) {
            const Eigen::Matrix3f& C = srf_obj.cov_odo;
            const float trace = C.trace();
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(C);
            float cond = 1.f;
            float e0 = 0.f, e1 = 0.f, e2 = 0.f;
            if (es.info() == Eigen::Success) {
                Eigen::Vector3f ev = es.eigenvalues().cwiseAbs();
                // sort ascending
                std::sort(ev.data(), ev.data() + 3);
                e0 = ev(0); e1 = ev(1); e2 = ev(2);
                const float emin = std::max(e0, 1e-12f);
                cond = e2 / emin;
            }
            const float kai_n = srf_obj.kai_loc.norm();
            // P9: IRLS weight stats (soft-weight health)
            float w_sum = 0.f;
            int w_cnt = 0;
            const int n_w = static_cast<int>(srf_obj.weights_12.size());
            for (int u = 0; u < n_w; ++u) {
                const float w12 = srf_obj.weights_12(u);
                const float w13 = srf_obj.weights_13(u);
                if (w12 > 1e-6f) { w_sum += w12; ++w_cnt; }
                if (w13 > 1e-6f) { w_sum += w13; ++w_cnt; }
            }
            const float mean_w = (w_cnt > 0) ? (w_sum / static_cast<float>(w_cnt)) : 0.f;
            int n_out = 0;
            for (int u = 0; u < static_cast<int>(srf_obj.outliers.size()); ++u) {
                if (srf_obj.outliers(u)) ++n_out;
            }
            const float out_frac = (srf_obj.outliers.size() > 0)
                ? static_cast<float>(n_out) / static_cast<float>(srf_obj.outliers.size())
                : 0.f;

            const float r_cov = 1.f / (1.f + trace / static_cast<float>(rel_cov_scale_));
            const float r_cond = 1.f / (1.f + (cond - 1.f) / static_cast<float>(rel_cond_scale_));
            const float r_w = std::max(0.f, std::min(1.f, mean_w));  // weights already normalized ~[0,1]
            const float r_out = 1.f / (1.f + 4.f * out_frac);
            float reliab = 0.40f * r_cov + 0.25f * r_cond + 0.25f * r_w + 0.10f * r_out;
            if (!std::isfinite(reliab))
                reliab = 0.f;
            reliab = std::max(0.f, std::min(1.f, reliab));

            std_msgs::Float32MultiArray diag;
            diag.layout.dim.resize(1);
            diag.layout.dim[0].label = "srf_diag";
            diag.layout.dim[0].size = 10;
            diag.layout.dim[0].stride = 10;
            diag.data = {
                reliab, trace, cond, kai_n,
                static_cast<float>(srf_obj.num_valid_range), e0, e1, e2,
                mean_w, out_frac
            };
            srf_diag_pub_.publish(diag);

            std_msgs::Float32 rmsg;
            rmsg.data = reliab;
            srf_reliability_pub_.publish(rmsg);
        }
    }

    //Clear current laser
    new_scan_available = false;
}


//-----------------------------------------------------------------------------------
//                                   CALLBACKS
//-----------------------------------------------------------------------------------

void CLaserOdometry2D::LaserCallBack(const sensor_msgs::LaserScan::ConstPtr& new_scan)
{
    if (GT_pose_initialized)
    {
        //Keep in memory the last received laser_scan
        last_scan = *new_scan;
        laser_counter++;

        //Initialize module on first scan (retry until TF is available)
        if (first_laser_scan)
        {
            if (!Init())
                return;

            first_laser_scan = false;
            //Load first scan and build pyramid
            for (unsigned int i = 0; i < last_scan.ranges.size(); i++)
            {
                const float r = last_scan.ranges[i];
                if (!std::isfinite(r))
                    srf_obj.range_wf(i) = 0.f;
                else
                    srf_obj.range_wf(i) = r;
            }
            srf_obj.createScanPyramid();

            //Set laser min-max distances (if not set as parameters)
            if (laser_min_range == -1)
                laser_min_range = 0.0;
            if (laser_max_range == -1)
                laser_max_range = 0.99f * last_scan.range_max;
        }
        else
        {
            if (laser_counter % laser_decimation == 0)
            {
                //copy laser scan to internal srf variable
                for (unsigned int i = 0; i < last_scan.ranges.size(); i++)
                {
                    const float r = last_scan.ranges[i];
                    //Check min-max distances / non-finite, e.g. to avoid including points of the own robot
                    if (!std::isfinite(r) || (r > laser_max_range) || (r < laser_min_range))
                    {
                        srf_obj.range_wf(i) = 0.f; //invalid measurement
                        last_scan.ranges[i] = 0.0;
                    }
                    else
                        srf_obj.range_wf(i) = r;
                }

                //Publish truncated laser for visualization
                laser_pub.publish(last_scan);

                //Check the average laser measurement
                float sum = 0.f;
                for (unsigned int i = 0; i < last_scan.ranges.size(); i++)
                {
                    sum += srf_obj.range_wf(i);
                }

                ROS_DEBUG_THROTTLE(5.0, "[SRF] The average laser measurement is %f", sum);

                //Process odometry estimation
                srf_obj.odometryCalculation();
                publishPoseFromSRF();
            }
        }
    }
}


void CLaserOdometry2D::initPoseCallBack(const nav_msgs::Odometry::ConstPtr& new_initPose)
{
    // Initialize robot pose on first GT pose. Else do Nothing!
    // Usefull for comparison with other odometry methods.
    if (!GT_pose_initialized)
    {
        initial_robot_pose = *new_initPose;
        GT_pose_initialized = true;
    }
}


//-----------------------------------------------------------------------------------
//                                   MAIN
//-----------------------------------------------------------------------------------
int main(int argc, char** argv)
{
    ros::init(argc, argv, "SRF_LaserOdom");

    //Wrapper class
    CLaserOdometry2D myLaserOdom;

    //Main Loop
    //----------
    ros::spin();

    return (0);
}
