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

#ifndef CLaserOdometry2D_H
#define CLaserOdometry2D_H

#include <ros/ros.h>
#include <stdlib.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <Eigen/Eigenvalues>

// SRF
#include "laser_odometry_refscans.h"

// MRPT poses only (no obs/slam headers required by this wrapper)
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose2D.h>

class CLaserOdometry2D
{
public:
    //Variables
    std::string laser_scan_topic;
    std::string base_frame_id;
    std::string odom_topic, odom_frame_id;
    std::string init_pose_from_topic;
    std::string operation_mode;
    double laser_min_range, laser_max_range;
    bool publish_tf;
    mrpt::poses::CPose3D robot_pose, robot_oldpose;
    int laser_counter, laser_decimation;

    //Core class of SRF
    SRF_RefS srf_obj;

    //methods
    CLaserOdometry2D();
    ~CLaserOdometry2D();
    bool is_initialized();
    bool scan_available();
    bool Init();
    void odometryCalculation();     //Update odometric pose
    void publishPoseFromSRF();     //Publishes the last odometric pose with ROS format

protected:
    ros::NodeHandle n;
    sensor_msgs::LaserScan last_scan;
    bool module_initialized, first_laser_scan, new_scan_available, GT_pose_initialized;
    tf::TransformListener tf_listener;                              //Do not put inside the callback
    tf::TransformBroadcaster odom_broadcaster;
    ros::Time last_odom_time;
    nav_msgs::Odometry initial_robot_pose;

    // Extrinsics base_frame ← laser (from TF at Init): used for twist rotate + lever-arm
    double laser_x_on_base_;
    double laser_y_on_base_;
    double laser_yaw_on_base_;
    bool laser_extrinsics_valid_;

    // Optional non-zero covariances (ROS defaults are 0 → EKF over-trust)
    double pose_cov_x_;
    double pose_cov_y_;
    double pose_cov_yaw_;
    double twist_cov_vx_;
    double twist_cov_vy_;
    double twist_cov_wz_;

    //Subscriptions & Publishers
    ros::Subscriber laser_sub, initPose_sub;
    ros::Publisher odom_pub, laser_pub;
    ros::Publisher srf_diag_pub_;
    ros::Publisher srf_reliability_pub_;
    bool publish_diagnostics_;
    double rel_cov_scale_;
    double rel_cond_scale_;

    //CallBacks
    void LaserCallBack(const sensor_msgs::LaserScan::ConstPtr& new_scan);
    void initPoseCallBack(const nav_msgs::Odometry::ConstPtr& new_initPose);

    // Twist: laser-local kai_loc/dt → base_footprint (rotate + lever-arm)
    void laserTwistToBase(double v_lx, double v_ly, double wz,
                          double& v_bx, double& v_by, double& wz_b) const;
};

#endif
