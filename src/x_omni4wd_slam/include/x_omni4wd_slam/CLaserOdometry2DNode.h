/** ****************************************************************************************
*  This node presents a fast and precise method to estimate the planar motion of a lidar
*  from consecutive range scans. It is very useful for the estimation of the robot odometry from
*  2D laser range measurements.
*  This module is developed for mobile robots with innacurate or inexistent built-in odometry.
*  It allows the estimation of a precise odometry with low computational cost.
*  For more information, please refer to:
*
*  Planar Odometry from a Radial Laser Scanner. A Range Flow-based Approach. ICRA'16.
*  Available at: http://mapir.isa.uma.es/mapirwebsite/index.php/mapir-downloads/papers/217
*
* Maintainer: Javier G. Monroy
* MAPIR group: http://mapir.isa.uma.es/
*
* Modifications: Jeremie Deray
* Ported to ROS 1: x_omni4wd_slam
******************************************************************************************** */

#ifndef CLASERODOMETRY2DNODE_H
#define CLASERODOMETRY2DNODE_H

#include "x_omni4wd_slam/CLaserOdometry2D.h"

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf2/convert.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/impl/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

namespace rf2o {

class CLaserOdometry2DNode
{
public:
  CLaserOdometry2DNode(ros::NodeHandle& nh);
  void process();
  void publish();
  bool setLaserPoseFromTf();

  CLaserOdometry2D rf2o_ref;
  bool publish_tf, new_scan_available;

  double freq;
  double laser_fps;  // Scan rate in Hz (phải khớp lidar, e.g. A2M8 = 10 Hz) để odom scale đúng

  std::string         laser_scan_topic;
  std::string         odom_topic;
  std::string         base_frame_id;
  std::string         odom_frame_id;
  std::string         init_pose_from_topic;

  sensor_msgs::LaserScan      last_scan;
  bool                        GT_pose_initialized;
  tf2_ros::Buffer             buffer_;
  tf2_ros::TransformListener  tf_listener_;
  tf2_ros::TransformBroadcaster odom_broadcaster;
  nav_msgs::Odometry     initial_robot_pose;

  //Subscriptions & Publishers
  ros::Subscriber  laser_sub;
  ros::Subscriber  initPose_sub;
  ros::Publisher   odom_pub;

  bool scan_available();

  //CallBacks
  void LaserCallBack(const sensor_msgs::LaserScan::ConstPtr& new_scan);
  void initPoseCallBack(const nav_msgs::Odometry::ConstPtr& new_initPose);

private:
  ros::NodeHandle nh_;
};

} /* namespace rf2o */

#endif // CLASERODOMETRY2DNODE_H

