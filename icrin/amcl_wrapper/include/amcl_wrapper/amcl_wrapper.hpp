/**
 * @file      amcl_wrapper.hpp
 * @brief     AMCL wrapper, provides up-to-date robot pose for Environment
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-08-06
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#ifndef AMCL_WRAPPER_HPP
#define AMCL_WRAPPER_HPP

#include <ros/ros.h>

#include <tf/tf.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>

class AMCLWrapper {
 public:
  explicit AMCLWrapper(ros::NodeHandle* nh);
  ~AMCLWrapper();

  void init();
  void rosSetup();
  void loadParams();

  void AMCLPoseCB(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void OdomCB(const nav_msgs::Odometry::ConstPtr& msg);
  void SetPoseCB(const geometry_msgs::Pose2D::ConstPtr& msg);
  void pubRobotPose();
  void pubRobotVel();
  void calcAMCL();
  void calcOdomDiff();
  double calcYaw(nav_msgs::Odometry odom);

 private:
  // Flags
  bool amcl_received_;
  bool odom_received_;
  bool odom_init_;
  bool use_amcl_;
  // Variables
  std::string robot_name_;

  // ROS
  ros::NodeHandle* nh_;
  geometry_msgs::PoseWithCovarianceStamped amcl_pose_;
  geometry_msgs::Pose2D robot_pose_;
  geometry_msgs::Twist robot_vel_;
  nav_msgs::Odometry prev_odom_;
  nav_msgs::Odometry curr_odom_;
  geometry_msgs::Pose2D odom_diff_;

  ros::Publisher robot_pose_pub_;
  ros::Publisher robot_vel_pub_;
  ros::Publisher initial_pose_pub_;
  ros::Subscriber amcl_pose_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber set_pose_;
};

#endif /* AMCL_WRAPPER_HPP */
