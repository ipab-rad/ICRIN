/**
 * @file      environment.hpp
 * @brief     Main environment for Youbot-ROS-icrin interface
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-08-01
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#ifndef ENVIRONMENT_HPP
#define ENVIRONMENT_HPP

#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>

// #include "environment/youbot_wrapper.hpp"
// #include "environment/amcl_wrapper.hpp"
// #include "environment/bumper_wrapper.hpp"
#include <environment_msgs/EnvironmentData.h>
#include <tracker_msgs/TrackerData.h>
#include <robot_comms_msgs/CommsData.h>
#include <planner_msgs/SetupRVOPlanner.h>

class Environment {
 public:
  Environment(ros::NodeHandle* nh);
  ~Environment();

  void init();
  void rosSetup();
  void loadParams();

  void pubRobotPose();
  void pubRobotGoal();
  void pubRobotVelocity();
  void pubEnvironmentData();

  void trackerDataCB(const tracker_msgs::TrackerData::ConstPtr& msg);
  void amclPoseCB(const geometry_msgs::Pose2D::ConstPtr& msg);
  void commsDataCB(const robot_comms_msgs::CommsData::ConstPtr& msg);
  void plannerCmdVelCB(const geometry_msgs::Twist::ConstPtr& msg);
  void planningCB(const std_msgs::Bool::ConstPtr& msg);

 private:
  // Flags
  bool planning_;
  bool track_robots_;
  bool active_;
  bool amcl_;
  bool bumper_;
  bool rvo_planner_;

  // Variables
  std::string robot_name_;
  geometry_msgs::Vector3 zero_vect_;
  tracker_msgs::TrackerData tracker_data_;
  nav_msgs::Odometry robot_odom_;
  robot_comms_msgs::CommsData comms_data_;
  geometry_msgs::Pose2D robot_amcl_pose_;
  geometry_msgs::Pose2D robot_curr_pose_;
  geometry_msgs::Pose2D robot_target_goal_;
  geometry_msgs::Twist robot_cmd_velocity_;
  geometry_msgs::Twist planner_cmd_velocity_;

  // ROS
  ros::NodeHandle* nh_;
  ros::Publisher curr_pose_pub_;
  ros::Publisher target_goal_pub_;
  ros::Publisher robot_cmd_velocity_pub_;
  ros::Publisher environment_data_pub_;
  ros::ServiceClient setup_rvo_planner_;
  ros::Subscriber tracker_data_sub_;
  ros::Subscriber amcl_pose_sub_;
  ros::Subscriber comms_data_sub_;
  ros::Subscriber planner_cmd_vel_sub_;
  ros::Subscriber planning_sub_;
};

#endif /* ENVIRONMENT_HPP */
