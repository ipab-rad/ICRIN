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

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>

#include <tracker_msgs/TrackerData.h>
// #include "environment/youbot_wrapper.hpp"
// #include "environment/amcl_wrapper.hpp"
// #include "environment/bumper_wrapper.hpp"

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

  void odomCB(const nav_msgs::Odometry::ConstPtr& msg);
  void trackerDataCB(const tracker_msgs::TrackerData::ConstPtr& msg);
  void commsDataCB(const robot_comms_msgs::CommsData::ConstPtr& msg);
  void plannerCmdVelCB(const geometry_msgs::Twist::ConstPtr& msg);

  // Flags
  bool planning_;
 private:
  bool track_robots_;
  bool active_;
  bool amcl_;
  bool bumper_;
  bool rvo_planner_;

  // Variables
  std::string robot_name_;
  std::vector<uint32_t> agent_trackerID_;
  std::vector<geometry_msgs::Pose2D> agent_positions_;
  std::vector<geometry_msgs::Pose2D> agent_pos_std_dev_;
  std::vector<geometry_msgs::Twist> agent_velocities_;
  std::vector<geometry_msgs::Twist> agent_avg_velocities_;
  nav_msgs::Odometry robot_odom_;
  geometry_msgs::Pose2D robot_curr_pose_;
  geometry_msgs::Pose2D robot_target_goal_;
  geometry_msgs::Twist robot_cmd_velocity_;
  geometry_msgs::Twist planner_cmd_velocity_;
  // ROS
  ros::NodeHandle* nh_;
  ros::Publisher curr_pose_pub_;
  ros::Publisher target_goal_pub_;
  ros::Publisher robot_cmd_velocity_pub_;
  ros::ServiceClient setup_rvo_planner_;
  ros::Subscriber odom_sub_;
  ros::Subscriber tracker_data_sub_;
  ros::Subscriber comms_data_sub_;
  ros::Subscriber planner_cmd_vel_sub_;

};

#endif /* ENVIRONMENT_HPP */
