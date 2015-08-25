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
#include <csignal>

#include <std_msgs/Bool.h>
#include <std_msgs/Int32MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>

#include <model_msgs/ModelHypotheses.h>
#include <environment_msgs/EnvironmentData.h>
#include <tracker_msgs/TrackerData.h>
#include <robot_comms_msgs/CommsData.h>
#include <planner_msgs/SetupNewPlanner.h>
#include <experiment_msgs/Goals.h>
#include <experiment_msgs/Plans.h>
#include <experiment_msgs/Plan.h>

class Environment {
 public:
  Environment(ros::NodeHandle* nh);
  ~Environment();

  void init();
  void rosSetup();
  void loadParams();

  static void interrupt(int s);
  static bool isInterrupted() {return interrupted_;}
  void setupEnvironment();
  void setReady(bool ready);

  void pubRobotPose();
  void pubRobotGoal();
  void pubRobotVelocity();
  void pubEnvironmentData();
  void pubPlanning();
  void pubModelHypotheses();

  void goalsCB(const experiment_msgs::Goals::ConstPtr& msg);
  void plansCB(const experiment_msgs::Plans::ConstPtr& msg);
  void planningCB(const std_msgs::Bool::ConstPtr& msg);
  void arrivedCB(const std_msgs::Bool::ConstPtr& msg);
  void bumperKiltCB(const std_msgs::Int32MultiArray::ConstPtr& msg);
  void trackerDataCB(const tracker_msgs::TrackerData::ConstPtr& msg);
  void amclPoseCB(const geometry_msgs::Pose2D::ConstPtr& msg);
  void commsDataCB(const robot_comms_msgs::CommsData::ConstPtr& msg);
  void plannerCmdVelCB(const geometry_msgs::Twist::ConstPtr& msg);

  void checkGoalPlan();
  void modelStep();
  void stopRobot();

 private:
  // Flags
  static bool interrupted_;
  bool planning_;
  bool arrived_;
  bool modelling_;
  bool track_robots_;
  bool amcl_;
  bool bumper_;
  bool rvo_planner_;
  bool collision_;

  // Variables
  std::vector<std::string> robots_;
  std::string robot_name_;
  size_t agent_no_;
  uint16_t robot_id_;
  uint16_t goal_id_;
  geometry_msgs::Vector3 zero_vect_;
  std_msgs::Int32MultiArray bumper_kilt_;
  tracker_msgs::TrackerData tracker_data_;
  robot_comms_msgs::CommsData comms_data_;
  geometry_msgs::Pose2D robot_amcl_pose_;
  geometry_msgs::Pose2D robot_curr_pose_;
  geometry_msgs::Pose2D robot_target_goal_;
  std::vector<geometry_msgs::Pose2D> goals_;
  experiment_msgs::Plan curr_plan_;
  geometry_msgs::Twist robot_cmd_velocity_;
  geometry_msgs::Twist planner_cmd_velocity_;

  // ROS
  ros::NodeHandle* nh_;
  ros::Publisher curr_pose_pub_;
  ros::Publisher target_goal_pub_;
  ros::Publisher robot_cmd_velocity_pub_;
  ros::Publisher environment_data_pub_;
  ros::Publisher planning_pub_;
  ros::Publisher model_pub_;
  ros::ServiceClient setup_new_planner_;
  ros::Subscriber goals_sub_;
  ros::Subscriber plans_sub_;
  ros::Subscriber planning_sub_;
  ros::Subscriber arrived_sub_;
  ros::Subscriber bumper_kilt_sub_;
  ros::Subscriber tracker_data_sub_;
  ros::Subscriber amcl_pose_sub_;
  ros::Subscriber comms_data_sub_;
  ros::Subscriber planner_cmd_vel_sub_;
};

#endif /* ENVIRONMENT_HPP */
