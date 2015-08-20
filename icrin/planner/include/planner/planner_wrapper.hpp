/**
 * @file      planner_wrapper.hpp
 * @brief     Planner wrapper, connecting the differing planners through ROS
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-06-29
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#ifndef PLANNER_WRAPPER_HPP
#define PLANNER_WRAPPER_HPP

#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <common_msgs/Vector2.h>

#include <environment_msgs/EnvironmentData.h>
#include <planner_msgs/SetupNewPlanner.h>
#include <planner_msgs/SetupRVOPlanner.h>

#include <planner/rvo_planner.hpp>

class PlannerWrapper {
 public:
  PlannerWrapper(ros::NodeHandle* nh);
  ~PlannerWrapper();

  void init();

  void rosSetup();

  void pubPlanning(bool planning);
  void pubArrived(bool arrived);

  bool setupNewPlanner(planner_msgs::SetupNewPlanner::Request& req,
                       planner_msgs::SetupNewPlanner::Response& res);

  bool setupRVOPlanner(planner_msgs::SetupRVOPlanner::Request& req,
                       planner_msgs::SetupRVOPlanner::Response& res);

  void plannerStep();

  void currPoseCB(const geometry_msgs::Pose2D::ConstPtr& msg);
  void targetGoalCB(const geometry_msgs::Pose2D::ConstPtr& msg);
  void planningCB(const std_msgs::Bool::ConstPtr& msg);
  void environmentDataCB(const environment_msgs::EnvironmentData::ConstPtr& msg);

 private:
  // Flags
  bool use_rvo_planner_;
  bool planning_;
  bool arrived_;
  bool planner_init;
  // Variables
  std::string robot_name_;
  common_msgs::Vector2 curr_pose_;
  common_msgs::Vector2 goal_pose_;
  common_msgs::Vector2 rvo_planner_vel_;
  geometry_msgs::Twist cmd_vel_;
  environment_msgs::EnvironmentData environment_;
  // ROS
  ros::NodeHandle* nh_;

  ros::Publisher cmd_vel_pub_;
  ros::Publisher planning_pub_;
  ros::Publisher arrived_pub_;
  ros::Subscriber curr_pose_sub_;
  ros::Subscriber target_goal_sub_;
  ros::Subscriber planning_sub_;
  ros::ServiceServer srv_setup_new_planner_;
  ros::ServiceServer srv_setup_rvo_planner_;

  // Class pointers
  RVOPlanner* rvo_planner_;
};

#endif  /* PLANNER_WRAPPER_HPP */
