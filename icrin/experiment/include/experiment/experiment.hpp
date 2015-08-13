/**
 * @file      experiment.hpp
 * @brief     Experiment "State Machine" for controlling flow
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-08-11
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#ifndef EXPERIMENT_HPP_
#define EXPERIMENT_HPP_

#include <string>
#include <csignal>
#include <ros/ros.h>
#include <experiment/console.hpp>

#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose2D.h>
#include <experiment_msgs/Goals.h>
#include <experiment_msgs/Plan.h>
#include <experiment_msgs/Plans.h>
#include <experiment_msgs/SetGoal.h>
#include <experiment_msgs/SetPlan.h>

class Experiment {
 public:
  Experiment(ros::NodeHandle* nh);
  ~Experiment();

  void init();
  void rosSetup();
  void loadParams();

  static void interrupt(int s);
  void pubPlanning();
  void pubGoals();
  void pubPlans();
  bool setGoal(experiment_msgs::SetGoal::Request& req,
               experiment_msgs::SetGoal::Response& res);
  bool setPlan(experiment_msgs::SetPlan::Request& req,
               experiment_msgs::SetPlan::Response& res);
  void stopExperiment();
  static bool isInterrupted() {return interrupted_;}

 private:
  // Flags
  static bool interrupted_;

  // Variables
  std::vector<std::string> robots_;
  size_t goal_no_;
  // std::vector<geometry_msgs::Pose2D> goals_;
  experiment_msgs::Goals goals_;
  // std::vector<experiment_msgs::Plan> plans_;
  experiment_msgs::Plans plans_;

  // ROS
  ros::NodeHandle* nh_;
  std::vector<ros::Publisher> planning_pub_;
  std::vector<bool> robots_planning_;
  ros::Publisher goals_pub_;
  ros::Publisher plans_pub_;
  ros::ServiceServer srv_set_goal_;
  ros::ServiceServer srv_set_plan_;
};

#endif /* EXPERIMENT_HPP_ */
