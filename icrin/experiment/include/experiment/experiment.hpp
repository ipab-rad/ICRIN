/**
 * @file      experiment.hpp
 * @brief     Experiment "State Machine" for controlling flow
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-08-11
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#ifndef EXPERIMENT_HPP_
#define EXPERIMENT_HPP_

#include <stdlib.h>
#include <ros/ros.h>
#include <experiment/console.hpp>

#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose2D.h>
#include <experiment_msgs/Goals.h>
#include <experiment_msgs/Plan.h>
#include <experiment_msgs/Plans.h>
#include <experiment_msgs/SetGoal.h>
#include <experiment_msgs/SetPlan.h>

#include <string>
#include <csignal>

class Experiment {
 public:
  explicit Experiment(ros::NodeHandle* nh);
  ~Experiment();

  void init();
  void rosSetup();
  void loadParams();

  static void interrupt(int s);
  void pubPlanning();
  void pubGoals();
  void pubPlans(bool setup_plan);
  void planningCB(const std_msgs::Bool::ConstPtr& msg,
                  const std::string& robot);
  bool setGoal(experiment_msgs::SetGoal::Request& req,
               experiment_msgs::SetGoal::Response& res);
  bool setPlan(experiment_msgs::SetPlan::Request& req,
               experiment_msgs::SetPlan::Response& res);
  void setPlanning(size_t robot_no, bool planning)
  {robots_planning_[robot_no] = planning;}
  bool isPlanning(size_t robot_no) {return robots_planning_[robot_no];}
  bool checkReadyRobots();
  void stopExperiment();
  void waitReturn();
  void progSpin();
  bool robotsReady() {return robots_ready_;}
  static bool isInterrupted() {return interrupted_;}
  std::vector<std::string> getRobots() {return robots_;}

 private:
  // Flags
  bool robots_ready_;
  static bool interrupted_;

  // Variables
  std::vector<std::string> robots_;
  size_t goal_no_;
  int prog_;
  experiment_msgs::Goals goals_;
  experiment_msgs::Plans plans_;
  experiment_msgs::Plans setup_plans_;

  // ROS
  ros::NodeHandle* nh_;
  std::vector<ros::Publisher> planning_pub_;
  std::vector<ros::Subscriber> planning_sub_;
  std::vector<bool> robots_planning_;
  ros::Publisher goals_pub_;
  ros::Publisher plans_pub_;
  ros::ServiceServer srv_set_goal_;
  ros::ServiceServer srv_set_plan_;
};

#endif /* EXPERIMENT_HPP_ */
