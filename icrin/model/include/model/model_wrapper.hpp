/**
 * @file      model_wrapper.hpp
 * @brief     Model Wrapper to manage modelling functions and sim wrapper
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-08-22
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#ifndef MODEL_WRAPPER_HPP
#define MODEL_WRAPPER_HPP

#include <ros/ros.h>

#include <std_msgs/Bool.h>

#include <model/sim_wrapper.hpp>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <environment_msgs/EnvironmentData.h>
#include <model_msgs/ModelHypotheses.h>
#include <model_msgs/GoalHypothesis.h>
#include <model_msgs/AwareHypothesis.h>
#include <model_msgs/InteractivePrediction.h>

class ModelWrapper {
 public:
  ModelWrapper(ros::NodeHandle* nh);
  ~ModelWrapper();

  void loadParams();
  void init();
  void rosSetup();

  void robotPoseCB(const geometry_msgs::Pose2D::ConstPtr& msg);
  void robotGoalCB(const geometry_msgs::Pose2D::ConstPtr& msg);
  void robotVelCB(const geometry_msgs::Twist::ConstPtr& msg);
  void envDataCB(const environment_msgs::EnvironmentData::ConstPtr& msg);
  void modelCB(const model_msgs::ModelHypotheses::ConstPtr& msg);

  void runModel();
  void inferGoals();
  void setupModel();
  void runSims();
  void interactivePrediction();

 private:
  // Flags
  bool debug_;
  bool use_rvo_lib_;
  bool interactive_costmap_;
  bool initialised_;

  // Constants
  bool robot_model_;
  float goal_sum_prior_;
  float goal_history_discount_;
  int goal_inference_history_;
  int velocity_average_window_;
  float prior_lambda_;
  float max_accel_;

  // Variables
  std::string robot_name_;
  std::string model_name_;
  environment_msgs::EnvironmentData env_data_;
  geometry_msgs::Pose2D robot_pose_;
  geometry_msgs::Pose2D robot_goal_;
  geometry_msgs::Twist robot_vel_;
  model_msgs::ModelHypotheses hypotheses_;
  std::vector<uint32_t> sampling_sims_;
  std::vector<common_msgs::Vector2> sampling_sim_vels;
  std::vector<uint32_t> sequence_sims_;
  std::vector<common_msgs::Vector2> sequence_sim_vels;
  std::vector<uint32_t> awareness_sims_;
  // std::vector<std::vector<float> > inferred_goals_history_;
  // std::vector<bool> init_liks_;
  // std::vector<float> prev_prior_;
  std::vector<uint32_t> costmap_sims_;
  std::vector<std::vector<bool> > init_liks_;
  std::vector<std::vector<float> > prev_prior_;
  std::vector<std::vector<float> > agent_goal_inference_;

  // ROS
  ros::NodeHandle* nh_;
  ros::Subscriber robot_pose_sub_;
  ros::Subscriber robot_goal_sub_;
  ros::Subscriber robot_vel_sub_;
  ros::Subscriber env_data_sub_;
  ros::Subscriber model_hyp_sub_;
  ros::Publisher inter_pred_pub_;

  // Class pointers
  SimWrapper* sim_wrapper_;
};

#endif  /* MODEL_WRAPPER_HPP */
