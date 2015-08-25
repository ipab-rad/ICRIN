/**
 * @file      sim_wrapper.hpp
 * @brief     Sim Wrapper communicates with rvo_wrapper for running simulations
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-08-22
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#ifndef SIM_WRAPPER_HPP
#define SIM_WRAPPER_HPP

#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <common_msgs/Vector2.h>

#include <rvo_wrapper_msgs/AddAgent.h>
#include <rvo_wrapper_msgs/CalcPrefVelocities.h>
#include <rvo_wrapper_msgs/CreateRVOSim.h>
#include <rvo_wrapper_msgs/DeleteSimVector.h>
#include <rvo_wrapper_msgs/DoStep.h>
#include <rvo_wrapper_msgs/GetAgentVelocity.h>
#include <rvo_wrapper_msgs/SetAgentGoals.h>
#include <rvo_wrapper_msgs/SetAgentVelocity.h>

class SimWrapper {
 public:
  SimWrapper(ros::NodeHandle* nh);
  ~SimWrapper();

  void loadParams();
  void init();
  void rosSetup();

  std::vector<uint32_t> goalSampling(std::vector<geometry_msgs::Pose2D>
                                     sample_space,
                                     float sample_resolution);
  std::vector<uint32_t> goalSequence(std::vector<geometry_msgs::Pose2D>
                                     goal_sequence);

  std::vector<common_msgs::Vector2> calcSimVels(std::vector<uint32_t> sims,
                                                size_t n_goals);

  void setRobotModel(bool robot_model);
  void setModelAgents(std::vector<uint8_t> model_agents);
  void setRobotGoal(geometry_msgs::Pose2D robot_goal);
  void setEnvironment(std::vector<geometry_msgs::Pose2D> agent_poses,
                      std::vector<geometry_msgs::Twist> agent_vels);

 private:
  // Flags
  bool use_rvo_lib_;

  // Constants
  common_msgs::Vector2 null_vect_;

  // Sim params
  float time_step_;
  float neighbor_dist_;
  size_t max_neighbors_;
  float time_horizon_agent_;
  float time_horizon_obst_;
  float radius_;
  float max_speed_;

  // Variables
  bool robot_model_;
  std::string robot_name_;
  std::string model_name_;
  common_msgs::Vector2 robot_goal_;
  size_t agent_no_;
  std::vector<uint8_t> model_agents_;
  size_t model_agent_no_;
  std::vector<common_msgs::Vector2> agent_poses_;
  std::vector<common_msgs::Vector2> agent_vels_;

  // ROS
  ros::NodeHandle* nh_;
  ros::ServiceClient add_sim_agent_client_;
  ros::ServiceClient calc_pref_velocities_client_;
  ros::ServiceClient create_sims_client_;
  ros::ServiceClient delete_sims_client_;
  ros::ServiceClient do_sim_step_client_;
  ros::ServiceClient get_agent_vel_client_;
  ros::ServiceClient set_agent_goals_client_;
  ros::ServiceClient set_agent_vel_client_;
};

#endif  /* SIM_WRAPPER_HPP */
