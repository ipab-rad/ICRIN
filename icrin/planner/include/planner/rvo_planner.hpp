/**
 * @file      rvo_planner.hpp
 * @brief     RVO Planner class
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-06-30
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#ifndef RVO_PLANNER_HPP
#define RVO_PLANNER_HPP

#include <ros/ros.h>
#include <vector>

#include <geometry_msgs/Pose2D.h>

#include <rvo_wrapper/rvo_wrapper.hpp>

#include <rvo_wrapper_msgs/AddAgent.h>
#include <rvo_wrapper_msgs/CalcPrefVelocities.h>
#include <rvo_wrapper_msgs/CheckReachedGoal.h>
#include <rvo_wrapper_msgs/CreateRVOSim.h>
#include <rvo_wrapper_msgs/AgentDefaults.h>
#include <rvo_wrapper_msgs/DeleteSimVector.h>
#include <rvo_wrapper_msgs/DoStep.h>
#include <rvo_wrapper_msgs/GetAgentPosition.h>
#include <rvo_wrapper_msgs/GetNumAgents.h>
#include <rvo_wrapper_msgs/SetAgentGoals.h>
#include <rvo_wrapper_msgs/SetAgentDefaults.h>
#include <rvo_wrapper_msgs/SetAgentPosition.h>
#include <rvo_wrapper_msgs/SetTimeStep.h>

class RVOPlanner {
 public:
  RVOPlanner(ros::NodeHandle* nh);
  ~RVOPlanner();

  void init();

  void rosSetup();

  size_t addPlannerAgent(common_msgs::Vector2 agent_pos);

  void calcPrefVelocities();

  bool checkReachedGoal();

  void setPlannerGoal(common_msgs::Vector2 goal);

  void createPlanner();

  void doSimStep();

  common_msgs::Vector2 getAgentPos(size_t agent_no);

  void setPlannerSettings(float time_step,
                          rvo_wrapper_msgs::AgentDefaults defaults);

  void planStep();

  common_msgs::Vector2 getPlannerVel();
  void setCurrPose(common_msgs::Vector2 curr_pose);

 private:
  // Constants
  uint8_t PLANNER_ROBOT_;
  // Variables
  rvo_wrapper_msgs::CreateRVOSim planner_settings_;
  // ROS
  ros::NodeHandle* nh_;
  ros::ServiceClient add_planner_agent_client_;
  ros::ServiceClient check_reached_goal_client_;
  ros::ServiceClient calc_pref_velocities_client_;
  ros::ServiceClient create_planner_client_;
  ros::ServiceClient delete_planner_client_;
  ros::ServiceClient do_planner_step_client_;
  ros::ServiceClient get_agent_pos_client_;
  ros::ServiceClient get_agent_vel_client_;
  ros::ServiceClient get_num_agents_;
  ros::ServiceClient set_agent_goals_client_;
  ros::ServiceClient set_agent_defaults_;
  ros::ServiceClient set_agent_position_;
  ros::ServiceClient set_time_step_;

  // Class pointers

};

#endif /* RVO_PLANNER_HPP */
