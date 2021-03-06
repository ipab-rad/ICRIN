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
#include <rvo_wrapper/rvo_wrapper.hpp>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>

#include <rvo_wrapper_msgs/AddAgent.h>
#include <rvo_wrapper_msgs/CalcPrefVelocities.h>
#include <rvo_wrapper_msgs/CheckReachedGoal.h>
#include <rvo_wrapper_msgs/CreateRVOSim.h>
#include <rvo_wrapper_msgs/DeleteSimVector.h>
#include <rvo_wrapper_msgs/DoStep.h>
#include <rvo_wrapper_msgs/GetAgentVelocity.h>
#include <rvo_wrapper_msgs/SetAgentGoals.h>
#include <rvo_wrapper_msgs/SetAgentPosition.h>
#include <rvo_wrapper_msgs/SetAgentVelocity.h>

#include <vector>

class RVOPlanner {
 public:
  explicit RVOPlanner(ros::NodeHandle* nh);
  ~RVOPlanner();

  void loadParams();

  void init();

  void rosSetup();

  size_t addPlannerAgent(common_msgs::Vector2 agent_pos);
  void setPlannerVel(common_msgs::Vector2 planner_vel);

  bool checkReachedGoal();

  common_msgs::Vector2 planStep();

  void createPlanner();

  void setupPlanner();

  void calcPrefVelocity();

  void doSimStep();

  void deletePlanner();
  void setupEnvironment(std::vector<uint32_t> tracker_ids,
                        std::vector<common_msgs::Vector2> agent_poses,
                        std::vector<common_msgs::Vector2> agent_vels);

  common_msgs::Vector2 getPlannerVel();
  void setAgentVelocities(std::vector<common_msgs::Vector2> agent_velocities);

  void setPlannerSettings(float time_step,
                          rvo_wrapper_msgs::AgentDefaults defaults);
  void setCurrPose(common_msgs::Vector2 curr_pose);
  void setCurrVel(common_msgs::Vector2 curr_vel);
  void setPlannerGoal(common_msgs::Vector2 goal);
  bool getArrived() {return arrived_;}

 private:
  // Flags
  bool arrived_;
  bool persistence_;

  // Constants
  uint8_t PLANNER_ROBOT_;

  // Variables
  std::string robot_name_;
  common_msgs::Vector2 curr_pose_;
  common_msgs::Vector2 planner_goal_;
  common_msgs::Vector2 planner_vel_;
  std::vector<uint32_t> tracker_ids_;
  std::vector<common_msgs::Vector2> agent_positions_;
  std::vector<common_msgs::Vector2> agent_velocities_;
  rvo_wrapper_msgs::CreateRVOSim planner_settings_;

  // ROS
  ros::NodeHandle* nh_;
  ros::ServiceClient add_planner_agent_client_;
  ros::ServiceClient calc_pref_velocities_client_;
  ros::ServiceClient check_reached_goal_client_;
  ros::ServiceClient create_planner_client_;
  ros::ServiceClient delete_planner_client_;
  ros::ServiceClient do_planner_step_client_;
  ros::ServiceClient get_agent_vel_client_;
  ros::ServiceClient set_agent_goals_client_;
  ros::ServiceClient set_agent_position_;
  ros::ServiceClient set_agent_velocity_;
};

#endif /* RVO_PLANNER_HPP */
