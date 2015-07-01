/**
 * @file      rvo_planner.hpp
 * @brief     RVO Planner class
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-06-30
 * @copyright (MIT) 2015 Edinferno
 */

#ifndef RVO_PLANNER_HPP
#define RVO_PLANNER_HPP

#include <ros/ros.h>
#include <vector>

#include <rvo_wrapper/rvo_wrapper.hpp>

#include <rvo_wrapper_msgs/CreateRVOSim.h>
#include <rvo_wrapper_msgs/AddAgent.h>
#include <rvo_wrapper_msgs/GetAgentPosition.h>
#include <rvo_wrapper_msgs/DoStep.h>

class RVOPlanner {
 public:
  RVOPlanner(ros::NodeHandle* nh);
  ~RVOPlanner();

  void rosSetup();

  void createPlanner();

  void doPlannerStep();

  size_t addPlannerAgent(common_msgs::Vector2 agent_pos);

  common_msgs::Vector2 getAgentPos(size_t agent_no);

 private:
  // ROS
  ros::NodeHandle* nh_;
  ros::ServiceClient create_planner_client_;
  ros::ServiceClient do_planner_step_client_;
  ros::ServiceClient add_planner_agent_client_;
  ros::ServiceClient get_agent_pos_client_;

  // Class pointers

};

#endif /* RVO_PLANNER_HPP */
