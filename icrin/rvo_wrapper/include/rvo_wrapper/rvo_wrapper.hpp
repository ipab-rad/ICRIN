/**
 * @file      rvo_wrapper.hpp
 * @brief     Add file description...
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-06-29
 * @copyright (MIT) 2015 Edinferno
 */

#ifndef RVO_WRAPPER_HPP
#define RVO_WRAPPER_HPP

#include <ros/ros.h>

#include <rvo_wrapper/RVO.h>
#include <rvo_wrapper/Definitions.h>
#include <rvo_wrapper/Vector2.h>

#include <std_srvs/Empty.h>
#include <rvo_wrapper_msgs/GetAgentPos.h>
#include <rvo_wrapper_msgs/AddPlannerAgent.h>

class RVOWrapper {
 public:
  RVOWrapper(ros::NodeHandle* nh);
  ~RVOWrapper();

  void rosSetup();

  bool createPlanner(std_srvs::Empty::Request& req,
                     std_srvs::Empty::Response& res);

  bool doPlannerStep(std_srvs::Empty::Request& req,
                     std_srvs::Empty::Response& res);

  bool addPlannerAgent(rvo_wrapper_msgs::AddPlannerAgent::Request& req,
                       rvo_wrapper_msgs::AddPlannerAgent::Response& res);

  bool getAgentPos(rvo_wrapper_msgs::GetAgentPos::Request& req,
                   rvo_wrapper_msgs::GetAgentPos::Response& res);
 private:
  // ROS
  ros::NodeHandle* nh_;
  ros::ServiceServer srv_create_planner_;
  ros::ServiceServer srv_do_planner_step_;
  ros::ServiceServer srv_add_planner_agent_;
  ros::ServiceServer srv_get_agent_pos_;

  // Class pointers
  RVO::RVOSimulator* planner_;
  std::vector<RVO::RVOSimulator*> sim_vect_;

};

#endif /* RVO_WRAPPER_HPP */
