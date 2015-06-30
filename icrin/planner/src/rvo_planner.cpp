/**
 * @file      rvo_planner.cpp
 * @brief     RVO Planner class
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-06-30
 * @copyright (MIT) 2015 Edinferno
 */

#include <planner/rvo_planner.hpp>

RVOPlanner::RVOPlanner(ros::NodeHandle* nh) {
  nh_ = nh;
  this->rosSetup();
  this->createPlanner();
}

RVOPlanner::~RVOPlanner() {;}

void RVOPlanner::rosSetup() {
  create_planner_client_ =
    nh_->serviceClient<std_srvs::Empty>("/rvo_wrapper/create_planner");
  create_planner_client_.waitForExistence();
  do_planner_step_client_ =
    nh_->serviceClient<std_srvs::Empty>("/rvo_wrapper/do_planner_step");
  do_planner_step_client_.waitForExistence();
  add_planner_agent_client =
    nh_->serviceClient<rvo_wrapper_msgs::AddPlannerAgent>("/rvo_wrapper/add_planner_agent");
  add_planner_agent_client.waitForExistence();
  get_agent_pos_client =
    nh_->serviceClient<rvo_wrapper_msgs::GetAgentPos>("/rvo_wrapper/get_agent_pos");
  get_agent_pos_client.waitForExistence();
}

void RVOPlanner::createPlanner() {
  create_planner_client_.call(empty_srv_);
}

void RVOPlanner::doPlannerStep() {
  do_planner_step_client_.call(empty_srv_);
}

RVO::Vector2 RVOPlanner::getAgentPos(size_t agent_no) {
  rvo_wrapper_msgs::GetAgentPos msg;
  msg.request.agent_id = agent_no;
  get_agent_pos_client.call(msg);
  RVO::Vector2 agent_pos_(msg.response.agent_pos.x, msg.response.agent_pos.y);
  return agent_pos_;
}
