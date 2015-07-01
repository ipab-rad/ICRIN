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
    nh_->serviceClient<rvo_wrapper_msgs::CreateRVOSim>("/rvo_wrapper/create_rvosim");
  create_planner_client_.waitForExistence();
  do_planner_step_client_ =
    nh_->serviceClient<rvo_wrapper_msgs::DoStep>("/rvo_wrapper/do_step");
  do_planner_step_client_.waitForExistence();
  add_planner_agent_client_ =
    nh_->serviceClient<rvo_wrapper_msgs::AddAgent>("/rvo_wrapper/add_agent");
  add_planner_agent_client_.waitForExistence();
  get_agent_pos_client_ =
    nh_->serviceClient<rvo_wrapper_msgs::GetAgentPosition>("/rvo_wrapper/get_agent_position");
  get_agent_pos_client_.waitForExistence();
}

void RVOPlanner::createPlanner() {
  rvo_wrapper_msgs::CreateRVOSim msg;
  msg.request.sim_num = 0;
  msg.request.time_step = 0.1f;
  msg.request.defaults.neighbor_dist = 2.0f;
  msg.request.defaults.max_neighbors = 20;
  msg.request.defaults.time_horizon_agent = 5.0f;
  msg.request.defaults.time_horizon_obst = 5.0f;
  msg.request.defaults.radius = 1.2f;
  msg.request.defaults.max_speed = 0.6f;
  create_planner_client_.call(msg);
}

void RVOPlanner::doPlannerStep() {
  rvo_wrapper_msgs::DoStep msg;
  do_planner_step_client_.call(msg);
}

size_t RVOPlanner::addPlannerAgent(common_msgs::Vector2 agent_pos) {
  rvo_wrapper_msgs::AddAgent msg;
  msg.request.position = agent_pos;
  add_planner_agent_client_.call(msg);
  return msg.response.agent_id;
}

common_msgs::Vector2 RVOPlanner::getAgentPos(size_t agent_no) {
  rvo_wrapper_msgs::GetAgentPosition msg;
  msg.request.agent_id = agent_no;
  get_agent_pos_client_.call(msg);
  return msg.response.position;
}
