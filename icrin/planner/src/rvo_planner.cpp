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
  add_planner_agent_client_ =
    nh_->serviceClient<rvo_wrapper_msgs::AddAgent>(
      "/rvo_wrapper/add_agent", true);
  add_planner_agent_client_.waitForExistence();
  check_reached_goal_client_ =
    nh_->serviceClient<rvo_wrapper_msgs::CheckReachedGoal>(
      "/rvo_wrapper/check_reached_goal", true);
  check_reached_goal_client_.waitForExistence();
  calc_pref_velocities_client_ =
    nh_->serviceClient<rvo_wrapper_msgs::CalcPrefVelocities>(
      "/rvo_wrapper/calc_pref_velocities", true);
  calc_pref_velocities_client_.waitForExistence();
  create_planner_client_ =
    nh_->serviceClient<rvo_wrapper_msgs::CreateRVOSim>(
      "/rvo_wrapper/create_rvosim", true);
  create_planner_client_.waitForExistence();
  do_planner_step_client_ =
    nh_->serviceClient<rvo_wrapper_msgs::DoStep>(
      "/rvo_wrapper/do_step", true);
  do_planner_step_client_.waitForExistence();
  get_agent_pos_client_ =
    nh_->serviceClient<rvo_wrapper_msgs::GetAgentPosition>(
      "/rvo_wrapper/get_agent_position", true);
  get_agent_pos_client_.waitForExistence();
  get_num_agents_ =
    nh_->serviceClient<rvo_wrapper_msgs::GetNumAgents>(
      "/rvo_wrapper/get_num_agents", true);
  get_num_agents_.waitForExistence();
  set_agent_goals_client_ =
    nh_->serviceClient<rvo_wrapper_msgs::SetAgentGoals>(
      "/rvo_wrapper/set_agent_goals", true);
  set_agent_goals_client_.waitForExistence();
}

size_t RVOPlanner::addPlannerAgent(common_msgs::Vector2 agent_pos) {
  rvo_wrapper_msgs::AddAgent msg;
  msg.request.position = agent_pos;
  add_planner_agent_client_.call(msg);
  return msg.response.agent_id;
}

void RVOPlanner::calcPrefVelocities() {
  rvo_wrapper_msgs::CalcPrefVelocities msg;
  calc_pref_velocities_client_.call(msg);
}

bool RVOPlanner::checkReachedGoal() {
  rvo_wrapper_msgs::CheckReachedGoal msg;
  // msg.request.goal = goal;
  check_reached_goal_client_.call(msg);
  return msg.response.reached;
}

void RVOPlanner::createPlanner() {
  rvo_wrapper_msgs::CreateRVOSim msg;
  msg.request.sim_num = 0;
  msg.request.time_step = 0.1f;
  msg.request.defaults.neighbor_dist = 2.0f;
  msg.request.defaults.max_neighbors = 20;
  msg.request.defaults.time_horizon_agent = 5.0f;
  msg.request.defaults.time_horizon_obst = 5.0f;
  msg.request.defaults.radius = 0.5f;
  msg.request.defaults.max_speed = 0.3f;
  create_planner_client_.call(msg);
  if (msg.response.res) {
    ROS_INFO("Planner created");
  } else { ROS_ERROR("Planner not created!"); };
}

void RVOPlanner::doPlannerStep() {
  rvo_wrapper_msgs::DoStep msg;
  do_planner_step_client_.call(msg);
}

common_msgs::Vector2 RVOPlanner::getAgentPos(size_t agent_no) {
  rvo_wrapper_msgs::GetAgentPosition msg;
  msg.request.agent_id = agent_no;
  get_agent_pos_client_.call(msg);
  return msg.response.position;
}

void RVOPlanner::setPlannerGoal(common_msgs::Vector2 goal) {
  rvo_wrapper_msgs::GetNumAgents num_agents_msg;
  get_num_agents_.call(num_agents_msg);
  rvo_wrapper_msgs::SetAgentGoals agent_goal_msg;
  rvo_wrapper_msgs::SimGoals empty;
  agent_goal_msg.request.sim.push_back(empty);
  for (uint32_t i = 0; i < num_agents_msg.response.num_agents; ++i) {
    agent_goal_msg.request.sim[0].agent.push_back(goal);
  }
  set_agent_goals_client_.call(agent_goal_msg);
}
