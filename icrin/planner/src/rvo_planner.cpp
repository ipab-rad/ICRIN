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

void RVOPlanner::init() {
  curr_pos_.x = 0.0f;
  curr_pos_.y = 0.0f;
  target_pos_.x = 0.0f;
  target_pos_.y = 0.0f;
  planner_settings_.request.sim_num = 0;
  planner_settings_.request.time_step = 0.1f;
  planner_settings_.request.defaults.neighbor_dist = 2.0f;
  planner_settings_.request.defaults.max_neighbors = 20;
  planner_settings_.request.defaults.time_horizon_agent = 5.0f;
  planner_settings_.request.defaults.time_horizon_obst = 5.0f;
  planner_settings_.request.defaults.radius = 0.5f;
  planner_settings_.request.defaults.max_speed = 0.3f;
}

void RVOPlanner::rosSetup() {
  ros::service::waitForService("/rvo_wrapper/add_agent");
  add_planner_agent_client_ =
    nh_->serviceClient<rvo_wrapper_msgs::AddAgent>(
      "/rvo_wrapper/add_agent", true);
  ros::service::waitForService("/rvo_wrapper/check_reached_goal");
  check_reached_goal_client_ =
    nh_->serviceClient<rvo_wrapper_msgs::CheckReachedGoal>(
      "/rvo_wrapper/check_reached_goal", true);
  ros::service::waitForService("/rvo_wrapper/calc_pref_velocities");
  calc_pref_velocities_client_ =
    nh_->serviceClient<rvo_wrapper_msgs::CalcPrefVelocities>(
      "/rvo_wrapper/calc_pref_velocities", true);
  ros::service::waitForService("/rvo_wrapper/create_rvosim");
  create_planner_client_ =
    nh_->serviceClient<rvo_wrapper_msgs::CreateRVOSim>(
      "/rvo_wrapper/create_rvosim", true);
  ros::service::waitForService("/rvo_wrapper/do_step");
  do_planner_step_client_ =
    nh_->serviceClient<rvo_wrapper_msgs::DoStep>(
      "/rvo_wrapper/do_step", true);
  ros::service::waitForService("/rvo_wrapper/get_agent_position");
  get_agent_pos_client_ =
    nh_->serviceClient<rvo_wrapper_msgs::GetAgentPosition>(
      "/rvo_wrapper/get_agent_position", true);
  ros::service::waitForService("/rvo_wrapper/get_num_agents");
  get_num_agents_ =
    nh_->serviceClient<rvo_wrapper_msgs::GetNumAgents>(
      "/rvo_wrapper/get_num_agents", true);
  ros::service::waitForService("/rvo_wrapper/set_agent_goals");
  set_agent_goals_client_ =
    nh_->serviceClient<rvo_wrapper_msgs::SetAgentGoals>(
      "/rvo_wrapper/set_agent_goals", true);
  set_agent_defaults_ =
    nh_->serviceClient<rvo_wrapper_msgs::SetAgentDefaults>(
      "/rvo_wrapper/set_agent_defaults", true);
  set_time_step_ =
    nh_->serviceClient<rvo_wrapper_msgs::SetTimeStep>(
      "/rvo_wrapper/set_time_step", true);
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
  create_planner_client_.call(planner_settings_);
  if (planner_settings_.response.res) {
    ROS_INFO("Planner created");
  } else { ROS_ERROR("Planner not created!"); };
}

void RVOPlanner::doSimStep() {
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
  target_pos_ = goal;
  set_agent_goals_client_.call(agent_goal_msg);
}

void RVOPlanner::setPlannerSettings(float time_step,
                                    rvo_wrapper_msgs::AgentDefaults defaults) {
  // Save for new planner generation
  planner_settings_.request.time_step = time_step;
  planner_settings_.request.defaults.neighbor_dist = defaults.neighbor_dist;
  planner_settings_.request.defaults.max_neighbors = defaults.max_neighbors;
  planner_settings_.request.defaults.time_horizon_agent =
    defaults.time_horizon_agent;
  planner_settings_.request.defaults.time_horizon_obst =
    defaults.time_horizon_obst;
  planner_settings_.request.defaults.radius = defaults.radius;
  planner_settings_.request.defaults.max_speed = defaults.max_speed;
  // Set new planner settings
  rvo_wrapper_msgs::SetTimeStep planner_time_step;
  // planner_time_step.request.sim_ids = planner_settings_.request.sim_num;
  planner_time_step.request.time_step = planner_settings_.request.time_step;
  set_time_step_.call(planner_time_step);
  rvo_wrapper_msgs::SetAgentDefaults agent_defaults;
  // agent_defaults.request.sim_ids = planner_settings_.request.sim_num;
  agent_defaults.request.defaults = planner_settings_.request.defaults;
  set_agent_defaults_.call(agent_defaults);
}

void RVOPlanner::planStep() {
  curr_pos_ = this->getAgentPos(0);
  this->calcPrefVelocities();
  this->doSimStep();
}

geometry_msgs::Pose2D RVOPlanner::getCurrPose() {
  geometry_msgs::Pose2D msg;
  msg.x = curr_pos_.x;
  msg.y = curr_pos_.y;
  msg.theta = 0.0f;
  return msg;
}

geometry_msgs::Pose2D RVOPlanner::getTargetPose() {
  geometry_msgs::Pose2D msg;
  msg.x = target_pos_.x;
  msg.y = target_pos_.y;
  msg.theta = 0.0f;
  return msg;
}
