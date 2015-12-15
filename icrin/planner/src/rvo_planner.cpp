/**
 * @file      rvo_planner.cpp
 * @brief     RVO Planner class
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-06-30
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#include <planner/rvo_planner.hpp>

RVOPlanner::RVOPlanner(ros::NodeHandle* nh) {
  nh_ = nh;
  robot_name_ = ros::this_node::getNamespace();
  robot_name_.erase(0, 1);  // Remove 1 forward slash from robot_name
  this->loadParams();
  this->init();
  this->rosSetup();
}

RVOPlanner::~RVOPlanner() {;}

void RVOPlanner::loadParams() {
  if (!ros::param::has(robot_name_ + "/planner/time_step"))
  {ROS_WARN("Planner- Using default RVO Planner Sim params");}
  int max_neighbors;
  ros::param::param(robot_name_ + "/planner/time_step",
                    planner_settings_.request.time_step, 0.1f);
  ros::param::param(robot_name_ + "/planner/neighbor_dist",
                    planner_settings_.request.defaults.neighbor_dist, 2.0f);
  ros::param::param(robot_name_ + "/planner/max_neighbors", max_neighbors, 20);
  planner_settings_.request.defaults.max_neighbors = uint(max_neighbors);
  ros::param::param(robot_name_ + "/planner/time_horizon_agent",
                    planner_settings_.request.defaults.time_horizon_agent,
                    5.0f);
  ros::param::param(robot_name_ + "/planner/time_horizon_obst",
                    planner_settings_.request.defaults.time_horizon_obst, 5.0f);
  ros::param::param(robot_name_ + "/planner/radius",
                    planner_settings_.request.defaults.radius, 0.5f);
  ros::param::param(robot_name_ + "/planner/max_speed",
                    planner_settings_.request.defaults.max_speed, 0.3f);
  ros::param::param(robot_name_ + "/planner/max_accel",
                    planner_settings_.request.defaults.max_accel, 1.2f);
  ros::param::param(robot_name_ + "/planner/pref_speed",
                    planner_settings_.request.defaults.pref_speed, 0.3f);
}

void RVOPlanner::init() {
  arrived_ = false;
  persistence_ = true;
  PLANNER_ROBOT_ = 0;
  planner_settings_.request.sim_num = 0;
  planner_vel_.x = 0.0f;
  planner_vel_.y = 0.0f;
}

void RVOPlanner::rosSetup() {
  ros::service::waitForService(robot_name_ +
                               "/rvo_wrapper/add_agent");
  ros::service::waitForService(robot_name_ +
                               "/rvo_wrapper/calc_pref_velocities");
  ros::service::waitForService(robot_name_ +
                               "/rvo_wrapper/check_reached_goal");
  ros::service::waitForService(robot_name_ +
                               "/rvo_wrapper/create_rvosim");
  ros::service::waitForService(robot_name_ +
                               "/rvo_wrapper/delete_sim_vector");
  ros::service::waitForService(robot_name_ +
                               "/rvo_wrapper/do_step");
  ros::service::waitForService(robot_name_ +
                               "/rvo_wrapper/get_agent_velocity");
  ros::service::waitForService(robot_name_ +
                               "/rvo_wrapper/set_agent_goals");
  ros::service::waitForService(robot_name_ +
                               "/rvo_wrapper/set_agent_position");
  ros::service::waitForService(robot_name_ +
                               "/rvo_wrapper/set_agent_velocity");
  add_planner_agent_client_ =
    nh_->serviceClient<rvo_wrapper_msgs::AddAgent>(
      robot_name_ + "/rvo_wrapper/add_agent", persistence_);
  calc_pref_velocities_client_ =
    nh_->serviceClient<rvo_wrapper_msgs::CalcPrefVelocities>(
      robot_name_ + "/rvo_wrapper/calc_pref_velocities", persistence_);
  check_reached_goal_client_ =
    nh_->serviceClient<rvo_wrapper_msgs::CheckReachedGoal>(
      robot_name_ + "/rvo_wrapper/check_reached_goal", persistence_);
  create_planner_client_ =
    nh_->serviceClient<rvo_wrapper_msgs::CreateRVOSim>(
      robot_name_ + "/rvo_wrapper/create_rvosim", persistence_);
  delete_planner_client_ =
    nh_->serviceClient<rvo_wrapper_msgs::DeleteSimVector>(
      robot_name_ + "/rvo_wrapper/delete_sim_vector", persistence_);
  do_planner_step_client_ =
    nh_->serviceClient<rvo_wrapper_msgs::DoStep>(
      robot_name_ + "/rvo_wrapper/do_step", persistence_);
  get_agent_vel_client_ =
    nh_->serviceClient<rvo_wrapper_msgs::GetAgentVelocity>(
      robot_name_ + "/rvo_wrapper/get_agent_velocity", persistence_);
  set_agent_goals_client_ =
    nh_->serviceClient<rvo_wrapper_msgs::SetAgentGoals>(
      robot_name_ + "/rvo_wrapper/set_agent_goals", persistence_);
  set_agent_position_ =
    nh_->serviceClient<rvo_wrapper_msgs::SetAgentPosition>(
      robot_name_ + "/rvo_wrapper/set_agent_position", persistence_);
  set_agent_velocity_ =
    nh_->serviceClient<rvo_wrapper_msgs::SetAgentVelocity>(
      robot_name_ + "/rvo_wrapper/set_agent_velocity", persistence_);
}

size_t RVOPlanner::addPlannerAgent(common_msgs::Vector2 agent_pos) {
  rvo_wrapper_msgs::AddAgent msg;
  msg.request.position = agent_pos;
  add_planner_agent_client_.call(msg);
  return msg.response.agent_id;
}

void RVOPlanner::setPlannerVel(common_msgs::Vector2 planner_vel) {
  rvo_wrapper_msgs::SetAgentVelocity msg;
  msg.request.agent_id = 0;
  msg.request.velocity = planner_vel;
  set_agent_velocity_.call(msg);
}

bool RVOPlanner::checkReachedGoal() {
  rvo_wrapper_msgs::CheckReachedGoal msg;
  check_reached_goal_client_.call(msg);
  return msg.response.reached;
}

void RVOPlanner::createPlanner() {
  create_planner_client_.call(planner_settings_);
  if (!planner_settings_.response.ok) {
    ROS_ERROR("RVO Planner not created!");
  }
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
}

void RVOPlanner::setupEnvironment(std::vector<uint32_t> tracker_ids,
                                  std::vector<common_msgs::Vector2> agent_poses,
                                  std::vector<common_msgs::Vector2> agent_vels) {
  tracker_ids_ = tracker_ids;
  agent_positions_ = agent_poses;
  agent_velocities_ = agent_vels;
}

common_msgs::Vector2 RVOPlanner::planStep() {
  this->createPlanner();
  this->setupPlanner();
  this->calcPrefVelocity();
  this->doSimStep();
  arrived_ = this->checkReachedGoal();
  planner_vel_ = this->getPlannerVel();
  this->deletePlanner();
  return planner_vel_;
}

void RVOPlanner::setupPlanner() {
  // Setup Planner agent
  this->addPlannerAgent(curr_pose_);
  this->setPlannerVel(planner_vel_);
  // Setup other agents
  for (uint64_t i = 0; i < agent_positions_.size(); ++i) {
    this->addPlannerAgent(agent_positions_[i]);
  }
  this->setAgentVelocities(agent_velocities_);
  // Set planner goal
  rvo_wrapper_msgs::SetAgentGoals agent_goal_msg;
  rvo_wrapper_msgs::SimGoals empty;
  agent_goal_msg.request.sim.push_back(empty);
  agent_goal_msg.request.sim[0].agent.push_back(planner_goal_);
  set_agent_goals_client_.call(agent_goal_msg);
}

void RVOPlanner::calcPrefVelocity() {
  rvo_wrapper_msgs::CalcPrefVelocities msg;
  calc_pref_velocities_client_.call(msg);
}

void RVOPlanner::doSimStep() {
  rvo_wrapper_msgs::DoStep msg;
  do_planner_step_client_.call(msg);
}

void RVOPlanner::deletePlanner() {
  rvo_wrapper_msgs::DeleteSimVector msg;
  delete_planner_client_.call(msg);
}

common_msgs::Vector2 RVOPlanner::getPlannerVel() {
  rvo_wrapper_msgs::GetAgentVelocity msg;
  msg.request.agent_id.push_back(PLANNER_ROBOT_);
  get_agent_vel_client_.call(msg);
  if (!msg.response.ok) {
    ROS_ERROR("Planner- Velocity not received from RVO Library");
    common_msgs::Vector2 vel;
    vel.x = 0.0f;
    vel.y = 0.0f;
    return vel;
  }
  return msg.response.velocity[PLANNER_ROBOT_];
}

/*void RVOPlanner::setAgentPositions(std::vector<common_msgs::Vector2>
                                   agent_positions) {
  rvo_wrapper_msgs::SetAgentPosition msg;
  for (uint8_t i = 1; i < agent_positions.size(); ++i) {
    msg.request.agent_id = i;
    msg.request.position.x = agent_positions[i].x;
    msg.request.position.y = agent_positions[i].y;
    set_agent_position_.call(msg);
  }
}*/

void RVOPlanner::setAgentVelocities(std::vector<common_msgs::Vector2>
                                    agent_velocities) {
  rvo_wrapper_msgs::SetAgentVelocity msg;
  for (uint8_t i = 1; i < agent_velocities.size(); ++i) {
    msg.request.agent_id = i;
    msg.request.velocity.x = agent_velocities[i].x;
    msg.request.velocity.y = agent_velocities[i].y;
    set_agent_velocity_.call(msg);
  }
}

void RVOPlanner::setCurrPose(common_msgs::Vector2 curr_pose) {
  curr_pose_ = curr_pose;
  // rvo_wrapper_msgs::SetAgentPosition msg;
  // msg.request.agent_id = PLANNER_ROBOT_;
  // msg.request.position.x = curr_pose.x;
  // msg.request.position.y = curr_pose.y;
  // set_agent_position_.call(msg);
}

void RVOPlanner::setCurrVel(common_msgs::Vector2 curr_vel) {
  planner_vel_ = curr_vel;
}

void RVOPlanner::setPlannerGoal(common_msgs::Vector2 goal) {
  planner_goal_ = goal;
}
