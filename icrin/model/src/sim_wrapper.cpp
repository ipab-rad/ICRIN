/**
 * @file      sim_wrapper.cpp
 * @brief     Sim Wrapper communicates with rvo_wrapper for running simulations
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-08-22
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#include <model/sim_wrapper.hpp>

SimWrapper::SimWrapper(ros::NodeHandle* nh) {
  nh_ = nh;
  robot_name_ = ros::this_node::getNamespace();
  robot_name_.erase (0, 1); // Remove 1 forward slash from robot_name
  model_name_ = ros::this_node::getName();
  model_name_.erase (0, robot_name_.length()); // Remove robot name
  this->loadParams();
  this->init();
  this->rosSetup();
}

SimWrapper::~SimWrapper() {
}

void SimWrapper::loadParams() {
  if (!ros::param::has("/model/time_step"))
  {ROS_WARN("Model- Using default Sim params");}
  int max_neighbors;
  ros::param::param("/model/time_step", time_step_, 0.1f);
  ros::param::param("/model/neighbor_dist", neighbor_dist_, 2.0f);
  ros::param::param("/model/max_neighbors", max_neighbors, 20);
  ros::param::param("/model/time_horizon_agent", time_horizon_agent_, 5.0f);
  ros::param::param("/model/time_horizon_obst", time_horizon_obst_, 5.0f);
  ros::param::param("/model/radius", radius_, 0.5f);
  ros::param::param("/model/max_speed", max_speed_, 0.3f);
  max_neighbors_ = max_neighbors;
}

void SimWrapper::init() {
  use_rvo_lib_ = true;
  null_vect_.x = 0.0f;
  null_vect_.y = 0.0f;
}

void SimWrapper::rosSetup() {
  ros::service::waitForService(robot_name_ + model_name_ +
                               "/rvo_wrapper/add_agent");
  ros::service::waitForService(robot_name_ + model_name_ +
                               "/rvo_wrapper/calc_pref_velocities");
  ros::service::waitForService(robot_name_ + model_name_ +
                               "/rvo_wrapper/create_rvosim");
  ros::service::waitForService(robot_name_ + model_name_ +
                               "/rvo_wrapper/delete_sim_vector");
  ros::service::waitForService(robot_name_ + model_name_ +
                               "/rvo_wrapper/do_step");
  ros::service::waitForService(robot_name_ + model_name_ +
                               "/rvo_wrapper/get_agent_velocity");
  ros::service::waitForService(robot_name_ + model_name_ +
                               "/rvo_wrapper/set_agent_goals");
  ros::service::waitForService(robot_name_ + model_name_ +
                               "/rvo_wrapper/set_agent_velocity");

  add_sim_agent_client_ =
    nh_->serviceClient<rvo_wrapper_msgs::AddAgent>(
      robot_name_ + model_name_ + "/rvo_wrapper/add_agent", true);
  calc_pref_velocities_client_ =
    nh_->serviceClient<rvo_wrapper_msgs::CalcPrefVelocities>(
      robot_name_ + model_name_ + "/rvo_wrapper/calc_pref_velocities", true);
  create_sims_client_ =
    nh_->serviceClient<rvo_wrapper_msgs::CreateRVOSim>(
      robot_name_ + model_name_ + "/rvo_wrapper/create_rvosim", true);
  delete_sims_client_ =
    nh_->serviceClient<rvo_wrapper_msgs::DeleteSimVector>(
      robot_name_ + model_name_ + "/rvo_wrapper/delete_sim_vector", true);
  do_sim_step_client_ =
    nh_->serviceClient<rvo_wrapper_msgs::DoStep>(
      robot_name_ + model_name_ + "/rvo_wrapper/do_step", true);
  get_agent_vel_client_ =
    nh_->serviceClient<rvo_wrapper_msgs::GetAgentVelocity>(
      robot_name_ + model_name_ + "/rvo_wrapper/get_agent_velocity", true);
  set_agent_goals_client_ =
    nh_->serviceClient<rvo_wrapper_msgs::SetAgentGoals>(
      robot_name_ + model_name_ + "/rvo_wrapper/set_agent_goals", true);
  set_agent_vel_client_ =
    nh_->serviceClient<rvo_wrapper_msgs::SetAgentVelocity>(
      robot_name_ + model_name_ + "/rvo_wrapper/set_agent_velocity", true);
}

std::vector<uint32_t> SimWrapper::goalSampling(
  std::vector<geometry_msgs::Pose2D> sample_space, float sample_resolution) {
  std::vector<uint32_t> sim_ids;
  ROS_INFO("Model- Sampling!");
  return sim_ids;
}

std::vector<uint32_t> SimWrapper::goalSequence(
  std::vector<geometry_msgs::Pose2D> goal_sequence) {
  size_t goal_no = goal_sequence.size();
  // Create simulations
  rvo_wrapper_msgs::CreateRVOSim sim_msg;
  sim_msg.request.sim_num = model_agent_no_ * goal_no;
  sim_msg.request.time_step = time_step_;
  sim_msg.request.defaults.neighbor_dist = neighbor_dist_;
  sim_msg.request.defaults.max_neighbors = max_neighbors_;
  sim_msg.request.defaults.time_horizon_agent = time_horizon_agent_;
  sim_msg.request.defaults.time_horizon_obst = time_horizon_obst_;
  sim_msg.request.defaults.radius = radius_;
  sim_msg.request.defaults.max_speed = max_speed_;
  create_sims_client_.call(sim_msg);
  std::vector<uint32_t> sim_ids = sim_msg.response.sim_ids;
  if (sim_msg.response.ok) {
    ROS_INFO("Model- Goal sequence sims created!");
  } else {
    ROS_WARN("Model- Goal sequence sims failed!");
  }

  // Create Agents with Positions
  for (size_t i = 0; i < agent_no_; ++i) {
    rvo_wrapper_msgs::AddAgent agent_msg;
    agent_msg.request.sim_ids = sim_ids;
    agent_msg.request.position = agent_poses_[i];
    add_sim_agent_client_.call(agent_msg);
  }

  // Set Agent Velocities
  for (size_t i = 0; i < agent_no_; ++i) {
    rvo_wrapper_msgs::SetAgentVelocity vel_msg;
    vel_msg.request.sim_ids = sim_ids;
    vel_msg.request.velocity = agent_vels_[i];
    set_agent_vel_client_.call(vel_msg);
  }

  // Transform Pose2D goals into Vector2 goals
  std::vector<common_msgs::Vector2> goals;
  goals.resize(goal_no);
  for (size_t i = 0; i < goal_no; ++i) {
    goals[i].x = goal_sequence[i].x;
    goals[i].y = goal_sequence[i].y;
  }

  // Set Agent Goals
  rvo_wrapper_msgs::SetAgentGoals goal_msg;
  goal_msg.request.sim_ids = sim_ids;
  size_t sim_no = sim_ids[1] - sim_ids[0] + 1;
  goal_msg.request.sim.resize(sim_no);
  size_t sim_id = 0;
  for (size_t model_agent = 0; model_agent < model_agent_no_; ++model_agent) {
    for (size_t goal = 0; goal < goal_no; ++goal) {
      for (size_t agent = 0; agent < agent_no_; ++agent) {
        if (agent == model_agents_[model_agent]) {
          goal_msg.request.sim[sim_id].agent.push_back(goals[goal]);
        } else {
          goal_msg.request.sim[sim_id].agent.push_back(null_vect_);
        }
      }
      sim_id++;
    }
  }
  if (sim_id == (sim_no - 1)) {
    ROS_INFO("Model- Set Sim Goals check");
    set_agent_goals_client_.call(goal_msg);
  } else {
    ROS_WARN("Model- Set Sim Goals check failed!");
  }

  // Calc Pref Velocities (Opt)

  return sim_ids;
}

void SimWrapper::setRobotModel(bool robot_model) {
  robot_model_ = robot_model;
}

void SimWrapper::setModelAgents(std::vector<uint8_t> model_agents) {
  model_agents_ = model_agents;
  model_agent_no_ = model_agents_.size();
}

void SimWrapper::setRobotGoal(geometry_msgs::Pose2D robot_goal) {
  robot_goal_.x = robot_goal.x;
  robot_goal_.y = robot_goal.y;
}

void SimWrapper::setEnvironment(std::vector<geometry_msgs::Pose2D> agent_poses,
                                std::vector<geometry_msgs::Twist> agent_vels) {
  agent_poses_.clear();
  agent_vels_.clear();
  agent_no_ = agent_poses.size();
  agent_poses_.resize(agent_no_);
  agent_vels_.resize(agent_no_);
  for (size_t i = 0; i < agent_no_; ++i) {
    agent_poses_[i].x = agent_poses[i].x;
    agent_poses_[i].y = agent_poses[i].y;
    agent_vels_[i].x = agent_vels[i].linear.x;
    agent_vels_[i].y = agent_vels[i].linear.y;
  }
}
