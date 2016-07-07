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
  robot_name_.erase(0, 1);  // Remove 1 forward slash from robot_name
  robot_name_ = "";
  model_name_ = ros::this_node::getName();
  model_name_.erase(0, robot_name_.length());  // Remove robot name
  model_name_ = "/model";
  this->loadParams();
  this->init();
  this->rosSetup();
}

SimWrapper::~SimWrapper() {
}

void SimWrapper::loadParams() {
  if (!ros::param::has(robot_name_ + model_name_ + "/time_step"))
  {ROS_WARN("Model- Using default RVO Model Sim params");}
  int max_neighbors;
  ros::param::param(robot_name_ + model_name_ + "/time_step", time_step_, 0.1f);
  ros::param::param(robot_name_ + model_name_ +
                    "/neighbor_dist", neighbor_dist_, 2.0f);
  ros::param::param(robot_name_ + model_name_ + "/max_neighbors", max_neighbors,
                    20);
  ros::param::param(robot_name_ + model_name_ + "/time_horizon_agent",
                    time_horizon_agent_, 5.0f);
  ros::param::param(robot_name_ + model_name_ + "/time_horizon_obst",
                    time_horizon_obst_, 5.0f);
  ros::param::param(robot_name_ + model_name_ + "/radius", radius_, 0.5f);
  ros::param::param(robot_name_ + model_name_ + "/max_speed", max_speed_, 1.2f);
  ros::param::param(robot_name_ + model_name_ + "/max_accel", max_accel_, 2.4f);
  ros::param::param(robot_name_ + model_name_ +
                    "/pref_speed", pref_speed_, 0.6f);
  max_neighbors_ = max_neighbors;
  bool robot_model;
  ros::param::param(robot_name_ + model_name_ + "/robot_model",
                    robot_model, true);
  if (robot_model) {
    ros::param::param(robot_name_ + "/planner/max_speed",
                      planner_max_speed_, 0.6f);
    ros::param::param(robot_name_ + "/planner/max_accel",
                      planner_max_accel_, 1.2f);
    ros::param::param(robot_name_ + "/planner/pref_speed",
                      planner_pref_speed_, 0.3f);
  }
}

void SimWrapper::init() {
  use_rvo_lib_ = true;
  debug_ = false;
  persistence_ = true;
  null_vect_.x = 0.0f;
  null_vect_.y = 0.0f;
}

void SimWrapper::rosSetup() {
  ros::service::waitForService(robot_name_ + "/rvo_wrapper/add_agent");
  ros::service::waitForService(robot_name_ + "/rvo_wrapper/calc_pref_velocities");
  ros::service::waitForService(robot_name_ + "/rvo_wrapper/create_rvosim");
  ros::service::waitForService(robot_name_ + "/rvo_wrapper/delete_sim_vector");
  ros::service::waitForService(robot_name_ + "/rvo_wrapper/do_step");
  ros::service::waitForService(robot_name_ + "/rvo_wrapper/get_agent_velocity");
  ros::service::waitForService(robot_name_ + "/rvo_wrapper/set_agent_goals");
  ros::service::waitForService(robot_name_ + "/rvo_wrapper/set_agent_velocity");
  ros::service::waitForService(robot_name_ + "/rvo_wrapper/get_agent_position");
  ros::service::waitForService(robot_name_ + "/rvo_wrapper/set_agent_max_speed");
  ros::service::waitForService(robot_name_ + "/rvo_wrapper/set_agent_max_accel");
  ros::service::waitForService(robot_name_ + "/rvo_wrapper/set_agent_pref_speed");
  add_sim_agent_client_ =
    nh_->serviceClient<rvo_wrapper_msgs::AddAgent>(
      robot_name_ + "/rvo_wrapper/add_agent", persistence_);
  calc_pref_velocities_client_ =
    nh_->serviceClient<rvo_wrapper_msgs::CalcPrefVelocities>(
      robot_name_ + "/rvo_wrapper/calc_pref_velocities", persistence_);
  create_sims_client_ =
    nh_->serviceClient<rvo_wrapper_msgs::CreateRVOSim>(
      robot_name_ + "/rvo_wrapper/create_rvosim", persistence_);
  delete_sims_client_ =
    nh_->serviceClient<rvo_wrapper_msgs::DeleteSimVector>(
      robot_name_ + "/rvo_wrapper/delete_sim_vector", persistence_);
  do_sim_step_client_ =
    nh_->serviceClient<rvo_wrapper_msgs::DoStep>(
      robot_name_ + "/rvo_wrapper/do_step", persistence_);
  get_agent_vel_client_ =
    nh_->serviceClient<rvo_wrapper_msgs::GetAgentVelocity>(
      robot_name_ + "/rvo_wrapper/get_agent_velocity", persistence_);
  set_agent_goals_client_ =
    nh_->serviceClient<rvo_wrapper_msgs::SetAgentGoals>(
      robot_name_ + "/rvo_wrapper/set_agent_goals", persistence_);
  set_agent_vel_client_ =
    nh_->serviceClient<rvo_wrapper_msgs::SetAgentVelocity>(
      robot_name_ + "/rvo_wrapper/set_agent_velocity", persistence_);
  get_agent_position_client_ =
    nh_->serviceClient<rvo_wrapper_msgs::GetAgentPosition>(
      robot_name_ + "/rvo_wrapper/get_agent_position", persistence_);
  set_agent_max_speed_client_ =
    nh_->serviceClient<rvo_wrapper_msgs::SetAgentMaxSpeed>(
      robot_name_ + "/rvo_wrapper/set_agent_max_speed", persistence_);
  set_agent_max_accel_client_ =
    nh_->serviceClient<rvo_wrapper_msgs::SetAgentMaxAccel>(
      robot_name_ + "/rvo_wrapper/set_agent_max_accel", persistence_);
  set_agent_pref_speed_client_ =
    nh_->serviceClient<rvo_wrapper_msgs::SetAgentPrefSpeed>(
      robot_name_ + "/rvo_wrapper/set_agent_pref_speed", persistence_);
}

std::vector<uint32_t> SimWrapper::goalSampling(
  std::vector<geometry_msgs::Pose2D> sample_space, float sample_resolution) {
  std::vector<uint32_t> sim_ids;
  if (debug_) {ROS_INFO("ModelS- Sampling!");}
  float min_x = sample_space[0].x;
  float min_y = sample_space[0].y;
  float max_x = sample_space[1].x;
  float max_y = sample_space[1].y;
  float sample_res = sample_resolution;
  if (!(min_x <= max_x) || !(min_y <= max_y)) {
    ROS_ERROR("ModelWrapper: Sampling x,y values are incorrect!");
  } else {
    size_t size_x = max_x - min_x;
    size_t size_y = max_y - min_y;
    size_t n_goals = ((size_x / sample_res) + 1) * ((size_y / sample_res) + 1);
    std::vector<geometry_msgs::Pose2D> goal_sequence;
    geometry_msgs::Pose2D goal;
    goal.theta = 0.0;
    for (int i = 0; (min_x + (sample_res * i)) <= max_x; ++i) {
      for (int j = 0; (min_y + (sample_res * j)) <= max_y; ++j) {
        goal.x = min_x + (sample_res * i);
        goal.y = min_y + (sample_res * j);
        goal_sequence.push_back(goal);
        if (debug_) {
          ROS_INFO_STREAM("i: " << i << " min_x: " << min_x <<
                          " max_x: " << max_x << " Gx: " << goal.x <<
                          " j: " << j << " min_y: " << min_y <<
                          " max_y: " << max_y << " Gy: " << goal.y);
        }
      }
    }
    if (debug_) {
      ROS_INFO_STREAM("SimWrapper- min_x: " << min_x << " min_y: " << min_y <<
                      " max_x: " << max_x << " max_y: " << max_y <<
                      " GSeqSize: " << goal_sequence.size() <<
                      " nGoals: " << n_goals);
    }
    if (goal_sequence.size() == n_goals) {
      sim_ids = goalSequence(goal_sequence);
      sampling_goal_sequence_ = goal_sequence;
    } else {ROS_ERROR("SimWrapper: Sampling goal number does not match!");}
  }
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
  sim_msg.request.defaults.max_accel = max_accel_;
  sim_msg.request.defaults.pref_speed = pref_speed_;
  create_sims_client_.call(sim_msg);
  std::vector<uint32_t> sim_ids = sim_msg.response.sim_ids;
  if (debug_) {
    ROS_INFO_STREAM("ModelANo: " << model_agent_no_ <<
                    " GoalNo: " << goal_no <<
                    " SimNum: " << sim_msg.request.sim_num);
    ROS_INFO_STREAM("IDs:" << sim_ids.front() << " - " << sim_ids.back());
  }

  if (!sim_msg.response.ok) {
    ROS_ERROR("ModelS- Goal sequence sims failed!");
  }

  // Create Agents with Positions
  for (size_t i = 0; i < agent_no_; ++i) {
    rvo_wrapper_msgs::AddAgent agent_msg;
    agent_msg.request.sim_ids = sim_ids;
    agent_msg.request.position = agent_poses_[i];
    if (debug_) {
      ROS_INFO_STREAM("SIMW- A:" << i << "Px: " << agent_poses_[i].x <<
                      ", Py:" << agent_poses_[i].y);
    }
    add_sim_agent_client_.call(agent_msg);
  }

  // Set Agent Velocities
  for (size_t i = 0; i < agent_no_; ++i) {
    rvo_wrapper_msgs::SetAgentVelocity vel_msg;
    vel_msg.request.sim_ids = sim_ids;
    vel_msg.request.velocity = agent_vels_[i];
    if (debug_) {
      ROS_INFO_STREAM("SIMW- A:" << i << "Vx: " << agent_vels_[i].x <<
                      ", Vy:" << agent_vels_[i].y);
    }
    set_agent_vel_client_.call(vel_msg);
  }

  // Transform Pose2D goals into Vector2 goals
  std::vector<common_msgs::Vector2> goals;
  // ROS_ERROR_STREAM("GOAL_NO: " << goal_no);
  // ROS_ERROR_STREAM("GOAL_SEQ_SIZE: " << goal_sequence.size());
  goals.resize(goal_no);
  for (size_t i = 0; i < goal_no; ++i) {
    goals[i].x = goal_sequence[i].x;
    goals[i].y = goal_sequence[i].y;
    // ROS_ERROR_STREAM("GOAL_X: " << goal_sequence[i].x);
  }

  // Set Agent Goals
  rvo_wrapper_msgs::SetAgentGoals goal_msg;
  goal_msg.request.sim_ids = sim_ids;
  size_t sim_no = sim_ids[1] - sim_ids[0] + 1;
  goal_msg.request.sim.resize(sim_no);
  // sim_msg.request.sim_num
  size_t sim_id = 0;
  for (size_t model_agent = 0; model_agent < model_agent_no_; ++model_agent) {
    for (size_t goal = 0; goal < goal_no; ++goal) {
      for (size_t agent = 0; agent < agent_no_; ++agent) {
        if ((agent == 0) && (robot_model_)) {
          goal_msg.request.sim[sim_id].agent.push_back(robot_goal_);
          if (debug_) {
            ROS_INFO_STREAM("SIMW- A:" << agent << " Gx: " << robot_goal_.x <<
                            ", Gy: " << robot_goal_.y);
          }
        } else {
          if (agent == model_agents_[model_agent]) {
          goal_msg.request.sim[sim_id].agent.push_back(goals[goal]);
          if (debug_) {
          // ROS_ERROR("MODELLING!");
          ROS_INFO_STREAM("SIMW- A:" << agent << " Gx: " << goals[goal].x
                          << ", Gy: " << goals[goal].y);
          }
          } else {
            goal_msg.request.sim[sim_id].agent.push_back(null_vect_);
            if (debug_) {
              // ROS_ERROR("NOT MODELLING!");
              ROS_INFO_STREAM("SIMW- A:" << agent << " Gx: " << "null"
                              << ", Gy: " << "null");
            }
          }
        }
      }
      sim_id++;
    }
  }
  if (sim_id == sim_no) {
    set_agent_goals_client_.call(goal_msg);
  } else {
    ROS_ERROR("ModelS- Set Sim Goals check failed!");
  }

  return sim_ids;
}

std::vector<common_msgs::Vector2> SimWrapper::calcSimVels(
  std::vector<uint32_t> sims, size_t n_goals) {
  if (debug_) {
    ROS_INFO_STREAM("SimsSize: " << sims.size());
    ROS_INFO_STREAM("SimIDs: " << sims[0] << " - " << sims[1]);
  }
  // Calc Preferred Velocities
  rvo_wrapper_msgs::CalcPrefVelocities pref_vel;
  pref_vel.request.sim_ids = sims;
  calc_pref_velocities_client_.call(pref_vel);

  // Run Sims
  rvo_wrapper_msgs::DoStep run_sims;
  run_sims.request.sim_ids = sims;
  do_sim_step_client_.call(run_sims);
  if (!run_sims.response.ok) {ROS_ERROR("SimSteps could not be run!");}

  // Get Velocities
  rvo_wrapper_msgs::GetAgentVelocity get_vels;
  get_vels.request.sim_ids = sims;
  for (size_t model_agent = 0; model_agent < model_agent_no_; ++model_agent) {
    for (size_t goal = 0; goal < n_goals; ++goal) {
      get_vels.request.agent_id.push_back(model_agents_[model_agent]);
    }
  }
  // ROS_INFO_STREAM("NGoals:" << n_goals);
  get_agent_vel_client_.call(get_vels);
  if (!get_vels.response.ok) {ROS_ERROR("SimVels could not be acquired!");}

  // Delete Sims
  rvo_wrapper_msgs::DeleteSimVector del_sims;
  del_sims.request.sim_ids = sims;
  delete_sims_client_.call(del_sims);
  if (!del_sims.response.ok) {ROS_ERROR("Sims could not be deleted!");}
  // Return Velocities
  return get_vels.response.velocity;
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

model_msgs::InteractivePrediction SimWrapper::interactiveSim(
  std::vector<common_msgs::Vector2> a_goals, size_t foresight, float time_step
  // , std::vector<geometry_msgs::Pose2D> goals
) {
  model_msgs::InteractivePrediction inter_pred_msg;
  inter_pred_msg.foresight = foresight;
  inter_pred_msg.agent.resize(agent_no_ - 1);  // TODO(Alex): Bad Practice
  rvo_wrapper_msgs::CreateRVOSim sim_msg;
  sim_msg.request.sim_num = 1;
  sim_msg.request.time_step = time_step;
  sim_msg.request.defaults.neighbor_dist = neighbor_dist_;
  sim_msg.request.defaults.max_neighbors = max_neighbors_;
  sim_msg.request.defaults.time_horizon_agent = time_horizon_agent_;
  sim_msg.request.defaults.time_horizon_obst = time_horizon_obst_;
  sim_msg.request.defaults.radius = radius_;
  sim_msg.request.defaults.max_speed = max_speed_;
  sim_msg.request.defaults.max_accel = max_accel_;
  sim_msg.request.defaults.pref_speed = pref_speed_;
  create_sims_client_.call(sim_msg);
  uint sim_id = sim_msg.response.sim_ids[0];

  // Create Agents with Positions
  for (size_t agent = 0; agent < agent_no_; ++agent) {
    rvo_wrapper_msgs::AddAgent agent_msg;
    agent_msg.request.sim_ids.push_back(sim_id);
    agent_msg.request.position = agent_poses_[agent];
    add_sim_agent_client_.call(agent_msg);
  }

  // Set Agent Velocities
  for (size_t agent = 0; agent < agent_no_; ++agent) {
    rvo_wrapper_msgs::SetAgentVelocity vel_msg;
    vel_msg.request.sim_ids.push_back(sim_id);
    vel_msg.request.velocity = agent_vels_[agent];
    set_agent_vel_client_.call(vel_msg);
  }

  // Setup Planner parameters (Different from modelling params)
  if (robot_model_) {
    rvo_wrapper_msgs::SetAgentMaxSpeed max_speed_msg;
    max_speed_msg.request.sim_ids.push_back(sim_id);
    max_speed_msg.request.agent_id = 0;
    max_speed_msg.request.max_speed = planner_max_speed_;
    rvo_wrapper_msgs::SetAgentMaxAccel max_accel_msg;
    max_accel_msg.request.sim_ids.push_back(sim_id);
    max_accel_msg.request.agent_id = 0;
    max_accel_msg.request.max_accel = planner_max_accel_;
    rvo_wrapper_msgs::SetAgentPrefSpeed pref_speed_msg;
    pref_speed_msg.request.sim_ids.push_back(sim_id);
    pref_speed_msg.request.agent_id = 0;
    pref_speed_msg.request.pref_speed = planner_pref_speed_;
    set_agent_max_speed_client_.call(max_speed_msg);
    set_agent_max_accel_client_.call(max_accel_msg);
    set_agent_pref_speed_client_.call(pref_speed_msg);
  }

  // Transform Pose2D goals into Vector2 goals
  // std::vector<common_msgs::Vector2> a_goals;
  // size_t goal_no = goals.size();
  // a_goals.resize(goal_no);
  // for (size_t goal = 0; goal < goal_no; ++goal) {
  //   a_goals[goal].x = goals[goal].x;
  //   a_goals[goal].y = goals[goal].y;
  // }

  rvo_wrapper_msgs::SimGoals sim_goals_msg;
  // for (size_t agent = 0; agent < agent_no_; ++agent) {
  //   sim_goals_msg.agent.push_back(a_goals[agent]);
  // }
  sim_goals_msg.agent = a_goals;
  rvo_wrapper_msgs::SetAgentGoals goal_msg;
  goal_msg.request.sim_ids.push_back(sim_id);
  goal_msg.request.sim.push_back(sim_goals_msg);
  set_agent_goals_client_.call(goal_msg);


  // Run Sims
  for (size_t i = 0; i < foresight; ++i) {
    // Calc Preferred Velocities
    rvo_wrapper_msgs::CalcPrefVelocities pref_vel;
    pref_vel.request.sim_ids.push_back(sim_id);
    calc_pref_velocities_client_.call(pref_vel);

    rvo_wrapper_msgs::DoStep run_sims;
    run_sims.request.sim_ids.push_back(sim_id);
    do_sim_step_client_.call(run_sims);
    if (!run_sims.response.ok) {ROS_ERROR("SimSteps could not be run!");}

    // Get Positions
    for (size_t agent = 0; agent < agent_no_; ++agent) {
      rvo_wrapper_msgs::GetAgentPosition get_poses;
      get_poses.request.sim_ids.push_back(sim_id);
      get_poses.request.agent_id = agent;
      get_agent_position_client_.call(get_poses);
      if (!get_poses.response.ok) {
        ROS_ERROR("SimPoses could not be acquired!");
      } else {
        geometry_msgs::Pose2D pose;
        pose.x = get_poses.response.position.x;
        pose.y = get_poses.response.position.y;
        if ((agent == 0) && (robot_model_)) {
          inter_pred_msg.planner_pose.push_back(pose);
        } else {
          inter_pred_msg.agent[agent - 1].pose.push_back(pose);
          // TODO(Alex): Bad practice
        }
      }
    }
  }

  rvo_wrapper_msgs::DeleteSimVector del_msg;
  del_msg.request.sim_ids.push_back(sim_id);
  delete_sims_client_.call(del_msg);

  return inter_pred_msg;
}
