/**
 * @file      model_wrapper.cpp
 * @brief     Model Wrapper to manage modelling functions and sim wrapper
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-08-22
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#include "model/model_wrapper.hpp"

ModelWrapper::ModelWrapper(ros::NodeHandle* nh) {
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
  sim_wrapper_ = new SimWrapper(nh_);
  ROS_INFO("MODEL_WRAPPER: Initialised!");
}

ModelWrapper::~ModelWrapper() {
  ros::param::del(robot_name_ + model_name_);
  if (use_rvo_lib_) {
    delete sim_wrapper_;
    sim_wrapper_ = NULL;
  }
}

void ModelWrapper::loadParams() {
  // Model Params
  if (!ros::param::has(robot_name_ + model_name_ + "/robot_model"))
  {ROS_WARN("ModelW- Robot model by default");}
  ros::param::param(robot_name_ + model_name_ + "/robot_model",
                    robot_model_, true);
  if (!ros::param::has(robot_name_ + model_name_ + "/goal_sum_prior"))
  {ROS_WARN("ModelW- Using default Model params");}
  ros::param::param(robot_name_ + model_name_ + "/goal_sum_prior",
                    goal_sum_prior_, 0.001f);
  ros::param::param(robot_name_ + model_name_ + "/goal_history_discount",
                    goal_history_discount_, 0.5f);
  ros::param::param(robot_name_ + model_name_ + "/goal_inference_history",
                    goal_inference_history_, 10);
  ros::param::param(robot_name_ + model_name_ + "/velocity_average_window",
                    velocity_average_window_, 10);
  ros::param::param(robot_name_ + model_name_ + "/prior_lambda",
                    prior_lambda_, 0.5f);
  ros::param::param(robot_name_ + model_name_ + "/max_accel", max_accel_, 1.2f);
  ros::param::param(robot_name_ + model_name_ + "/foresight_steps",
                    foresight_steps_, 10);
  ros::param::param(robot_name_ + model_name_ + "/foresight_time_step",
                    foresight_time_step_, 0.1f);
}

void ModelWrapper::init() {
  got_env_data_ = false;
  got_hypotheses_ = false;
  use_rvo_lib_ = true;
  interactive_costmap_ = false;
  debug_ = false;
  initialised_ = false;
  n_sampling_goals = 0;
  n_sequence_goals = 0;
  // inferred_goals_history_.resize(3);
  // init_liks_.resize(3, false);
  // prev_prior_.resize(3);
}

void ModelWrapper::rosSetup() {
  robot_pose_sub_ = nh_->subscribe("/environment/curr_pose", 1000,
                                   &ModelWrapper::robotPoseCB, this);
  robot_goal_sub_ = nh_->subscribe("/environment/target_goal",
                                   1000, &ModelWrapper::robotGoalCB, this);
  robot_vel_sub_ = nh_->subscribe("/cmd_vel", 1000,
                                  &ModelWrapper::robotVelCB, this);
  env_data_sub_ = nh_->subscribe("/driver_env/data", 1000,
                                 &ModelWrapper::envDataCB, this);
  model_hyp_sub_ = nh_->subscribe("/model/hypotheses", 1000,
                                  &ModelWrapper::modelCB, this);
  inter_pred_pub_ = nh_->advertise<model_msgs::GoalInference>
                    ("/model/inference", 1);
  model_inference_pub_ = nh_->advertise<model_msgs::InteractivePrediction>
                         ("/model/interactive_prediction", 1);
}

void ModelWrapper::robotPoseCB(const geometry_msgs::Pose2D::ConstPtr& msg) {
  robot_pose_ = *msg;
}

void ModelWrapper::robotGoalCB(const geometry_msgs::Pose2D::ConstPtr& msg) {
  robot_goal_ = *msg;
}

void ModelWrapper::robotVelCB(const geometry_msgs::Twist::ConstPtr& msg) {
  robot_vel_ = *msg;
}

void ModelWrapper::envDataCB(const environment_msgs::EnvironmentData::ConstPtr&
                             msg) {
  got_env_data_ = true;
  env_data_ = *msg;
}

void ModelWrapper::modelCB(const model_msgs::ModelHypotheses::ConstPtr&
                           msg) {
  got_hypotheses_ = true;
  hypotheses_ = *msg;
}

void ModelWrapper::runModel() {
  if (!got_env_data_ || !got_hypotheses_) {
    ROS_WARN("Model has not received all data yet!");
  } else {
    if (debug_) {ROS_INFO("BeginModel");}
    this->setupModel();
    if (hypotheses_.agents.size() > 0) {
      this->runSims();
      this->inferGoals();
    }
    if (interactive_costmap_) {this->interactivePrediction();}
    if (debug_) {ROS_INFO_STREAM("EndModel" << std::endl);}
  }
}

void ModelWrapper::inferGoals() {
  if (debug_) {ROS_INFO("Infer");}
  model_msgs::GoalInference goal_msg;
  float ros_freq = 0.1f;
  float PI = 3.14159265358979323846f;
  bool reset_priors = false;
  // size_t inf_hist = 10;
  common_msgs::Vector2 curr_vel;
  size_t n_goals;
  if (hypotheses_.goal_hypothesis.sampling) {
    n_goals = n_sampling_goals;
  } else {
    n_goals = n_sequence_goals;
  }
  size_t n_agents = hypotheses_.agents.size();
  std::vector<float> goals;
  goals.resize(n_goals);
  agent_goal_inference_.assign(n_agents, goals);
  for (size_t agent = 0; agent < n_agents; ++agent) {
    curr_vel.x = env_data_.agent_vels[agent].linear.x;
    curr_vel.y = env_data_.agent_vels[agent].linear.y;
    std::vector<float> g_likelihoods;
    g_likelihoods.resize(n_goals);
    std::vector<common_msgs::Vector2> agent_sim_vels;
    size_t begin = (agent * n_goals);
    size_t end = begin + (n_goals);
    if (hypotheses_.goal_hypothesis.sampling) {
      agent_sim_vels.insert(agent_sim_vels.begin(),
                            sampling_sim_vels.begin() + begin,
                            sampling_sim_vels.begin() + end);
    } else {
      agent_sim_vels.insert(agent_sim_vels.begin(),
                            sequence_sim_vels.begin() + begin,
                            sequence_sim_vels.begin() + end);
    }
    if (debug_) {ROS_WARN_STREAM("SimVelSize: " << agent_sim_vels.size());}
    for (size_t goal = 0; goal < n_goals; ++goal) {
      // if (DISPLAY_INFERENCE_VALUES) {
      if (debug_) {
        ROS_INFO_STREAM("curr_vel=[" << curr_vel.x <<
                        ", " << curr_vel.y << "] " <<
                        "simVel=[" << agent_sim_vels[goal].x <<
                        ", " << agent_sim_vels[goal].y << "]" << std::endl);
      }
      // }
      // Bivariate Gaussian
      float x = agent_sim_vels[goal].x;
      float y = agent_sim_vels[goal].y;
      float ux = curr_vel.x;
      float uy = curr_vel.y;
      float ox = (max_accel_ / 2) * ros_freq;  // 2 std.dev = max vel change
      float oy = (max_accel_ / 2) * ros_freq;
      float o2x = pow(ox, 2);
      float o2y = pow(oy, 2);
      float corr = 0.0f;
      float corr2 = pow(corr, 2.0f);
      float t1 = pow(x - ux, 2.0f) / o2x;
      float t2 = pow(y - uy, 2.0f) / o2y;
      float t3 = (2 * corr * (x - ux) * (y - uy)) / (ox * oy);
      float p = (1 / (2 * PI * ox * oy * sqrtf(1 - corr2)));
      float e = exp(-(1 / (2 * (1 - corr2))) * (t1 + t2 - t3));
      g_likelihoods[goal] = p * e;
      // if (debug_) {ROS_INFO_STREAM("Goal: " << goal <<
      // " Lik: " << g_likelihoods[goal] <<
      //                 " t1: " << t1 << " t2: " << t2 << " t3: " << t3);}
    }

    float posterior_norm = 0.0f;
    std::vector<float> g_posteriors;
    g_posteriors.resize(n_goals);
    float uniform_prior = 1.0f / n_goals;
    for (std::size_t goal = 0; goal < n_goals; ++goal) {
      if (reset_priors || !init_liks_[agent][goal]) {
        g_posteriors[goal] = uniform_prior;
        init_liks_[agent][goal] = true;
      } else {
        g_posteriors[goal] = g_likelihoods[goal] * prev_prior_[agent][goal];
      }

      posterior_norm += g_posteriors[goal];
      if (debug_) {
        // ROS_INFO_STREAM("GoalPosteriors" << goal << "=" <<
        //                 g_posteriors[goal] << " " << std::endl);
      }
    }

    float norm_posterior = 0.0f;
    std::vector<float> norm_posteriors;
    norm_posteriors.resize(n_goals);
    for (std::size_t goal = 0; goal < n_goals; ++goal) {
      if (posterior_norm == 0) {
        norm_posterior = uniform_prior;
        // if (debug_) {ROS_INFO_STREAM("Uni: " << norm_posterior);
        // Avoiding NaNs when likelihoods are all 0
      } else {
        norm_posterior = g_posteriors[goal] / posterior_norm;
        if (norm_posterior > 0.01f) {
          prev_prior_[agent][goal] = norm_posterior;
        } else {
          prev_prior_[agent][goal] = 0.005f;
        }
      }
      norm_posteriors[goal] = prev_prior_[agent][goal];
      agent_goal_inference_[agent][goal] = prev_prior_[agent][goal];
    }
    // if (debug_) {
    model_msgs::GoalEstimate msg;
    msg.agent_id = hypotheses_.agents[agent];
    // ROS_INFO_STREAM("InferAgent: " << (int)hypotheses_.agents[agent]);
    for (int i = 0; i < n_goals; ++i) {
      msg.goal_id.push_back(i);
      msg.goal_posterior.push_back(norm_posteriors[i]);
      // ROS_INFO_STREAM("G" << i << ": " << norm_posteriors[i]);
    }
    goal_msg.goal_inference.push_back(msg);
  }
  model_inference_pub_.publish(goal_msg);
}

void ModelWrapper::setupModel() {
  // if (!initialised_) {
  //   prev_prior_.resize(3);
  //   initialised_ = true;
  // }
  size_t n_goals;
  if (hypotheses_.goal_hypothesis.sampling) {
    float min_x = hypotheses_.goal_hypothesis.sample_space[0].x;
    float min_y = hypotheses_.goal_hypothesis.sample_space[0].y;
    float max_x = hypotheses_.goal_hypothesis.sample_space[1].x;
    float max_y = hypotheses_.goal_hypothesis.sample_space[1].y;
    float sample_res = hypotheses_.goal_hypothesis.sample_resolution;
    size_t size_x = max_x - min_x;
    size_t size_y = max_y - min_y;
    n_sampling_goals = ((size_x / sample_res) + 1) * ((size_y / sample_res) + 1);
    n_goals = n_sampling_goals;
  } else {
    n_sequence_goals = hypotheses_.goal_hypothesis.goal_sequence.size();
    n_goals = n_sequence_goals;
  }
  init_liks_.resize(hypotheses_.agents.size());
  for (size_t i = 0; i < hypotheses_.agents.size(); ++i) {
    init_liks_[i].resize(n_goals, false);
  }
  prev_prior_.resize(hypotheses_.agents.size());
  for (size_t i = 0; i < hypotheses_.agents.size(); ++i) {
    prev_prior_[i].resize(n_goals, 1.0f / n_goals);
  }

  std::vector<geometry_msgs::Pose2D> agent_poses_;
  std::vector<geometry_msgs::Twist> agent_vels_;
  sim_wrapper_->setRobotModel(robot_model_);
  if (robot_model_) {
    agent_poses_.push_back(robot_pose_);
    agent_vels_.push_back(robot_vel_);
    sim_wrapper_->setRobotGoal(robot_goal_);
  }
  agent_poses_.insert(agent_poses_.end(), env_data_.agent_poses.begin(),
                      env_data_.agent_poses.end());
  agent_vels_.insert(agent_vels_.end(), env_data_.agent_vels.begin(),
                     env_data_.agent_vels.end());
  sim_wrapper_->setEnvironment(agent_poses_, agent_vels_);

  sim_wrapper_->setModelAgents(hypotheses_.agents);
  if (hypotheses_.agents.size() > 0) {
    if (hypotheses_.goals) {
      if (hypotheses_.goal_hypothesis.sampling) {
        sampling_sims_ = sim_wrapper_->goalSampling(
                           hypotheses_.goal_hypothesis.sample_space,
                           hypotheses_.goal_hypothesis.sample_resolution);
      } else {
        sequence_sims_ = sim_wrapper_->goalSequence(
                           hypotheses_.goal_hypothesis.goal_sequence);
      }
    }
    if (hypotheses_.awareness) {
      ROS_WARN("ModelW- Awareness modelling not implemented yet!");
    }
  }
}

void ModelWrapper::runSims() {
  sampling_sim_vels.clear();
  sequence_sim_vels.clear();
  if (hypotheses_.goals) {
    if (hypotheses_.goal_hypothesis.sampling) {
      if (debug_) {ROS_WARN("ModelW- Run Goal Sampling Sims!");}
      sampling_sim_vels = sim_wrapper_->calcSimVels(sampling_sims_, n_sampling_goals);
    } else {
      if (debug_) {ROS_WARN("ModelW- Run Goal Sequence Sims!");}
      sequence_sim_vels = sim_wrapper_->calcSimVels(sequence_sims_, n_sequence_goals);
    }
  }
  if (hypotheses_.awareness) {
    ROS_WARN("ModelW- Awareness modelling not implemented yet!");
  }
  // for (size_t i = 0; i < sequence_sim_vels.size(); ++i) {
  // ROS_INFO_STREAM("SimVel" << i << ": " << sequence_sim_vels[i].x <<
  //                 ", " << sequence_sim_vels[i].y);
  // }
}

void ModelWrapper::interactivePrediction() {
  // Find Maximum Likelihood Goals for each agent given predictions
  std::vector<common_msgs::Vector2> a_goals;
  common_msgs::Vector2 goal;
  size_t n_agents = hypotheses_.agents.size();
  size_t n_goals;
  model_msgs::InteractivePrediction inter_pred_msg;
  if (hypotheses_.goal_hypothesis.sampling) {
    n_goals = n_sampling_goals;
  } else {n_goals = n_sequence_goals;}
  if (robot_model_) {  // TODO(Alex): Add proper check for non-modelled agents
    goal.x = robot_goal_.x;
    goal.y = robot_goal_.y;
    a_goals.push_back(goal);
    // ROS_INFO_STREAM("R_Goal: " << a_goals.back().x <<
    // ", " << a_goals.back().y);
  }
  for (size_t a = 0; a < n_agents; ++a) {
    size_t max_goal = 0;
    for (size_t g = 1; g < n_goals; ++g) {
      if (agent_goal_inference_[a][g] > agent_goal_inference_[a][max_goal]) {
        max_goal = g;
      }
    }
    if (hypotheses_.goal_hypothesis.sampling) {
      std::vector<geometry_msgs::Pose2D> sampling_goal_sequence_ =
        sim_wrapper_->getSamplingGoals();
      goal.x = sampling_goal_sequence_[max_goal].x;
      goal.y = sampling_goal_sequence_[max_goal].y;
    } else {
      goal.x = hypotheses_.goal_hypothesis.goal_sequence[max_goal].x;
      goal.y = hypotheses_.goal_hypothesis.goal_sequence[max_goal].y;
    }
    a_goals.push_back(goal);
    geometry_msgs::Pose2D goal_pose;
    goal_pose.x = goal.x;
    goal_pose.y = goal.y;
    // inter_pred_msg.agent_goal.push_back(goal_pose);
    if (debug_) {
      ROS_INFO_STREAM("A_Goal: " << " x: " << a_goals.back().x
                      << " y: " << a_goals.back().y);
    }
  }

// Run simulation for all agents given goals and foresight
  // if (n_agents > 0) {
  inter_pred_msg = sim_wrapper_->interactiveSim
                   (a_goals, foresight_steps_, foresight_time_step_
                    // , hypotheses_.goal_hypothesis.goal_sequence
                   );
  inter_pred_pub_.publish(inter_pred_msg);
  // }
}
