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
  robot_name_.erase (0, 1); // Remove 1 forward slash from robot_name
  model_name_ = ros::this_node::getName();
  model_name_.erase (0, robot_name_.length()); // Remove robot name
  this->loadParams();
  this->init();
  this->rosSetup();
  sim_wrapper_ = new SimWrapper(nh_);
}

ModelWrapper::~ModelWrapper() {
  ros::param::del(model_name_);
  if (use_rvo_lib_) {
    delete sim_wrapper_;
    sim_wrapper_ = NULL;
  }
}

void ModelWrapper::loadParams() {
  // Model Params
  if (!ros::param::has("/model/robot_model"))
  {ROS_WARN("Model- Robot model by default");}
  ros::param::param(robot_name_ + model_name_ + "/robot_model",
                    robot_model_, true);
  if (!ros::param::has("/model/goal_sum_prior"))
  {ROS_WARN("Model- Using default Model params");}
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
}

void ModelWrapper::init() {
  use_rvo_lib_ = true;
}

void ModelWrapper::rosSetup() {
  robot_pose_sub_ = nh_->subscribe(robot_name_ + "/environment/curr_pose", 1000,
                                   &ModelWrapper::robotPoseCB, this);
  robot_goal_sub_ = nh_->subscribe(robot_name_ + "/environment/target_goal",
                                   1000, &ModelWrapper::robotGoalCB, this);
  robot_vel_sub_ = nh_->subscribe(robot_name_ + "/cmd_vel", 1000,
                                  &ModelWrapper::robotVelCB, this);
  env_data_sub_ = nh_->subscribe(robot_name_ + "/environment/data", 1000,
                                 &ModelWrapper::envDataCB, this);
  model_hyp_sub_ = nh_->subscribe(robot_name_ + "/model/hypotheses", 1000,
                                  &ModelWrapper::modelCB, this);
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
  env_data_ = *msg;
}

void ModelWrapper::modelCB(const model_msgs::ModelHypotheses::ConstPtr&
                           msg) {
  hypotheses_ = *msg;
}

void ModelWrapper::runModel() {
  this->inferGoals();
  this->setupModel();
  this->runSims();
}

void ModelWrapper::inferGoals() {

}

void ModelWrapper::setupModel() {
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
    ROS_WARN("Model- Awareness modelling not implemented yet!");
  }
}

void ModelWrapper::runSims() {
  if (hypotheses_.goals) {
    if (hypotheses_.goal_hypothesis.sampling) {
      ROS_WARN("Model- Run Goal Sampling Sims!");
    } else {
      ROS_WARN("Model- Run Goal Sequence Sims!");
    }
  }
  if (hypotheses_.awareness) {
    ROS_WARN("Model- Awareness modelling not implemented yet!");
  }
}
