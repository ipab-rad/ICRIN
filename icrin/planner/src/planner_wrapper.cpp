/**
 * @file      planner_wrapper.cpp
 * @brief     Planner wrapper, connecting the differing planners through ROS
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-06-29
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#include "planner/planner_wrapper.hpp"

PlannerWrapper::PlannerWrapper(ros::NodeHandle* nh) {
  nh_ = nh;
  robot_name_ = ros::this_node::getNamespace();
  robot_name_.erase (0, 1); // Remove 1 forward slash from robot_name
  this->init();
  this->rosSetup();
}

PlannerWrapper::~PlannerWrapper() {
  if (use_rvo_planner_) {
    delete rvo_planner_;
    rvo_planner_ = NULL;
  }
}

void PlannerWrapper::init() {
  use_rvo_planner_ = false;
  planning_ = false;
  arrived_ = false;
  planner_init = false;
  curr_pose_.x = 0.0f;
  curr_pose_.y = 0.0f;
  goal_pose_ = curr_pose_;
  cmd_vel_.linear.x = 0.0f;
  cmd_vel_.linear.y = 0.0f;
  cmd_vel_.linear.z = 0.0f;
  cmd_vel_.angular.x = 0.0f;
  cmd_vel_.angular.y = 0.0f;
  cmd_vel_.angular.z = 0.0f;
}

void PlannerWrapper::rosSetup() {
  cmd_vel_pub_ = nh_->advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
  planning_pub_ = nh_->advertise<std_msgs::Bool>(robot_name_ +
                                                 "/environment/planning",
                                                 1);
  arrived_pub_ = nh_->advertise<std_msgs::Bool>(robot_name_ +
                                                "/environment/arrived", 1);
  srv_setup_new_planner_ =
    nh_->advertiseService("setup_new_planner",
                          &PlannerWrapper::setupNewPlanner, this);
  srv_setup_rvo_planner_ =
    nh_->advertiseService("setup_rvo_planner",
                          &PlannerWrapper::setupRVOPlanner, this);
  // srv_add_planner_agents_ =
  //   nh_->advertiseService("add_planner_agents",
  //                         &PlannerWrapper::checkReachedGoal, this);
  curr_pose_sub_ = nh_->subscribe(robot_name_ + "/environment/curr_pose", 1000,
                                  &PlannerWrapper::currPoseCB, this);
  target_goal_sub_ = nh_->subscribe(robot_name_ + "/environment/target_goal",
                                    1000,
                                    &PlannerWrapper::targetGoalCB, this);
  planning_sub_ = nh_->subscribe(robot_name_ + "/environment/planning", 1000,
                                 &PlannerWrapper::planningCB, this);
}

void PlannerWrapper::pubPlanning(bool planning) {
  planning_ = planning;
  std_msgs::Bool msg;
  msg.data = planning;
  planning_pub_.publish(msg);
}

void PlannerWrapper::pubArrived(bool arrived) {
  arrived_ = arrived;
  std_msgs::Bool msg;
  msg.data = arrived;
  arrived_pub_.publish(msg);
}

bool PlannerWrapper::setupNewPlanner(
  planner_msgs::SetupNewPlanner::Request& req,
  planner_msgs::SetupNewPlanner::Response& res) {
  res.res = true;
  if (req.planner_type == req.RVO_PLANNER && !planner_init) {
    rvo_planner_ = new RVOPlanner(nh_);
    use_rvo_planner_ = true;
    planner_init = true;
    ROS_INFO("RVO Planner setup");
  } else if (req.planner_type == req.ROS_NAVIGATION && !planner_init) {
    ROS_ERROR("ROS_NAVIGATION not implemented yet, sorry!");
  } else {res.res = false;}
  return true;
}

bool PlannerWrapper::setupRVOPlanner(
  planner_msgs::SetupRVOPlanner::Request& req,
  planner_msgs::SetupRVOPlanner::Response& res) {
  rvo_planner_->setPlannerSettings(req.time_step, req.defaults);
  res.res = true;
  return true;
}

void PlannerWrapper::plannerStep() {
  if (planning_) {
    if (use_rvo_planner_) {
      ROS_INFO("Planner Wrapper- Planning!");
      rvo_planner_vel_ = rvo_planner_->planStep();
      cmd_vel_.linear.x = rvo_planner_vel_.x;
      cmd_vel_.linear.y = rvo_planner_vel_.y;
    }
    cmd_vel_pub_.publish(cmd_vel_);
    arrived_ = rvo_planner_->getArrived();
  }
  if (planning_ && arrived_) {
    this->pubArrived(true);
    this->pubPlanning(false);
    ROS_INFO("Planner Wrapper- Robot %s reached goal", robot_name_.c_str());
  }
}

void PlannerWrapper::currPoseCB(const geometry_msgs::Pose2D::ConstPtr& msg) {
  curr_pose_.x = msg->x;
  curr_pose_.y = msg->y;
  if (use_rvo_planner_) {
    rvo_planner_->setCurrPose(curr_pose_);
  }
}

void PlannerWrapper::targetGoalCB(const geometry_msgs::Pose2D::ConstPtr& msg) {
  goal_pose_.x = msg->x;
  goal_pose_.y = msg->y;
  if (use_rvo_planner_) {
    rvo_planner_->setPlannerGoal(goal_pose_);
  }
}

void PlannerWrapper::planningCB(const std_msgs::Bool::ConstPtr& msg) {
  planning_ = msg->data;
}

void PlannerWrapper::environmentDataCB(
  const environment_msgs::EnvironmentData::ConstPtr& msg) {
  environment_ = *msg;
  if (use_rvo_planner_) {
    uint64_t nAgents = environment_.agent_poses.size();
    std::vector<common_msgs::Vector2> agent_poses;
    std::vector<common_msgs::Vector2> agent_vels;
    agent_poses.resize(nAgents);
    agent_vels.resize(nAgents);
    for (uint64_t i = 0; i < nAgents; ++i) {
      agent_poses[i].x = environment_.agent_poses[i].x;
      agent_poses[i].y = environment_.agent_poses[i].y;
      agent_vels[i].x = environment_.agent_vels[i].linear.x;
      agent_vels[i].y = environment_.agent_vels[i].linear.y;
    }
    rvo_planner_->setupEnvironment(environment_.tracker_ids,
                                   agent_poses, agent_vels);
  }
}
