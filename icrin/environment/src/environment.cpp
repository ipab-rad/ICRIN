/**
 * @file      environment.cpp
 * @brief     Main environment for Youbot-ROS-icrin interface
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-08-01
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#include "environment/environment.hpp"

Environment::Environment(ros::NodeHandle* nh) {
  nh_ = nh;
  robot_name_ = ros::this_node::getNamespace();
  robot_name_.erase (0, 1); // Remove 1 forward slash from robot_name
  this->init();
  this->rosSetup();
  this->loadParams();
}

Environment::~Environment() {
  ;
}

void Environment::init() {
  planning_ = false;
  // ROS
  zero_vect_.x = 0.0f;
  zero_vect_.y = 0.0f;
  zero_vect_.z = 0.0f;
  robot_curr_pose_.x = 0.0f;
  robot_curr_pose_.y = 0.0f;
  robot_curr_pose_.theta = 0.0f;
  robot_target_goal_.x = 0.0f;
  robot_target_goal_.y = 0.0f;
  robot_target_goal_.theta = 0.0f;
  robot_cmd_velocity_.linear = zero_vect_;
  robot_cmd_velocity_.angular = zero_vect_;
}

void Environment::rosSetup() {
  curr_pose_pub_ = nh_->advertise<geometry_msgs::Pose2D>("curr_pose", 1, true);
  target_goal_pub_ = nh_->advertise<geometry_msgs::Pose2D>("target_goal", 1,
                                                           true);
  robot_cmd_velocity_pub_ = nh_->advertise<geometry_msgs::Twist>(
                              robot_name_ + "/cmd_vel", 1, true);
  environment_data_pub_ = nh_->advertise<environment_msgs::EnvironmentData>(
                            "data", 1, true);
  ros::service::waitForService(robot_name_ + "/planner/setup_rvo_planner");
  setup_rvo_planner_ = nh_->serviceClient<planner_msgs::SetupRVOPlanner>(
                         robot_name_ + "/planner/setup_rvo_planner", true);
  planning_sub_ = nh_->subscribe(robot_name_ + "/environment/planning", 1000,
                                 &Environment::planningCB, this);
  // Youbot
  bumper_kilt_sub_ = nh_->subscribe(robot_name_ + "/bumper_kilt", 1000,
                                    &Environment::bumperKiltCB, this);
  // Tracker
  tracker_data_sub_ = nh_->subscribe("/tracker/data", 1000,
                                     &Environment::trackerDataCB, this);
  // AMCL Wrapper
  amcl_pose_sub_ = nh_->subscribe(robot_name_ + "/amcl_wrapper/curr_pose",
                                  1000, &Environment::amclPoseCB, this);
  // Robot Comms
  comms_data_sub_ = nh_->subscribe(robot_name_ + "/robot_comms/data", 1000,
                                   &Environment::commsDataCB, this);
  // Planner
  planner_cmd_vel_sub_ = nh_->subscribe(robot_name_ + "/planner/cmd_vel", 1000,
                                        &Environment::plannerCmdVelCB, this);
}

void Environment::loadParams() {
  // Experiment
  ros::param::param(robot_name_ + "/environment/track_robots",
                    track_robots_, false);
  // Robot specific
  ros::param::param(robot_name_ + "/environment/active", active_, false);
  if (!active_) {
    ROS_WARN("WARNING: Robot %s not active but environment created!",
             robot_name_.c_str());
  }
  ros::param::param(robot_name_ + "/environment/amcl", amcl_, true);
  ros::param::param(robot_name_ + "/environment/bumper", bumper_, false);
  ros::param::param(robot_name_ + "/environment/rvo_planner", rvo_planner_, true);
}

void Environment::pubRobotPose() {
  robot_curr_pose_ = robot_amcl_pose_;
  curr_pose_pub_.publish(robot_curr_pose_);
}

void Environment::pubRobotGoal() {
  target_goal_pub_.publish(robot_target_goal_);
}

void Environment::pubRobotVelocity() {
  if (planning_ && !collision_) {
    robot_cmd_velocity_ = planner_cmd_velocity_;
  } else {
    robot_cmd_velocity_.linear = zero_vect_;
    robot_cmd_velocity_.angular = zero_vect_;
  }
  robot_cmd_velocity_pub_.publish(robot_cmd_velocity_);
}

void Environment::bumperKiltCB(const std_msgs::Int32MultiArray::ConstPtr& msg) {
  bumper_kilt_ = *msg;  // 8 Directions, clockwise starting at front
  collision_ = false;
  for (uint8_t i = 0; i < bumper_kilt_.data.size(); ++i) {
    if (bumper_kilt_.data[i] > 0) {collision_ = true;}
  }
}

void Environment::trackerDataCB(const tracker_msgs::TrackerData::ConstPtr&
                                msg) {
  tracker_data_ = *msg;
}

void Environment::amclPoseCB(const geometry_msgs::Pose2D::ConstPtr& msg) {
  robot_amcl_pose_ = *msg;
}

void Environment::commsDataCB(const robot_comms_msgs::CommsData::ConstPtr&
                              msg) {
  comms_data_ = *msg;
}

void Environment::plannerCmdVelCB(const geometry_msgs::Twist::ConstPtr& msg) {
  planner_cmd_velocity_ = *msg;
  this->pubRobotVelocity();
}

void Environment::planningCB(const std_msgs::Bool::ConstPtr& msg) {
  bool plan_now = msg->data;
  if (!planning_ && plan_now) {
    ROS_WARN("Robot %s now planning!", robot_name_.c_str());
  } else if (planning_ && !plan_now) {
    ROS_WARN("Robot %s stop planning!", robot_name_.c_str());
  }
  planning_ = plan_now;
}

void Environment::pubEnvironmentData() {
  environment_msgs::EnvironmentData env_data;
  uint64_t nrobots = comms_data_.robot_poses.size();
  uint64_t ntrackers = tracker_data_.identity.size();
  // Add other robots info
  if (!track_robots_) {
    env_data.tracker_ids.resize(nrobots, 0);  // If robots are not tracked
    env_data.agent_poses = comms_data_.robot_poses;
    env_data.agent_vels = comms_data_.robot_vels;
  }
  // Add people tracking info
  for (uint64_t i = 0; i < ntrackers; ++i) {
    env_data.tracker_ids.push_back(tracker_data_.identity[i]);
    env_data.agent_poses.push_back(tracker_data_.agent_position[i]);
    env_data.agent_vels.push_back(tracker_data_.agent_avg_velocity[i]);
  }
  environment_data_pub_.publish(env_data);
  this->pubRobotPose();
  this->pubRobotGoal();
  this->pubRobotVelocity();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "environment");
  ros::NodeHandle nh("environment");
  Environment environment(&nh);

  ros::Rate r(10);

  while (ros::ok()) {
    ros::spinOnce();
    environment.pubEnvironmentData();
    r.sleep();
  }

  ros::shutdown();

  return 0;
}
