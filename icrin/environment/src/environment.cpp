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
  this->init();
  this->rosSetup();
}

Environment::~Environment() {
  ;
}

void Environment::init() {
  planning_ = true;
  use_rvo_planner_ = true;
  use_odometry_ = true;
  // ROS
  robot_curr_pose_.x = 0.0f;
  robot_curr_pose_.y = 0.0f;
  robot_curr_pose_.theta = 0.0f;
  robot_target_goal_.x = 1.0f;
  robot_target_goal_.y = 0.0f;
  robot_target_goal_.theta = 0.0f;
}

void Environment::rosSetup() {
  curr_pose_pub_ = nh_->advertise<geometry_msgs::Pose2D>("curr_pose", 1, true);
  target_goal_pub_ = nh_->advertise<geometry_msgs::Pose2D>("target_goal", 1,
                                                           true);
  robot_cmd_velocity_pub_ = nh_->advertise<geometry_msgs::Twist>(
                              robot_name_ + "/cmd_vel", 1, true);
  ros::service::waitForService(robot_name_ + "/planner/setup_rvo_planner");
  setup_rvo_planner_ =
    nh_->serviceClient<planner_msgs::SetupRVOPlanner>(
      robot_name_ + "/planner/setup_rvo_planner", true);
  // Youbot
  odom_sub_ = nh_->subscribe(robot_name_ + "/odom", 1000,
                             &Environment::odomCB, this);
  // Tracker
  tracker_pose_sub_ = nh_->subscribe("/tracker/data", 1000,
                                     &Environment::trackerDataCB, this);
  // Planner
  planner_cmd_vel_sub_ = nh_->subscribe(robot_name_ + "/planner/cmd_vel", 1000,
                                        &Environment::plannerCmdVelCB, this);
}

void Environment::pubRobotPose() {
  curr_pose_pub_.publish(robot_curr_pose_);
}

void Environment::pubRobotGoal() {
  target_goal_pub_.publish(robot_target_goal_);
}

void Environment::pubRobotVelocity() {
  // TODO: Set vels to absolute zero if small enough
  robot_cmd_velocity_ = planner_cmd_velocity_;
  robot_cmd_velocity_pub_.publish(robot_cmd_velocity_);
}

void Environment::odomCB(const nav_msgs::Odometry::ConstPtr& msg) {
  robot_odom_ = *msg;
}

void Environment::trackerDataCB(const geometry_msgs::Pose2D::ConstPtr& msg) {
  ;
}

void Environment::plannerCmdVelCB(const geometry_msgs::Twist::ConstPtr& msg) {
  planner_cmd_velocity_ = *msg;
  this->pubRobotVelocity();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "environment");
  ros::NodeHandle nh("environment");
  Environment environment(&nh);

  ros::Rate r(10);

  while (ros::ok()) {
    if (environment.planning_) {
      environment.pubRobotPose();
      environment.pubRobotGoal();
      if (environment.use_rvo_planner_) {
        environment.pubRobotVelocity();
      }
    }
    ros::spinOnce();
    r.sleep();
  }

  ros::shutdown();

  return 0;
}
