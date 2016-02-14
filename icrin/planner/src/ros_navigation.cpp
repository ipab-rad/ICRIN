/**
 * @file      ros_navigation.cpp
 * @brief     ROS Navigation wrapper
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-09-02
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#include <planner/ros_navigation.hpp>

ROSNavigation::ROSNavigation(ros::NodeHandle* nh) : nh_(nh),
  move_ac("move_base", true) {
  robot_name_ = ros::this_node::getNamespace();
  robot_name_.erase(0, 1);  // Remove 1 forward slash from robot_name
  this->loadParams();
  this->init();
  this->rosSetup();
}

ROSNavigation::~ROSNavigation() {;}

void ROSNavigation::loadParams() {
}

void ROSNavigation::init() {
  aborted_ = false;
  arrived_ = false;
  moving_ = false;
  stop_msgs_ = 4;
  stop_msg_.linear.x = 0.0;
  stop_msg_.linear.y = 0.0;
  stop_msg_.linear.z = 0.0;
  stop_msg_.angular.x = 0.0;
  stop_msg_.angular.y = 0.0;
  stop_msg_.angular.z = 0.0;
  ROS_INFO_STREAM("Robot name: " << robot_name_);
}

void ROSNavigation::rosSetup() {
  // cmd_vel_pub_ = nh_->advertise<geometry_msgs::Twist>
  // (robot_name_ + "/cmd_vel", 1000);
  ROS_INFO("Waiting for Move Base Action Server...");
  move_ac.waitForServer();
  ROS_INFO("Move Base Action Server ready!");
  ROS_INFO("Waiting for Clear Costmaps service");
  ros::service::waitForService(robot_name_ + "/move_base_node/clear_costmaps");
  clear_costmaps_client_ = nh_->serviceClient<std_srvs::Empty>
                           (robot_name_ + "/move_base_node/clear_costmaps");
  ROS_INFO("Clear Costmaps service ready!");
}

void ROSNavigation::stopRobot() {
  ROS_INFO("ROS_NAV: STOP!");
  // ros::Rate r(10);
  // for (size_t i = 0; i < stop_msgs_; i++) {
  //   cmd_vel_pub_.publish(stop_msg_);
  //   ros::spinOnce();
  //   r.sleep();
  // }
}

void ROSNavigation::planStep() {
  this->sendGoal();
  this->monitorNavigation();
}

void ROSNavigation::sendGoal() {
  // Check if goal is different
  if ((target_pose_.pose.position.x !=
       move_goal_.target_pose.pose.position.x) ||
      (target_pose_.pose.position.y !=
       move_goal_.target_pose.pose.position.y)) {
    target_pose_.header.frame_id = robot_name_ + "/map";
    // move_goal_.target_pose.header.frame_id = robot_name_ + "/map";
    ROS_INFO_STREAM("ROS_NAV: Prev Move Goal" <<
                    move_goal_.target_pose.pose.position.x
                    << ", " << move_goal_.target_pose.pose.position.y);
    ROS_INFO("ROS_NAV:New Target Pose!");
    ROS_INFO_STREAM("ROS_NAV: Targ Pose" << target_pose_.pose.position.x << ", "
                    << target_pose_.pose.position.y << ", " <<
                    target_pose_.header.frame_id);
    move_goal_.target_pose = target_pose_;
    move_ac.sendGoal(move_goal_);
    arrived_ = false;
    moving_ = true;
  }
}

void ROSNavigation::monitorNavigation() {
  if (move_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    this->stopRobot();
    arrived_ = true;
    moving_ = false;
    ROS_INFO("ROS_NAV:Stopped!");
  } else if (move_ac.getState() == actionlib::SimpleClientGoalState::ABORTED) {
    std_srvs::Empty empty;
    clear_costmaps_client_.call(empty);
    this->stopRobot();
    arrived_ = false;
    moving_ = false;
    // aborted_ = true;
    ROS_WARN("ROS_NAV:Aborted!");
    this->sendGoal();
  }
}

void ROSNavigation::setPlannerGoal(geometry_msgs::PoseStamped target_pose) {
  target_pose_ = target_pose;
}
