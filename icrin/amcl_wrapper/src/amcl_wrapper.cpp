/**
 * @file      amcl_wrapper.cpp
 * @brief     AMCL wrapper, provides up-to-date robot pose for Environment
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-08-06
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */
#include "amcl_wrapper/amcl_wrapper.hpp"

AMCLWrapper::AMCLWrapper(ros::NodeHandle* nh) {
  nh_ = nh;
  robot_name_ = ros::this_node::getNamespace();
  robot_name_.erase (0, 1); // Remove 1 forward slash from robot_name
  this->init();
  this->loadParams();
  this->rosSetup();
}

AMCLWrapper::~AMCLWrapper() {
  ;
}

void AMCLWrapper::init() {
  amcl_received_ = false;
  odom_received_ = false;
  odom_init_ = false;
  use_amcl_ = true;
}

void AMCLWrapper::rosSetup() {
  robot_pose_pub_ = nh_->advertise<geometry_msgs::Pose2D>("curr_pose", 1, true);
  robot_vel_pub_ = nh_->advertise<geometry_msgs::Twist>("curr_vel", 1, true);
  initial_pose_pub_ = nh_->advertise<geometry_msgs::PoseWithCovarianceStamped>
                      (robot_name_ + "/initialpose", 1, true);
  amcl_pose_sub_ = nh_->subscribe(robot_name_ + "/amcl_pose",
                                  1, &AMCLWrapper::AMCLPoseCB, this);
  odom_sub_ = nh_->subscribe(robot_name_ + "/odom",
                             1, &AMCLWrapper::OdomCB, this);
  set_pose_ = nh_->subscribe(robot_name_ + "/amcl_wrapper/set_pose",
                             1, &AMCLWrapper::SetPoseCB, this);
}

void AMCLWrapper::loadParams() {
  bool robot_active;
  ros::param::param(robot_name_ + "/environment/active", robot_active, false);
  if (!robot_active)
  {ROS_WARN("WARNING: Robot %s not active but AMCLWrapper created!", robot_name_.c_str());}
  bool use_amcl_;
  ros::param::param(robot_name_ + "/environment/amcl", use_amcl_, false);
  if (!use_amcl_)
  {ROS_WARN("WARNING: Robot %s not using AMCL!", robot_name_.c_str());}
}

void AMCLWrapper::AMCLPoseCB(const geometry_msgs::
                             PoseWithCovarianceStamped::ConstPtr& msg) {
  amcl_pose_ = *msg;
  amcl_received_ = true;
  prev_odom_ = curr_odom_;
}

void AMCLWrapper::OdomCB(const nav_msgs::Odometry::ConstPtr& msg) {
  if (!odom_init_) {
    prev_odom_ = *msg;
    curr_odom_ = prev_odom_;
    odom_init_ = true;
  } else {
    curr_odom_ = *msg;
    robot_vel_ = msg->twist.twist;
    odom_received_ = true;
  }
}

void AMCLWrapper::SetPoseCB(const geometry_msgs::Pose2D::ConstPtr& msg) {
  robot_pose_ = *msg;
  geometry_msgs::PoseWithCovarianceStamped init_pose;
  init_pose.pose.pose.position.x = robot_pose_.x;
  init_pose.pose.pose.position.y = robot_pose_.y;
  tf::Quaternion q = tf::createQuaternionFromRPY(0.0, 0.0, robot_pose_.theta);
  tf::quaternionTFToMsg(q, init_pose.pose.pose.orientation);
  initial_pose_pub_.publish(init_pose);
}

void AMCLWrapper::pubRobotPose() {
  if (!odom_received_) {
    ROS_WARN("WARNING: No odometry received from robot");
  } else {
    if (use_amcl_) {this->calcAMCL();}
    this->calcOdomDiff(); // Since AMCL msg may be sporadic at best
    robot_pose_pub_.publish(robot_pose_);
  }
}

void AMCLWrapper::pubRobotVel() {
  if (!odom_received_) {
    ROS_WARN("WARNING: No odometry received from robot");
  } else {
    robot_vel_pub_.publish(robot_vel_);
  }
}

void AMCLWrapper::calcAMCL() {
  tf::Quaternion q;
  tf::quaternionMsgToTF(amcl_pose_.pose.pose.orientation, q);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  robot_pose_.x = amcl_pose_.pose.pose.position.x;
  robot_pose_.y = amcl_pose_.pose.pose.position.y;
  robot_pose_.theta = yaw;
}

void AMCLWrapper::calcOdomDiff() {
  odom_diff_.x = curr_odom_.pose.pose.position.x -
                 prev_odom_.pose.pose.position.x;
  odom_diff_.y = curr_odom_.pose.pose.position.y -
                 prev_odom_.pose.pose.position.y;
  odom_diff_.theta = calcYaw(curr_odom_) - calcYaw(prev_odom_);

  robot_pose_.x = robot_pose_.x - odom_diff_.x;
  robot_pose_.y =  robot_pose_.y - odom_diff_.y;
  robot_pose_.theta =  robot_pose_.theta - odom_diff_.theta;
  odom_received_ = false;
}

double AMCLWrapper::calcYaw(nav_msgs::Odometry odom) {
  tf::Quaternion q;
  tf::quaternionMsgToTF(odom.pose.pose.orientation, q);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "amcl_wrapper");
  ros::NodeHandle nh("amcl_wrapper");
  AMCLWrapper amcl_wrapper(&nh);

  ros::Rate r(10);

  while (ros::ok()) {
    ros::spinOnce();
    amcl_wrapper.pubRobotVel();
    amcl_wrapper.pubRobotPose();
    r.sleep();
  }

  ros::shutdown();

  return 0;
}
