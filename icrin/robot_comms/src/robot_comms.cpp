/**
 * @file      robot_comms.cpp
 * @brief     Subscriber/Publisher class to manage inter-robot comms
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-08-03
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#include "robot_comms/robot_comms.hpp"

RobotComms::RobotComms(ros::NodeHandle* nh) {
  nh_ = nh;
  robot_name_ = ros::this_node::getNamespace();
  robot_name_.erase (0, 1); // Remove 1 forward slash from robot_name
  this->init();
  this->loadParams();
  this->rosSetup();
}

RobotComms::~RobotComms() {
  ;
}

void RobotComms::init() {
  robot_names_.push_back("/megatron");
  robot_names_.push_back("/soundwave");
  robot_names_.push_back("/starscream");
  robot_names_.push_back("/blackout");
  robot_names_.push_back("/thundercracker");
}

void RobotComms::rosSetup() {
  // After checking which robots are active, sub to their pose and cmd_vel
  for (uint8_t i = 0; i < active_robots_.size(); ++i) {
    robot_pose_sub_.push_back(nh_->subscribe(active_robots_[i] +
                                             "/environment/curr_pose",
                                             1, &RobotComms::RobotPoseCB, this));
    robot_vel_sub_.push_back(nh_->subscribe(active_robots_[i] +
                                            "/cmd_vel",
                                            1, &RobotComms::RobotVelCB, this));
  }
}

void RobotComms::loadParams() {
  bool robot_active;
  ros::param::param(robot_name_ + "/environment/active", robot_active, false);
  if (!robot_active)
  {ROS_WARN("WARNING: Robot %s not active but robot_comms created!", robot_name_.c_str());}
  for (uint8_t i = 0; i < robot_names_.size(); ++i) {
    bool active;
    ROS_INFO("Checking if %s is active", robot_names_[i].c_str());
    ros::param::param(robot_names_[i] + "/environment/active", active, false);
    if (active && (robot_names_[i].compare(robot_name_) != 0)) {
      active_robots_.push_back(robot_names_[i]);
    } else if (!active) {
      ROS_INFO("%s not active", robot_names_[i].c_str());
    } else {
      ROS_INFO("%s is me!", robot_names_[i].c_str());
    }
  }
  ROS_INFO("%s is listening to %lu other robots",
           robot_name_.c_str(), active_robots_.size());
}

void RobotComms::RobotPoseCB(const geometry_msgs::Pose2D::ConstPtr& msg) {
  ROS_INFO("RobotPoseCB");
}

void RobotComms::RobotVelCB(const geometry_msgs::Twist::ConstPtr& msg) {
  ROS_INFO("RobotVelCB");
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_comms");
  ros::NodeHandle nh("robot_comms");
  RobotComms robot_comms(&nh);

  ros::Rate r(10);

  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }

  ros::shutdown();

  return 0;
}
