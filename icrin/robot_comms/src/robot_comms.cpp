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
  comms_data_pub_ = nh_->advertise<robot_comms_msgs::CommsData>("data", 1,
                                                                true);
  for (uint8_t i = 0; i < active_robots_.size(); ++i) {
    robot_pose_sub_.push_back(nh_->subscribe<geometry_msgs::Pose2D>
                              (active_robots_[i] + "/environment/curr_pose", 1,
                               boost::bind(&RobotComms::robotPoseCB,
                                           this, _1, active_robots_[i])));
    robot_vel_sub_.push_back(nh_->subscribe<geometry_msgs::Twist>
                             (active_robots_[i] + "/cmd_vel", 1,
                              boost::bind(&RobotComms::robotVelCB,
                                          this, _1, active_robots_[i])));
  }
}

void RobotComms::loadParams() {
  bool robot_active;
  ros::param::param(robot_name_ + "/environment/active", robot_active, false);
  if (!robot_active)
  {ROS_WARN("WARNING: Robot %s not active but robot_comms created!", robot_name_.c_str());}
  for (uint8_t i = 0; i < robot_names_.size(); ++i) {
    bool active;
    ros::param::param(robot_names_[i] + "/environment/active", active, false);
    if (active
        && (robot_names_[i].compare(robot_name_) != 0)
       ) {
      ROS_INFO("%s active!", robot_names_[i].c_str());
      active_robots_.push_back(robot_names_[i]);
    }
  }
  ROS_INFO("%s is listening to %lu other robots",
           robot_name_.c_str(), active_robots_.size());
  robot_poses_.resize(active_robots_.size());
  robot_vels_.resize(active_robots_.size());
}

void RobotComms::pubCommsData() {
  comms_data_.robot_poses = robot_poses_;
  comms_data_.robot_vels = robot_vels_;
  comms_data_pub_.publish(comms_data_);
}

void RobotComms::robotPoseCB(const geometry_msgs::Pose2D::ConstPtr& msg,
                             const std::string& robot) {
  for (uint32_t i = 0; i < active_robots_.size(); ++i) {
    if (robot.compare(active_robots_[i]) == 0) {
      robot_poses_[i] = *msg;
      break;
    }
  }
}

void RobotComms::robotVelCB(const geometry_msgs::Twist::ConstPtr& msg,
                            const std::string& robot) {
  for (uint32_t i = 0; i < active_robots_.size(); ++i) {
    if (robot.compare(active_robots_[i]) == 0) {
      robot_vels_[i] = *msg;
      break;
    }
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_comms");
  ros::NodeHandle nh("robot_comms");
  RobotComms robot_comms(&nh);

  ros::Rate r(10);

  while (ros::ok()) {
    ros::spinOnce();
    robot_comms.pubCommsData();
    r.sleep();
  }

  ros::shutdown();

  return 0;
}
