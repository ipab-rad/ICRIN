/**
 * @file      robot_comms.hpp
 * @brief     Subscriber/Publisher class to manage inter-robot comms
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-08-03
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#ifndef ROBOT_COMMS_HPP
#define ROBOT_COMMS_HPP

#include <ros/ros.h>
#include <ros/message_event.h>
#include <sstream>
// #include <boost/bind.hpp>

#include <std_msgs/String.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>

#include <robot_comms_msgs/CommsData.h>

class RobotComms {
 public:
  RobotComms(ros::NodeHandle* nh);
  ~RobotComms();

  void init();
  void rosSetup();
  void loadParams();

  void pubCommsData();
  void robotPoseCB(const geometry_msgs::Pose2D::ConstPtr& msg,
                   const std::string& robot);
  void robotVelCB(const geometry_msgs::Twist::ConstPtr& msg,
                  const std::string& robot);

 private:
  // Variables
  std::string robot_name_;
  std::vector<std::string> active_robots_;

  // ROS
  ros::NodeHandle* nh_;
  ros::Publisher comms_data_pub_;
  std::vector<ros::Subscriber> robot_pose_sub_;
  std::vector<ros::Subscriber> robot_vel_sub_;
  std::vector<geometry_msgs::Pose2D> robot_poses_;
  std::vector<geometry_msgs::Twist> robot_vels_;

  robot_comms_msgs::CommsData comms_data_;
};

#endif /* ROBOT_COMMS_HPP */
