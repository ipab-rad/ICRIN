/**
 * @file      ros_navigation.hpp
 * @brief     ROS Navigation wrapper
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-09-02
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#ifndef ROS_NAVIGATION_HPP
#define ROS_NAVIGATION_HPP

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Twist.h>

class ROSNavigation {
 public:
  ROSNavigation(ros::NodeHandle* nh);
  ~ROSNavigation();

  void loadParams();
  void init();
  void rosSetup();

  void stopRobot();
  void planStep();
  void sendGoal();
  void monitorNavigation();

  void setPlannerGoal(geometry_msgs::PoseStamped target_pose);
  bool getArrived() {return arrived_;}
  bool getAborted() {return aborted_;}

 private:
  // Flags
  bool aborted_;
  bool arrived_;
  bool moving_;

  // Constants
  uint stop_msgs_;
  geometry_msgs::Twist stop_msg_;

  // Variables
  std::string robot_name_;
  geometry_msgs::PoseStamped target_pose_;
  move_base_msgs::MoveBaseGoal move_goal_;

  // ROS
  ros::NodeHandle* nh_;
  ros::Publisher cmd_vel_pub_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_ac;

};

#endif /* ROS_NAVIGATION_HPP */
