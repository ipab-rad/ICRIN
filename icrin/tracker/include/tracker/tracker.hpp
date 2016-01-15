/**
 * @file      tracker.hpp
 * @brief     Tracker wrapper, subs to PTracking and pubs to Environment
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-08-04
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#ifndef TRACKER_HPP
#define TRACKER_HPP

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>

#include <tracker_msgs/TrackerData.h>
#include <tracker_msgs/TargetEstimations.h>
#include <people_msgs/People.h>
#include <people_msgs/Person.h>

#include <string>

class Tracker {
 public:
  explicit Tracker(ros::NodeHandle* nh);
  ~Tracker();

  void init();
  void rosSetup();
  void loadParams();

  void receivePTrackerData(const tracker_msgs::
                           TargetEstimations::ConstPtr& msg);
  void pubTrackerData();

 private:
  // Flags
  bool ptracker_rec_;
  bool ptracker_sent_;

  // Constants
  int8_t invert_x_;
  float vel_reduct_fact_;
  std::string camera_agent_;

  // Variables
  tracker_msgs::TargetEstimations ptracker_msg_;

  // ROS
  ros::NodeHandle* nh_;
  ros::Publisher tracker_pub_;
  ros::Publisher people_pub_;
  ros::Subscriber ptracker_sub_;
};

#endif /* TRACKER_HPP */
