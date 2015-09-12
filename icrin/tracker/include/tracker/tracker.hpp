/**
 * @file      tracker.hpp
 * @brief     Tracker wrapper, subs to PTrackingBridge and pubs to Environment
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-08-04
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#ifndef TRACKER_HPP
#define TRACKER_HPP

#include <ros/ros.h>
#include <string>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>

#include <PTrackingBridge/TargetEstimations.h>
#include <tracker_msgs/TrackerData.h>
#include <people_msgs/People.h>
#include <people_msgs/Person.h>

class Tracker {
 public:
  Tracker(ros::NodeHandle* nh);
  ~Tracker();

  void init();
  void rosSetup();
  void loadParams();

  void receivePTrackerData(const PTrackingBridge::
                           TargetEstimations::ConstPtr& msg);
  void pubTrackerData();

 private:
  // Flags
  bool ptracker_rec_;
  bool ptracker_sent_;
  int8_t invert_x_;
  float vel_reduct_fact_;

  // Variables
  PTrackingBridge::TargetEstimations ptracker_msg_;

  // ROS
  ros::NodeHandle* nh_;
  ros::Publisher tracker_pub_;
  ros::Publisher people_pub_;
  ros::Subscriber ptracker_sub_;

};

#endif /* TRACKER_HPP */
