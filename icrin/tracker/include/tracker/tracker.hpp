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

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <common_msgs/Enable.h>

#include <PTrackingBridge/TargetEstimations.h>
#include <tracker_msgs/TrackerData.h>

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

  // Variables
  PTrackingBridge::TargetEstimations ptracker_msg_;
  // tracker_msgs::TrackerData tracker_data_;

  // ROS
  ros::NodeHandle* nh_;
  ros::Publisher tracker_pub_;
  ros::Subscriber ptracker_sub_;

};

#endif /* TRACKER_HPP */
