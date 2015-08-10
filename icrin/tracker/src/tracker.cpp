/**
 * @file      tracker.cpp
 * @brief     Tracker wrapper, subs to PTrackingBridge and pubs to Environment
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-08-04
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */


#include "tracker/tracker.hpp"

#ifndef PI
const float PI = 3.14159265358979323846f;
#endif

Tracker::Tracker(ros::NodeHandle* nh) {
  nh_ = nh;
  this->init();
  this->loadParams();
  this->rosSetup();
}

Tracker::~Tracker() {
  ;
}

void Tracker::init() {
  ptracker_rec_ = false;
  ptracker_sent_ = true;
  invert_x_ = -1; // Set to -1 to invert x axis relative to robot frame
}

void Tracker::rosSetup() {
  tracker_pub_ = nh_->advertise<tracker_msgs::TrackerData>("data", 1, true);
  ptracker_sub_ = nh_->subscribe("/agent_1/PTrackingBridge/targetEstimations",
                                 1, &Tracker::receivePTrackerData, this);
}

void Tracker::loadParams() {
}

void Tracker::receivePTrackerData(const PTrackingBridge::
                                  TargetEstimations::ConstPtr& msg) {
  ptracker_msg_ = *msg;
  ptracker_rec_ = true;
}

void Tracker::pubTrackerData() {
  if (ptracker_rec_) {
    // Unpack tracker data
    tracker_msgs::TrackerData tracker_data;
    uint32_t nagents = ptracker_msg_.identities.size();
    for (uint32_t i = 0; i < nagents; ++i) {
      geometry_msgs::Pose2D pos;
      pos.x = ptracker_msg_.positions[i].x * invert_x_;
      pos.y = ptracker_msg_.positions[i].y;
      pos.theta = atan2(ptracker_msg_.velocities[i].y,
                        ptracker_msg_.velocities[i].x * invert_x_) * (180 / PI);
      tracker_data.agent_position.push_back(pos);
      geometry_msgs::Pose2D pos_dev;
      pos_dev.x = ptracker_msg_.standardDeviations[i].x;
      pos_dev.y = ptracker_msg_.standardDeviations[i].y;
      tracker_data.standard_deviation.push_back(pos_dev);
      geometry_msgs::Twist vel;
      vel.linear.x = ptracker_msg_.velocities[i].x * invert_x_;
      vel.linear.y = ptracker_msg_.velocities[i].y;
      tracker_data.agent_velocity.push_back(vel);
      geometry_msgs::Twist vel_avg;
      vel_avg.linear.x = ptracker_msg_.averagedVelocities[i].x * invert_x_;
      vel_avg.linear.y = ptracker_msg_.averagedVelocities[i].y;
      tracker_data.agent_avg_velocity.push_back(vel_avg);
    }
    tracker_pub_.publish(tracker_data);
    if (!ptracker_sent_) {ROS_INFO("Tracker data received!");}
    ptracker_rec_ = false;
    ptracker_sent_ = true;
  } else {
    if (ptracker_sent_) {ROS_WARN("Tracker data not received!");}
    ptracker_sent_ = false;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "tracker");
  ros::NodeHandle nh("tracker");
  Tracker tracker(&nh);

  ros::Rate r(10);

  while (ros::ok()) {
    ros::spinOnce();
    tracker.pubTrackerData();
    r.sleep();
  }

  ros::shutdown();

  return 0;
}
