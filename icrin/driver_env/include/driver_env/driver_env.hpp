/**
 * @file      driver_env.hpp
 * @brief     Main environment for Youbot-ROS-icrin interface
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-08-01
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#ifndef DRIVER_ENV_HPP
#define DRIVER_ENV_HPP

#include <ros/ros.h>
#include <driver_env_msgs/Cars.h>
#include <driver_env_msgs/Car.h>

class DriverEnv {
 public:
  explicit DriverEnv(ros::NodeHandle* nh);
  ~DriverEnv();

  void init();
  void rosSetup();
  void loadParams();

  void carDataCB(const driver_env_msgs::Cars::ConstPtr& msg);

 private:
  // Flags

  // Variables

  // ROS
  ros::NodeHandle* nh_;
  ros::Subscriber car_data_sub_;
};

#endif /* DRIVER_ENV_HPP */
