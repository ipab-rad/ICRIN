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
#include <environment_msgs/EnvironmentData.h>
#include <model_msgs/ModelHypotheses.h>
#include <model_msgs/GoalInference.h>
#include <model_msgs/GoalEstimate.h>

class DriverEnv {
 public:
  explicit DriverEnv(ros::NodeHandle* nh);
  ~DriverEnv();

  void init();
  void rosSetup();
  void loadParams();

  void carDataCB(const driver_env_msgs::Cars::ConstPtr& msg);

  void runModel();
  void pubEnvData();
  void pubHypotheses();

 private:
  // Flags

  // Variables
  driver_env_msgs::Cars car_data_;
  std::vector<geometry_msgs::Pose2D> goals_;
  std::vector<uint> agent_ids_;


  // ROS
  ros::NodeHandle* nh_;
  ros::Publisher environment_data_pub_;
  ros::Publisher model_pub_;
  ros::Subscriber car_data_sub_;
  ros::Subscriber model_sub_;
};

#endif /* DRIVER_ENV_HPP */
