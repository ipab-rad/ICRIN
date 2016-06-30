/**
 * @file      driver_env.cpp
 * @brief     Driver Environment processes driving data set for ICRIN
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-08-01
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#include "driver_env/driver_env.hpp"

DriverEnv::DriverEnv(ros::NodeHandle* nh) {
  nh_ = nh;
  this->loadParams();
  this->init();
  this->rosSetup();
}

DriverEnv::~DriverEnv() {
  ros::param::del("driver_env");
}

void DriverEnv::init() {

}

void DriverEnv::rosSetup() {
  car_data_sub_ = nh_->subscribe("/driver_env/data", 1000,
                                 &DriverEnv::carDataCB, this);
}

void DriverEnv::loadParams() {
}

void DriverEnv::carDataCB(const driver_env_msgs::Cars::ConstPtr& msg) {

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "driver_env");
  ros::NodeHandle nh("driver_env");
  DriverEnv driver_env(&nh);

  ros::Rate r(10);

  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }

  ros::shutdown();

  return 0;
}
