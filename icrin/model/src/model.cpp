/**
 * @file      model.cpp
 * @brief     Model class for querying motion library and calculate posteriors
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-08-22
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#include <ros/ros.h>

#include "model/model_wrapper.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "model");
  ros::NodeHandle nh("model");
  ModelWrapper model_wrapper(&nh);

  ros::Rate r(10);

  while (ros::ok()) {
    ros::spinOnce();
    model_wrapper.runModel();
    r.sleep();
  }

  ros::shutdown();

  return 0;
}
