/**
 * @file      planner.cpp
 * @brief     Planner master class harboring main planner loop
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-06-29
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#include <ros/ros.h>

#include "planner/planner_wrapper.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "planner");
  ros::NodeHandle nh("planner");
  PlannerWrapper planner_wrapper(&nh);

  ros::Rate r(10);

  while (ros::ok()) {
    ros::spinOnce();
    planner_wrapper.plannerStep();
    r.sleep();
  }

  ros::shutdown();

  return 0;
}
