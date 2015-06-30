/**
 * @file      planner.cpp
 * @brief     Planner master class, which instantiates different planners
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-06-29
 * @copyright (MIT) 2015 Edinferno
 */

#include "planner/planner.hpp"
#include "planner/rvo_planner.hpp"

Planner::Planner() {;}

Planner::~Planner() {;}

int main(int argc, char** argv) {
  ros::init(argc, argv, "planner");
  ros::NodeHandle nh("planner");

  RVOPlanner rvo_planner(&nh);

  ros::spin();

  ros::shutdown();

  return 0;
}
