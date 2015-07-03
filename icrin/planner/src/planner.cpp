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

  common_msgs::Vector2 init_pos;
  common_msgs::Vector2 target_pos;
  common_msgs::Vector2 curr_pos;

  init_pos.x = 0.0f;
  init_pos.y = 0.0f;

  target_pos.x = 1.0f;
  target_pos.y = 0.0f;

  rvo_planner.addPlannerAgent(init_pos);

  rvo_planner.getAgentPos(0);

  rvo_planner.setPlannerGoal(0, target_pos);

  ros::Rate r(10);

  while (ros::ok()) {
    curr_pos = rvo_planner.getAgentPos(0);
    ROS_INFO("X:%f, Y:%f", curr_pos.x, curr_pos.y);
    rvo_planner.calcPrefVelocities();
    rvo_planner.doPlannerStep();
    ros::spinOnce();
    r.sleep();
  }

  ros::shutdown();

  return 0;
}
