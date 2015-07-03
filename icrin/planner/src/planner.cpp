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
  common_msgs::Vector2 goal1;
  common_msgs::Vector2 goal2;
  common_msgs::Vector2 curr_pos;

  init_pos.x = 0.0f;
  init_pos.y = 0.0f;

  goal1.x = 1.0f;
  goal1.y = 0.0f;

  goal2.x = -1 * goal1.x;
  goal2.y = -1 * goal1.y;

  rvo_planner.addPlannerAgent(init_pos);

  rvo_planner.getAgentPos(0);

  target_pos = goal1;
  rvo_planner.setPlannerGoal(goal1);
  bool going1 = true;

  ros::Rate r(10);

  while (ros::ok()) {
    curr_pos = rvo_planner.getAgentPos(0);
    ROS_INFO("X:%f, Y:%f, TargX:%f, TargY:%f",
             curr_pos.x, curr_pos.y, target_pos.x, target_pos.y);
    rvo_planner.calcPrefVelocities();
    rvo_planner.doPlannerStep();
    if (rvo_planner.checkReachedGoal()) {
      if (going1) {
        target_pos = goal2; going1 = false;
      } else {target_pos = goal1; going1 = true;}
      rvo_planner.setPlannerGoal(target_pos);
    }
    ros::spinOnce();
    r.sleep();
  }

  ros::shutdown();

  return 0;
}
