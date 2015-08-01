/**
 * @file      planner_wrapper.hpp
 * @brief     Planner wrapper, connecting the differing planners through ROS
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-06-29
 * @copyright (MIT) 2015 Edinferno
 */

#ifndef PLANNER_WRAPPER_HPP
#define PLANNER_WRAPPER_HPP

#include <ros/ros.h>

#include <planner/rvo_planner.hpp>
#include <common_msgs/Vector2.h>

#include <planner_msgs/SetupNewPlanner.h>
#include <planner_msgs/SetupRVOPlanner.h>

class PlannerWrapper {
 public:
  PlannerWrapper(ros::NodeHandle* nh);
  ~PlannerWrapper();

  void init();

  void rosSetup();

  bool setupNewPlanner(planner_msgs::SetupNewPlanner::Request& req,
                       planner_msgs::SetupNewPlanner::Response& res);

  bool setupRVOPlanner(planner_msgs::SetupRVOPlanner::Request& req,
                       planner_msgs::SetupRVOPlanner::Response& res);

  void plannerStep();

 private:
  // Flags
  bool use_rvo_planner_;
  // ROS
  ros::NodeHandle* nh_;

  ros::Publisher curr_pose_pub_;
  ros::Publisher target_pose_pub_;
  ros::ServiceServer srv_setup_new_planner_;
  ros::ServiceServer srv_setup_rvo_planner_;

  // Class pointers
  RVOPlanner* rvo_planner_;

};

#endif  /* PLANNER_WRAPPER_HPP */
