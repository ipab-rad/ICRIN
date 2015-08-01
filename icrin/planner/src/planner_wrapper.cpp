/**
 * @file      planner_wrapper.cpp
 * @brief     Planner wrapper, connecting the differing planners through ROS
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-06-29
 * @copyright (MIT) 2015 Edinferno
 */

#include "planner/planner_wrapper.hpp"

PlannerWrapper::PlannerWrapper(ros::NodeHandle* nh) {
  nh_ = nh;
  this->init();
  this->rosSetup();
}

PlannerWrapper::~PlannerWrapper() {
  if (use_rvo_planner_) {
    delete rvo_planner_;
    rvo_planner_ = NULL;
  }
}

void PlannerWrapper::init() {
  use_rvo_planner_ = false;
}

void PlannerWrapper::rosSetup() {
  curr_pose_pub_ = nh_->advertise<geometry_msgs::Pose2D>("curr_pose", 1, true);
  target_pose_pub_ = nh_->advertise<geometry_msgs::Pose2D>("target_pose", 1,
                                                           true);
  srv_setup_new_planner_ =
    nh_->advertiseService("setup_new_planner",
                          &PlannerWrapper::setupNewPlanner, this);
  srv_setup_rvo_planner_ =
    nh_->advertiseService("setup_rvo_planner",
                          &PlannerWrapper::setupRVOPlanner, this);
  // srv_add_planner_agents_ =
  //   nh_->advertiseService("add_planner_agents",
  //                         &PlannerWrapper::checkReachedGoal, this);
  // srv_set_planner_goal_ =
  //   nh_->advertiseService("check_reached_goal",
  //                         &PlannerWrapper::checkReachedGoal, this);
  // srv_set_planner_pose_ =
  //   nh_->advertiseService("check_reached_goal",
  //                         &PlannerWrapper::checkReachedGoal, this);
}

bool PlannerWrapper::setupNewPlanner(
  planner_msgs::SetupNewPlanner::Request& req,
  planner_msgs::SetupNewPlanner::Response& res) {
  rvo_planner_ = new RVOPlanner(nh_);
  use_rvo_planner_ = true;
  return true;
}

bool PlannerWrapper::setupRVOPlanner(
  planner_msgs::SetupRVOPlanner::Request& req,
  planner_msgs::SetupRVOPlanner::Response& res) {
  rvo_planner_->setPlannerSettings(req.time_step, req.defaults);
  return true;
}

void PlannerWrapper::plannerStep() {
  if (use_rvo_planner_) {
    rvo_planner_->planStep();
    curr_pose_pub_.publish(rvo_planner_->getCurrPose());
    target_pose_pub_.publish(rvo_planner_->getTargetPose());
  }
}

// bool Planner::addPlannerAgents(planner_msgs::SetAgentsInitPos::Request& req,
//                                planner_msgs::SetAgentsInitPos::Response& res) {
//   ;
// }

// bool Planner::setPlannerDefaults(planner_msgs::SetPlannerDefaults::Request& req,
//                                  planner_msgs::SetPlannerDefaults::Response& res) {
//   ;
// }

// bool Planner::setPlannerGoal(planner_msgs::SetPlannerGoal::Request& req,
//                              planner_msgs::SetPlannerGoal::Response& res) {
//   ;
// }

// int main(int argc, char** argv) {
//   ros::init(argc, argv, "planner");
//   ros::NodeHandle nh("planner");
//   PlannerWrapper planner(&nh);

//   if (planner.use_rvo_planner_) {
//     rvo_planner_->addPlannerAgent(planner.init_pos_);
//     rvo_planner_->setPlannerGoal(planner.goal1_);

//     ros::Rate r(10);

//     // Flags
//     bool going1;
//     // Variables
//     common_msgs::Vector2 target_pos;
//     while (ros::ok()) {
//       rvo_planner_->planStep();
//       if (rvo_planner_->checkReachedGoal()) {
//         if (going1) {
//           target_pos = planner.goal2_; going1 = false;
//         } else {target_pos = planner.goal1_; going1 = true;}
//         rvo_planner_->setPlannerGoal(target_pos);
//       }
//       ros::spinOnce();
//       r.sleep();
//     }
//   }

//   ros::shutdown();

//   return 0;
// }

