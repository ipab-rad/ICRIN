/**
 * @file      planner_wrapper.cpp
 * @brief     Planner wrapper, connecting the differing planners through ROS
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-06-29
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#include "planner/planner_wrapper.hpp"

PlannerWrapper::PlannerWrapper(ros::NodeHandle* nh) {
  nh_ = nh;
  robot_name_ = ros::this_node::getNamespace();
  robot_name_.erase (0, 1); // Remove 1 forward slash from robot_name
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
  zero_vect_.x = 0.0f;
  zero_vect_.y = 0.0f;
  goal_vect_.x = 0.0f;
  goal_vect_.y = 0.0f;
  cmd_vel.linear.x = 0.0f;
  cmd_vel.linear.y = 0.0f;
  cmd_vel.linear.z = 0.0f;
  cmd_vel.angular.x = 0.0f;
  cmd_vel.angular.y = 0.0f;
  cmd_vel.angular.z = 0.0f;
}

void PlannerWrapper::rosSetup() {
  cmd_vel_pub_ = nh_->advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
  srv_setup_new_planner_ =
    nh_->advertiseService("setup_new_planner",
                          &PlannerWrapper::setupNewPlanner, this);
  srv_setup_rvo_planner_ =
    nh_->advertiseService("setup_rvo_planner",
                          &PlannerWrapper::setupRVOPlanner, this);
  // srv_add_planner_agents_ =
  //   nh_->advertiseService("add_planner_agents",
  //                         &PlannerWrapper::checkReachedGoal, this);
  curr_pose_sub_ = nh_->subscribe(robot_name_ + "/environment/curr_pose", 1000,
                                  &PlannerWrapper::currPoseCB, this);
  target_goal_sub_ = nh_->subscribe(robot_name_ + "/environment/target_goal",
                                    1000,
                                    &PlannerWrapper::targetGoalCB, this);
}

bool PlannerWrapper::setupNewPlanner(
  planner_msgs::SetupNewPlanner::Request& req,
  planner_msgs::SetupNewPlanner::Response& res) {
  rvo_planner_ = new RVOPlanner(nh_);
  rvo_planner_->addPlannerAgent(zero_vect_);
  // rvo_planner_->setCurrPose(zero_vect_);
  rvo_planner_->setPlannerGoal(goal_vect_);
  use_rvo_planner_ = true;
  res.res = true;
  return true;
}

bool PlannerWrapper::setupRVOPlanner(
  planner_msgs::SetupRVOPlanner::Request& req,
  planner_msgs::SetupRVOPlanner::Response& res) {
  rvo_planner_->setPlannerSettings(req.time_step, req.defaults);
  res.res = true;
  return true;
}

void PlannerWrapper::plannerStep() {
  if (use_rvo_planner_) {
    rvo_planner_->planStep();

    rvo_planner_vel = rvo_planner_->getPlannerVel();
    cmd_vel.linear.x = rvo_planner_vel.x;
    cmd_vel.linear.y = rvo_planner_vel.y;
    cmd_vel_pub_.publish(cmd_vel);
  }
}

void PlannerWrapper::currPoseCB(const geometry_msgs::Pose2D::ConstPtr& msg) {
  if (use_rvo_planner_) {
    common_msgs::Vector2 curr_pose;
    curr_pose.x = msg->x;
    curr_pose.y = msg->y;
    // rvo_planner_->setCurrPose(curr_pose);
  }
}

void PlannerWrapper::targetGoalCB(const geometry_msgs::Pose2D::ConstPtr& msg) {
  if (use_rvo_planner_) {
    common_msgs::Vector2 goal_pose;
    goal_pose.x = msg->x;
    goal_pose.y = msg->y;
    rvo_planner_->setPlannerGoal(goal_pose);
  }
}

// bool Planner::addPlannerAgents(planner_msgs::SetAgentsInitPos::Request& req,
//                                planner_msgs::SetAgentsInitPos::Response& res) {
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

