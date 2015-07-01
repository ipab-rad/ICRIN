/**
 * @file      rvo_wrapper.cpp
 * @brief     RVO Wrapper class, manages the handling of the RVO library
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-06-29
 * @copyright (MIT) 2015 Edinferno
 */

#include "rvo_wrapper/rvo_wrapper.hpp"

RVOWrapper::RVOWrapper(ros::NodeHandle* nh) {
  nh_ = nh;
  this->rosSetup();
}

RVOWrapper::~RVOWrapper() {;}

void RVOWrapper::rosSetup() {
  srv_create_planner_ = nh_->advertiseService("create_planner",
                                              &RVOWrapper::createPlanner, this);
  srv_do_planner_step_ = nh_->advertiseService("do_planner_step",
                                               &RVOWrapper::doPlannerStep, this);
  srv_add_planner_agent_ = nh_->advertiseService("add_planner_agent",
                                                 &RVOWrapper::addPlannerAgent, this);
  srv_get_agent_pos_ = nh_->advertiseService("get_agent_pos",
                                             &RVOWrapper::getPlannerAgentPos, this);
}
// RVO::RVOSimulator* RVOWrapper::createSimulation() {;}

bool RVOWrapper::createPlanner(std_srvs::Empty::Request& req,
                               std_srvs::Empty::Response& res) {
  planner_ = new RVO::RVOSimulator();
  planner_->setTimeStep(0.1f);
  return true;
}

bool RVOWrapper::addPlannerAgent(rvo_wrapper_msgs::AddPlannerAgent::Request&
                                 req,
                                 rvo_wrapper_msgs::AddPlannerAgent::Response&
                                 res) {
  RVO::Vector2 agent_pos(req.agent_pos.x, req.agent_pos.y);
  res.agent_id = planner_->addAgent(agent_pos, req.neighbor_dist,
                                    req.max_neighbors, req.time_horizon,
                                    req.time_horizon_obst, req.radius,
                                    req.max_speed);
  RVO::Vector2 temp_goal(1.0f, 1.0f);
  planner_->setAgentPrefVelocity(res.agent_id, temp_goal);
  return true;
}

bool RVOWrapper::doPlannerStep(std_srvs::Empty::Request& req,
                               std_srvs::Empty::Response& res) {
  planner_->doStep();
  return true;
}

bool RVOWrapper::getPlannerAgentPos(
  rvo_wrapper_msgs::GetPlannerAgentPos::Request& req,
  rvo_wrapper_msgs::GetPlannerAgentPos::Response& res) {
  RVO::Vector2 agent_pos = planner_->getAgentPosition(req.agent_id);
  res.agent_pos.x = agent_pos.x();
  res.agent_pos.y = agent_pos.y();
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "rvo_wrapper");
  ros::NodeHandle nh("rvo_wrapper");

  RVOWrapper rvo_wrapper(&nh);

  ros::spin();

  ros::shutdown();

  return 0;
}
