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
  this->init();
  this->rosSetup();
}

RVOWrapper::~RVOWrapper() {;}

void RVOWrapper::init() {
  planner_init_ = false;
}

void RVOWrapper::rosSetup() {
  srv_create_rvosim_ =
    nh_->advertiseService("create_rvosim",
                          &RVOWrapper::createRVOSim, this);
  srv_add_agent_ =
    nh_->advertiseService("add_agent",
                          &RVOWrapper::addAgent, this);
  srv_add_osbtacle_ =
    nh_->advertiseService("add_osbtacle",
                          &RVOWrapper::addObstacle, this);
  srv_do_step_ =
    nh_->advertiseService("do_step",
                          &RVOWrapper::doStep, this);
  srv_get_agent_agent_neighbor_ =
    nh_->advertiseService("get_agent_agent_neighbor",
                          &RVOWrapper::getAgentAgentNeighbor, this);
  srv_get_agent_max_neighbors_ =
    nh_->advertiseService("get_agent_max_neighbors",
                          &RVOWrapper::getAgentMaxNeighbors, this);
  srv_get_agent_max_speed_ =
    nh_->advertiseService("get_agent_max_speed",
                          &RVOWrapper::getAgentMaxSpeed, this);
  srv_get_agent_neighbor_dist_ =
    nh_->advertiseService("get_agent_neighbor_dist",
                          &RVOWrapper::getAgentNeighborDist, this);
  srv_get_agent_num_agent_neighbors_ =
    nh_->advertiseService("get_agent_num_agent_neighbors",
                          &RVOWrapper::getAgentNumAgentNeighbors, this);
  srv_get_agent_num_obstacle_neighbors_ =
    nh_->advertiseService("get_agent_num_obstacle_neighbors",
                          &RVOWrapper::getAgentNumObstacleNeighbors, this);
  srv_get_agent_obstacle_neighbor_ =
    nh_->advertiseService("get_agent_obstacle_neighbor",
                          &RVOWrapper::getAgentObstacleNeighbor, this);
  srv_get_agent_position_ =
    nh_->advertiseService("get_agent_position",
                          &RVOWrapper::getAgentPosition, this);
  srv_get_agent_pref_velocity_ =
    nh_->advertiseService("get_agent_pref_velocity",
                          &RVOWrapper::getAgentPrefVelocity, this);
  srv_get_agent_radius_ =
    nh_->advertiseService("get_agent_radius",
                          &RVOWrapper::getAgentRadius, this);
  srv_get_agent_time_horizon_ =
    nh_->advertiseService("get_agent_time_horizon",
                          &RVOWrapper::getAgentTimeHorizon, this);
  srv_get_agent_time_horizon_obst_ =
    nh_->advertiseService("get_agent_time_horizon_obst",
                          &RVOWrapper::getAgentTimeHorizonObst, this);
  srv_get_agent_velocity_ =
    nh_->advertiseService("get_agent_velocity",
                          &RVOWrapper::getAgentVelocity, this);
  srv_get_global_time_ =
    nh_->advertiseService("get_global_time",
                          &RVOWrapper::getGlobalTime, this);
  srv_get_num_agents_ =
    nh_->advertiseService("get_num_agents",
                          &RVOWrapper::getNumAgents, this);
  srv_get_time_step_ =
    nh_->advertiseService("get_time_step",
                          &RVOWrapper::getTimeStep, this);
  srv_process_obstacles_ =
    nh_->advertiseService("process_obstacles",
                          &RVOWrapper::processObstacles, this);
  srv_query_visibility_ =
    nh_->advertiseService("query_visibility",
                          &RVOWrapper::queryVisibility, this);
  srv_set_agent_defaults_ =
    nh_->advertiseService("set_agent_defaults",
                          &RVOWrapper::setAgentDefaults, this);
  srv_set_agent_max_neighbors_ =
    nh_->advertiseService("set_agent_max_neighbors",
                          &RVOWrapper::setAgentMaxNeighbors, this);
  srv_set_agent_max_speed_ =
    nh_->advertiseService("set_agent_max_speed",
                          &RVOWrapper::setAgentMaxSpeed, this);
  srv_set_agent_neighbor_dist_ =
    nh_->advertiseService("set_agent_neighbor_dist",
                          &RVOWrapper::setAgentNeighborDist, this);
  srv_set_agent_position_ =
    nh_->advertiseService("set_agent_position",
                          &RVOWrapper::setAgentPosition, this);
  srv_set_agent_pref_velocity_ =
    nh_->advertiseService("set_agent_pref_velocity",
                          &RVOWrapper::setAgentPrefVelocity, this);
  srv_set_agent_radius_ =
    nh_->advertiseService("set_agent_radius",
                          &RVOWrapper::setAgentRadius, this);
  srv_set_agent_time_horizon_ =
    nh_->advertiseService("set_agent_time_horizon",
                          &RVOWrapper::setAgentTimeHorizon, this);
  srv_set_agent_time_horizon_obst_ =
    nh_->advertiseService("set_agent_time_horizon_obst",
                          &RVOWrapper::setAgentTimeHorizonObst, this);
  srv_set_agent_velocity_ =
    nh_->advertiseService("set_agent_velocity",
                          &RVOWrapper::setAgentVelocity, this);
  srv_set_time_step_ =
    nh_->advertiseService("set_time_step",
                          &RVOWrapper::setTimeStep, this);
}

bool RVOWrapper::createRVOSim(
  rvo_wrapper_msgs::CreateRVOSim::Request& req,
  rvo_wrapper_msgs::CreateRVOSim::Response& res) {
  res.res = true;
  if (req.sim_num == 0 && !planner_init_) { // If Planner
    if (req.time_step == 0.0f) { // If defaults not set
      planner_ = new RVO::RVOSimulator();
    } else {
      planner_ = new RVO::RVOSimulator(req.time_step,
                                       req.defaults.neighbor_dist,
                                       req.defaults.max_neighbors,
                                       req.defaults.time_horizon_agent,
                                       req.defaults.time_horizon_obst,
                                       req.defaults.radius,
                                       req.defaults.max_speed);
    }
    res.sim_ids.push_back(0);
    planner_init_ = true;
  } else {
    uint32_t sim_vect_size = sim_vect_.size(); // If Sim Vector
    res.sim_ids.push_back(sim_vect_size); // Store first sim_vector id
    if (req.time_step == 0.0f) { // If defaults not set
      for (uint32_t i = sim_vect_size; i < req.sim_num; ++i) {
        sim_vect_.push_back(new RVO::RVOSimulator());
      }
    } else {
      for (uint32_t i = sim_vect_size; i < req.sim_num; ++i) {
        sim_vect_.push_back(new RVO::RVOSimulator(req.time_step,
                                                  req.defaults.neighbor_dist,
                                                  req.defaults.max_neighbors,
                                                  req.defaults.time_horizon_agent,
                                                  req.defaults.time_horizon_obst,
                                                  req.defaults.radius,
                                                  req.defaults.max_speed));
      }
      // Store last sim_vector id
      res.sim_ids.push_back(sim_vect_.size());
    }
  }
  return true;
}

bool RVOWrapper::addAgent(
  rvo_wrapper_msgs::AddAgent::Request& req,
  rvo_wrapper_msgs::AddAgent::Response& res) {
  res.res = true;
  RVO::Vector2 agent_pos(req.position.x, req.position.y);
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    if (req.defaults.radius == 0.0f) { // If defaults not set
      res.agent_id = planner_->addAgent(agent_pos);
    } else {
      res.agent_id = planner_->addAgent(agent_pos,
                                        req.defaults.neighbor_dist,
                                        req.defaults.max_neighbors,
                                        req.defaults.time_horizon_agent,
                                        req.defaults.time_horizon_obst,
                                        req.defaults.radius,
                                        req.defaults.max_speed);
    }
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if (req.sim_ids.back() >= req.sim_ids.front()) { // If good sim id range
      if (req.defaults.radius == 0.0f) { // If defaults not set
        for (uint32_t i = req.sim_ids.front(); i < req.sim_ids.back(); ++i) {
          res.agent_id = sim_vect_[i]->addAgent(agent_pos);
        }
      } else {
        for (uint32_t i = req.sim_ids.front(); i < req.sim_ids.back(); ++i) {
          res.agent_id = sim_vect_[i]->addAgent(agent_pos,
                                                req.defaults.neighbor_dist,
                                                req.defaults.max_neighbors,
                                                req.defaults.time_horizon_agent,
                                                req.defaults.time_horizon_obst,
                                                req.defaults.radius,
                                                req.defaults.max_speed);
        }
      }
    } else {
      ROS_WARN("Please provide a proper id range for sim_vector");
      res.res = false;
    }
  } else {
    ROS_WARN("RVO Planner not initialised!");
    res.res = false;
  }
  return true;
}

bool RVOWrapper::addObstacle(
  rvo_wrapper_msgs::AddObstacle::Request& req,
  rvo_wrapper_msgs::AddObstacle::Response& res) {
  res.res = true;
  std::vector<RVO::Vector2> vertices; // Get msg obstacle vertices
  for (uint32_t i = 0; i < req.vertices.size(); ++i) {
    vertices.push_back(RVO::Vector2(req.vertices[i].x, req.vertices[i].y));
  }
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    planner_->addObstacle(vertices);
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if (req.sim_ids.back() >= req.sim_ids.front()) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i < req.sim_ids.back(); ++i) {
        sim_vect_[i]->addObstacle(vertices);
      }
    } else {
      ROS_WARN("Please provide a proper id range for sim_vector");
      res.res = false;
    }
  } else {
    ROS_WARN("RVO Planner not initialised!");
    res.res = false;
  }
  return true;
}

bool RVOWrapper::doStep(
  rvo_wrapper_msgs::DoStep::Request& req,
  rvo_wrapper_msgs::DoStep::Response& res) {
  res.res = true;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    planner_->doStep();
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if (req.sim_ids.back() >= req.sim_ids.front()) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i < req.sim_ids.back(); ++i) {
        sim_vect_[i]->doStep();
      }
    } else {
      ROS_WARN("Please provide a proper id range for sim_vector");
      res.res = false;
    }
  } else {
    ROS_WARN("RVO Planner not initialised!");
    res.res = false;
  }
  return true;
}

bool RVOWrapper::getAgentAgentNeighbor(
  rvo_wrapper_msgs::GetAgentAgentNeighbor::Request& req,
  rvo_wrapper_msgs::GetAgentAgentNeighbor::Response& res) {
  res.res = true;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    res.neighbor_id = planner_->getAgentAgentNeighbor(req.agent_id,
                                                      req.agent_neighbor);
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if (req.sim_ids.back() >= req.sim_ids.front()) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i < req.sim_ids.back(); ++i) {
        res.neighbor_id = sim_vect_[i]->getAgentAgentNeighbor(req.agent_id,
                                                              req.agent_neighbor);
      }
    } else {
      ROS_WARN("Please provide a proper id range for sim_vector");
      res.res = false;
    }
  } else {
    ROS_WARN("RVO Planner not initialised!");
    res.res = false;
  }
  return true;
}

bool RVOWrapper::getAgentMaxNeighbors(
  rvo_wrapper_msgs::GetAgentMaxNeighbor::Request& req,
  rvo_wrapper_msgs::GetAgentMaxNeighbor::Response& res) {
  return true;
}

bool RVOWrapper::getAgentMaxSpeed(
  rvo_wrapper_msgs::GetAgentMaxSpeed::Request& req,
  rvo_wrapper_msgs::GetAgentMaxSpeed::Response& res) {
  return true;
}

bool RVOWrapper::getAgentNeighborDist(
  rvo_wrapper_msgs::GetAgentNeighborDist::Request& req,
  rvo_wrapper_msgs::GetAgentNeighborDist::Response& res) {
  return true;
}

bool RVOWrapper::getAgentNumAgentNeighbors(
  rvo_wrapper_msgs::GetAgentNumAgentNeighbors::Request& req,
  rvo_wrapper_msgs::GetAgentNumAgentNeighbors::Response& res) {
  return true;
}

bool RVOWrapper::getAgentNumObstacleNeighbors(
  rvo_wrapper_msgs::GetAgentNumObstacleNeighbors::Request& req,
  rvo_wrapper_msgs::GetAgentNumObstacleNeighbors::Response& res) {
  return true;
}

bool RVOWrapper::getAgentObstacleNeighbor(
  rvo_wrapper_msgs::GetAgentObstacleNeighbor::Request& req,
  rvo_wrapper_msgs::GetAgentObstacleNeighbor::Response& res) {
  return true;
}

bool RVOWrapper::getAgentPosition(
  rvo_wrapper_msgs::GetAgentPosition::Request& req,
  rvo_wrapper_msgs::GetAgentPosition::Response& res) {
  return true;
}

bool RVOWrapper::getAgentPrefVelocity(
  rvo_wrapper_msgs::GetAgentPrefVelocity::Request& req,
  rvo_wrapper_msgs::GetAgentPrefVelocity::Response& res) {
  return true;
}

bool RVOWrapper::getAgentRadius(
  rvo_wrapper_msgs::GetAgentRadius::Request& req,
  rvo_wrapper_msgs::GetAgentRadius::Response& res) {
  return true;
}

bool RVOWrapper::getAgentTimeHorizon(
  rvo_wrapper_msgs::GetAgentTimeHorizon::Request& req,
  rvo_wrapper_msgs::GetAgentTimeHorizon::Response& res) {
  return true;
}

bool RVOWrapper::getAgentTimeHorizonObst(
  rvo_wrapper_msgs::GetAgentTimeHorizonObst::Request& req,
  rvo_wrapper_msgs::GetAgentTimeHorizonObst::Response& res) {
  return true;
}

bool RVOWrapper::getAgentVelocity(
  rvo_wrapper_msgs::GetAgentVelocity::Request& req,
  rvo_wrapper_msgs::GetAgentVelocity::Response& res) {
  return true;
}

bool RVOWrapper::getGlobalTime(
  rvo_wrapper_msgs::GetGlobalTime::Request& req,
  rvo_wrapper_msgs::GetGlobalTime::Response& res) {
  return true;
}

bool RVOWrapper::getNumAgents(
  rvo_wrapper_msgs::GetNumAgents::Request& req,
  rvo_wrapper_msgs::GetNumAgents::Response& res) {
  return true;
}

bool RVOWrapper::getTimeStep(
  rvo_wrapper_msgs::GetTimeStep::Request& req,
  rvo_wrapper_msgs::GetTimeStep::Response& res) {
  return true;
}

bool RVOWrapper::processObstacles(
  rvo_wrapper_msgs::ProcessObstacles::Request& req,
  rvo_wrapper_msgs::ProcessObstacles::Response& res) {
  return true;
}

bool RVOWrapper::queryVisibility(
  rvo_wrapper_msgs::QueryVisibility::Request& req,
  rvo_wrapper_msgs::QueryVisibility::Response& res) {
  return true;
}

bool RVOWrapper::setAgentDefaults(
  rvo_wrapper_msgs::SetAgentDefaults::Request& req,
  rvo_wrapper_msgs::SetAgentDefaults::Response& res) {
  return true;
}

bool RVOWrapper::setAgentMaxNeighbors(
  rvo_wrapper_msgs::SetAgentMaxNeighbors::Request& req,
  rvo_wrapper_msgs::SetAgentMaxNeighbors::Response& res) {
  return true;
}

bool RVOWrapper::setAgentMaxSpeed(
  rvo_wrapper_msgs::SetAgentMaxSpeed::Request& req,
  rvo_wrapper_msgs::SetAgentMaxSpeed::Response& res) {
  return true;
}

bool RVOWrapper::setAgentNeighborDist(
  rvo_wrapper_msgs::SetAgentNeighborDist::Request& req,
  rvo_wrapper_msgs::SetAgentNeighborDist::Response& res) {
  return true;
}

bool RVOWrapper::setAgentPosition(
  rvo_wrapper_msgs::SetAgentPosition::Request& req,
  rvo_wrapper_msgs::SetAgentPosition::Response& res) {
  return true;
}

bool RVOWrapper::setAgentPrefVelocity(
  rvo_wrapper_msgs::SetAgentPrefVelocity::Request& req,
  rvo_wrapper_msgs::SetAgentPrefVelocity::Response& res) {
  return true;
}

bool RVOWrapper::setAgentRadius(
  rvo_wrapper_msgs::SetAgentRadius::Request& req,
  rvo_wrapper_msgs::SetAgentRadius::Response& res) {
  return true;
}

bool RVOWrapper::setAgentTimeHorizon(
  rvo_wrapper_msgs::SetAgentTimeHorizon::Request& req,
  rvo_wrapper_msgs::SetAgentTimeHorizon::Response& res) {
  return true;
}

bool RVOWrapper::setAgentTimeHorizonObst(
  rvo_wrapper_msgs::SetAgentTimeHorizonObst::Request& req,
  rvo_wrapper_msgs::SetAgentTimeHorizonObst::Response& res) {
  return true;
}

bool RVOWrapper::setAgentVelocity(
  rvo_wrapper_msgs::SetAgentVelocity::Request& req,
  rvo_wrapper_msgs::SetAgentVelocity::Response& res) {
  return true;
}

bool RVOWrapper::setTimeStep(
  rvo_wrapper_msgs::SetTimeStep::Request& req,
  rvo_wrapper_msgs::SetTimeStep::Response& res) {
  return true;
}

// bool RVOWrapper::doPlannerStep(std_srvs::Empty::Request& req,
//                                std_srvs::Empty::Response& res) {
//   planner_->doStep();
//   return true;
// }

// bool RVOWrapper::getPlannerAgentPos(
//   rvo_wrapper_msgs::GetPlannerAgentPos::Request& req,
//   rvo_wrapper_msgs::GetPlannerAgentPos::Response& res) {
//   RVO::Vector2 agent_pos = planner_->getAgentPosition(req.agent_id);
//   res.agent_pos.x = agent_pos.x();
//   res.agent_pos.y = agent_pos.y();
//   return true;
// }

int main(int argc, char** argv) {
  ros::init(argc, argv, "rvo_wrapper");
  ros::NodeHandle nh("rvo_wrapper");

  RVOWrapper rvo_wrapper(&nh);

  ros::spin();

  ros::shutdown();

  return 0;
}
