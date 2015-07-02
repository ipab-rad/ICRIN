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
  rvo_wrapper_msgs::GetAgentMaxNeighbors::Request& req,
  rvo_wrapper_msgs::GetAgentMaxNeighbors::Response& res) {
  res.res = true;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    res.max_neighbors = planner_->getAgentMaxNeighbors(req.agent_id);
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if (req.sim_ids.back() >= req.sim_ids.front()) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i < req.sim_ids.back(); ++i) {
        res.max_neighbors = sim_vect_[i]->getAgentMaxNeighbors(req.agent_id);
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

bool RVOWrapper::getAgentMaxSpeed(
  rvo_wrapper_msgs::GetAgentMaxSpeed::Request& req,
  rvo_wrapper_msgs::GetAgentMaxSpeed::Response& res) {
  res.res = true;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    res.max_speed = planner_->getAgentMaxSpeed(req.agent_id);
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if (req.sim_ids.back() >= req.sim_ids.front()) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i < req.sim_ids.back(); ++i) {
        res.max_speed = sim_vect_[i]->getAgentMaxSpeed(req.agent_id);
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

bool RVOWrapper::getAgentNeighborDist(
  rvo_wrapper_msgs::GetAgentNeighborDist::Request& req,
  rvo_wrapper_msgs::GetAgentNeighborDist::Response& res) {
  res.res = true;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    res.max_neighbor_dist = planner_->getAgentNeighborDist(req.agent_id);
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if (req.sim_ids.back() >= req.sim_ids.front()) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i < req.sim_ids.back(); ++i) {
        res.max_neighbor_dist = sim_vect_[i]->getAgentNeighborDist(req.agent_id);
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

bool RVOWrapper::getAgentNumAgentNeighbors(
  rvo_wrapper_msgs::GetAgentNumAgentNeighbors::Request& req,
  rvo_wrapper_msgs::GetAgentNumAgentNeighbors::Response& res) {
  res.res = true;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    res.num_neighbors = planner_->getAgentNumAgentNeighbors(req.agent_id);
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if (req.sim_ids.back() >= req.sim_ids.front()) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i < req.sim_ids.back(); ++i) {
        res.num_neighbors = sim_vect_[i]->getAgentNumAgentNeighbors(req.agent_id);
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

bool RVOWrapper::getAgentNumObstacleNeighbors(
  rvo_wrapper_msgs::GetAgentNumObstacleNeighbors::Request& req,
  rvo_wrapper_msgs::GetAgentNumObstacleNeighbors::Response& res) {
  res.res = true;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    res.num_obstacles = planner_->getAgentNumObstacleNeighbors(req.agent_id);
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if (req.sim_ids.back() >= req.sim_ids.front()) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i < req.sim_ids.back(); ++i) {
        res.num_obstacles = sim_vect_[i]->getAgentNumObstacleNeighbors(req.agent_id);
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

bool RVOWrapper::getAgentObstacleNeighbor(
  rvo_wrapper_msgs::GetAgentObstacleNeighbor::Request& req,
  rvo_wrapper_msgs::GetAgentObstacleNeighbor::Response& res) {
  res.res = true;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    res.obstacle_vertex = planner_->getAgentObstacleNeighbor(req.agent_id,
                                                             req.agent_obstacle);
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if (req.sim_ids.back() >= req.sim_ids.front()) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i < req.sim_ids.back(); ++i) {
        res.obstacle_vertex = sim_vect_[i]->getAgentObstacleNeighbor(req.agent_id,
                                                                     req.agent_obstacle);
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

bool RVOWrapper::getAgentPosition(
  rvo_wrapper_msgs::GetAgentPosition::Request& req,
  rvo_wrapper_msgs::GetAgentPosition::Response& res) {
  res.res = true;
  RVO::Vector2 position;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    position = planner_->getAgentPosition(req.agent_id);
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if (req.sim_ids.back() >= req.sim_ids.front()) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i < req.sim_ids.back(); ++i) {
        position = sim_vect_[i]->getAgentPosition(req.agent_id);
      }
    } else {
      ROS_WARN("Please provide a proper id range for sim_vector");
      res.res = false;
    }
  } else {
    ROS_WARN("RVO Planner not initialised!");
    res.res = false;
  }
  res.position.x = position.x();
  res.position.y = position.y();
  return true;
}

bool RVOWrapper::getAgentPrefVelocity(
  rvo_wrapper_msgs::GetAgentPrefVelocity::Request& req,
  rvo_wrapper_msgs::GetAgentPrefVelocity::Response& res) {
  res.res = true;
  RVO::Vector2 pref_velocity;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    pref_velocity = planner_->getAgentPrefVelocity(req.agent_id);
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if (req.sim_ids.back() >= req.sim_ids.front()) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i < req.sim_ids.back(); ++i) {
        pref_velocity = sim_vect_[i]->getAgentPrefVelocity(req.agent_id);
      }
    } else {
      ROS_WARN("Please provide a proper id range for sim_vector");
      res.res = false;
    }
  } else {
    ROS_WARN("RVO Planner not initialised!");
    res.res = false;
  }
  res.pref_velocity.x = pref_velocity.x();
  res.pref_velocity.y = pref_velocity.y();
  return true;
}

bool RVOWrapper::getAgentRadius(
  rvo_wrapper_msgs::GetAgentRadius::Request& req,
  rvo_wrapper_msgs::GetAgentRadius::Response& res) {
  res.res = true;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    res.radius = planner_->getAgentRadius(req.agent_id);
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if (req.sim_ids.back() >= req.sim_ids.front()) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i < req.sim_ids.back(); ++i) {
        res.radius = sim_vect_[i]->getAgentRadius(req.agent_id);
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

bool RVOWrapper::getAgentTimeHorizon(
  rvo_wrapper_msgs::GetAgentTimeHorizon::Request& req,
  rvo_wrapper_msgs::GetAgentTimeHorizon::Response& res) {
  res.res = true;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    res.agent_time_horizon = planner_->getAgentTimeHorizon(req.agent_id);
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if (req.sim_ids.back() >= req.sim_ids.front()) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i < req.sim_ids.back(); ++i) {
        res.agent_time_horizon = sim_vect_[i]->getAgentTimeHorizon(req.agent_id);
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

bool RVOWrapper::getAgentTimeHorizonObst(
  rvo_wrapper_msgs::GetAgentTimeHorizonObst::Request& req,
  rvo_wrapper_msgs::GetAgentTimeHorizonObst::Response& res) {
  res.res = true;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    res.obst_time_horizon = planner_->getAgentTimeHorizonObst(req.agent_id);
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if (req.sim_ids.back() >= req.sim_ids.front()) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i < req.sim_ids.back(); ++i) {
        res.obst_time_horizon = sim_vect_[i]->getAgentTimeHorizonObst(req.agent_id);
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

bool RVOWrapper::getAgentVelocity(
  rvo_wrapper_msgs::GetAgentVelocity::Request& req,
  rvo_wrapper_msgs::GetAgentVelocity::Response& res) {
  res.res = true;
  RVO::Vector2 velocity;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    velocity = planner_->getAgentVelocity(req.agent_id);
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if (req.sim_ids.back() >= req.sim_ids.front()) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i < req.sim_ids.back(); ++i) {
        velocity = sim_vect_[i]->getAgentVelocity(req.agent_id);
      }
    } else {
      ROS_WARN("Please provide a proper id range for sim_vector");
      res.res = false;
    }
  } else {
    ROS_WARN("RVO Planner not initialised!");
    res.res = false;
  }
  res.velocity.x = velocity.x();
  res.velocity.y = velocity.y();
  return true;
}

bool RVOWrapper::getGlobalTime(
  rvo_wrapper_msgs::GetGlobalTime::Request& req,
  rvo_wrapper_msgs::GetGlobalTime::Response& res) {
  res.res = true;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    res.global_time = planner_->getGlobalTime();
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if (req.sim_ids.back() >= req.sim_ids.front()) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i < req.sim_ids.back(); ++i) {
        res.global_time = sim_vect_[i]->getGlobalTime();
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

bool RVOWrapper::getNumAgents(
  rvo_wrapper_msgs::GetNumAgents::Request& req,
  rvo_wrapper_msgs::GetNumAgents::Response& res) {
  res.res = true;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    res.num_agents = planner_->getNumAgents();
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if (req.sim_ids.back() >= req.sim_ids.front()) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i < req.sim_ids.back(); ++i) {
        res.num_agents = sim_vect_[i]->getNumAgents();
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

bool RVOWrapper::getTimeStep(
  rvo_wrapper_msgs::GetTimeStep::Request& req,
  rvo_wrapper_msgs::GetTimeStep::Response& res) {
  res.res = true;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    res.time_step = planner_->getTimeStep();
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if (req.sim_ids.back() >= req.sim_ids.front()) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i < req.sim_ids.back(); ++i) {
        res.time_step = sim_vect_[i]->getTimeStep();
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

bool RVOWrapper::processObstacles(
  rvo_wrapper_msgs::ProcessObstacles::Request& req,
  rvo_wrapper_msgs::ProcessObstacles::Response& res) {
  res.res = true;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    planner_->processObstacles();
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if (req.sim_ids.back() >= req.sim_ids.front()) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i < req.sim_ids.back(); ++i) {
        sim_vect_[i]->processObstacles();
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

bool RVOWrapper::queryVisibility(
  rvo_wrapper_msgs::QueryVisibility::Request& req,
  rvo_wrapper_msgs::QueryVisibility::Response& res) {
  res.res = true;
  RVO::Vector2 point1(req.point1.x, req.point1.y);
  RVO::Vector2 point2(req.point2.x, req.point2.y);
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    res.visible = planner_->queryVisibility(point1, point2, req.radius);
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if (req.sim_ids.back() >= req.sim_ids.front()) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i < req.sim_ids.back(); ++i) {
        res.visible = sim_vect_[i]->queryVisibility(point1, point2, req.radius);
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

bool RVOWrapper::setAgentDefaults(
  rvo_wrapper_msgs::SetAgentDefaults::Request& req,
  rvo_wrapper_msgs::SetAgentDefaults::Response& res) {
  res.res = true;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    planner_->setAgentDefaults(req.defaults.neighbor_dist,
                               req.defaults.max_neighbors,
                               req.defaults.time_horizon_agent,
                               req.defaults.time_horizon_obst,
                               req.defaults.radius,
                               req.defaults.max_speed);
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if (req.sim_ids.back() >= req.sim_ids.front()) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i < req.sim_ids.back(); ++i) {
        sim_vect_[i]->setAgentDefaults(req.defaults.neighbor_dist,
                                       req.defaults.max_neighbors,
                                       req.defaults.time_horizon_agent,
                                       req.defaults.time_horizon_obst,
                                       req.defaults.radius,
                                       req.defaults.max_speed);
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

bool RVOWrapper::setAgentMaxNeighbors(
  rvo_wrapper_msgs::SetAgentMaxNeighbors::Request& req,
  rvo_wrapper_msgs::SetAgentMaxNeighbors::Response& res) {
  res.res = true;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    planner_->setAgentMaxNeighbors(req.agent_id, req.max_neighbors);
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if (req.sim_ids.back() >= req.sim_ids.front()) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i < req.sim_ids.back(); ++i) {
        sim_vect_[i]->setAgentMaxNeighbors(req.agent_id, req.max_neighbors);
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

bool RVOWrapper::setAgentMaxSpeed(
  rvo_wrapper_msgs::SetAgentMaxSpeed::Request& req,
  rvo_wrapper_msgs::SetAgentMaxSpeed::Response& res) {
  res.res = true;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    planner_->setAgentMaxSpeed(req.agent_id, req.max_speed);
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if (req.sim_ids.back() >= req.sim_ids.front()) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i < req.sim_ids.back(); ++i) {
        sim_vect_[i]->setAgentMaxSpeed(req.agent_id, req.max_speed);
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

bool RVOWrapper::setAgentNeighborDist(
  rvo_wrapper_msgs::SetAgentNeighborDist::Request& req,
  rvo_wrapper_msgs::SetAgentNeighborDist::Response& res) {
  res.res = true;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    planner_->setAgentNeighborDist(req.agent_id, req.neighbor_dist);
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if (req.sim_ids.back() >= req.sim_ids.front()) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i < req.sim_ids.back(); ++i) {
        sim_vect_[i]->setAgentNeighborDist(req.agent_id, req.neighbor_dist);
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

bool RVOWrapper::setAgentPosition(
  rvo_wrapper_msgs::SetAgentPosition::Request& req,
  rvo_wrapper_msgs::SetAgentPosition::Response& res) {
  res.res = true;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    planner_->setAgentPosition(req.agent_id, RVO::Vector2(req.position.x,
                                                          req.position.y));
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if (req.sim_ids.back() >= req.sim_ids.front()) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i < req.sim_ids.back(); ++i) {
        sim_vect_[i]->setAgentPosition(req.agent_id, RVO::Vector2(req.position.x,
                                                                  req.position.y));
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
