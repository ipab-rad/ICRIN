/**
 * @file      rvo_wrapper.cpp
 * @brief     RVO Wrapper class, manages the handling of the RVO library
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-06-29
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#include "rvo_wrapper/rvo_wrapper.hpp"

RVOWrapper::RVOWrapper(ros::NodeHandle* nh) {
  nh_ = nh;
  this->init();
  this->rosSetup();
}

RVOWrapper::~RVOWrapper() {
  if (planner_init_) {
    delete planner_;
    planner_ = NULL;
  }
  for (uint32_t i = 0; i < sim_vect_.size(); ++i) {
    delete (sim_vect_[i]);
  }
  sim_vect_.clear();
}

void RVOWrapper::init() {
  planner_init_ = false;
}

void RVOWrapper::rosSetup() {
  srv_add_agent_ =
    nh_->advertiseService("add_agent",
                          &RVOWrapper::addAgent, this);
  srv_add_osbtacle_ =
    nh_->advertiseService("add_osbtacle",
                          &RVOWrapper::addObstacle, this);
  srv_check_reached_goal_ =
    nh_->advertiseService("check_reached_goal",
                          &RVOWrapper::checkReachedGoal, this);
  srv_calc_pref_velocities_ =
    nh_->advertiseService("calc_pref_velocities",
                          &RVOWrapper::calcPrefVelocities, this);
  srv_create_rvosim_ =
    nh_->advertiseService("create_rvosim",
                          &RVOWrapper::createRVOSim, this);
  srv_delete_sim_vector_ =
    nh_->advertiseService("delete_sim_vector",
                          &RVOWrapper::deleteSimVector, this);
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
  srv_set_agent_goals_ =
    nh_->advertiseService("set_agent_goals",
                          &RVOWrapper::setAgentGoals, this);
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

bool RVOWrapper::addAgent(
  rvo_wrapper_msgs::AddAgent::Request& req,
  rvo_wrapper_msgs::AddAgent::Response& res) {
  res.ok = true;
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
                                        req.defaults.max_speed,
                                        req.defaults.max_accel,
                                        req.defaults.pref_speed);
    }
    planner_goals_.push_back(RVO::Vector2(0.0f, 0.0f));
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if ((req.sim_ids.back() >= req.sim_ids.front()) &&
        (req.sim_ids.back() < sim_vect_.size())) { // If good sim id range
      if (req.defaults.radius == 0.0f) { // If defaults not set
        for (uint32_t i = req.sim_ids.front(); i <= req.sim_ids.back(); ++i) {
          res.agent_id = sim_vect_[i]->addAgent(agent_pos);
          sim_vect_goals_[i].push_back(RVO::Vector2(0.0f, 0.0f));
        }
      } else {
        for (uint32_t i = req.sim_ids.front(); i <= req.sim_ids.back(); ++i) {
          res.agent_id = sim_vect_[i]->addAgent(agent_pos,
                                                req.defaults.neighbor_dist,
                                                req.defaults.max_neighbors,
                                                req.defaults.time_horizon_agent,
                                                req.defaults.time_horizon_obst,
                                                req.defaults.radius,
                                                req.defaults.max_speed,
                                                req.defaults.max_accel,
                                                req.defaults.pref_speed);
          sim_vect_goals_[i].push_back(RVO::Vector2(0.0f, 0.0f));
        }
      }
    } else {
      ROS_WARN("Please provide a proper id range for sim_vector");
      res.ok = false;
    }
  } else {
    ROS_WARN("RVO Planner not initialised!");
    res.ok = false;
  }
  return true;
}

bool RVOWrapper::addObstacle(
  rvo_wrapper_msgs::AddObstacle::Request& req,
  rvo_wrapper_msgs::AddObstacle::Response& res) {
  res.ok = true;
  std::vector<RVO::Vector2> vertices; // Get msg obstacle vertices
  for (uint32_t i = 0; i < req.vertices.size(); ++i) {
    vertices.push_back(RVO::Vector2(req.vertices[i].x, req.vertices[i].y));
  }
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    planner_->addObstacle(vertices);
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if ((req.sim_ids.back() >= req.sim_ids.front()) &&
        (req.sim_ids.back() < sim_vect_.size())) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i <= req.sim_ids.back(); ++i) {
        sim_vect_[i]->addObstacle(vertices);
      }
    } else {
      ROS_WARN("Please provide a proper id range for sim_vector");
      res.ok = false;
    }
  } else {
    ROS_WARN("RVO Planner not initialised!");
    res.ok = false;
  }
  return true;
}

bool RVOWrapper::calcPrefVelocities(
  rvo_wrapper_msgs::CalcPrefVelocities::Request& req,
  rvo_wrapper_msgs::CalcPrefVelocities::Response& res) {
  res.ok = true;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    for (uint32_t i = 0; i < planner_->getNumAgents(); ++i) {
      RVO::Vector2 goalVector = planner_goals_[i] - planner_->getAgentPosition(i);
      RVO::Vector2 prefVel;
      if (i == 0) {
        if (RVO::absSq(goalVector) > 1.0f) {
          goalVector = RVO::normalize(goalVector);
        }
        prefVel = planner_->getAgentPrefSpeed(i) * goalVector;
      } else {
        prefVel = planner_->getAgentVelocity(i);
      }
      planner_->setAgentPrefVelocity(i, prefVel);
    }
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if ((req.sim_ids.back() >= req.sim_ids.front()) &&
        (req.sim_ids.back() < sim_vect_.size())) { // If good sim id range
      for (uint32_t j = req.sim_ids.front(); j <= req.sim_ids.back(); ++j) {
        for (uint32_t i = 0; i < sim_vect_[j]->getNumAgents(); ++i) {
          if (sim_vect_goals_[j][i] != null_vect_) { // Goal has been set
            RVO::Vector2 goalVector = sim_vect_goals_[j][i] -
                                      sim_vect_[j]->getAgentPosition(i);
            if (RVO::absSq(goalVector) > 1.0f) {
              goalVector = RVO::normalize(goalVector);
            }
            RVO::Vector2 prefVel = sim_vect_[j]->getAgentPrefSpeed(i) * goalVector;
            sim_vect_[j]->setAgentPrefVelocity(i, prefVel);
          } else { // Unmodelled agents have no goals, so pref vel is current vel
            sim_vect_[j]->setAgentPrefVelocity(
              i, sim_vect_[j]->getAgentVelocity(i));
          }
        }
      }
    } else {
      ROS_WARN("Please provide a proper id range for sim_vector");
      res.ok = false;
    }
  } else {
    ROS_WARN("RVO Planner not initialised!");
    res.ok = false;
  }
  return true;
}

bool RVOWrapper::checkReachedGoal(
  rvo_wrapper_msgs::CheckReachedGoal::Request& req,
  rvo_wrapper_msgs::CheckReachedGoal::Response& res) {
  res.ok = true;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    ROS_INFO("RVO Wrapper- Goal: %f, %f. Dist: %f",
             planner_goals_[0].x(), planner_goals_[0].y(),
             RVO::absSq(planner_->getAgentPosition(0) - planner_goals_[0]));
    if (RVO::absSq(planner_->getAgentPosition(0) - planner_goals_[0]) <
        planner_->getAgentRadius(0) / 2) {
      res.reached = true;
    } else { res.reached = false; }
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    ROS_WARN("Not yet implemented for sim goals!");
    res.ok = false;
  } else {
    ROS_WARN("RVO Planner not initialised!");
    res.ok = false;
  }
  return true;
}

bool RVOWrapper::createRVOSim(
  rvo_wrapper_msgs::CreateRVOSim::Request& req,
  rvo_wrapper_msgs::CreateRVOSim::Response& res) {
  res.ok = true;
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
                                       req.defaults.max_speed,
                                       req.defaults.max_accel,
                                       req.defaults.pref_speed);
    }
    res.sim_ids.push_back(0);
    planner_init_ = true;
  } else if (req.sim_num > 0) {
    uint32_t sim_vect_size = sim_vect_.size(); // If Sim Vector
    res.sim_ids.push_back(sim_vect_size); // Store first sim_vector id
    if (req.time_step == 0.0f) { // If defaults not set
      for (uint32_t i = sim_vect_size; i < req.sim_num + sim_vect_size; ++i) {
        sim_vect_.push_back(new RVO::RVOSimulator());
        std::vector<RVO::Vector2> empty;
        sim_vect_goals_.push_back(empty);
      }
    } else {
      for (uint32_t i = sim_vect_size; i < req.sim_num + sim_vect_size; ++i) {
        sim_vect_.push_back(new RVO::RVOSimulator(req.time_step,
                                                  req.defaults.neighbor_dist,
                                                  req.defaults.max_neighbors,
                                                  req.defaults.time_horizon_agent,
                                                  req.defaults.time_horizon_obst,
                                                  req.defaults.radius,
                                                  req.defaults.max_speed,
                                                  req.defaults.max_accel,
                                                  req.defaults.pref_speed));
        std::vector<RVO::Vector2> empty;
        sim_vect_goals_.push_back(empty);
      }
      // Store last sim_vector id
      res.sim_ids.push_back(sim_vect_.size() - 1);
    }
  } else if (planner_init_) {
    ROS_WARN("Planner already initialised!");
    res.ok = false;
  }
  return true;
}

bool RVOWrapper::deleteSimVector(
  rvo_wrapper_msgs::DeleteSimVector::Request& req,
  rvo_wrapper_msgs::DeleteSimVector::Response& res) {
  res.ok = true;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    delete planner_;
    planner_ = NULL;
    planner_init_ = false;
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    for (uint32_t i = 0; i < sim_vect_.size(); ++i) {
      delete (sim_vect_[i]);
    }
    sim_vect_.clear();
    sim_vect_goals_.clear();
  }
  return true;
}

bool RVOWrapper::doStep(
  rvo_wrapper_msgs::DoStep::Request& req,
  rvo_wrapper_msgs::DoStep::Response& res) {
  // ROS_INFO("RVO Wrapper- SimStep start");
  res.ok = true;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    planner_->doStep();
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if ((req.sim_ids.back() >= req.sim_ids.front()) &&
        (req.sim_ids.back() < sim_vect_.size())) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i <= req.sim_ids.back(); ++i) {
        sim_vect_[i]->doStep();
      }
    } else {
      ROS_WARN("Please provide a proper id range for sim_vector");
      res.ok = false;
    }
  } else {
    ROS_WARN("RVO Planner not initialised!");
    res.ok = false;
  }
  // ROS_INFO("RVO Wrapper- SimStep end");
  return true;
}

bool RVOWrapper::getAgentAgentNeighbor(
  rvo_wrapper_msgs::GetAgentAgentNeighbor::Request& req,
  rvo_wrapper_msgs::GetAgentAgentNeighbor::Response& res) {
  res.ok = true;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    res.neighbor_id = planner_->getAgentAgentNeighbor(req.agent_id,
                                                      req.agent_neighbor);
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if ((req.sim_ids.back() >= req.sim_ids.front()) &&
        (req.sim_ids.back() < sim_vect_.size())) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i <= req.sim_ids.back(); ++i) {
        res.neighbor_id = sim_vect_[i]->getAgentAgentNeighbor(req.agent_id,
                                                              req.agent_neighbor);
      }
    } else {
      ROS_WARN("Please provide a proper id range for sim_vector");
      res.ok = false;
    }
  } else {
    ROS_WARN("RVO Planner not initialised!");
    res.ok = false;
  }
  return true;
}

bool RVOWrapper::getAgentMaxNeighbors(
  rvo_wrapper_msgs::GetAgentMaxNeighbors::Request& req,
  rvo_wrapper_msgs::GetAgentMaxNeighbors::Response& res) {
  res.ok = true;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    res.max_neighbors = planner_->getAgentMaxNeighbors(req.agent_id);
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if ((req.sim_ids.back() >= req.sim_ids.front()) &&
        (req.sim_ids.back() < sim_vect_.size())) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i <= req.sim_ids.back(); ++i) {
        res.max_neighbors = sim_vect_[i]->getAgentMaxNeighbors(req.agent_id);
      }
    } else {
      ROS_WARN("Please provide a proper id range for sim_vector");
      res.ok = false;
    }
  } else {
    ROS_WARN("RVO Planner not initialised!");
    res.ok = false;
  }
  return true;
}

bool RVOWrapper::getAgentMaxSpeed(
  rvo_wrapper_msgs::GetAgentMaxSpeed::Request& req,
  rvo_wrapper_msgs::GetAgentMaxSpeed::Response& res) {
  res.ok = true;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    res.max_speed = planner_->getAgentMaxSpeed(req.agent_id);
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if ((req.sim_ids.back() >= req.sim_ids.front()) &&
        (req.sim_ids.back() < sim_vect_.size())) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i <= req.sim_ids.back(); ++i) {
        res.max_speed = sim_vect_[i]->getAgentMaxSpeed(req.agent_id);
      }
    } else {
      ROS_WARN("Please provide a proper id range for sim_vector");
      res.ok = false;
    }
  } else {
    ROS_WARN("RVO Planner not initialised!");
    res.ok = false;
  }
  return true;
}

bool RVOWrapper::getAgentNeighborDist(
  rvo_wrapper_msgs::GetAgentNeighborDist::Request& req,
  rvo_wrapper_msgs::GetAgentNeighborDist::Response& res) {
  res.ok = true;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    res.max_neighbor_dist = planner_->getAgentNeighborDist(req.agent_id);
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if ((req.sim_ids.back() >= req.sim_ids.front()) &&
        (req.sim_ids.back() < sim_vect_.size())) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i <= req.sim_ids.back(); ++i) {
        res.max_neighbor_dist = sim_vect_[i]->getAgentNeighborDist(req.agent_id);
      }
    } else {
      ROS_WARN("Please provide a proper id range for sim_vector");
      res.ok = false;
    }
  } else {
    ROS_WARN("RVO Planner not initialised!");
    res.ok = false;
  }
  return true;
}

bool RVOWrapper::getAgentNumAgentNeighbors(
  rvo_wrapper_msgs::GetAgentNumAgentNeighbors::Request& req,
  rvo_wrapper_msgs::GetAgentNumAgentNeighbors::Response& res) {
  res.ok = true;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    res.num_neighbors = planner_->getAgentNumAgentNeighbors(req.agent_id);
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if ((req.sim_ids.back() >= req.sim_ids.front()) &&
        (req.sim_ids.back() < sim_vect_.size())) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i <= req.sim_ids.back(); ++i) {
        res.num_neighbors = sim_vect_[i]->getAgentNumAgentNeighbors(req.agent_id);
      }
    } else {
      ROS_WARN("Please provide a proper id range for sim_vector");
      res.ok = false;
    }
  } else {
    ROS_WARN("RVO Planner not initialised!");
    res.ok = false;
  }
  return true;
}

bool RVOWrapper::getAgentNumObstacleNeighbors(
  rvo_wrapper_msgs::GetAgentNumObstacleNeighbors::Request& req,
  rvo_wrapper_msgs::GetAgentNumObstacleNeighbors::Response& res) {
  res.ok = true;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    res.num_obstacles = planner_->getAgentNumObstacleNeighbors(req.agent_id);
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if ((req.sim_ids.back() >= req.sim_ids.front()) &&
        (req.sim_ids.back() < sim_vect_.size())) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i <= req.sim_ids.back(); ++i) {
        res.num_obstacles = sim_vect_[i]->getAgentNumObstacleNeighbors(req.agent_id);
      }
    } else {
      ROS_WARN("Please provide a proper id range for sim_vector");
      res.ok = false;
    }
  } else {
    ROS_WARN("RVO Planner not initialised!");
    res.ok = false;
  }
  return true;
}

bool RVOWrapper::getAgentObstacleNeighbor(
  rvo_wrapper_msgs::GetAgentObstacleNeighbor::Request& req,
  rvo_wrapper_msgs::GetAgentObstacleNeighbor::Response& res) {
  res.ok = true;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    res.obstacle_vertex = planner_->getAgentObstacleNeighbor(req.agent_id,
                                                             req.agent_obstacle);
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if ((req.sim_ids.back() >= req.sim_ids.front()) &&
        (req.sim_ids.back() < sim_vect_.size())) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i <= req.sim_ids.back(); ++i) {
        res.obstacle_vertex = sim_vect_[i]->getAgentObstacleNeighbor(req.agent_id,
                                                                     req.agent_obstacle);
      }
    } else {
      ROS_WARN("Please provide a proper id range for sim_vector");
      res.ok = false;
    }
  } else {
    ROS_WARN("RVO Planner not initialised!");
    res.ok = false;
  }
  return true;
}

bool RVOWrapper::getAgentPosition(
  rvo_wrapper_msgs::GetAgentPosition::Request& req,
  rvo_wrapper_msgs::GetAgentPosition::Response& res) {
  res.ok = true;
  RVO::Vector2 position;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    position = planner_->getAgentPosition(req.agent_id);
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if ((req.sim_ids.back() >= req.sim_ids.front()) &&
        (req.sim_ids.back() < sim_vect_.size())) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i <= req.sim_ids.back(); ++i) {
        position = sim_vect_[i]->getAgentPosition(req.agent_id);
      }
    } else {
      ROS_WARN("Please provide a proper id range for sim_vector");
      res.ok = false;
    }
  } else {
    ROS_WARN("RVO Planner not initialised!");
    res.ok = false;
  }
  res.position.x = position.x();
  res.position.y = position.y();
  return true;
}

bool RVOWrapper::getAgentPrefVelocity(
  rvo_wrapper_msgs::GetAgentPrefVelocity::Request& req,
  rvo_wrapper_msgs::GetAgentPrefVelocity::Response& res) {
  res.ok = true;
  RVO::Vector2 pref_velocity;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    pref_velocity = planner_->getAgentPrefVelocity(req.agent_id);
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if ((req.sim_ids.back() >= req.sim_ids.front()) &&
        (req.sim_ids.back() < sim_vect_.size())) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i <= req.sim_ids.back(); ++i) {
        pref_velocity = sim_vect_[i]->getAgentPrefVelocity(req.agent_id);
      }
    } else {
      ROS_WARN("Please provide a proper id range for sim_vector");
      res.ok = false;
    }
  } else {
    ROS_WARN("RVO Planner not initialised!");
    res.ok = false;
  }
  res.pref_velocity.x = pref_velocity.x();
  res.pref_velocity.y = pref_velocity.y();
  return true;
}

bool RVOWrapper::getAgentRadius(
  rvo_wrapper_msgs::GetAgentRadius::Request& req,
  rvo_wrapper_msgs::GetAgentRadius::Response& res) {
  res.ok = true;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    res.radius = planner_->getAgentRadius(req.agent_id);
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if ((req.sim_ids.back() >= req.sim_ids.front()) &&
        (req.sim_ids.back() < sim_vect_.size())) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i <= req.sim_ids.back(); ++i) {
        res.radius = sim_vect_[i]->getAgentRadius(req.agent_id);
      }
    } else {
      ROS_WARN("Please provide a proper id range for sim_vector");
      res.ok = false;
    }
  } else {
    ROS_WARN("RVO Planner not initialised!");
    res.ok = false;
  }
  return true;
}

bool RVOWrapper::getAgentTimeHorizon(
  rvo_wrapper_msgs::GetAgentTimeHorizon::Request& req,
  rvo_wrapper_msgs::GetAgentTimeHorizon::Response& res) {
  res.ok = true;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    res.agent_time_horizon = planner_->getAgentTimeHorizon(req.agent_id);
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if ((req.sim_ids.back() >= req.sim_ids.front()) &&
        (req.sim_ids.back() < sim_vect_.size())) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i <= req.sim_ids.back(); ++i) {
        res.agent_time_horizon = sim_vect_[i]->getAgentTimeHorizon(req.agent_id);
      }
    } else {
      ROS_WARN("Please provide a proper id range for sim_vector");
      res.ok = false;
    }
  } else {
    ROS_WARN("RVO Planner not initialised!");
    res.ok = false;
  }
  return true;
}

bool RVOWrapper::getAgentTimeHorizonObst(
  rvo_wrapper_msgs::GetAgentTimeHorizonObst::Request& req,
  rvo_wrapper_msgs::GetAgentTimeHorizonObst::Response& res) {
  res.ok = true;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    res.obst_time_horizon = planner_->getAgentTimeHorizonObst(req.agent_id);
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if ((req.sim_ids.back() >= req.sim_ids.front()) &&
        (req.sim_ids.back() < sim_vect_.size())) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i <= req.sim_ids.back(); ++i) {
        res.obst_time_horizon = sim_vect_[i]->getAgentTimeHorizonObst(req.agent_id);
      }
    } else {
      ROS_WARN("Please provide a proper id range for sim_vector");
      res.ok = false;
    }
  } else {
    ROS_WARN("RVO Planner not initialised!");
    res.ok = false;
  }
  return true;
}

bool RVOWrapper::getAgentVelocity(
  rvo_wrapper_msgs::GetAgentVelocity::Request& req,
  rvo_wrapper_msgs::GetAgentVelocity::Response& res) {
  res.ok = true;
  std::vector<RVO::Vector2> velocity;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    velocity.push_back(planner_->getAgentVelocity(req.agent_id[0]));
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if ((req.sim_ids.back() >= req.sim_ids.front()) &&
        (req.sim_ids.back() < sim_vect_.size())) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i <= req.sim_ids.back(); ++i) {
        velocity.push_back(sim_vect_[i]->getAgentVelocity(req.agent_id[i]));
      }
    } else {
      ROS_WARN("Please provide a proper id range for sim_vector");
      res.ok = false;
    }
  } else {
    ROS_WARN("RVO Planner not initialised!");
    res.ok = false;
  }
  res.velocity.resize(velocity.size());
  for (size_t i = 0; i < velocity.size(); ++i) {
    res.velocity[i].x = velocity[i].x();
    res.velocity[i].y = velocity[i].y();
  }
  return true;
}

bool RVOWrapper::getGlobalTime(
  rvo_wrapper_msgs::GetGlobalTime::Request& req,
  rvo_wrapper_msgs::GetGlobalTime::Response& res) {
  res.ok = true;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    res.global_time = planner_->getGlobalTime();
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if ((req.sim_ids.back() >= req.sim_ids.front()) &&
        (req.sim_ids.back() < sim_vect_.size())) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i <= req.sim_ids.back(); ++i) {
        res.global_time = sim_vect_[i]->getGlobalTime();
      }
    } else {
      ROS_WARN("Please provide a proper id range for sim_vector");
      res.ok = false;
    }
  } else {
    ROS_WARN("RVO Planner not initialised!");
    res.ok = false;
  }
  return true;
}

bool RVOWrapper::getNumAgents(
  rvo_wrapper_msgs::GetNumAgents::Request& req,
  rvo_wrapper_msgs::GetNumAgents::Response& res) {
  res.ok = true;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    res.num_agents = planner_->getNumAgents();
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if ((req.sim_ids.back() >= req.sim_ids.front()) &&
        (req.sim_ids.back() < sim_vect_.size())) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i <= req.sim_ids.back(); ++i) {
        res.num_agents = sim_vect_[i]->getNumAgents();
      }
    } else {
      ROS_WARN("Please provide a proper id range for sim_vector");
      res.ok = false;
    }
  } else {
    ROS_WARN("RVO Planner not initialised!");
    res.ok = false;
  }
  return true;
}

bool RVOWrapper::getTimeStep(
  rvo_wrapper_msgs::GetTimeStep::Request& req,
  rvo_wrapper_msgs::GetTimeStep::Response& res) {
  res.ok = true;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    res.time_step = planner_->getTimeStep();
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if ((req.sim_ids.back() >= req.sim_ids.front()) &&
        (req.sim_ids.back() < sim_vect_.size())) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i <= req.sim_ids.back(); ++i) {
        res.time_step = sim_vect_[i]->getTimeStep();
      }
    } else {
      ROS_WARN("Please provide a proper id range for sim_vector");
      res.ok = false;
    }
  } else {
    ROS_WARN("RVO Planner not initialised!");
    res.ok = false;
  }
  return true;
}

bool RVOWrapper::processObstacles(
  rvo_wrapper_msgs::ProcessObstacles::Request& req,
  rvo_wrapper_msgs::ProcessObstacles::Response& res) {
  res.ok = true;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    planner_->processObstacles();
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if ((req.sim_ids.back() >= req.sim_ids.front()) &&
        (req.sim_ids.back() < sim_vect_.size())) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i <= req.sim_ids.back(); ++i) {
        sim_vect_[i]->processObstacles();
      }
    } else {
      ROS_WARN("Please provide a proper id range for sim_vector");
      res.ok = false;
    }
  } else {
    ROS_WARN("RVO Planner not initialised!");
    res.ok = false;
  }
  return true;
}

bool RVOWrapper::queryVisibility(
  rvo_wrapper_msgs::QueryVisibility::Request& req,
  rvo_wrapper_msgs::QueryVisibility::Response& res) {
  res.ok = true;
  RVO::Vector2 point1(req.point1.x, req.point1.y);
  RVO::Vector2 point2(req.point2.x, req.point2.y);
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    res.visible = planner_->queryVisibility(point1, point2, req.radius);
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if ((req.sim_ids.back() >= req.sim_ids.front()) &&
        (req.sim_ids.back() < sim_vect_.size())) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i <= req.sim_ids.back(); ++i) {
        res.visible = sim_vect_[i]->queryVisibility(point1, point2, req.radius);
      }
    } else {
      ROS_WARN("Please provide a proper id range for sim_vector");
      res.ok = false;
    }
  } else {
    ROS_WARN("RVO Planner not initialised!");
    res.ok = false;
  }
  return true;
}

bool RVOWrapper::setAgentDefaults(
  rvo_wrapper_msgs::SetAgentDefaults::Request& req,
  rvo_wrapper_msgs::SetAgentDefaults::Response& res) {
  res.ok = true;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    planner_->setAgentDefaults(req.defaults.neighbor_dist,
                               req.defaults.max_neighbors,
                               req.defaults.time_horizon_agent,
                               req.defaults.time_horizon_obst,
                               req.defaults.radius,
                               req.defaults.max_speed,
                               req.defaults.max_accel,
                               req.defaults.pref_speed);
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if ((req.sim_ids.back() >= req.sim_ids.front()) &&
        (req.sim_ids.back() < sim_vect_.size())) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i <= req.sim_ids.back(); ++i) {
        sim_vect_[i]->setAgentDefaults(req.defaults.neighbor_dist,
                                       req.defaults.max_neighbors,
                                       req.defaults.time_horizon_agent,
                                       req.defaults.time_horizon_obst,
                                       req.defaults.radius,
                                       req.defaults.max_speed,
                                       req.defaults.max_accel,
                                       req.defaults.pref_speed);
      }
    } else {
      ROS_WARN("Please provide a proper id range for sim_vector");
      res.ok = false;
    }
  } else {
    ROS_WARN("RVO Planner not initialised!");
    res.ok = false;
  }
  return true;
}

bool RVOWrapper::setAgentGoals(
  rvo_wrapper_msgs::SetAgentGoals::Request& req,
  rvo_wrapper_msgs::SetAgentGoals::Response& res) {
  res.ok = true;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    uint32_t num_agents = planner_->getNumAgents();
    for (uint32_t i = 0; i < num_agents; ++i) {
      planner_goals_[i] = RVO::Vector2(req.sim[0].agent[i].x,
                                       req.sim[0].agent[i].y);
    }
  } else if (req.sim_ids.size() == 1) { // If specific simulation
    if (req.sim_ids[0] < sim_vect_.size()) { // If good sim id
      uint32_t num_agents = sim_vect_[req.sim_ids[0]]->getNumAgents();
      for (uint32_t i = 0; i < num_agents; ++i) { // Cycle through sim agents
        sim_vect_goals_[req.sim_ids[0]][i] = RVO::Vector2(req.sim[0].agent[i].x,
                                                          req.sim[0].agent[i].y);
      }
    } else {
      ROS_WARN("Please provide a proper sim id within range");
      res.ok = false;
    }
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if ((req.sim_ids.back() >= req.sim_ids.front()) &&
        (req.sim_ids.back() < sim_vect_.size())) { // If good sim id range
      for (uint32_t j = req.sim_ids.front(); j <= req.sim_ids.back(); ++j) {
        uint32_t num_agents = sim_vect_[req.sim_ids[j]]->getNumAgents();
        size_t sim_no = j - req.sim_ids.front();
        for (uint32_t i = 0; i < num_agents; ++i) { // Cycle through sim agents
          sim_vect_goals_[j][i] = RVO::Vector2(req.sim[sim_no].agent[i].x,
                                               req.sim[sim_no].agent[i].y);
          ROS_INFO_STREAM("SimID: " << j << " No: " << sim_no << " A: " << i <<
                          " G: " << req.sim[sim_no].agent[i].x << ", " <<
                          req.sim[sim_no].agent[i].y);
        }
      }
    } else {
      ROS_WARN("Please provide a proper id range for sim_vector");
      res.ok = false;
    }
  } else {
    ROS_WARN("RVO Planner not initialised!");
    res.ok = false;
  }
  return true;
}

bool RVOWrapper::setAgentMaxNeighbors(
  rvo_wrapper_msgs::SetAgentMaxNeighbors::Request& req,
  rvo_wrapper_msgs::SetAgentMaxNeighbors::Response& res) {
  res.ok = true;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    planner_->setAgentMaxNeighbors(req.agent_id, req.max_neighbors);
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if ((req.sim_ids.back() >= req.sim_ids.front()) &&
        (req.sim_ids.back() < sim_vect_.size())) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i <= req.sim_ids.back(); ++i) {
        sim_vect_[i]->setAgentMaxNeighbors(req.agent_id, req.max_neighbors);
      }
    } else {
      ROS_WARN("Please provide a proper id range for sim_vector");
      res.ok = false;
    }
  } else {
    ROS_WARN("RVO Planner not initialised!");
    res.ok = false;
  }
  return true;
}

bool RVOWrapper::setAgentMaxSpeed(
  rvo_wrapper_msgs::SetAgentMaxSpeed::Request& req,
  rvo_wrapper_msgs::SetAgentMaxSpeed::Response& res) {
  res.ok = true;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    planner_->setAgentMaxSpeed(req.agent_id, req.max_speed);
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if ((req.sim_ids.back() >= req.sim_ids.front()) &&
        (req.sim_ids.back() < sim_vect_.size())) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i <= req.sim_ids.back(); ++i) {
        sim_vect_[i]->setAgentMaxSpeed(req.agent_id, req.max_speed);
      }
    } else {
      ROS_WARN("Please provide a proper id range for sim_vector");
      res.ok = false;
    }
  } else {
    ROS_WARN("RVO Planner not initialised!");
    res.ok = false;
  }
  return true;
}

bool RVOWrapper::setAgentNeighborDist(
  rvo_wrapper_msgs::SetAgentNeighborDist::Request& req,
  rvo_wrapper_msgs::SetAgentNeighborDist::Response& res) {
  res.ok = true;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    planner_->setAgentNeighborDist(req.agent_id, req.neighbor_dist);
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if ((req.sim_ids.back() >= req.sim_ids.front()) &&
        (req.sim_ids.back() < sim_vect_.size())) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i <= req.sim_ids.back(); ++i) {
        sim_vect_[i]->setAgentNeighborDist(req.agent_id, req.neighbor_dist);
      }
    } else {
      ROS_WARN("Please provide a proper id range for sim_vector");
      res.ok = false;
    }
  } else {
    ROS_WARN("RVO Planner not initialised!");
    res.ok = false;
  }
  return true;
}

bool RVOWrapper::setAgentPosition(
  rvo_wrapper_msgs::SetAgentPosition::Request& req,
  rvo_wrapper_msgs::SetAgentPosition::Response& res) {
  res.ok = true;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    planner_->setAgentPosition(req.agent_id, RVO::Vector2(req.position.x,
                                                          req.position.y));
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if ((req.sim_ids.back() >= req.sim_ids.front()) &&
        (req.sim_ids.back() < sim_vect_.size())) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i <= req.sim_ids.back(); ++i) {
        sim_vect_[i]->setAgentPosition(req.agent_id, RVO::Vector2(req.position.x,
                                                                  req.position.y));
      }
    } else {
      ROS_WARN("Please provide a proper id range for sim_vector");
      res.ok = false;
    }
  } else {
    ROS_WARN("RVO Planner not initialised!");
    res.ok = false;
  }
  return true;
}

bool RVOWrapper::setAgentPrefVelocity(
  rvo_wrapper_msgs::SetAgentPrefVelocity::Request& req,
  rvo_wrapper_msgs::SetAgentPrefVelocity::Response& res) {
  res.ok = true;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    planner_->setAgentPrefVelocity(req.agent_id, RVO::Vector2(req.pref_velocity.x,
                                                              req.pref_velocity.y));
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if ((req.sim_ids.back() >= req.sim_ids.front()) &&
        (req.sim_ids.back() < sim_vect_.size())) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i <= req.sim_ids.back(); ++i) {
        sim_vect_[i]->setAgentPrefVelocity(req.agent_id,
                                           RVO::Vector2(req.pref_velocity.x,
                                                        req.pref_velocity.y));
      }
    } else {
      ROS_WARN("Please provide a proper id range for sim_vector");
      res.ok = false;
    }
  } else {
    ROS_WARN("RVO Planner not initialised!");
    res.ok = false;
  }
  return true;
}

bool RVOWrapper::setAgentRadius(
  rvo_wrapper_msgs::SetAgentRadius::Request& req,
  rvo_wrapper_msgs::SetAgentRadius::Response& res) {
  res.ok = true;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    planner_->setAgentRadius(req.agent_id, req.radius);
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if ((req.sim_ids.back() >= req.sim_ids.front()) &&
        (req.sim_ids.back() < sim_vect_.size())) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i <= req.sim_ids.back(); ++i) {
        sim_vect_[i]->setAgentRadius(req.agent_id, req.radius);
      }
    } else {
      ROS_WARN("Please provide a proper id range for sim_vector");
      res.ok = false;
    }
  } else {
    ROS_WARN("RVO Planner not initialised!");
    res.ok = false;
  }
  return true;
}

bool RVOWrapper::setAgentTimeHorizon(
  rvo_wrapper_msgs::SetAgentTimeHorizon::Request& req,
  rvo_wrapper_msgs::SetAgentTimeHorizon::Response& res) {
  res.ok = true;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    planner_->setAgentTimeHorizon(req.agent_id, req.agent_time_horizon);
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if ((req.sim_ids.back() >= req.sim_ids.front()) &&
        (req.sim_ids.back() < sim_vect_.size())) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i <= req.sim_ids.back(); ++i) {
        sim_vect_[i]->setAgentTimeHorizon(req.agent_id, req.agent_time_horizon);
      }
    } else {
      ROS_WARN("Please provide a proper id range for sim_vector");
      res.ok = false;
    }
  } else {
    ROS_WARN("RVO Planner not initialised!");
    res.ok = false;
  }
  return true;
}

bool RVOWrapper::setAgentTimeHorizonObst(
  rvo_wrapper_msgs::SetAgentTimeHorizonObst::Request& req,
  rvo_wrapper_msgs::SetAgentTimeHorizonObst::Response& res) {
  res.ok = true;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    planner_->setAgentTimeHorizonObst(req.agent_id, req.obst_time_horizon);
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if ((req.sim_ids.back() >= req.sim_ids.front()) &&
        (req.sim_ids.back() < sim_vect_.size())) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i <= req.sim_ids.back(); ++i) {
        sim_vect_[i]->setAgentTimeHorizonObst(req.agent_id, req.obst_time_horizon);
      }
    } else {
      ROS_WARN("Please provide a proper id range for sim_vector");
      res.ok = false;
    }
  } else {
    ROS_WARN("RVO Planner not initialised!");
    res.ok = false;
  }
  return true;
}

bool RVOWrapper::setAgentVelocity(
  rvo_wrapper_msgs::SetAgentVelocity::Request& req,
  rvo_wrapper_msgs::SetAgentVelocity::Response& res) {
  res.ok = true;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    planner_->setAgentVelocity(req.agent_id, RVO::Vector2(req.velocity.x,
                                                          req.velocity.y));
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if ((req.sim_ids.back() >= req.sim_ids.front()) &&
        (req.sim_ids.back() < sim_vect_.size())) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i <= req.sim_ids.back(); ++i) {
        sim_vect_[i]->setAgentVelocity(req.agent_id, RVO::Vector2(req.velocity.x,
                                                                  req.velocity.y));
      }
    } else {
      ROS_WARN("Please provide a proper id range for sim_vector");
      res.ok = false;
    }
  } else {
    ROS_WARN("RVO Planner not initialised!");
    res.ok = false;
  }
  return true;
}

bool RVOWrapper::setTimeStep(
  rvo_wrapper_msgs::SetTimeStep::Request& req,
  rvo_wrapper_msgs::SetTimeStep::Response& res) {
  res.ok = true;
  if (req.sim_ids.size() == 0 && planner_init_) { // If Planner
    planner_->setTimeStep(req.time_step);
  } else if (req.sim_ids.size() > 0) { // If Sim Vector
    if ((req.sim_ids.back() >= req.sim_ids.front()) &&
        (req.sim_ids.back() < sim_vect_.size())) { // If good sim id range
      for (uint32_t i = req.sim_ids.front(); i <= req.sim_ids.back(); ++i) {
        sim_vect_[i]->setTimeStep(req.time_step);
      }
    } else {
      ROS_WARN("Please provide a proper id range for sim_vector");
      res.ok = false;
    }
  } else {
    ROS_WARN("RVO Planner not initialised!");
    res.ok = false;
  }
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
