/**
 * @file      rvo_wrapper.hpp
 * @brief     Add file description...
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-06-29
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#ifndef RVO_WRAPPER_HPP
#define RVO_WRAPPER_HPP

#include <ros/ros.h>

#include <rvo_wrapper/RVO.h>
#include <rvo_wrapper/Definitions.h>
#include <rvo_wrapper/Vector2.h>

#include <std_srvs/Empty.h>

#include <rvo_wrapper_msgs/AddAgent.h>
#include <rvo_wrapper_msgs/AddObstacle.h>
#include <rvo_wrapper_msgs/CalcPrefVelocities.h>
#include <rvo_wrapper_msgs/CheckReachedGoal.h>
#include <rvo_wrapper_msgs/CreateRVOSim.h>
#include <rvo_wrapper_msgs/DeleteSimVector.h>
#include <rvo_wrapper_msgs/DoStep.h>
#include <rvo_wrapper_msgs/GetAgentAgentNeighbor.h>
#include <rvo_wrapper_msgs/GetAgentMaxNeighbors.h>
#include <rvo_wrapper_msgs/GetAgentMaxSpeed.h>
#include <rvo_wrapper_msgs/GetAgentNeighborDist.h>
#include <rvo_wrapper_msgs/GetAgentNumAgentNeighbors.h>
#include <rvo_wrapper_msgs/GetAgentNumObstacleNeighbors.h>
#include <rvo_wrapper_msgs/GetAgentObstacleNeighbor.h>
#include <rvo_wrapper_msgs/GetAgentPosition.h>
#include <rvo_wrapper_msgs/GetAgentPrefVelocity.h>
#include <rvo_wrapper_msgs/GetAgentRadius.h>
#include <rvo_wrapper_msgs/GetAgentTimeHorizon.h>
#include <rvo_wrapper_msgs/GetAgentTimeHorizonObst.h>
#include <rvo_wrapper_msgs/GetAgentVelocity.h>
#include <rvo_wrapper_msgs/GetGlobalTime.h>
#include <rvo_wrapper_msgs/GetNumAgents.h>
#include <rvo_wrapper_msgs/GetTimeStep.h>
#include <rvo_wrapper_msgs/ProcessObstacles.h>
#include <rvo_wrapper_msgs/QueryVisibility.h>
#include <rvo_wrapper_msgs/SetAgentDefaults.h>
#include <rvo_wrapper_msgs/SetAgentGoals.h>
#include <rvo_wrapper_msgs/SetAgentMaxNeighbors.h>
#include <rvo_wrapper_msgs/SetAgentMaxSpeed.h>
#include <rvo_wrapper_msgs/SetAgentNeighborDist.h>
#include <rvo_wrapper_msgs/SetAgentPosition.h>
#include <rvo_wrapper_msgs/SetAgentPrefVelocity.h>
#include <rvo_wrapper_msgs/SetAgentRadius.h>
#include <rvo_wrapper_msgs/SetAgentTimeHorizon.h>
#include <rvo_wrapper_msgs/SetAgentTimeHorizonObst.h>
#include <rvo_wrapper_msgs/SetAgentVelocity.h>
#include <rvo_wrapper_msgs/SetTimeStep.h>

class RVOWrapper {
 public:
  RVOWrapper(ros::NodeHandle* nh);
  ~RVOWrapper();

  void init();

  void rosSetup();

  bool addAgent(rvo_wrapper_msgs::AddAgent::Request& req,
                rvo_wrapper_msgs::AddAgent::Response& res);

  bool addObstacle(rvo_wrapper_msgs::AddObstacle::Request& req,
                   rvo_wrapper_msgs::AddObstacle::Response& res);

  bool calcPrefVelocities(rvo_wrapper_msgs::CalcPrefVelocities::Request& req,
                          rvo_wrapper_msgs::CalcPrefVelocities::Response& res);

  bool checkReachedGoal(rvo_wrapper_msgs::CheckReachedGoal::Request& req,
                        rvo_wrapper_msgs::CheckReachedGoal::Response& res);

  bool createRVOSim(rvo_wrapper_msgs::CreateRVOSim::Request& req,
                    rvo_wrapper_msgs::CreateRVOSim::Response& res);

  bool deleteSimVector(
    rvo_wrapper_msgs::DeleteSimVector::Request& req,
    rvo_wrapper_msgs::DeleteSimVector::Response& res);

  bool doStep(rvo_wrapper_msgs::DoStep::Request& req,
              rvo_wrapper_msgs::DoStep::Response& res);

  bool getAgentAgentNeighbor(
    rvo_wrapper_msgs::GetAgentAgentNeighbor::Request& req,
    rvo_wrapper_msgs::GetAgentAgentNeighbor::Response& res);

  bool getAgentMaxNeighbors(
    rvo_wrapper_msgs::GetAgentMaxNeighbors::Request& req,
    rvo_wrapper_msgs::GetAgentMaxNeighbors::Response& res);

  bool getAgentMaxSpeed(
    rvo_wrapper_msgs::GetAgentMaxSpeed::Request& req,
    rvo_wrapper_msgs::GetAgentMaxSpeed::Response& res);

  bool getAgentNeighborDist(
    rvo_wrapper_msgs::GetAgentNeighborDist::Request& req,
    rvo_wrapper_msgs::GetAgentNeighborDist::Response& res);

  bool getAgentNumAgentNeighbors(
    rvo_wrapper_msgs::GetAgentNumAgentNeighbors::Request& req,
    rvo_wrapper_msgs::GetAgentNumAgentNeighbors::Response& res);

  bool getAgentNumObstacleNeighbors(
    rvo_wrapper_msgs::GetAgentNumObstacleNeighbors::Request& req,
    rvo_wrapper_msgs::GetAgentNumObstacleNeighbors::Response& res);

  bool getAgentObstacleNeighbor(
    rvo_wrapper_msgs::GetAgentObstacleNeighbor::Request& req,
    rvo_wrapper_msgs::GetAgentObstacleNeighbor::Response& res);

  bool getAgentPosition(
    rvo_wrapper_msgs::GetAgentPosition::Request& req,
    rvo_wrapper_msgs::GetAgentPosition::Response& res);

  bool getAgentPrefVelocity(
    rvo_wrapper_msgs::GetAgentPrefVelocity::Request& req,
    rvo_wrapper_msgs::GetAgentPrefVelocity::Response& res);

  bool getAgentRadius(
    rvo_wrapper_msgs::GetAgentRadius::Request& req,
    rvo_wrapper_msgs::GetAgentRadius::Response& res);

  bool getAgentTimeHorizon(
    rvo_wrapper_msgs::GetAgentTimeHorizon::Request& req,
    rvo_wrapper_msgs::GetAgentTimeHorizon::Response& res);

  bool getAgentTimeHorizonObst(
    rvo_wrapper_msgs::GetAgentTimeHorizonObst::Request& req,
    rvo_wrapper_msgs::GetAgentTimeHorizonObst::Response& res);

  bool getAgentVelocity(
    rvo_wrapper_msgs::GetAgentVelocity::Request& req,
    rvo_wrapper_msgs::GetAgentVelocity::Response& res);

  bool getGlobalTime(
    rvo_wrapper_msgs::GetGlobalTime::Request& req,
    rvo_wrapper_msgs::GetGlobalTime::Response& res);

  bool getNumAgents(
    rvo_wrapper_msgs::GetNumAgents::Request& req,
    rvo_wrapper_msgs::GetNumAgents::Response& res);

  bool getTimeStep(
    rvo_wrapper_msgs::GetTimeStep::Request& req,
    rvo_wrapper_msgs::GetTimeStep::Response& res);

  bool processObstacles(
    rvo_wrapper_msgs::ProcessObstacles::Request& req,
    rvo_wrapper_msgs::ProcessObstacles::Response& res);

  bool queryVisibility(
    rvo_wrapper_msgs::QueryVisibility::Request& req,
    rvo_wrapper_msgs::QueryVisibility::Response& res);

  bool setAgentDefaults(
    rvo_wrapper_msgs::SetAgentDefaults::Request& req,
    rvo_wrapper_msgs::SetAgentDefaults::Response& res);

  bool setAgentGoals(
    rvo_wrapper_msgs::SetAgentGoals::Request& req,
    rvo_wrapper_msgs::SetAgentGoals::Response& res);

  bool setAgentMaxNeighbors(
    rvo_wrapper_msgs::SetAgentMaxNeighbors::Request& req,
    rvo_wrapper_msgs::SetAgentMaxNeighbors::Response& res);

  bool setAgentMaxSpeed(
    rvo_wrapper_msgs::SetAgentMaxSpeed::Request& req,
    rvo_wrapper_msgs::SetAgentMaxSpeed::Response& res);

  bool setAgentNeighborDist(
    rvo_wrapper_msgs::SetAgentNeighborDist::Request& req,
    rvo_wrapper_msgs::SetAgentNeighborDist::Response& res);

  bool setAgentPosition(
    rvo_wrapper_msgs::SetAgentPosition::Request& req,
    rvo_wrapper_msgs::SetAgentPosition::Response& res);

  bool setAgentPrefVelocity(
    rvo_wrapper_msgs::SetAgentPrefVelocity::Request& req,
    rvo_wrapper_msgs::SetAgentPrefVelocity::Response& res);

  bool setAgentRadius(
    rvo_wrapper_msgs::SetAgentRadius::Request& req,
    rvo_wrapper_msgs::SetAgentRadius::Response& res);

  bool setAgentTimeHorizon(
    rvo_wrapper_msgs::SetAgentTimeHorizon::Request& req,
    rvo_wrapper_msgs::SetAgentTimeHorizon::Response& res);

  bool setAgentTimeHorizonObst(
    rvo_wrapper_msgs::SetAgentTimeHorizonObst::Request& req,
    rvo_wrapper_msgs::SetAgentTimeHorizonObst::Response& res);

  bool setAgentVelocity(
    rvo_wrapper_msgs::SetAgentVelocity::Request& req,
    rvo_wrapper_msgs::SetAgentVelocity::Response& res);

  bool setTimeStep(
    rvo_wrapper_msgs::SetTimeStep::Request& req,
    rvo_wrapper_msgs::SetTimeStep::Response& res);

 private:
  // ROS
  ros::NodeHandle* nh_;
  ros::ServiceServer srv_add_agent_;
  ros::ServiceServer srv_add_osbtacle_;
  ros::ServiceServer srv_calc_pref_velocities_;
  ros::ServiceServer srv_check_reached_goal_;
  ros::ServiceServer srv_create_rvosim_;
  ros::ServiceServer srv_delete_sim_vector_;
  ros::ServiceServer srv_do_step_;
  ros::ServiceServer srv_get_agent_agent_neighbor_;
  ros::ServiceServer srv_get_agent_max_neighbors_;
  ros::ServiceServer srv_get_agent_max_speed_;
  ros::ServiceServer srv_get_agent_neighbor_dist_;
  ros::ServiceServer srv_get_agent_num_agent_neighbors_;
  ros::ServiceServer srv_get_agent_num_obstacle_neighbors_;
  ros::ServiceServer srv_get_agent_obstacle_neighbor_;
  ros::ServiceServer srv_get_agent_position_;
  ros::ServiceServer srv_get_agent_pref_velocity_;
  ros::ServiceServer srv_get_agent_radius_;
  ros::ServiceServer srv_get_agent_time_horizon_;
  ros::ServiceServer srv_get_agent_time_horizon_obst_;
  ros::ServiceServer srv_get_agent_velocity_;
  ros::ServiceServer srv_get_global_time_;
  ros::ServiceServer srv_get_num_agents_;
  ros::ServiceServer srv_get_time_step_;
  ros::ServiceServer srv_process_obstacles_;
  ros::ServiceServer srv_query_visibility_;
  ros::ServiceServer srv_set_agent_defaults_;
  ros::ServiceServer srv_set_agent_goals_;
  ros::ServiceServer srv_set_agent_max_neighbors_;
  ros::ServiceServer srv_set_agent_max_speed_;
  ros::ServiceServer srv_set_agent_neighbor_dist_;
  ros::ServiceServer srv_set_agent_position_;
  ros::ServiceServer srv_set_agent_pref_velocity_;
  ros::ServiceServer srv_set_agent_radius_;
  ros::ServiceServer srv_set_agent_time_horizon_;
  ros::ServiceServer srv_set_agent_time_horizon_obst_;
  ros::ServiceServer srv_set_agent_velocity_;
  ros::ServiceServer srv_set_time_step_;

  // Class pointers
  RVO::RVOSimulator* planner_;
  std::vector<RVO::Vector2> planner_goals_;
  std::vector<RVO::RVOSimulator*> sim_vect_;
  std::vector< std::vector<RVO::Vector2> > sim_vect_goals_;

  // Variables/Flags
  bool planner_init_;

};

#endif /* RVO_WRAPPER_HPP */
