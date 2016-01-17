/**
 * @file      environment.cpp
 * @brief     Main environment for Youbot-ROS-icrin interface
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-08-01
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#include "environment/environment.hpp"

Environment::Environment(ros::NodeHandle* nh) {
  nh_ = nh;
  robot_name_ = ros::this_node::getNamespace();
  robot_name_.erase(0, 1);  // Remove 1 forward slash from robot_name
  this->loadParams();
  this->init();
  this->rosSetup();
}

Environment::~Environment() {
  ros::param::del("environment");
}

void Environment::init() {
  planning_ = false;
  arrived_ = false;
  modelling_ = true;
  goal_id_ = 0;
  collision_ = false;
  // ROS
  zero_vect_.x = 0.0f;
  zero_vect_.y = 0.0f;
  zero_vect_.z = 0.0f;
  robot_curr_pose_.x = 0.0f;
  robot_curr_pose_.y = 0.0f;
  robot_curr_pose_.theta = 0.0f;
  robot_target_goal_.x = 0.0f;
  robot_target_goal_.y = 0.0f;
  robot_target_goal_.theta = 0.0f;
  robot_cmd_velocity_.linear = zero_vect_;
  robot_cmd_velocity_.angular = zero_vect_;

  bool active = false;
  for (size_t i = 0; i < robots_.size(); ++i) {
    if (("/" + robots_[i]).compare(robot_name_) == 0) {
      robot_id_ = i;
      active = true;
      break;
    }
  }
  if (!active) {
    ROS_ERROR("ERROR: Robot launched but not meant to be active!");
    // ros::shutdown();
  }
}

void Environment::rosSetup() {
  curr_pose_pub_ = nh_->advertise<geometry_msgs::Pose2D>
                   ("curr_pose", 1, true);
  target_goal_pub_ = nh_->advertise<geometry_msgs::Pose2D>
                     ("target_goal", 1, true);
  robot_cmd_velocity_pub_ = nh_->advertise<geometry_msgs::Twist>
                            (robot_name_ + "/cmd_vel", 1, true);
  environment_data_pub_ = nh_->advertise<environment_msgs::EnvironmentData>
                          ("data", 1, true);
  planning_pub_ = nh_->advertise<std_msgs::Bool>
                  ("planning", 1);
  // Model
  model_pub_ = nh_->advertise<model_msgs::ModelHypotheses>
               (robot_name_ + "/model/hypotheses", 1);
  // Planner
  ros::service::waitForService(robot_name_ + "/planner/setup_new_planner");
  setup_new_planner_ = nh_->serviceClient<planner_msgs::SetupNewPlanner>
                       (robot_name_ + "/planner/setup_new_planner", true);
  // Experiment
  goals_sub_ = nh_->subscribe("/experiment/goals", 1000,
                              &Environment::goalsCB, this);
  plans_sub_ = nh_->subscribe("/experiment/plans", 1000,
                              &Environment::plansCB, this);
  // Environment
  planning_sub_ = nh_->subscribe(robot_name_ + "/environment/planning", 1000,
                                 &Environment::planningCB, this);
  arrived_sub_ = nh_->subscribe(robot_name_ + "/environment/arrived", 1000,
                                &Environment::arrivedCB, this);
  // Youbot
  bumper_kilt_sub_ = nh_->subscribe(robot_name_ + "/bumper_kilt", 1000,
                                    &Environment::bumperKiltCB, this);
  // Tracker
  tracker_data_sub_ = nh_->subscribe("/tracker/data", 1000,
                                     &Environment::trackerDataCB, this);
  // AMCL Wrapper
  amcl_pose_sub_ = nh_->subscribe(robot_name_ + "/amcl_wrapper/curr_pose",
                                  1000, &Environment::amclPoseCB, this);
  // Robot Comms
  comms_data_sub_ = nh_->subscribe(robot_name_ + "/robot_comms/data", 1000,
                                   &Environment::commsDataCB, this);
  // Planner
  planner_cmd_vel_sub_ = nh_->subscribe(robot_name_ + "/planner/cmd_vel", 1000,
                                        &Environment::plannerCmdVelCB, this);
}

void Environment::loadParams() {
  // Experiment
  ros::param::param("/experiment/track_robots", track_robots_, false);
  ros::param::get("/experiment/robots", robots_);
  // Robot specific
  ros::param::param("environment/amcl", amcl_, true);
  ros::param::param("environment/bumper", bumper_, false);
  ros::param::param("environment/rvo_planner", rvo_planner_, true);
}

bool Environment::interrupted_;

void Environment::interrupt(int s) {
  Environment::interrupted_ = true;
}

void Environment::setupEnvironment() {
  planning_ = false;
  this->pubRobotVelocity();
  planner_msgs::SetupNewPlanner new_planner;
  if (rvo_planner_) {
    new_planner.request.planner_type = new_planner.request.RVO_PLANNER;
  } else {
    new_planner.request.planner_type = new_planner.request.ROS_NAVIGATION;
  }
  setup_new_planner_.call(new_planner);
  while (goals_.size() == 0) {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  ROS_INFO("Environment- Goals received");
  robot_target_goal_ = goals_[goal_id_];
  this->setReady(true);
}

void Environment::setReady(bool ready) {
  ros::param::set("environment/ready", ready);
}

void Environment::pubRobotPose() {
  robot_curr_pose_ = robot_amcl_pose_;
  curr_pose_pub_.publish(robot_curr_pose_);
}

void Environment::pubRobotGoal() {
  target_goal_pub_.publish(robot_target_goal_);
}

void Environment::pubRobotVelocity() {
  if (planning_ && !collision_) {
    robot_cmd_velocity_ = planner_cmd_velocity_;
  } else {
    robot_cmd_velocity_.linear = zero_vect_;
    robot_cmd_velocity_.angular = zero_vect_;
  }
  robot_cmd_velocity_pub_.publish(robot_cmd_velocity_);
}

void Environment::pubEnvironmentData() {
  environment_msgs::EnvironmentData env_data;
  uint64_t nrobots = comms_data_.robot_poses.size();
  uint64_t ntrackers = tracker_data_.identity.size();
  // Add other robots info
  if (!track_robots_) {
    env_data.tracker_ids.resize(nrobots, 0);  // If robots are not tracked
    env_data.agent_poses = comms_data_.robot_poses;
    env_data.agent_vels = comms_data_.robot_vels;
  }
  // Add people tracking info
  for (uint64_t i = 0; i < ntrackers; ++i) {
    env_data.tracker_ids.push_back(tracker_data_.identity[i]);
    env_data.agent_poses.push_back(tracker_data_.agent_position[i]);
    env_data.agent_vels.push_back(tracker_data_.agent_avg_velocity[i]);
  }
  agent_no_ = env_data.agent_poses.size() + 1;
  environment_data_pub_.publish(env_data);
  this->pubRobotPose();
  this->pubRobotGoal();
  this->pubRobotVelocity();
}

void Environment::pubPlanning() {
  std_msgs::Bool planning;
  planning.data = planning_;
  planning_pub_.publish(planning);
}

void Environment::pubModelHypotheses() {
  // Temporary Modelling test request
  model_msgs::ModelHypotheses model_hypotheses;
  // ROS_INFO_STREAM("ENV- NAgents: " << agent_no_);
  // for (size_t i = 1; i < agent_no_; ++i) { // i = 0 includes Robot Agent
  //   model_hypotheses.agents.push_back(i);
  // }
  if (agent_no_ > 1) {model_hypotheses.agents.push_back(1);}  // Temp fix
  model_hypotheses.goals = true;
  model_hypotheses.awareness = false;
  model_hypotheses.goal_hypothesis.sampling = false;
  model_hypotheses.goal_hypothesis.goal_sequence = goals_;
  model_pub_.publish(model_hypotheses);
}

void Environment::goalsCB(const experiment_msgs::Goals::ConstPtr& msg) {
  goals_ = msg->goal;
}

void Environment::plansCB(const experiment_msgs::Plans::ConstPtr& msg) {
  curr_plan_ = msg->plan[robot_id_];
}

void Environment::planningCB(const std_msgs::Bool::ConstPtr& msg) {
  bool plan_now = msg->data;
  if (!planning_ && plan_now) {
    ROS_INFO("Environment- Robot %s now planning!", robot_name_.c_str());
  } else if (planning_ && !plan_now) {
    ROS_INFO("Environment- Robot %s stop planning!", robot_name_.c_str());
  }
  planning_ = plan_now;
}

void Environment::arrivedCB(const std_msgs::Bool::ConstPtr& msg) {
  arrived_ = msg->data;
}

void Environment::bumperKiltCB(const std_msgs::Int32MultiArray::ConstPtr& msg) {
  bumper_kilt_ = *msg;  // 8 Directions, clockwise starting at front
  collision_ = false;
  for (uint8_t i = 0; i < bumper_kilt_.data.size(); ++i) {
    if (bumper_kilt_.data[i] > 0) {collision_ = true;}
  }
}

void Environment::trackerDataCB(const tracker_msgs::TrackerData::ConstPtr&
                                msg) {
  tracker_data_ = *msg;
}

void Environment::amclPoseCB(const geometry_msgs::Pose2D::ConstPtr& msg) {
  robot_amcl_pose_ = *msg;
}

void Environment::commsDataCB(const robot_comms_msgs::CommsData::ConstPtr&
                              msg) {
  comms_data_ = *msg;
}

void Environment::plannerCmdVelCB(const geometry_msgs::Twist::ConstPtr& msg) {
  planner_cmd_velocity_ = *msg;
  // this->pubRobotVelocity(); // TODO: Figure out why I was publishing this twice
}

void Environment::checkGoalPlan() {
  if (!arrived_) {
    robot_target_goal_ = goals_[curr_plan_.sequence[goal_id_]];
  } else {
    planning_ = false;
    uint16_t next_goal = goal_id_ + 1;
    if (next_goal < curr_plan_.sequence.size()) {
      ROS_INFO_STREAM("Environment- New goal: " << next_goal);
      robot_target_goal_ = goals_[curr_plan_.sequence[next_goal]];
      goal_id_ = next_goal;
      arrived_ = false;
      planning_ = true;
    } else if (curr_plan_.repeat) {
      ROS_INFO_STREAM("Environment- Restarting goal sequence ");
      goal_id_ = 0;
      robot_target_goal_ = goals_[curr_plan_.sequence[goal_id_]];
      arrived_ = false;
      planning_ = true;
    }
    this->pubPlanning();
  }
}

void Environment::modelStep() {
  if (modelling_) {
    // Check what we want to model
    // Publish hypotheses request
    this->pubModelHypotheses();
  }
}

void Environment::stopRobot() {
  ROS_INFO("Stop!");
  planning_ = false;
  this->pubPlanning();
  this->pubRobotVelocity();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "environment");
  ros::NodeHandle nh("environment");
  Environment environment(&nh);

  std::signal(SIGINT, Environment::interrupt);

  ros::Rate r(10);
  environment.setupEnvironment();

  while (ros::ok() && !Environment::isInterrupted()) {
    ros::spinOnce();
    environment.checkGoalPlan();
    environment.pubEnvironmentData();
    environment.modelStep();
    r.sleep();
  }

  environment.stopRobot();

  ros::shutdown();

  return 0;
}
