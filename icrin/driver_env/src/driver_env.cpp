/**
 * @file      driver_env.cpp
 * @brief     Main driver_env for Youbot-ROS-icrin interface
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>, Derek Phillips <djp42@stanford.edu>
 * @date      2015-08-01
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#include "driver_env/driver_env.hpp"

Driver_env::Driver_env(ros::NodeHandle* nh) {
  nh_ = nh;
  robot_name_ = ros::this_node::getNamespace();
  robot_name_.erase(0, 1);  // Remove 1 forward slash from robot_name
  this->loadParams();
  this->init();
  this->rosSetup();
}

Driver_env::Driver_env() {
  ros::param::del("driver_env");
}

void Driver_env::init() {
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

void Driver_env::rosSetup() {
  curr_pose_pub_ = nh_->advertise<geometry_msgs::Pose2D>
                   ("curr_pose", 1, true);
  target_goal_pub_ = nh_->advertise<geometry_msgs::Pose2D>
                     ("target_goal", 1, true);
  robot_cmd_velocity_pub_ = nh_->advertise<geometry_msgs::Twist>
                            (robot_name_ + "/cmd_vel", 1, true);
  driver_env_data_pub_ = nh_->advertise<driver_env_msgs::driver_envData>
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
                              &Driver_env::goalsCB, this);
  plans_sub_ = nh_->subscribe("/experiment/plans", 1000,
                              &Driver_env::plansCB, this);
  // driver_env
  planning_sub_ = nh_->subscribe(robot_name_ + "/driver_env/planning", 1000,
                                 &Driver_env::planningCB, this);
  arrived_sub_ = nh_->subscribe(robot_name_ + "/driver_env/arrived", 1000,
                                &Driver_env::arrivedCB, this);
  // Youbot
  bumper_kilt_sub_ = nh_->subscribe(robot_name_ + "/bumper_kilt", 1000,
                                    &Driver_env::bumperKiltCB, this);
  // Tracker
  tracker_data_sub_ = nh_->subscribe("/tracker/data", 1000,
                                     &Driver_env::trackerDataCB, this);
  // AMCL Wrapper
  amcl_pose_sub_ = nh_->subscribe(robot_name_ + "/amcl_wrapper/curr_pose",
                                  1000, &Driver_env::amclPoseCB, this);
  // Robot Comms
  comms_data_sub_ = nh_->subscribe(robot_name_ + "/robot_comms/data", 1000,
                                   &Driver_env::commsDataCB, this);
  // Planner
  planner_cmd_vel_sub_ = nh_->subscribe(robot_name_ + "/planner/cmd_vel", 1000,
                                        &Driver_env::plannerCmdVelCB, this);
}

void Driver_env::loadParams() {
  // Experiment
  ros::param::param("/experiment/track_robots", track_robots_, false);
  ros::param::get("/experiment/robots", robots_);
  // Robot specific
  ros::param::param("driver_env/amcl", amcl_, true);
  ros::param::param("driver_env/bumper", bumper_, false);
  ros::param::param("driver_env/rvo_planner", rvo_planner_, true);
  // Modelling
  float min_x, max_x, min_y, max_y;
  ros::param::param("/experiment/sampling", sampling_, false);
  ros::param::param("/experiment/sample_min/x", min_x, 0.0f);
  ros::param::param("/experiment/sample_max/x", max_x, 1.0f);
  ros::param::param("/experiment/sample_min/y", min_y, 0.0f);
  ros::param::param("/experiment/sample_max/y", max_y, 1.0f);
  ros::param::param("/experiment/sample_res", sample_res_, 0.1f);
  sample_min_.x = min_x;
  sample_min_.y = min_y;
  sample_max_.x = max_x;
  sample_max_.y = max_y;
}

bool Driver_env::interrupted_;

void Driver_env::interrupt(int s) {
  Driver_env::interrupted_ = true;
}

void Driver_env::setupdriver_env() {
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
  ROS_INFO("driver_env- Goals received");
  robot_target_goal_ = goals_[goal_id_];
  this->setReady(true);
}

void Driver_env::setReady(bool ready) {
  ros::param::set("driver_env/ready", ready);
}

void Driver_env::pubRobotPose() {
  robot_curr_pose_ = robot_amcl_pose_;
  curr_pose_pub_.publish(robot_curr_pose_);
}

void Driver_env::pubRobotGoal() {
  target_goal_pub_.publish(robot_target_goal_);
}

void Driver_env::pubRobotVelocity() {
  if (planning_ && !collision_) {
    robot_cmd_velocity_ = planner_cmd_velocity_;
  } else {
    robot_cmd_velocity_.linear = zero_vect_;
    robot_cmd_velocity_.angular = zero_vect_;
  }
  robot_cmd_velocity_pub_.publish(robot_cmd_velocity_);
}

void Driver_env::pubdriver_envData() {
  driver_env_msgs::driver_envData env_data;
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
  driver_env_data_pub_.publish(env_data);
  this->pubRobotPose();
  this->pubRobotGoal();
  this->pubRobotVelocity();
}

void Driver_env::pubPlanning() {
  std_msgs::Bool planning;
  planning.data = planning_;
  planning_pub_.publish(planning);
}

void Driver_env::pubModelHypotheses() {
  // Temporary Modelling test request
  model_msgs::ModelHypotheses model_hypotheses;
  // ROS_INFO_STREAM("ENV- NAgents: " << agent_no_);
  // for (size_t i = 1; i < agent_no_; ++i) { // i = 0 includes Robot Agent
  //   model_hypotheses.agents.push_back(i);
  // }
  if (agent_no_ > 1) {model_hypotheses.agents.push_back(1);}  // Temp fix
  model_hypotheses.goals = true;
  model_hypotheses.awareness = false;
  // FOR GOAL SEQUENCE
  model_hypotheses.goal_hypothesis.goal_sequence = goals_;
  // FOR GOAL SAMPLING
  model_hypotheses.goal_hypothesis.sampling = sampling_;
  model_hypotheses.goal_hypothesis.sample_space.push_back(sample_min_);
  model_hypotheses.goal_hypothesis.sample_space.push_back(sample_max_);
  model_hypotheses.goal_hypothesis.sample_resolution = sample_res_;
  model_pub_.publish(model_hypotheses);
}

void Driver_env::goalsCB(const experiment_msgs::Goals::ConstPtr& msg) {
  goals_ = msg->goal;
}

void Driver_env::plansCB(const experiment_msgs::Plans::ConstPtr& msg) {
  curr_plan_ = msg->plan[robot_id_];
}

void Driver_env::planningCB(const std_msgs::Bool::ConstPtr& msg) {
  bool plan_now = msg->data;
  if (!planning_ && plan_now) {
    ROS_INFO("driver_env- Robot %s now planning!", robot_name_.c_str());
  } else if (planning_ && !plan_now) {
    ROS_INFO("driver_env- Robot %s stop planning!", robot_name_.c_str());
  }
  planning_ = plan_now;
}

void Driver_env::arrivedCB(const std_msgs::Bool::ConstPtr& msg) {
  arrived_ = msg->data;
}

void Driver_env::bumperKiltCB(const std_msgs::Int32MultiArray::ConstPtr& msg) {
  bumper_kilt_ = *msg;  // 8 Directions, clockwise starting at front
  collision_ = false;
  for (uint8_t i = 0; i < bumper_kilt_.data.size(); ++i) {
    if (bumper_kilt_.data[i] > 0) {collision_ = true;}
  }
}

void Driver_env::trackerDataCB(const tracker_msgs::TrackerData::ConstPtr&
                                msg) {
  tracker_data_ = *msg;
}

void Driver_env::amclPoseCB(const geometry_msgs::Pose2D::ConstPtr& msg) {
  robot_amcl_pose_ = *msg;
}

void Driver_env::commsDataCB(const robot_comms_msgs::CommsData::ConstPtr&
                              msg) {
  comms_data_ = *msg;
}

void Driver_env::plannerCmdVelCB(const geometry_msgs::Twist::ConstPtr& msg) {
  planner_cmd_velocity_ = *msg;
  // this->pubRobotVelocity(); // TODO: Figure out why I was publishing this twice
}

void Driver_env::checkGoalPlan() {
  if (!arrived_) {
    robot_target_goal_ = goals_[curr_plan_.sequence[goal_id_]];
  } else {
    planning_ = false;
    uint16_t next_goal = goal_id_ + 1;
    if (next_goal < curr_plan_.sequence.size()) {
      ROS_INFO_STREAM("driver_env- New goal: " << next_goal);
      robot_target_goal_ = goals_[curr_plan_.sequence[next_goal]];
      goal_id_ = next_goal;
      arrived_ = false;
      planning_ = true;
    } else if (curr_plan_.repeat) {
      ROS_INFO_STREAM("driver_env- Restarting goal sequence ");
      goal_id_ = 0;
      robot_target_goal_ = goals_[curr_plan_.sequence[goal_id_]];
      arrived_ = false;
      planning_ = true;
    }
    this->pubPlanning();
  }
}

void Driver_env::modelStep() {
  if (modelling_) {
    // Check what we want to model
    // Publish hypotheses request
    this->pubModelHypotheses();
  }
}

void Driver_env::stopRobot() {
  ROS_INFO("Stop!");
  planning_ = false;
  this->pubPlanning();
  this->pubRobotVelocity();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "driver_env");
  ros::NodeHandle nh("driver_env");
  driver_env driver_env(&nh);

  std::signal(SIGINT, Driver_env::interrupt);

  ros::Rate r(10);
  driver_env.setupdriver_env();

  while (ros::ok() && !Driver_env::isInterrupted()) {
    ros::spinOnce();
    driver_env.checkGoalPlan();
    driver_env.pubdriver_envData();
    driver_env.modelStep();
    r.sleep();
  }

  driver_env.stopRobot();

  ros::shutdown();

  return 0;
}
