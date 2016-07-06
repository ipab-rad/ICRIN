/**
 * @file      driver_env.cpp
 * @brief     Driver Environment processes driving data set for ICRIN
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-08-01
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#include "driver_env/driver_env.hpp"

DriverEnv::DriverEnv(ros::NodeHandle* nh) {
  nh_ = nh;
  this->loadParams();
  this->init();
  this->rosSetup();
}

DriverEnv::~DriverEnv() {
  ros::param::del("driver_env");
}

void DriverEnv::loadParams() {
}

void DriverEnv::init() {
    //Set up for lankershim (8:30), yes this is very crude, sorry... (djp)
  geometry_msgs::Pose2D goal1;
  goal1.x = -6.419475524475522;
  goal1.y = 60.04413286713285;
  geometry_msgs::Pose2D goal2;
  goal2.x = -17.04736274509804;
  goal2.y = 64.07240196078426;
  geometry_msgs::Pose2D goal3;
  goal3.x = -27.503154320987658;
  goal3.y = 58.362308641975304;
  geometry_msgs::Pose2D goal4;
  goal4.x = 84.29676056338029;
  goal4.y = 409.7671408450705;
  geometry_msgs::Pose2D goal5;
  goal5.x = 86.3219037037037;
  goal5.y = 397.3591851851852;
  geometry_msgs::Pose2D goal6;
  goal6.x = 89.21276250000004;
  goal6.y = 388.82471250000015;
  geometry_msgs::Pose2D goal7;
  goal7.x = 60.66331034482761;
  goal7.y = 710.1502758620687;
  geometry_msgs::Pose2D goal8;
  goal8.x = 57.33545;
  goal8.y = 1075.7973500000003;
  geometry_msgs::Pose2D goal9;
  goal9.x = 60.9972;
  goal9.y = 1069.9632;
  geometry_msgs::Pose2D goal10;
  goal10.x = 46.304600;
  goal10.y = 1270.5684;
  geometry_msgs::Pose2D goal11;
  goal11.x = 94.08247619047619;
  goal11.y = 1541.9685714285715;
  geometry_msgs::Pose2D goal12;
  goal12.x = 95.98047368421048;
  goal12.y = 1535.4778684210523;
  geometry_msgs::Pose2D goal13;
  goal13.x = 45.281962962962965;
  goal13.y = 1613.079240740741;
  geometry_msgs::Pose2D goal14;
  goal14.x = 12.041342105263158;
  goal14.y = 1622.2986052631584;
  geometry_msgs::Pose2D goal15;
  goal15.x = 21.460899999999988;
  goal15.y = 1616.7741833333332;
  geometry_msgs::Pose2D goal16;
  goal16.x = 33.54720202020202;
  goal16.y = 1616.1990303030302;
  geometry_msgs::Pose2D goal17;
  goal17.x = -75.90733333333333;
  goal17.y = 1563.4226666666666;
  geometry_msgs::Pose2D goal18;
  goal18.x = -93.78888888888888;
  goal18.y = 1054.7534444444448;
  geometry_msgs::Pose2D goal19;
  goal19.x = -95.85615;
  goal19.y = 1061.22355;
  geometry_msgs::Pose2D goal20;
  goal20.x = -84.2815;
  goal20.y = 426.74291666666664;
  geometry_msgs::Pose2D goal21;
  goal21.x = -90.86164285714287;
  goal21.y = 425.9830714285714;
  geometry_msgs::Pose2D goal22;
  goal22.x = -92.14173529411764;
  goal22.y = 438.2764411764706;
  goals_.push_back(goal1);
  goals_.push_back(goal2);
  goals_.push_back(goal3);
  goals_.push_back(goal4);
  goals_.push_back(goal5);
  goals_.push_back(goal6);
  goals_.push_back(goal7);
  goals_.push_back(goal8);
  goals_.push_back(goal9);
  goals_.push_back(goal10);
  goals_.push_back(goal11);
  goals_.push_back(goal12);
  goals_.push_back(goal13);
  goals_.push_back(goal14);
  goals_.push_back(goal15);
  goals_.push_back(goal16);
  goals_.push_back(goal17);
  goals_.push_back(goal18);
  goals_.push_back(goal19);
  goals_.push_back(goal20);
  goals_.push_back(goal21);
  goals_.push_back(goal22);
}

void DriverEnv::rosSetup() {
  environment_data_pub_ = nh_->advertise<environment_msgs::EnvironmentData>
                          ("data", 1, true);
  model_pub_ = nh_->advertise<model_msgs::ModelHypotheses>
               ("/model/hypotheses", 1);

  car_data_sub_ = nh_->subscribe("/driver_env_msgs/car_data", 1000,
                                 &DriverEnv::carDataCB, this);
  model_sub_ = nh_->subscribe("/model/inference", 1000,
                              &DriverEnv::carDataCB, this);
}

void DriverEnv::carDataCB(const driver_env_msgs::Cars::ConstPtr& msg) {
  car_data_ = *msg;
  this->runModel();
}

void DriverEnv::runModel() {
  this->pubEnvData();
  this->pubHypotheses();
}

void DriverEnv::pubEnvData() {
  environment_msgs::EnvironmentData env_data;
  uint64_t ncars = car_data_.cars.size();
  for (uint64_t i = 0; i < ncars; ++i) {
    env_data.tracker_ids.push_back(car_data_.cars[i].card_id);
    geometry_msgs::Pose2D pose;
    pose.x = car_data_.cars[i].pose.position.x; 
    pose.y = car_data_.cars[i].pose.position.y;
    pose.theta = car_data_.cars[i].yaw; 
    env_data.agent_poses.push_back(pose);
    env_data.agent_vels.push_back(car_data_.cars[i].vel);
  }
  environment_data_pub_.publish(env_data);
  agent_ids_ = env_data.tracker_ids;
}

void DriverEnv::pubHypotheses() {
  model_msgs::ModelHypotheses model_hypotheses;
    //if (agent_no_ > 1) {model_hypotheses.agents.push_back(1);}  // Temp fix
  model_hypotheses.goals = true;
  model_hypotheses.awareness = false;
  uint64_t ncars = car_data_.cars.size();
  for (uint64_t i = 0; i < ncars; ++i) {
    model_hypotheses.agents.push_back(agent_ids_[i]);
  }
  model_hypotheses.goal_hypothesis.goal_sequence = goals_;
  model_pub_.publish(model_hypotheses);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "driver_env");
  ros::NodeHandle nh("driver_env");
  DriverEnv driver_env(&nh);
  driver_env.init();
  ros::Rate r(10);

  while (ros::ok()) {
    ros::spinOnce();
    driver_env.runModel();
    r.sleep();
  }

  ros::shutdown();

  return 0;
}
