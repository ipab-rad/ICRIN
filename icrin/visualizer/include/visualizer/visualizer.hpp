/**
 * @file      visualizer.hpp
 * @brief     Data Visualizer
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-08-04
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */


#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP

#include <ros/ros.h>
#include <Eigen/Geometry>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <driver_env_msgs/Car.h>
#include <driver_env_msgs/Cars.h>
#include <iostream>
#include <fstream>
#include <sstream>

struct car_struct {
  int car_id;
  int frame_id;
  int max_frames;
  float x_pos;
  float y_pos;
  float y_vel;
  float y_acc;
  float x_vel;
  float x_acc;
  int lane;
  int destination;
  int destLane;
  float destX;
  float destY;
  int direction;
  float length;
  float width;
  float orientation;
  int type;
} ;

struct color {
  float r;
  float g;
  float b;
} ;

class Visualizer {
 public:
  explicit Visualizer(ros::NodeHandle* nh);
  ~Visualizer();
  bool isModelReady();
  void init();
  void rosSetup();
  void loadParams();
  void pubCarData();
  void pubVizData();
  void setModelReady(bool isModelReady);
  void process_file();
  void modelReadyCB(const std_msgs::Bool::ConstPtr& msg);

  geometry_msgs::Quaternion euler2quat(double roll, double pitch, double yaw);

  std::vector<std::string>& split2(const std::string& s, char delim,
                                   std::vector<std::string>& elems);
  std::vector<std::string> split(const std::string& s, char delim);

 private:
  // Flags
  bool use_cardinal;  // Use cardinal orientaitons instead of estimated
  bool model_ready_;
  bool use_goal_labels_;
  bool use_car_labels_;
  // Constants

  // Variables
  int frame_;
  std::string datafile_;
  std::ifstream myFile_;
  std::vector<int> active_cars_;
  std::vector<int> existing_cars_;
  std::map<int, std::map<int, car_struct> > car_data_;
  std::map <int, color> car_color_;
  std::vector<geometry_msgs::Pose2D> goals_;
  // ROS
  ros::NodeHandle* nh_;
  ros::Publisher ready_pub_;
  ros::Publisher visualizer_pub_;
  ros::Publisher driver_env_data_pub_;
  ros::Subscriber model_ready_sub_;
};

#endif /* VISUALIZER_HPP */
