/**
 * @file      visualizer.cpp
 * @brief     Data Visualizer
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-08-04
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */


#include "visualizer/visualizer.hpp"
#include <math.h>
Visualizer::Visualizer(ros::NodeHandle* nh) {
  nh_ = nh;
  this->loadParams();
  this->init();
  this->rosSetup();
  ROS_INFO("Visualizer started");
}

Visualizer::~Visualizer() {
}

void Visualizer::loadParams() {
  nh_->param<std::string>("datafile", datafile_,
                          "/home/testdjp/Documents/NGSIM/driver_comp.txt");
}

void Visualizer::init() {
  frame_ = 0;
  myFile_.open(datafile_);
  if (myFile_.is_open()) {
    ROS_INFO("VIS: File was opened successfully!");
  } else {
    ROS_ERROR("VIS: Error, file could not be opened!");
  }
  this->process_file();
}

void Visualizer::rosSetup() {
  visualizer_pub_ =
    nh_->advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1,
                                                    true);
    driver_env_data_pub = nh_->advertise<driver_env_msgs::Cars>("/driver_env/data",1,true);

}

void Visualizer::pubCarData() {
    driver_env_msgs::Cars cars_msg;
    for (std::vector<int>::iterator i = existing_cars_.begin();
         i != existing_cars_.end(); ++i) {
        if (car_data_[*i].find(frame_) != car_data_[*i].end()) {
            driver_env_msgs::Car car_msg;
            car_struct car_frame(car_data_[*i][frame_]);
            car_msg.card_id = car_frame.car_id;
            car_msg.pose.position.x = car_frame.x_pos;
            car_msg.pose.position.y = car_frame.y_pos;
            double orientation = car_frame.orientation;
            car_msg.pose.orientation = euler2quat(0.0, 0.0, orientation);
            car_msg.vel.linear.x = car_frame.x_vel;
            car_msg.vel.linear.y = car_frame.y_vel;
            car_msg.vel.linear.z = 0;
            car_msg.acc.linear.x = car_frame.x_acc;
            car_msg.acc.linear.y = car_frame.y_acc;
            car_msg.acc.linear.z = 0;
            car_msg.goal.position.x = car_frame.destX;
            car_msg.goal.position.y = car_frame.destY;
            cars_msg.cars.push_back(car_msg);
         }
    }
    driver_env_data_pub.publish(cars_msg);
}

void Visualizer::pubVizData() {
  visualization_msgs::MarkerArray deletemsg;
  visualization_msgs::Marker deletemarkers;
  deletemarkers.action = 3;
  deletemsg.markers.push_back(deletemarkers);
  visualizer_pub_.publish(deletemsg);
  visualization_msgs::MarkerArray msg;
  for (std::vector<int>::iterator i = existing_cars_.begin();
       i != existing_cars_.end(); ++i) {
    if (car_data_[*i].find(frame_) != car_data_[*i].end()) {
      car_struct car_frame(car_data_[*i][frame_]);
      visualization_msgs::Marker data;
      data.header.stamp = ros::Time::now();
      data.header.frame_id = "map";
      data.ns = "visualizer";
      data.text = std::to_string(car_frame.car_id);
      data.id = car_frame.car_id;
      data.type = visualization_msgs::Marker::ARROW;
      data.action = visualization_msgs::Marker::ADD;
      data.scale.x = car_frame.length;
      data.scale.y = car_frame.width;
      data.scale.z = 2.5;
      data.color.a = 1.0;
      data.color.r = car_color_[car_frame.car_id].r;
      data.color.g = car_color_[car_frame.car_id].g;
      data.color.b = car_color_[car_frame.car_id].b;
      data.pose.position.x = car_frame.x_pos;
      data.pose.position.y = car_frame.y_pos;

      double orientation = car_frame.orientation;
      //data.pose.position.x = data.pose.position.x - car_frame.length*sin(orientation);
      //data.pose.position.y = data.pose.position.y - car_frame.length*cos(orientation);
      /**
      if (car_frame.direction == 1) { // East
        orientation = 0.0;
      } else if (car_frame.direction == 2) { // North
        orientation = M_PI / 2;
      } else if (car_frame.direction == 3) { // West
        orientation = M_PI;
      } else if (car_frame.direction == 4) { // South
        orientation = -M_PI / 2;
      }**/
      data.pose.orientation = euler2quat(0.0, 0.0, orientation);

      msg.markers.push_back(data);
    }
    visualizer_pub_.publish(msg);
  }
  frame_ += 1;
}

void Visualizer::process_file() {
  ROS_INFO("Begin");
  char output[256];
  if (myFile_.is_open()) {
    while (!myFile_.eof()) {
      myFile_.getline(output, 256);
      if (strcmp(output, "") == 0) {  // End of File!
        break;
      }
      std::vector<std::string> values = this->split(std::string(output), ' ');
      car_struct frame;
      /**self.preceding = augArray[9]
      self.following = augArray[10]
      self.spaceHeadway = augArray[11]
      **/
      frame.car_id = std::stoi(values[0]);
      frame.frame_id = std::stoi(values[1]);
      //frame.max_frames = std::stoi(values[2]);
      frame.x_pos = std::stof(values[2]);
      frame.y_pos = std::stof(values[3]);
      frame.y_vel = std::stof(values[4]);
      frame.y_acc = std::stof(values[5]);
      frame.x_vel = std::stof(values[6]);
      frame.x_acc = std::stof(values[7]);
      frame.lane = std::stof(values[8]);
      frame.destination = std::stof(values[12]);
      frame.destLane = std::stof(values[13]);
      frame.destX = std::stof(values[19]);
      frame.destY = std::stof(values[20]);
      frame.direction = std::stof(values[14]);
      frame.type = std::stof(values[15]);
      frame.length = std::stof(values[16]);
      frame.width = std::stof(values[17]);
      frame.orientation = std::stof(values[18]);
        // car_data_[frame.car_id].push_back(frame);
      car_data_[frame.car_id][frame.frame_id] = frame;

      if (std::find(existing_cars_.begin(), existing_cars_.end(),
                    frame.car_id) == existing_cars_.end()) {
        existing_cars_.push_back(frame.car_id); // Add car_id if not seen before
        color car_color;
        car_color.r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        car_color.g = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        car_color.b = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        car_color_[frame.car_id] = car_color;
      }
    }
  } else {
    ROS_ERROR("VIS: Error, file is not open!");
  }
}

geometry_msgs::Quaternion Visualizer::euler2quat(
  double roll, double pitch, double yaw) {
  Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond q = rollAngle * yawAngle * pitchAngle;
  geometry_msgs::Quaternion ori;
  ori.x = q.x(); ori.y = q.y(); ori.z = q.z(); ori.w = q.w();
  return ori;
}

std::vector<std::string>& Visualizer::split2(const std::string& s, char delim,
                                             std::vector<std::string>& elems) {
  std::stringstream ss(s);
  std::string item;
  while (getline(ss, item, delim)) {
    elems.push_back(item);
  }
  return elems;
}


std::vector<std::string> Visualizer::split(const std::string& s, char delim) {
  std::vector<std::string> elems;
  split2(s, delim, elems);
  return elems;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "visualizer");
  ros::NodeHandle nh("visualizer");
  Visualizer visualizer(&nh);

  ros::Rate r(10);

  while (ros::ok()) {
    ros::spinOnce();
    visualizer.pubVizData();
    r.sleep();
  }

  ros::shutdown();

  return 0;
}
