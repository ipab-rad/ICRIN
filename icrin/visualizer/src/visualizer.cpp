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
  ROS_INFO("VIS: Visualizer started");
}

Visualizer::~Visualizer() {
  ros::param::del(nh_->getNamespace());
}

void Visualizer::loadParams() {
  nh_->getParam("data_file", datafile_);
  nh_->param("use_cardinal", use_cardinal, false);
}

void Visualizer::init() {
  frame_ = 0;
  myFile_.open(datafile_);
  if (myFile_.is_open()) {
    ROS_INFO("VIS: File was opened successfully!");
  } else {
    ROS_ERROR("VIS: Error, file could not be opened!");
  }
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
  this->process_file();
}

void Visualizer::rosSetup() {
  visualizer_pub_ = nh_->advertise<visualization_msgs::MarkerArray>
                    ("visualization_marker_array", 1, true);
  driver_env_data_pub = nh_->advertise<driver_env_msgs::Cars>
                        ("/driver_env/car_data", 1, true);
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
      car_msg.yaw = orientation;
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
  //pub goal labels - should probably be other function...
  for (int goal = 0; goal < goals_.size(); goal++) {
    visualization_msgs::Marker label;
    label.header.stamp = ros::Time::now();
    label.header.frame_id = "map";
    label.ns = "visualizer";
    label.text = std::to_string(goal);
    label.id = 0-goal-goals_.size();
    label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    label.action = visualization_msgs::Marker::ADD;
    label.pose.position.x = goals_[goal].x;
    label.pose.position.y = goals_[goal].y;
    label.pose.position.z = 15;
    label.scale.x = 25;
    label.scale.y = 25;
    label.scale.z = 15;
    label.color.a = 1.0;
    label.color.r = 1;
    label.color.g = 1;
    label.color.b = 1;
    msg.markers.push_back(label);
    visualizer_pub_.publish(msg);
  }
  //pub goals - should probably be other function...
  for (int goal = 0; goal < goals_.size(); goal++) {
    visualization_msgs::Marker data;
    data.header.stamp = ros::Time::now();
    data.header.frame_id = "map";
    data.ns = "visualizer";
    data.text = std::to_string(goal);
    data.id = 0-goal;
    data.type = visualization_msgs::Marker::SPHERE;
    data.action = visualization_msgs::Marker::ADD;
    data.pose.position.x = goals_[goal].x;
    data.pose.position.y = goals_[goal].y;
    data.color.a = 1.0;
    data.color.r = 1;
    data.color.g = 0;
    data.color.b = 0;
    data.scale.x = 15.0;
    data.scale.y = 15.0;
    data.scale.z = 15.0;
    msg.markers.push_back(data);
    visualizer_pub_.publish(msg);
  }
  for (std::vector<int>::iterator i = existing_cars_.begin();
       i != existing_cars_.end(); ++i) {
    if (car_data_[*i].find(frame_) != car_data_[*i].end()) {
      car_struct car_frame(car_data_[*i][frame_]);
      visualization_msgs::Marker label;
      label.header.stamp = ros::Time::now();
      label.header.frame_id = "map";
      label.ns = "visualizer";
      label.text = std::to_string(car_frame.car_id);
      label.id = 0-(goals_.size()*2)-car_frame.car_id;
      label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::Marker::ADD;
      label.pose.position.x = car_frame.x_pos;
      label.pose.position.y = car_frame.y_pos;
      label.pose.position.z = 15;
      label.scale.x = 25;
      label.scale.y = 25;
      label.scale.z = 15;
      label.color.a = 1;
      label.color.r = 0;
      label.color.g = 0;
      label.color.b = 0;
      msg.markers.push_back(label);
      visualization_msgs::Marker data;
      data.header.stamp = ros::Time::now();
      data.header.frame_id = "map";
      data.ns = "visualizer";
      data.text = std::to_string(car_frame.car_id);
      data.id = car_frame.car_id;
      data.type = visualization_msgs::Marker::ARROW;
      data.action = visualization_msgs::Marker::ADD;
      data.color.a = 1.0;
      data.color.r = car_color_[car_frame.car_id].r;
      data.color.g = car_color_[car_frame.car_id].g;
      data.color.b = car_color_[car_frame.car_id].b;

      double orientation = car_frame.orientation;
      if (use_cardinal) {
        if (car_frame.direction == 1) { // East
          orientation = 0.0;
        } else if (car_frame.direction == 2) { // North
          orientation = M_PI / 2;
        } else if (car_frame.direction == 3) { // West
          orientation = M_PI;
        } else if (car_frame.direction == 4) { // South
          orientation = -M_PI / 2;
        }
      }
      data.pose.orientation = euler2quat(0.0, 0.0, orientation);

      /* first option is to set the scale*/
      data.scale.x = car_frame.length;
      data.scale.y = car_frame.width;
      data.scale.z = 2.5;
      data.pose.position.x = car_frame.x_pos;
      data.pose.position.y = car_frame.y_pos;
      msg.markers.push_back(data);
    }
    visualizer_pub_.publish(msg);
  }
  frame_ += 1;
}

void Visualizer::process_file() {
  ROS_INFO("VIS: Processing dataset, please stand by...");
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
      frame.lane = std::stoi(values[8]);
      frame.destination = std::stoi(values[12]);
      frame.destLane = std::stoi(values[13]);
      frame.destX = std::stof(values[19]);
      frame.destY = std::stof(values[20]);
      frame.direction = std::stoi(values[14]);
      frame.type = std::stoi(values[15]);
      frame.length = std::stof(values[16]);
      frame.width = std::stof(values[17]);
      frame.orientation = std::stof(values[18]);
      // car_data_[frame.car_id].push_back(frame);
      car_data_[frame.car_id][frame.frame_id] = frame;

      float dx = frame.length * cos(frame.orientation);
      float dy = frame.length * sin(frame.orientation);
      frame.x_pos = frame.x_pos - dx;
      frame.y_pos = frame.y_pos - dy;
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
    visualizer.pubCarData();
    r.sleep();
  }

  ros::shutdown();

  return 0;
}
