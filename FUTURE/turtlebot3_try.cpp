// Copyright 2019 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Taehun Lim (Darby), Ryan Shim

//#include "turtlebot3_gazebo/turtlebot3_try.hpp"
#include "turtlebot3_try.hpp"
#include <memory>
#include <iostream>

using namespace std::chrono_literals;


Turtlebot3Drive::Turtlebot3Drive()
: Node("turtlebot3_drive_node")
{
  /************************************************************
  ** Initialise variables
  ************************************************************/
  scan_data_[0] = 0.0;
  scan_data_[1] = 0.0;
  scan_data_[2] = 0.0;

  robot_pose_ = 0.0;
  prev_robot_pose_ = 0.0;

  /************************************************************
  ** Initialise ROS publishers and subscribers
  ************************************************************/
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  // Initialise publishers
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

  // Initialise subscribers
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", \
    rclcpp::SensorDataQoS(), \
    std::bind(
      &Turtlebot3Drive::scan_callback, \
      this, \
      std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", qos, std::bind(&Turtlebot3Drive::odom_callback, this, std::placeholders::_1));

  /************************************************************
  ** Initialise ROS timers
  ************************************************************/
  update_timer_ = this->create_wall_timer(10ms, std::bind(&Turtlebot3Drive::update_callback, this));

  RCLCPP_INFO(this->get_logger(), "Turtlebot3 simulation node has been initialised");
}

Turtlebot3Drive::~Turtlebot3Drive()
{
  RCLCPP_INFO(this->get_logger(), "Turtlebot3 simulation node has been terminated");
}

/********************************************************************************
** Callback functions for ROS subscribers
********************************************************************************/
void Turtlebot3Drive::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  robot_pose_ = yaw;
}

void Turtlebot3Drive::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  uint16_t scan_angle[5] = {0, 30, 330, 90, 270};

  for (int num = 0; num < 5; num++) {
    if (std::isinf(msg->ranges.at(scan_angle[num]))) {
      scan_data_[num] = msg->range_max;
    } else {
      scan_data_[num] = msg->ranges.at(scan_angle[num]);
    }
  }
}

void Turtlebot3Drive::update_cmd_vel(double linear, double angular)
{
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub_->publish(cmd_vel);
}

/********************************************************************************
** Update functions
********************************************************************************/

/* 
MAX STATS
forward dist3.5
left dist3.5
right dist3.5
*/
int Turtlebot3Drive::get_sector(int angle) {
    if (angle >= 0 && angle < 90) return TOP_RIGHT;
    else if (angle >= 90 && angle < 180) return BOTTOM_RIGHT;
    else if (angle >= 180 && angle < 270) return BOTTOM_LEFT;
    else return BOTTOM_RIGHT;
}

void Turtlebot3Drive::drive_forward() {
    update_cmd_vel(LINEAR_VELOCITY, 0.0);
}

// Function for a right turn
void Turtlebot3Drive::right_turn() {
    update_cmd_vel(LINEAR_VELOCITY, -1 * ANGULAR_VELOCITY);
}

// Function for a left turn
void Turtlebot3Drive::left_turn() {
    update_cmd_vel(LINEAR_VELOCITY, ANGULAR_VELOCITY);
}

// Function for a hard right turn
void Turtlebot3Drive::hard_right() {
    update_cmd_vel(NONE_VEL, -1 * ANGULAR_VELOCITY);
}

void Turtlebot3Drive::hard_left() {
    update_cmd_vel(NONE_VEL, 1 * ANGULAR_VELOCITY);
}

// Function for a slight left curve
void Turtlebot3Drive::curve_left() {
    update_cmd_vel(LINEAR_VELOCITY, CURVE_VEL);
}

void Turtlebot3Drive::stop() {
    update_cmd_vel(0, 0);
}

void Turtlebot3Drive::buffer_guard(){
    double safe_distance = 0.9;
    for (int angle = 0; angle < 360; ++angle) {
    double distance = scan_data_[angle];
    if (distance < safe_distance) {
        //std::cout<<"BUFFER IN EFFECT"<<std::endl;
        switch (get_sector(angle)) {
            case TOP_RIGHT:
                left_turn();
                break;
            case TOP_LEFT:
                right_turn();
                break;
            case BOTTOM_LEFT:
                drive_forward();
                break;
            case BOTTOM_RIGHT:
                drive_forward();
                break;

        }
      }
    }
}

void Turtlebot3Drive::corner_guard(double left, double right){
  if(left < MIN_LEFT/4){
    //right_turn();
    //std::cout<<"Corner right"<<std::endl;
  }else if(right < MIN_LEFT/4){
    //left_turn();
    //std::cout<<"Corner left"<<std::endl;
  }
}

void Turtlebot3Drive::wall_follow(){
  static uint8_t turtlebot3_state_num = 0;
  double escape_range = 30.0 * DEG2RAD;
  double check_forward_dist = 0.35; //original is 0.05
  double check_side_dist = 0.3; //ORIGINAL IS 0.05  //near is around 0.44505
  double check_corner_dist = 0.15;
  //  std::cout<<"forward dist "<< scan_data_[CENTER]<<std::endl;
  //  std::cout<<"left dist "<<scan_data_[LEFT]<<std::endl;
  //  std::cout<<"right dist "<<scan_data_[RIGHT]<<std::endl;
  double front = scan_data_[CENTER];
  double right = scan_data_[RIGHT];
  double left = scan_data_[LEFT];
  double left_corner = scan_data_[CORNER_LEFT];
  double right_corner = scan_data_[CORNER_RIGHT];

  switch (turtlebot3_state_num) {
    case GET_TB3_DIRECTION:

        if(front < check_forward_dist){
          turtlebot3_state_num = TB3_STOP;
          std::cout<<"SMTH IN FRONT"<<std::endl;
          if(right > left){
            turtlebot3_state_num = TB3_HARD_RIGHT;
          }else{
            turtlebot3_state_num = TB3_HARD_LEFT;
          }

        }else if(right_corner < check_corner_dist || right < check_side_dist){
          prev_robot_pose_ = robot_pose_;
          turtlebot3_state_num = TB3_HARD_LEFT;
          std::cout<<"left"<<std::endl;
              
        }else if(left_corner < check_corner_dist || left < check_side_dist){
          prev_robot_pose_ = robot_pose_;
          turtlebot3_state_num = TB3_HARD_RIGHT;
          std::cout<<"right"<<std::endl;

        }else if(front > check_forward_dist && left_corner>check_corner_dist && right_corner){
          prev_robot_pose_ = robot_pose_;
          turtlebot3_state_num = TB3_DRIVE_FORWARD;
          std::cout<<"forward"<<std::endl;

        }


      break;

    case TB3_DRIVE_FORWARD:
      drive_forward();
      turtlebot3_state_num = GET_TB3_DIRECTION;
      break;

    case TB3_RIGHT_TURN:
      if (fabs(prev_robot_pose_ - robot_pose_) >= escape_range) {
        turtlebot3_state_num = GET_TB3_DIRECTION;
      } else {
        right_turn();
      }
      break;

    case TB3_LEFT_TURN:
      if (fabs(prev_robot_pose_ - robot_pose_) >= escape_range) {
        turtlebot3_state_num = GET_TB3_DIRECTION;
      } else {
        left_turn();
      }
      break;

    case TB3_HARD_RIGHT:
      hard_right();
      if(right > check_side_dist){
          turtlebot3_state_num = TB3_DRIVE_FORWARD;
      }
      break;  

    case TB3_HARD_LEFT:
        
      hard_left();
      if(left > check_side_dist){
          turtlebot3_state_num = TB3_DRIVE_FORWARD;
      }

      break;  

    case TB3_CURVE_LEFT:
      if (fabs(prev_robot_pose_ - robot_pose_) >= escape_range) {
        turtlebot3_state_num = GET_TB3_DIRECTION;
      } else {
        update_cmd_vel(0.1, 0.3);
      }
      break;
      
    case TB3_STOP:
      if (fabs(prev_robot_pose_ - robot_pose_) >= escape_range) {
        turtlebot3_state_num = GET_TB3_DIRECTION;
      } else {
        stop();
      }
      break;

    default:
      turtlebot3_state_num = GET_TB3_DIRECTION;
      break;
  }
}

void Turtlebot3Drive::obstacle_avoidance(){
  static uint8_t turtlebot3_state_num = 0;
  double escape_range = 30.0 * DEG2RAD;
  double check_forward_dist = 0.8; //original is 0.05
  double check_side_dist = 0.2; //ORIGINAL IS 0.05  //near is around 0.44505
  std::cout<<"forward dist "<< scan_data_[CENTER]<<std::endl;
  std::cout<<"left dist "<<scan_data_[LEFT]<<std::endl;
  std::cout<<"right dist "<<scan_data_[RIGHT]<<std::endl;
  switch (turtlebot3_state_num) {
    case GET_TB3_DIRECTION:
      if (scan_data_[CENTER] > check_forward_dist) { 
        if (scan_data_[LEFT] < check_side_dist) {
          prev_robot_pose_ = robot_pose_;
          turtlebot3_state_num = TB3_RIGHT_TURN;
        } else if (scan_data_[RIGHT] < check_side_dist) {
          prev_robot_pose_ = robot_pose_;
          turtlebot3_state_num = TB3_LEFT_TURN;
        } else {
          turtlebot3_state_num = TB3_DRIVE_FORWARD;
        }
      }

      if (scan_data_[CENTER] < check_forward_dist) {
        prev_robot_pose_ = robot_pose_;
        turtlebot3_state_num = TB3_RIGHT_TURN;
      }
      break;

    case TB3_DRIVE_FORWARD:
      update_cmd_vel(LINEAR_VELOCITY, 0.0);
      turtlebot3_state_num = GET_TB3_DIRECTION;
      break;

    case TB3_RIGHT_TURN:
      if (fabs(prev_robot_pose_ - robot_pose_) >= escape_range) {
        turtlebot3_state_num = GET_TB3_DIRECTION;
      } else {
        update_cmd_vel(0.0, -1 * ANGULAR_VELOCITY);
      }
      break;

    case TB3_LEFT_TURN:
      if (fabs(prev_robot_pose_ - robot_pose_) >= escape_range) {
        turtlebot3_state_num = GET_TB3_DIRECTION;
      } else {
        update_cmd_vel(0.0, ANGULAR_VELOCITY);
      }
      break;

    default:
      turtlebot3_state_num = GET_TB3_DIRECTION;
      break;
  }
}

void Turtlebot3Drive::update_callback()
{
  wall_follow();
}

/*******************************************************************************
** Main
*******************************************************************************/
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Turtlebot3Drive>());
  rclcpp::shutdown();

  return 0;
}