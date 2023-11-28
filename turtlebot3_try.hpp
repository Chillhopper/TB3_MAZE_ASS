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

#ifndef TURTLEBOT3_GAZEBO__TURTLEBOT3_DRIVE_HPP_
#define TURTLEBOT3_GAZEBO__TURTLEBOT3_DRIVE_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#define SPEED_SCALE 0.5

#define MAX_DIST 1.5
#define MIN_LEFT 0.3
#define MAX_LEFT 0.4

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

#define CENTER 0
#define LEFT   1
#define RIGHT  2
#define CORNER_LEFT 3
#define CORNER_RIGHT 4

#define LINEAR_VELOCITY  0.1 * SPEED_SCALE
#define ANGULAR_VELOCITY 1.1 * SPEED_SCALE
#define CURVE_VEL 0.3 * SPEED_SCALE
#define NONE_VEL 0 * SPEED_SCALE

#define GET_TB3_DIRECTION 0
#define TB3_DRIVE_FORWARD 1
#define TB3_RIGHT_TURN    2
#define TB3_LEFT_TURN     3
#define TB3_HARD_RIGHT    4
#define TB3_CURVE_LEFT    5
#define TB3_CURVE_RIGHT   6
#define TB3_HARD_LEFT     7
#define TB3_STOP          8

#define TOP_RIGHT 11
#define TOP_LEFT 12
#define BOTTOM_RIGHT 13
#define BOTTOM_LEFT 14



class Turtlebot3Drive : public rclcpp::Node
{
public:
  Turtlebot3Drive();
  ~Turtlebot3Drive();

private:
  // ROS topic publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // ROS topic subscribers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Variables
  double robot_pose_;
  double prev_robot_pose_;
  double scan_data_[4];

  // ROS timer
  rclcpp::TimerBase::SharedPtr update_timer_;

  // Function prototypes
  void update_callback();
  void update_cmd_vel(double linear, double angular);
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void obstacle_avoidance();
  void wall_follow();

  int get_sector(int angle);
  void drive_forward();
  void right_turn();
  void left_turn();
  void hard_right();
  void buffer_guard();
  void curve_left();
  void corner_guard(double left = 100, double right = 100);
  void hard_left();
  void stop();
};
#endif  // TURTLEBOT3_GAZEBO__TURTLEBOT3_DRIVE_HPP_

