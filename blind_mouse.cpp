

#include "blind_mouse.hpp"

#include <memory>

using namespace std::chrono_literals;

Turtlebot3Drive::Turtlebot3Drive()
: Node("turtlebot3_drive_node")
{
  /************************************************************
  ** Initialise variables
  ************************************************************/
  scan_data_[0] = 0.0;
  scan_data_[1] = 0.0;
  scan_data_[2] = 0.0; //front

   scan_data_[3] = 0.0; //left

   scan_data_[4] = 0.0; //right

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

// void Turtlebot3Drive::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
// {
//   uint16_t scan_angle[3] = {0, 90, 180};

//   for (int num = 0; num < 3; num++) {
//     if (std::isinf(msg->ranges.at(scan_angle[num]))) {
//       scan_data_[num] = msg->range_max;
//     } else {
//       scan_data_[num] = msg->ranges.at(scan_angle[num]);
//     }
//   }
// }

void Turtlebot3Drive::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  uint16_t scan_angle[5] = {0, 30, 330, 90, 270}; //center, diag-left, diag-right, left, 270

  for (int num = 0; num < 5; num++) 
  {
    if (std::isinf(msg->ranges.at(scan_angle[num]))) 
    {
      scan_data_[num] = msg->range_max;
    } 
    
    else 
    {
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
void Turtlebot3Drive::update_callback()
{
  static uint8_t turtlebot3_state_num = 0;
  double check_forward_dist = 0.35; // Distance to check in front of the robot to consider it clear to move forward.
  double check_diag_dist = 0.3; // Distance to check in front of the robot to consider it clear to move forward.
  double check_side_dist = 0.15; // Distance to check on the sides of the robot to determine if it's close to walls/obstacles.

   std::cout << "Center Data: " << scan_data_[0] << std::endl;
   std::cout << "Diag-Left Data: " << scan_data_[1] << std::endl;
   std::cout << "Diag-Right Data: " << scan_data_[2] << std::endl; //front

   std::cout << "Left Data: " << scan_data_[3] << std::endl;
   std::cout << "Right Data: " << scan_data_[4] << std::endl;

  switch (turtlebot3_state_num) {
    case GET_TB3_DIRECTION:

      if (scan_data_[CENTER] < check_forward_dist) //front blocked
      {
        update_cmd_vel(0.0, 0.0); //stop

        if(scan_data_[RIGHT] > scan_data_[LEFT]) //if there is more space on the right, turn right
        {
          turtlebot3_state_num = TB3_RIGHT_TURN;
        }

        else //if there is more space on the left, turn left
        {
          turtlebot3_state_num = TB3_LEFT_TURN;
        }
        
      }    

      else if (scan_data_[DIAGRIGHT] < check_diag_dist || scan_data_[RIGHT] < check_side_dist) //if diag right is blocked or right side is too near to the wall, turn left
      {
        update_cmd_vel(0.0, 0.0);
        turtlebot3_state_num = TB3_LEFT_TURN;
      }

      else if (scan_data_[DIAGLEFT] < check_diag_dist || scan_data_[LEFT] < check_side_dist) //if diag left is blocked or left side is too near to the wall, turn right
      {
        update_cmd_vel(0.0, 0.0);
        turtlebot3_state_num = TB3_RIGHT_TURN;
      }



      else if (scan_data_[CENTER] > check_forward_dist && scan_data_[DIAGLEFT] > check_diag_dist && scan_data_[DIAGRIGHT] > check_diag_dist)
               //&& scan_data_[LEFT > check_side_dist && scan_data_[RIGHT] > check_side_dist]) //if front cone is clear move forward
      {
        // If there's no obstacle ahead, proceed to drive forward.
        turtlebot3_state_num = TB3_DRIVE_FORWARD;
      } 


      
      // else 
      // {
      //   update_cmd_vel(0.0, 0.0);
      //   // If there's an obstacle in front, check the left side.
      //   if (scan_data_[DIAGLEFT] > check_side_dist) {
      //     // If the left side is clear, turn left.
      //     turtlebot3_state_num = TB3_LEFT_TURN;
      //   } else if (scan_data_[DIAGRIGHT] > check_side_dist) {
      //     // If the left side is not clear, check the right side.
      //     turtlebot3_state_num = TB3_RIGHT_TURN;
      //   }
      // }
      break;

    case TB3_DRIVE_FORWARD:
      update_cmd_vel(LINEAR_VELOCITY, 0.0);
      turtlebot3_state_num = GET_TB3_DIRECTION;
      break;

    case TB3_LEFT_TURN:
      // Turn left until the left side has enough space.
      update_cmd_vel(0.0, ANGULAR_VELOCITY);
      if (scan_data_[LEFT] > check_side_dist) {
        // If there's enough space on the left, move forward again.
        turtlebot3_state_num = TB3_DRIVE_FORWARD;
      }
      break;

    case TB3_RIGHT_TURN:
      // Turn right until the right side has enough space.
      update_cmd_vel(0.0, -ANGULAR_VELOCITY);
      if (scan_data_[RIGHT] > check_side_dist) {
        // If there's enough space on the right, move forward again.
        turtlebot3_state_num = TB3_DRIVE_FORWARD;
      }
      break;

    

    default:
      turtlebot3_state_num = GET_TB3_DIRECTION;
      break;
  }
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

