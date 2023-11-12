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

#include "turtlebot3_gazebo/turtlebot3_drive.hpp"

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
      "scan",
      rclcpp::SensorDataQoS(),
      std::bind(
          &Turtlebot3Drive::scan_callback,
          this,
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
  x_ = msg->pose.pose.position.x;
  y_ = msg->pose.pose.position.y;
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
  uint16_t scan_angle[3] = {0, 60, 300};

  for (int num = 0; num < 3; num++)
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

void Turtlebot3Drive::measure_distance()
{

  static double x = 0;
  static double previous_position = 0;
  if (scan_data_[CENTER] > 0.45)
  {

    update_cmd_vel(LINEAR_VELOCITY, 0.0);

    x += x_ - previous_position;
  }

  if (scan_data_[CENTER] < 0.45)
  {
    update_cmd_vel(0, 0.0);
  }
  previous_position=x_;
  printf("DISTANCE: %f\n", x);

  // 0.83 wyszlo okolo
}
/********************************************************************************
** PID REGULATOR FOR SENSOR
********************************************************************************/
void Turtlebot3Drive::sensorPID()
{

 // update_cmd_vel(LINEAR_VELOCITY, 0.0);
  // static int stop = 0;
  // static int i = 1;
  static float I, prev_error;
  float error;
  float cs = 0; //  suma bledow P I D
  float Kp = 0.1, Ki = 0.00, Kd = 0.05;
  float P, D;

  error = scan_data_[LEFT] - scan_data_[RIGHT];

  //   if ((x_ - (0.90 * i))< 0.1 && (x_ - (0.90 * i)) > 0)
  // {
  //   stop = 1;
  //   printf("BOKS\n");
  // }

  // if (stop)
  // {
  //   update_cmd_vel(0, 0.0);
  //    printf("STOP\n");
  //   std::this_thread::sleep_for(2000ms);
  //   i++;
  //   stop = 0;
  // }

  P = error * Kp;
  I = (I + error) * Ki;
  D = (error - prev_error) * Kd;

  cs = P + I + D;

  if (cs > ANGULAR_VELOCITY)
    cs = ANGULAR_VELOCITY;
  if (cs < -ANGULAR_VELOCITY)
    cs = -ANGULAR_VELOCITY;

  update_cmd_vel(LINEAR_VELOCITY, cs);

  prev_error = error;
}

/********************************************************************************
** PID REGULATOR FOR ODOMETRY - TURNING ROBOT
********************************************************************************/
void Turtlebot3Drive::odomPID(double angle)
{

  angle += robot_pose_;
  angle *= DEG2RAD;
  update_cmd_vel(LINEAR_VELOCITY, 0.0);

  static float I, prev_error;
  float error;
  float cs = 0; //  suma bledow P I D
  float Kp = 0.1, Ki = 0.00, Kd = 0.05;
  float P, D;

  error = angle - robot_pose_;

  P = error * Kp;
  I = (I + error) * Ki;
  D = (error - prev_error) * Kd;

  cs = P + I + D;

  if (cs > ANGULAR_VELOCITY)
    cs = ANGULAR_VELOCITY;
  if (cs < -ANGULAR_VELOCITY)
    cs = -ANGULAR_VELOCITY;

  update_cmd_vel(0, cs);

  prev_error = error;
}
/********************************************************************************
** TURN 90 TO RIGHT
********************************************************************************/
void Turtlebot3Drive::turn_right(){


  update_cmd_vel(0, -ANGULAR_VELOCITY);


  std::this_thread::sleep_for(1600ms);
  update_cmd_vel(0, 0);

  std::this_thread::sleep_for(3500ms);

}

/********************************************************************************
** TURN 90 TO LEFT
********************************************************************************/
void Turtlebot3Drive::turn_left(){


  update_cmd_vel(0, ANGULAR_VELOCITY);
  

  std::this_thread::sleep_for(1600ms);
  update_cmd_vel(0, 0);

  std::this_thread::sleep_for(3500ms);



}


/********************************************************************************
** TURN BACK
********************************************************************************/
void Turtlebot3Drive::turn_back(){
  

  update_cmd_vel(0, ANGULAR_VELOCITY);
  

  std::this_thread::sleep_for(3300ms);
  update_cmd_vel(0, 0);
   std::this_thread::sleep_for(1000ms);
 

  

}



/********************************************************************************
** Update functions
********************************************************************************/
void Turtlebot3Drive::update_callback()
 {

  printf("X: %f  Y: %f A:  %f\n", x_, y_, robot_pose_);
  static uint8_t turtlebot3_state_num = 0;
  //  double escape_range = 30.0 * DEG2RAD;
  double check_forward_dist = 0.45;
  // double check_side_dist = 0.6;
  // printf("X: %f\n", x_);
  // printf("Y: %f\n", y_);
  // printf("A: %f\n", robot_pose_);
   //printf("ANGLE: %f\n", robot_pose_*RAD2DEG);
  switch (turtlebot3_state_num)
  {
  case GET_TB3_DIRECTION:
    if (scan_data_[CENTER] > check_forward_dist)
    {
      turtlebot3_state_num = TB3_DRIVE_FORWARD;
    }
    else
    {
      turtlebot3_state_num = 4;
    }
    break;

  case TB3_DRIVE_FORWARD:
   // sensorPID();
 //turn_back();

    turtlebot3_state_num = GET_TB3_DIRECTION;

    break;

  case 4:
   
    // odomPID(180);
   // turn_back();
    turtlebot3_state_num = GET_TB3_DIRECTION;

    break;

  default:
    turtlebot3_state_num = GET_TB3_DIRECTION;

    break;
  }

}

/*******************************************************************************
** Main
*******************************************************************************/
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Turtlebot3Drive>());
  rclcpp::shutdown();

  return 0;
}
