#ifndef OUTDOOR_ROBOT__MOTOR_CONTROLLER_HPP_
#define OUTDOOR_ROBOT__MOTOR_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class MotorController : public rclcpp::Node
{
public:
  MotorController();

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  
  // Robot physical params
  double wheel_base_;   // distance between left and right wheels (meters)
  double max_speed_;    // max wheel speed (m/s)
};

#endif
