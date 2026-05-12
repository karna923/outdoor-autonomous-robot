#include "outdoor_robot/motor_controller.hpp"

MotorController::MotorController()
: Node("motor_controller"),
  wheel_base_(0.25),   // 25cm — update when chassis is designed
  max_speed_(1.0)      // 1 m/s max
{
  // Subscribe to velocity commands from Nav2
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10,
    std::bind(&MotorController::cmd_vel_callback, this, std::placeholders::_1));
  
  RCLCPP_INFO(this->get_logger(), "Motor controller node started");
}

void MotorController::cmd_vel_callback(
  const geometry_msgs::msg::Twist::SharedPtr msg)
{
  double linear  = msg->linear.x;
  double angular = msg->angular.z;

  // Differential drive inverse kinematics
  double left_vel  = linear - (angular * wheel_base_ / 2.0);
  double right_vel = linear + (angular * wheel_base_ / 2.0);

  // Clamp to max speed
  left_vel  = std::clamp(left_vel,  -max_speed_, max_speed_);
  right_vel = std::clamp(right_vel, -max_speed_, max_speed_);

  // TODO: send left_vel and right_vel to ESP32 via micro-ROS
  // For now just log them
  RCLCPP_DEBUG(this->get_logger(),
    "CMD_VEL -> left: %.3f m/s  right: %.3f m/s", left_vel, right_vel);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorController>());
  rclcpp::shutdown();
  return 0;
}
