#include "outdoor_robot/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"

OdometryPublisher::OdometryPublisher()
: Node("odometry_publisher"),
  x_(0.0), y_(0.0), theta_(0.0),
  wheel_base_(0.25)
{
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  last_time_ = this->now();

  RCLCPP_INFO(this->get_logger(), "Odometry publisher node started");
}

void OdometryPublisher::update_odometry(
  double left_vel, double right_vel, double dt)
{
  // Forward kinematics
  double linear  = (left_vel + right_vel) / 2.0;
  double angular = (right_vel - left_vel) / wheel_base_;

  // Integrate pose
  theta_ += angular * dt;
  x_ += linear * std::cos(theta_) * dt;
  y_ += linear * std::sin(theta_) * dt;

  // Publish odometry message
  auto odom_msg = nav_msgs::msg::Odometry();
  odom_msg.header.stamp = this->now();
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id  = "base_link";

  odom_msg.pose.pose.position.x = x_;
  odom_msg.pose.pose.position.y = y_;

  tf2::Quaternion q;
  q.setRPY(0, 0, theta_);
  odom_msg.pose.pose.orientation.x = q.x();
  odom_msg.pose.pose.orientation.y = q.y();
  odom_msg.pose.pose.orientation.z = q.z();
  odom_msg.pose.pose.orientation.w = q.w();

  odom_pub_->publish(odom_msg);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryPublisher>());
  rclcpp::shutdown();
  return 0;
}
