#ifndef OUTDOOR_ROBOT__ODOMETRY_HPP_
#define OUTDOOR_ROBOT__ODOMETRY_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

class OdometryPublisher : public rclcpp::Node
{
public:
  OdometryPublisher();

private:
  void update_odometry(double left_vel, double right_vel, double dt);
  
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
  // Robot state
  double x_, y_, theta_;
  double wheel_base_;
  rclcpp::Time last_time_;
};

#endif
