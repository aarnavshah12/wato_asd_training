#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include <chrono>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

#include "control_core.hpp"

class ControlNode : public rclcpp::Node {
 public:
  ControlNode();

 private:
  void pathCallback(const nav_msgs::msg::Path::SharedPtr path);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odometry);
  void controlLoop();
  void stopRobot();

  robot::ControlCore control_;
  double lookahead_distance_m_;
  double goal_tolerance_m_;
  double linear_speed_mps_;
  double max_angular_speed_rps_;

  nav_msgs::msg::Path::SharedPtr current_path_;
  nav_msgs::msg::Odometry::SharedPtr latest_odometry_;
  std::chrono::steady_clock::time_point last_odometry_time_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

#endif  // CONTROL_NODE_HPP_
