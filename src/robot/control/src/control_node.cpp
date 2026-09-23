#include "control_node.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

ControlNode::ControlNode()
    : Node("control"),
      lookahead_distance_m_(this->declare_parameter<double>("lookahead_distance_m", 1.5)),
      goal_tolerance_m_(this->declare_parameter<double>("goal_tolerance_m", 0.2)),
      linear_speed_mps_(this->declare_parameter<double>("linear_speed_mps", 2.0)),
      max_angular_speed_rps_(this->declare_parameter<double>("max_angular_speed_rps", 1.5)) {
  path_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
      "/path", rclcpp::QoS(1).transient_local(),
      std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
  odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10,
      std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));
  cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10);
  control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&ControlNode::controlLoop, this));
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr path) {
  if (path->poses.empty()) {
    current_path_.reset();
    stopRobot();
    return;
  }
  current_path_ = path;
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odometry) {
  latest_odometry_ = odometry;
  last_odometry_time_ = std::chrono::steady_clock::now();
}

void ControlNode::controlLoop() {
  if (!current_path_) {
    return;
  }
  if (!latest_odometry_ ||
      std::chrono::steady_clock::now() - last_odometry_time_ >
          std::chrono::seconds(1)) {
    stopRobot();
    return;
  }
  if (current_path_->header.frame_id != latest_odometry_->header.frame_id) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Path and odometry must use the same frame");
    stopRobot();
    return;
  }

  const auto& robot_pose = latest_odometry_->pose.pose;
  const auto& goal = current_path_->poses.back().pose.position;
  if (std::hypot(goal.x - robot_pose.position.x,
                 goal.y - robot_pose.position.y) <= goal_tolerance_m_) {
    current_path_.reset();
    stopRobot();
    return;
  }

  const auto target = control_.findLookaheadPoint(
      *current_path_, robot_pose.position, lookahead_distance_m_);
  cmd_vel_publisher_->publish(control_.computeVelocity(
      robot_pose, target, linear_speed_mps_, max_angular_speed_rps_));
}

void ControlNode::stopRobot() {
  cmd_vel_publisher_->publish(geometry_msgs::msg::Twist());
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
