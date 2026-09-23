#include "planner_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

PlannerNode::PlannerNode()
    : Node("planner"),
      goal_tolerance_m_(this->declare_parameter<double>("goal_tolerance_m", 0.5)),
      replan_timeout_s_(this->declare_parameter<double>("replan_timeout_s", 10.0)) {
  map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(1).transient_local(),
      std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", 10,
      std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10,
      std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
  path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(
      "/path", rclcpp::QoS(1).transient_local());
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
  latest_map_ = map;
  if (state_ == State::FollowingGoal) {
    needs_replan_ = true;
  }
}

void PlannerNode::goalCallback(
    const geometry_msgs::msg::PointStamped::SharedPtr goal) {
  goal_ = goal;
  state_ = State::FollowingGoal;
  needs_replan_ = true;
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odometry) {
  latest_odometry_ = odometry;
}

void PlannerNode::timerCallback() {
  if (state_ != State::FollowingGoal || !goal_ ||
      !latest_map_ || !latest_odometry_) {
    return;
  }
  if (latest_odometry_->header.frame_id != latest_map_->header.frame_id ||
      (!goal_->header.frame_id.empty() &&
       goal_->header.frame_id != latest_map_->header.frame_id)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Goal, odometry, and map must use the same frame");
    return;
  }

  const double distance = std::hypot(
      goal_->point.x - latest_odometry_->pose.pose.position.x,
      goal_->point.y - latest_odometry_->pose.pose.position.y);
  if (distance <= goal_tolerance_m_) {
    state_ = State::WaitingForGoal;
    publishEmptyPath();
    RCLCPP_INFO(get_logger(), "Goal reached");
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  const auto since_plan =
      std::chrono::duration<double>(now - last_plan_time_).count();
  if (needs_replan_ || since_plan >= replan_timeout_s_) {
    planPath();
    needs_replan_ = false;
    last_plan_time_ = now;
  }
}

void PlannerNode::planPath() {
  const auto points = planner_.findPath(
      *latest_map_, latest_odometry_->pose.pose.position, goal_->point);
  if (points.empty()) {
    RCLCPP_WARN(get_logger(), "No path found to the goal");
    publishEmptyPath();
    return;
  }

  nav_msgs::msg::Path path;
  path.header.frame_id = latest_map_->header.frame_id;
  path.header.stamp = this->now();
  for (std::size_t index = 0; index < points.size(); ++index) {
    geometry_msgs::msg::PoseStamped waypoint;
    waypoint.header = path.header;
    waypoint.pose.position = points[index];
    const auto& next = points[std::min(index + 1, points.size() - 1)];
    const double heading = std::atan2(
        next.y - points[index].y, next.x - points[index].x);
    waypoint.pose.orientation.z = std::sin(heading / 2.0);
    waypoint.pose.orientation.w = std::cos(heading / 2.0);
    path.poses.push_back(waypoint);
  }
  path_publisher_->publish(path);
  RCLCPP_INFO(get_logger(), "Planned a path with %zu waypoints", path.poses.size());
}

void PlannerNode::publishEmptyPath() {
  nav_msgs::msg::Path path;
  path.header.frame_id = latest_map_->header.frame_id;
  path.header.stamp = this->now();
  path_publisher_->publish(path);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
