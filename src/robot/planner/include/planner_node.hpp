#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include <chrono>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node {
 public:
  PlannerNode();

 private:
  enum class State { WaitingForGoal, FollowingGoal };

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map);
  void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr goal);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odometry);
  void timerCallback();
  void planPath();
  void publishEmptyPath();

  robot::PlannerCore planner_;
  State state_ = State::WaitingForGoal;
  bool needs_replan_ = false;
  double goal_tolerance_m_;
  double replan_timeout_s_;
  std::chrono::steady_clock::time_point last_plan_time_;

  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
  nav_msgs::msg::Odometry::SharedPtr latest_odometry_;
  geometry_msgs::msg::PointStamped::SharedPtr goal_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // PLANNER_NODE_HPP_
