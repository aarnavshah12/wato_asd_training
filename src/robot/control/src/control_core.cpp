#include "control_core.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace robot {

geometry_msgs::msg::Point ControlCore::findLookaheadPoint(
    const nav_msgs::msg::Path& path,
    const geometry_msgs::msg::Point& robot_position,
    double lookahead_distance_m) const {
  if (path.poses.size() == 1) {
    return path.poses.front().pose.position;
  }

  // Start at the nearest place on the path, even between two waypoints.
  double nearest_distance = std::numeric_limits<double>::infinity();
  std::size_t nearest_segment = 0;
  double nearest_fraction = 0.0;
  for (std::size_t index = 0; index + 1 < path.poses.size(); ++index) {
    const auto& start = path.poses[index].pose.position;
    const auto& end = path.poses[index + 1].pose.position;
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double length_squared = dx * dx + dy * dy;
    const double fraction = length_squared > 0.0
        ? std::clamp(((robot_position.x - start.x) * dx +
                      (robot_position.y - start.y) * dy) / length_squared,
                     0.0, 1.0)
        : 0.0;
    const double distance = std::hypot(
        robot_position.x - (start.x + fraction * dx),
        robot_position.y - (start.y + fraction * dy));
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest_segment = index;
      nearest_fraction = fraction;
    }
  }

  // Look farther ahead on straights, but keep turns tight near a corner.
  double distance_left = lookahead_distance_m;
  if (nearest_segment + 2 < path.poses.size()) {
    const auto& start = path.poses[nearest_segment].pose.position;
    const auto& corner = path.poses[nearest_segment + 1].pose.position;
    const auto& after = path.poses[nearest_segment + 2].pose.position;
    const double first_heading = std::atan2(corner.y - start.y, corner.x - start.x);
    const double next_heading = std::atan2(after.y - corner.y, after.x - corner.x);
    const double bend = std::atan2(std::sin(next_heading - first_heading),
                                   std::cos(next_heading - first_heading));
    const double distance_to_corner = (1.0 - nearest_fraction) *
        std::hypot(corner.x - start.x, corner.y - start.y);
    if (std::abs(bend) > 0.35 && distance_to_corner < 2.0) {
      distance_left = 0.6;
    }
  }
  for (std::size_t index = nearest_segment; index + 1 < path.poses.size(); ++index) {
    const auto& start = path.poses[index].pose.position;
    const auto& end = path.poses[index + 1].pose.position;
    const double start_fraction = index == nearest_segment ? nearest_fraction : 0.0;
    const double segment_left =
        (1.0 - start_fraction) * std::hypot(end.x - start.x, end.y - start.y);
    if (distance_left <= segment_left && segment_left > 0.0) {
      const double fraction =
          start_fraction + (1.0 - start_fraction) * distance_left / segment_left;
      geometry_msgs::msg::Point target;
      target.x = start.x + fraction * (end.x - start.x);
      target.y = start.y + fraction * (end.y - start.y);
      return target;
    }
    distance_left -= segment_left;
  }
  return path.poses.back().pose.position;
}

geometry_msgs::msg::Twist ControlCore::computeVelocity(
    const geometry_msgs::msg::Pose& robot_pose,
    const geometry_msgs::msg::Point& target,
    double linear_speed_mps,
    double max_angular_speed_rps) const {
  const auto& rotation = robot_pose.orientation;
  const double yaw = std::atan2(
      2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
      1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z));
  const double dx = target.x - robot_pose.position.x;
  const double dy = target.y - robot_pose.position.y;
  const double target_angle = std::atan2(dy, dx);
  const double heading_error = std::atan2(
      std::sin(target_angle - yaw), std::cos(target_angle - yaw));

  geometry_msgs::msg::Twist command;
  if (std::abs(heading_error) > 0.9) {
    // Turn toward a target that is far to the side or behind the robot.
    command.angular.z = std::clamp(heading_error, -max_angular_speed_rps,
                                    max_angular_speed_rps);
    return command;
  }

  const double distance_squared = dx * dx + dy * dy;
  if (distance_squared > 0.0001) {
    const double target_left = -std::sin(yaw) * dx + std::cos(yaw) * dy;
    const double curvature = 2.0 * target_left / distance_squared;
    // Slow down on tight bends so the robot can actually make the turn.
    const double turn_speed = std::abs(curvature) > 0.001
        ? max_angular_speed_rps / std::abs(curvature)
        : linear_speed_mps;
    const double heading_speed = linear_speed_mps *
        std::clamp(1.0 - std::abs(heading_error) / 2.0, 0.5, 1.0);
    command.linear.x = std::min({linear_speed_mps, turn_speed, heading_speed});
    command.angular.z = command.linear.x * curvature;
  }
  return command;
}

}  // namespace robot
