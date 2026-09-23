#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"

namespace robot {

class ControlCore {
 public:
  geometry_msgs::msg::Point findLookaheadPoint(
      const nav_msgs::msg::Path& path,
      const geometry_msgs::msg::Point& robot_position,
      double lookahead_distance_m) const;

  geometry_msgs::msg::Twist computeVelocity(
      const geometry_msgs::msg::Pose& robot_pose,
      const geometry_msgs::msg::Point& target,
      double linear_speed_mps,
      double max_angular_speed_rps) const;
};

}  // namespace robot

#endif  // CONTROL_CORE_HPP_
