#include "control_core.hpp"

#include <algorithm>
#include <cmath>

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger)
  : logger_(logger) {}

void ControlCore::configure(double lookahead_distance, double linear_speed, double max_angular_speed,
                            double goal_tolerance, double rotate_threshold, double slowdown_radius) {
  lookahead_distance_ = lookahead_distance;
  linear_speed_ = linear_speed;
  max_angular_speed_ = max_angular_speed;
  goal_tolerance_ = goal_tolerance;
  rotate_threshold_ = rotate_threshold;
  slowdown_radius_ = slowdown_radius;
}

void ControlCore::setPath(const nav_msgs::msg::Path& path) {
  path_.clear();
  path_.reserve(path.poses.size());
  for (const auto& pose : path.poses) {
    path_.push_back(pose.pose.position);
  }
  progress_ = 0;
}

size_t ControlCore::findLookahead(double x, double y) {
  auto distance_to = [x, y](const geometry_msgs::msg::Point& p) { return std::hypot(p.x - x, p.y - y); };

  // Slide our progress marker forward to the closest point. Only search a little way ahead so a
  // path that comes back close to itself can't make us jump over a chunk of it.
  double best = distance_to(path_[progress_]);
  double searched = 0.0;
  for (size_t i = progress_ + 1; i < path_.size() && searched < 2.0 * lookahead_distance_; ++i) {
    searched += std::hypot(path_[i].x - path_[i - 1].x, path_[i].y - path_[i - 1].y);
    double d = distance_to(path_[i]);
    if (d < best) {
      best = d;
      progress_ = i;
    }
  }

  // Lookahead point is the first one past that which is at least a lookahead distance away
  for (size_t i = progress_; i < path_.size(); ++i) {
    if (distance_to(path_[i]) >= lookahead_distance_) {
      return i;
    }
  }
  return path_.size() - 1;  // near the end, just aim for the last point
}

ControlCore::Status ControlCore::update(double x, double y, double yaw, geometry_msgs::msg::Twist& cmd) {
  cmd = geometry_msgs::msg::Twist();
  if (path_.empty()) {
    return Status::NO_PATH;
  }

  const auto& goal = path_.back();
  double goal_distance = std::hypot(goal.x - x, goal.y - y);
  if (goal_distance < goal_tolerance_) {
    path_.clear();
    return Status::GOAL_REACHED;
  }

  const auto& target = path_[findLookahead(x, y)];

  // Lookahead point in the robot's frame (x forward, y left)
  double dx = target.x - x;
  double dy = target.y - y;
  double local_x = std::cos(yaw) * dx + std::sin(yaw) * dy;
  double local_y = -std::sin(yaw) * dx + std::cos(yaw) * dy;
  double alpha = std::atan2(local_y, local_x);

  // Pure pursuit assumes the point is roughly in front of us. If it's way off to the side or behind
  // (new goal, or a replan that goes a different way) turn on the spot first, we're diff drive so we can.
  if (std::abs(alpha) > rotate_threshold_) {
    cmd.angular.z = std::copysign(max_angular_speed_, alpha);
    return Status::FOLLOWING;
  }

  // Arc that starts along our heading and passes through the lookahead point: k = 2 sin(alpha) / L
  double curvature = 2.0 * std::sin(alpha) / std::hypot(dx, dy);

  // Ease off coming into the goal so we don't roll past it
  double speed = linear_speed_ * std::clamp(goal_distance / slowdown_radius_, 0.2, 1.0);
  double turn_rate = speed * curvature;

  // If that's a sharper turn than we're allowed, slow down rather than cap the turn rate.
  // Keeps us on the same arc instead of swinging wide.
  if (std::abs(turn_rate) > max_angular_speed_) {
    turn_rate = std::copysign(max_angular_speed_, turn_rate);
    speed = std::abs(turn_rate / curvature);
  }

  cmd.linear.x = speed;
  cmd.angular.z = turn_rate;
  return Status::FOLLOWING;
}

}
