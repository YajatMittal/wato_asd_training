#include "planner_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  int lethal_cost = this->declare_parameter("lethal_cost", 50);
  int unknown_cost = this->declare_parameter("unknown_cost", 10);
  double cost_weight = this->declare_parameter("cost_weight", 3.0);
  double goal_search_radius = this->declare_parameter("goal_search_radius", 1.5);
  planner_.configure(lethal_cost, unknown_cost, cost_weight, goal_search_radius);

  goal_tolerance_ = this->declare_parameter("goal_tolerance", 0.5);
  goal_timeout_ = this->declare_parameter("goal_timeout", 120.0);
  progress_timeout_ = this->declare_parameter("progress_timeout", 10.0);

  // Map memory publishes transient local, match it so we still get the map if we start after it
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", rclcpp::QoS(1).transient_local(), std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  planner_.setMap(msg);
  map_frame_ = msg->header.frame_id;

  // New map means we might know about obstacles the current path goes through
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    planPath();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  if (planner_.hasMap() && msg->header.frame_id != map_frame_) {
    RCLCPP_WARN(this->get_logger(), "Goal is in frame '%s' but the map is in '%s', using it as is",
                msg->header.frame_id.c_str(), map_frame_.c_str());
  }

  goal_ = msg->point;
  target_ = goal_;
  RCLCPP_INFO(this->get_logger(), "New goal: (%.2f, %.2f)", goal_.x, goal_.y);

  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  goal_start_time_ = this->now();
  last_progress_time_ = goal_start_time_;
  best_distance_ = distanceToGoal();
  planPath();

  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL &&
      std::hypot(target_.x - goal_.x, target_.y - goal_.y) > 0.2) {
    RCLCPP_INFO(this->get_logger(), "Goal is too close to an obstacle, heading for (%.2f, %.2f) instead",
                target_.x, target_.y);
  }
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_position_ = msg->pose.pose.position;
  odom_stamp_ = msg->header.stamp;
  have_odom_ = true;
}

void PlannerNode::timerCallback() {
  if (state_ != State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    return;
  }

  double distance = distanceToGoal();
  if (distance < goal_tolerance_) {
    // Control finishes off the last bit and stops at the end of the path on its own
    RCLCPP_INFO(this->get_logger(), "Goal reached!");
    state_ = State::WAITING_FOR_GOAL;
    return;
  }

  rclcpp::Time now = this->now();
  if ((now - goal_start_time_).seconds() > goal_timeout_) {
    abortGoal("Timed out");
    return;
  }

  // If we haven't gotten any closer in a while something's off (stuck, or following a bad plan), so replan
  if (distance < best_distance_ - 0.25) {
    best_distance_ = distance;
    last_progress_time_ = now;
  } else if ((now - last_progress_time_).seconds() > progress_timeout_) {
    RCLCPP_INFO(this->get_logger(), "Not making progress, replanning");
    last_progress_time_ = now;
    planPath();
  }
}

void PlannerNode::planPath() {
  if (!planner_.hasMap() || !have_odom_) {
    RCLCPP_WARN(this->get_logger(), "Can't plan yet, still waiting for %s",
                planner_.hasMap() ? "odometry" : "a map");
    return;
  }

  auto points = planner_.plan(robot_position_, goal_);
  if (points.empty()) {
    abortGoal("No path to the goal");
    return;
  }

  target_ = points.back();
  publishPath(points);
}

void PlannerNode::abortGoal(const std::string& reason) {
  RCLCPP_WARN(this->get_logger(), "%s, dropping this goal", reason.c_str());
  state_ = State::WAITING_FOR_GOAL;
  publishPath({});  // empty path tells control to stop
}

void PlannerNode::publishPath(const std::vector<geometry_msgs::msg::Point>& points) {
  nav_msgs::msg::Path path;
  path.header.stamp = odom_stamp_;
  path.header.frame_id = map_frame_;
  path.poses.reserve(points.size());

  for (size_t i = 0; i < points.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position = points[i];

    // Point each pose along the path, mostly so it looks right in foxglove
    size_t from = (i + 1 < points.size()) ? i : (i > 0 ? i - 1 : i);
    size_t to = std::min(from + 1, points.size() - 1);
    double yaw = std::atan2(points[to].y - points[from].y, points[to].x - points[from].x);
    pose.pose.orientation.z = std::sin(yaw / 2.0);
    pose.pose.orientation.w = std::cos(yaw / 2.0);

    path.poses.push_back(pose);
  }

  path_pub_->publish(path);
}

double PlannerNode::distanceToGoal() const {
  return std::hypot(target_.x - robot_position_.x, target_.y - robot_position_.y);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
