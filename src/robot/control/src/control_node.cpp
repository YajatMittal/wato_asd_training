#include "control_node.hpp"

#include <chrono>
#include <cmath>

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  double lookahead_distance = this->declare_parameter("lookahead_distance", 1.5);
  double linear_speed = this->declare_parameter("linear_speed", 0.5);
  double max_angular_speed = this->declare_parameter("max_angular_speed", 1.0);
  double goal_tolerance = this->declare_parameter("goal_tolerance", 0.25);
  double rotate_threshold = this->declare_parameter("rotate_threshold", 1.0);
  double slowdown_radius = this->declare_parameter("slowdown_radius", 1.5);
  control_.configure(lookahead_distance, linear_speed, max_angular_speed,
                     goal_tolerance, rotate_threshold, slowdown_radius);

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // 10 Hz
  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&ControlNode::controlLoop, this));
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  control_.setPath(*msg);
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  odom_ = msg;
}

void ControlNode::controlLoop() {
  if (!odom_) {
    return;
  }

  const auto& position = odom_->pose.pose.position;
  const auto& q = odom_->pose.pose.orientation;
  double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  geometry_msgs::msg::Twist cmd;
  auto status = control_.update(position.x, position.y, yaw, cmd);

  if (status == robot::ControlCore::Status::FOLLOWING) {
    cmd_vel_pub_->publish(cmd);
    driving_ = true;
  } else if (driving_) {
    // Send one stop instead of spamming zeros, so the teleop panel still works while we're idle
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    driving_ = false;
    if (status == robot::ControlCore::Status::GOAL_REACHED) {
      RCLCPP_INFO(this->get_logger(), "Made it to the end of the path");
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
