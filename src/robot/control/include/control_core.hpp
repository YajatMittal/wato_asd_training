#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"

namespace robot
{

class ControlCore {
  public:
    enum class Status { NO_PATH, FOLLOWING, GOAL_REACHED };

    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    ControlCore(const rclcpp::Logger& logger);

    void configure(double lookahead_distance, double linear_speed, double max_angular_speed,
                   double goal_tolerance, double rotate_threshold, double slowdown_radius);

    void setPath(const nav_msgs::msg::Path& path);

    // One pure pursuit step for the robot at (x, y, yaw). Fills in cmd and returns what we're doing.
    Status update(double x, double y, double yaw, geometry_msgs::msg::Twist& cmd);

  private:
    size_t findLookahead(double x, double y);

    rclcpp::Logger logger_;

    std::vector<geometry_msgs::msg::Point> path_;
    size_t progress_ = 0;  // path point we're closest to, only ever moves forward

    double lookahead_distance_ = 1.5;
    double linear_speed_ = 0.5;
    double max_angular_speed_ = 1.0;
    double goal_tolerance_ = 0.25;
    double rotate_threshold_ = 1.0;
    double slowdown_radius_ = 1.5;
};

}

#endif
