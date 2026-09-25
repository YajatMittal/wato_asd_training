#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();

  private:
    enum class State { WAITING_FOR_GOAL, WAITING_FOR_ROBOT_TO_REACH_GOAL };

    void planPath();
    void abortGoal(const std::string& reason);
    void publishPath(const std::vector<geometry_msgs::msg::Point>& points);
    double distanceToGoal() const;

    robot::PlannerCore planner_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    State state_ = State::WAITING_FOR_GOAL;
    std::string map_frame_;
    bool have_odom_ = false;

    geometry_msgs::msg::Point goal_;
    geometry_msgs::msg::Point target_;  // where the current path really ends, the goal can get nudged out of obstacles
    geometry_msgs::msg::Point robot_position_;
    builtin_interfaces::msg::Time odom_stamp_;

    // For the timeout and "are we still getting closer" checks
    rclcpp::Time goal_start_time_;
    rclcpp::Time last_progress_time_;
    double best_distance_ = 0.0;

    double goal_tolerance_;
    double goal_timeout_;
    double progress_timeout_;
};

#endif
