#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include <deque>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void updateMap();

  private:
    struct Pose2D {
      double x;
      double y;
      double yaw;
    };

    // Where the robot was at time t, interpolated from the odometry we've kept
    bool poseAt(const rclcpp::Time& t, Pose2D& pose) const;

    robot::MapMemoryCore map_memory_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::deque<nav_msgs::msg::OccupancyGrid::SharedPtr> recent_costmaps_;  // newest at the back
    std::deque<std::pair<rclcpp::Time, Pose2D>> odom_history_;
    std::string map_frame_;

    bool has_fused_ = false;
    double last_x_ = 0.0;
    double last_y_ = 0.0;
    double update_distance_;
};

#endif
