#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    // Square map centred on the world origin, everything starts out unknown
    void initMap(double resolution, double size);

    // Merges a costmap into the global map. (x, y, yaw) is where the costmap's frame was in the
    // world when its scan was taken.
    void fuse(const nav_msgs::msg::OccupancyGrid& costmap, double x, double y, double yaw);

    const nav_msgs::msg::OccupancyGrid& map() const { return map_; }

  private:
    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid map_;
};

}

#endif
