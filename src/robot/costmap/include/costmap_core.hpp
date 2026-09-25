#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include <cstdint>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/header.hpp"

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    // size is the side length of the square costmap in metres, the lidar sits in the middle of it
    void configure(double resolution, double size, double inflation_radius, int max_cost);

    // Rebuilds the whole costmap from a single scan. Returns false (and leaves the costmap
    // alone) if the scan is empty.
    bool update(const sensor_msgs::msg::LaserScan& scan);

    nav_msgs::msg::OccupancyGrid toMsg(const std_msgs::msg::Header& header) const;

  private:
    void clearRay(int x0, int y0, int x1, int y1);
    void inflateAround(int cx, int cy);

    rclcpp::Logger logger_;

    double resolution_ = 0.1;
    int cells_ = 0;        // grid is cells_ x cells_
    double origin_ = 0.0;  // x and y of the bottom left corner, in the lidar frame
    double inflation_radius_ = 1.0;
    int max_cost_ = 100;

    std::vector<int8_t> grid_;

    // Offsets around an obstacle and the cost each one gets. Worked out once in configure()
    // so we aren't doing a sqrt for every cell around every hit on every scan.
    struct KernelCell {
      int dx;
      int dy;
      int8_t cost;
    };
    std::vector<KernelCell> kernel_;
    int kernel_reach_ = 0;
};

}

#endif
