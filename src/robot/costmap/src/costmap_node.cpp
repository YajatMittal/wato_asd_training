#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  double resolution = this->declare_parameter("resolution", 0.1);
  double size = this->declare_parameter("size", 30.0);
  double inflation_radius = this->declare_parameter("inflation_radius", 2.0);
  int max_cost = this->declare_parameter("max_cost", 100);
  costmap_.configure(resolution, size, inflation_radius, max_cost);

  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::lidarCallback, this, std::placeholders::_1));
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

void CostmapNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  if (!costmap_.update(*msg)) {
    return;
  }
  // Keep the scan's frame and stamp, map memory needs to know exactly where the robot was for this scan
  costmap_pub_->publish(costmap_.toMsg(msg->header));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
