#include "costmap_core.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}

void CostmapCore::configure(double resolution, double size, double inflation_radius, int max_cost) {
  resolution_ = resolution;
  cells_ = static_cast<int>(std::round(size / resolution));
  origin_ = -0.5 * cells_ * resolution_;
  inflation_radius_ = inflation_radius;
  max_cost_ = max_cost;
  grid_.assign(cells_ * cells_, -1);

  // cost = max_cost * (1 - distance / inflation_radius), dropping anything that works out to 0
  kernel_.clear();
  kernel_reach_ = static_cast<int>(std::ceil(inflation_radius_ / resolution_));
  for (int dy = -kernel_reach_; dy <= kernel_reach_; ++dy) {
    for (int dx = -kernel_reach_; dx <= kernel_reach_; ++dx) {
      double distance = std::hypot(dx, dy) * resolution_;
      int cost = static_cast<int>(max_cost_ * (1.0 - distance / inflation_radius_));
      if (cost > 0) {
        kernel_.push_back({dx, dy, static_cast<int8_t>(cost)});
      }
    }
  }

  RCLCPP_INFO(logger_, "Costmap is %dx%d cells at %.2f m/cell, inflating %.2f m around obstacles",
              cells_, cells_, resolution_, inflation_radius_);
}

bool CostmapCore::update(const sensor_msgs::msg::LaserScan& scan) {
  // For the first few frames after the sim starts, before the world has loaded, every beam comes
  // back as inf. Taking that literally says everything around us is free and the planner happily
  // drives into whatever is actually there, so treat a scan with no returns at all as not ready yet.
  bool any_return = std::any_of(scan.ranges.begin(), scan.ranges.end(), [&scan](float r) {
    return r >= scan.range_min && r < scan.range_max;
  });
  if (!any_return) {
    RCLCPP_WARN_ONCE(logger_, "Got a scan with no returns at all, ignoring it");
    return false;
  }

  // Start from unknown instead of free. Only cells a beam actually went through get marked free,
  // so whatever is hidden behind an obstacle stays unknown instead of being passed on to map
  // memory as explored free space.
  std::fill(grid_.begin(), grid_.end(), -1);

  const int center = cells_ / 2;
  std::vector<std::pair<int, int>> hits;
  hits.reserve(scan.ranges.size());

  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    double range = scan.ranges[i];
    if (std::isnan(range) || range < scan.range_min) {
      continue;
    }

    // No return (inf or max range) doesn't mean nothing, it still tells us the beam's path is clear
    bool hit = range < scan.range_max;
    if (!hit) {
      range = scan.range_max;
    }

    double angle = scan.angle_min + i * scan.angle_increment;
    double x = range * std::cos(angle);
    double y = range * std::sin(angle);
    int gx = static_cast<int>(std::floor((x - origin_) / resolution_));
    int gy = static_cast<int>(std::floor((y - origin_) / resolution_));

    clearRay(center, center, gx, gy);
    if (hit) {
      hits.emplace_back(gx, gy);
    }
  }

  // Obstacles go in after all the clearing, otherwise a neighbouring beam could erase them.
  // A hit can be outside the grid and still have its inflation reach in.
  for (const auto& [gx, gy] : hits) {
    inflateAround(gx, gy);
  }
  return true;
}

// Bresenham from the lidar out to the end of the beam, marking everything on the way as free
void CostmapCore::clearRay(int x0, int y0, int x1, int y1) {
  int dx = std::abs(x1 - x0);
  int dy = -std::abs(y1 - y0);
  int sx = x0 < x1 ? 1 : -1;
  int sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;

  while (true) {
    // We start in the middle and only move outwards, so once we're off the grid we're done
    if (x0 < 0 || y0 < 0 || x0 >= cells_ || y0 >= cells_) {
      return;
    }
    grid_[y0 * cells_ + x0] = 0;
    if (x0 == x1 && y0 == y1) {
      return;
    }

    int e2 = 2 * err;
    if (e2 >= dy) {
      err += dy;
      x0 += sx;
    }
    if (e2 <= dx) {
      err += dx;
      y0 += sy;
    }
  }
}

void CostmapCore::inflateAround(int cx, int cy) {
  if (cx < -kernel_reach_ || cy < -kernel_reach_ ||
      cx >= cells_ + kernel_reach_ || cy >= cells_ + kernel_reach_) {
    return;  // too far outside to touch the grid at all
  }

  for (const auto& k : kernel_) {
    int x = cx + k.dx;
    int y = cy + k.dy;
    if (x < 0 || y < 0 || x >= cells_ || y >= cells_) {
      continue;
    }
    // Only ever raise a cell. Unknown (-1) and free (0) both lose to any real cost.
    int8_t& cell = grid_[y * cells_ + x];
    cell = std::max(cell, k.cost);
  }
}

nav_msgs::msg::OccupancyGrid CostmapCore::toMsg(const std_msgs::msg::Header& header) const {
  nav_msgs::msg::OccupancyGrid msg;
  msg.header = header;
  msg.info.map_load_time = header.stamp;
  msg.info.resolution = resolution_;
  msg.info.width = cells_;
  msg.info.height = cells_;
  msg.info.origin.position.x = origin_;
  msg.info.origin.position.y = origin_;
  msg.info.origin.orientation.w = 1.0;
  msg.data = grid_;
  return msg;
}

}
