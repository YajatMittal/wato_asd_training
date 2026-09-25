#include "map_memory_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger)
  : logger_(logger) {}

void MapMemoryCore::initMap(double resolution, double size) {
  int cells = static_cast<int>(std::round(size / resolution));
  map_.info.resolution = resolution;
  map_.info.width = cells;
  map_.info.height = cells;
  map_.info.origin.position.x = -0.5 * cells * resolution;
  map_.info.origin.position.y = -0.5 * cells * resolution;
  map_.info.origin.orientation.w = 1.0;
  map_.data.assign(cells * cells, -1);
}

void MapMemoryCore::fuse(const nav_msgs::msg::OccupancyGrid& costmap, double x, double y, double yaw) {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);

  const double local_res = costmap.info.resolution;
  const double local_ox = costmap.info.origin.position.x;
  const double local_oy = costmap.info.origin.position.y;
  const int local_w = costmap.info.width;
  const int local_h = costmap.info.height;

  const double res = map_.info.resolution;
  const double ox = map_.info.origin.position.x;
  const double oy = map_.info.origin.position.y;
  const int w = map_.info.width;
  const int h = map_.info.height;

  // World bounding box of the rotated costmap so we only loop over the part of the map it covers
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
  for (double lx : {local_ox, local_ox + local_w * local_res}) {
    for (double ly : {local_oy, local_oy + local_h * local_res}) {
      double wx = x + c * lx - s * ly;
      double wy = y + s * lx + c * ly;
      min_x = std::min(min_x, wx);
      max_x = std::max(max_x, wx);
      min_y = std::min(min_y, wy);
      max_y = std::max(max_y, wy);
    }
  }
  int x_begin = std::max(0, static_cast<int>(std::floor((min_x - ox) / res)));
  int x_end = std::min(w - 1, static_cast<int>(std::floor((max_x - ox) / res)));
  int y_begin = std::max(0, static_cast<int>(std::floor((min_y - oy) / res)));
  int y_end = std::min(h - 1, static_cast<int>(std::floor((max_y - oy) / res)));

  // Loop over map cells and look up the costmap cell underneath each one, instead of pushing
  // costmap cells into the map. That way every map cell gets exactly one value, so there are no
  // holes when the costmap is rotated or when the two grids have different resolutions.
  int updated = 0;
  for (int gy = y_begin; gy <= y_end; ++gy) {
    for (int gx = x_begin; gx <= x_end; ++gx) {
      // centre of this map cell relative to the robot, rotated into the costmap's frame
      double dx = ox + (gx + 0.5) * res - x;
      double dy = oy + (gy + 0.5) * res - y;
      double lx = c * dx + s * dy;
      double ly = -s * dx + c * dy;

      int cx = static_cast<int>(std::floor((lx - local_ox) / local_res));
      int cy = static_cast<int>(std::floor((ly - local_oy) / local_res));
      if (cx < 0 || cy < 0 || cx >= local_w || cy >= local_h) {
        continue;
      }

      int8_t value = costmap.data[cy * local_w + cx];
      if (value < 0) {
        continue;  // costmap doesn't know anything here, keep what we remembered
      }

      // Keep the higher of the old and new cost rather than just overwriting. A costmap cost only
      // accounts for obstacles that one scan could see, so when a beam skims past the far side of
      // something it marks cells free that are really right next to it. Overwriting would let
      // those beams wipe out inflation we already built up from other angles (the obstacles here
      // don't move, so a higher cost from earlier is still true). Free still replaces unknown.
      int8_t& cell = map_.data[gy * w + gx];
      if (value > cell) {
        cell = value;
        ++updated;
      }
    }
  }

  RCLCPP_DEBUG(logger_, "Fused costmap at (%.2f, %.2f), %d cells updated", x, y, updated);
}

}
