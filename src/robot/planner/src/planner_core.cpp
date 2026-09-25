#include "planner_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger)
: logger_(logger) {}

void PlannerCore::configure(int lethal_cost, int unknown_cost, double cost_weight, double goal_search_radius) {
  lethal_cost_ = lethal_cost;
  unknown_cost_ = unknown_cost;
  cost_weight_ = cost_weight;
  goal_search_radius_ = goal_search_radius;
}

void PlannerCore::setMap(const nav_msgs::msg::OccupancyGrid::SharedPtr& map) {
  map_ = map;
}

std::vector<geometry_msgs::msg::Point> PlannerCore::plan(const geometry_msgs::msg::Point& start,
                                                         const geometry_msgs::msg::Point& goal) const {
  std::vector<geometry_msgs::msg::Point> path;
  if (!map_) {
    return path;
  }

  CellIndex start_cell;
  CellIndex goal_cell;
  if (!toCell(start, start_cell)) {
    RCLCPP_WARN(logger_, "Robot is outside the map");
    return path;
  }
  if (!toCell(goal, goal_cell)) {
    RCLCPP_WARN(logger_, "Goal (%.2f, %.2f) is outside the map", goal.x, goal.y);
    return path;
  }

  // Goal too close to something for the robot to actually stand there, aim for the nearest spot it can
  bool goal_moved = false;
  if (blocked(goal_cell)) {
    CellIndex free_cell;
    if (!nearestFreeCell(goal_cell, free_cell)) {
      RCLCPP_WARN(logger_, "Goal is inside an obstacle and there's no free space near it");
      return path;
    }
    goal_cell = free_cell;
    goal_moved = true;
  }

  const int width = map_->info.width;
  const int total = width * map_->info.height;
  const double resolution = map_->info.resolution;

  // The grid is small and fixed size, so plain arrays indexed by cell do the job of the
  // usual g_score / came_from hash maps and are a lot faster
  std::vector<double> g_score(total, std::numeric_limits<double>::infinity());
  std::vector<int> came_from(total, -1);
  std::vector<bool> closed(total, false);
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open;

  g_score[index(start_cell)] = 0.0;
  open.emplace(start_cell, heuristic(start_cell, goal_cell));

  static const int kDx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  static const int kDy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

  bool found = false;
  while (!open.empty()) {
    CellIndex current = open.top().index;
    open.pop();

    const int current_idx = index(current);
    if (closed[current_idx]) {
      continue;  // old duplicate, we push again when a cell improves instead of updating it in place
    }
    closed[current_idx] = true;

    if (current == goal_cell) {
      found = true;
      break;
    }

    const int current_cost = cost(current);
    // If we're already inside the inflated zone (robot ended up too close to something), let A*
    // walk out of it through cells that are no worse than this one instead of failing right away
    const bool escaping = current_cost >= lethal_cost_;

    for (int i = 0; i < 8; ++i) {
      CellIndex next(current.x + kDx[i], current.y + kDy[i]);
      if (!inBounds(next)) {
        continue;
      }
      const int next_idx = index(next);
      if (closed[next_idx]) {
        continue;
      }

      const int next_cost = cost(next);
      if (next_cost >= lethal_cost_ && (!escaping || next_cost > current_cost)) {
        continue;
      }

      const bool diagonal = kDx[i] != 0 && kDy[i] != 0;
      if (diagonal && !escaping &&
          (blocked(CellIndex(current.x + kDx[i], current.y)) || blocked(CellIndex(current.x, current.y + kDy[i])))) {
        continue;  // don't cut corners past obstacles
      }

      // Distance, scaled up by how costly the cell is. The inflation gradient is what pushes the
      // path towards the middle of gaps instead of skimming along the edge of the lethal zone.
      double step = (diagonal ? M_SQRT2 : 1.0) * resolution;
      double tentative = g_score[current_idx] + step * (1.0 + cost_weight_ * next_cost / 100.0);

      if (tentative < g_score[next_idx]) {
        g_score[next_idx] = tentative;
        came_from[next_idx] = current_idx;
        open.emplace(next, tentative + heuristic(next, goal_cell));
      }
    }
  }

  if (!found) {
    RCLCPP_WARN(logger_, "A* couldn't find a path to (%.2f, %.2f)", goal.x, goal.y);
    return path;
  }

  for (int idx = index(goal_cell); idx != -1; idx = came_from[idx]) {
    path.push_back(toPoint(CellIndex(idx % width, idx / width)));
  }
  std::reverse(path.begin(), path.end());

  // Finish exactly on the requested goal rather than the middle of its cell
  if (!goal_moved) {
    path.back() = goal;
  }
  return path;
}

bool PlannerCore::toCell(const geometry_msgs::msg::Point& point, CellIndex& cell) const {
  cell.x = static_cast<int>(std::floor((point.x - map_->info.origin.position.x) / map_->info.resolution));
  cell.y = static_cast<int>(std::floor((point.y - map_->info.origin.position.y) / map_->info.resolution));
  return inBounds(cell);
}

geometry_msgs::msg::Point PlannerCore::toPoint(const CellIndex& cell) const {
  geometry_msgs::msg::Point point;
  point.x = map_->info.origin.position.x + (cell.x + 0.5) * map_->info.resolution;
  point.y = map_->info.origin.position.y + (cell.y + 0.5) * map_->info.resolution;
  return point;
}

bool PlannerCore::inBounds(const CellIndex& cell) const {
  return cell.x >= 0 && cell.y >= 0 &&
         cell.x < static_cast<int>(map_->info.width) && cell.y < static_cast<int>(map_->info.height);
}

int PlannerCore::index(const CellIndex& cell) const {
  return cell.y * static_cast<int>(map_->info.width) + cell.x;
}

int PlannerCore::cost(const CellIndex& cell) const {
  int8_t value = map_->data[index(cell)];
  return value < 0 ? unknown_cost_ : value;
}

bool PlannerCore::blocked(const CellIndex& cell) const {
  return !inBounds(cell) || cost(cell) >= lethal_cost_;
}

// Octile distance, the exact shortest path length on an empty 8-connected grid. Every step
// costs at least its length, so this never overestimates and A* stays optimal.
double PlannerCore::heuristic(const CellIndex& a, const CellIndex& b) const {
  double dx = std::abs(a.x - b.x);
  double dy = std::abs(a.y - b.y);
  return map_->info.resolution * ((dx + dy) + (M_SQRT2 - 2.0) * std::min(dx, dy));
}

bool PlannerCore::nearestFreeCell(const CellIndex& from, CellIndex& result) const {
  const int reach = static_cast<int>(std::ceil(goal_search_radius_ / map_->info.resolution));
  double best = std::numeric_limits<double>::infinity();

  for (int dy = -reach; dy <= reach; ++dy) {
    for (int dx = -reach; dx <= reach; ++dx) {
      double distance = std::hypot(dx, dy);
      CellIndex cell(from.x + dx, from.y + dy);
      if (distance <= reach && distance < best && !blocked(cell)) {
        best = distance;
        result = cell;
      }
    }
  }
  return std::isfinite(best);
}

}
