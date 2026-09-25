#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

// 2D grid index
struct CellIndex
{
  int x;
  int y;

  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex &other) const
  {
    return (x == other.x && y == other.y);
  }

  bool operator!=(const CellIndex &other) const
  {
    return (x != other.x || y != other.y);
  }
};

// Structure representing a node in the A* open set
struct AStarNode
{
  CellIndex index;
  double f_score;  // f = g + h

  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

// Comparator for the priority queue (min-heap by f_score)
struct CompareF
{
  bool operator()(const AStarNode &a, const AStarNode &b)
  {
    // We want the node with the smallest f_score on top
    return a.f_score > b.f_score;
  }
};

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    void configure(int lethal_cost, int unknown_cost, double cost_weight, double goal_search_radius);

    void setMap(const nav_msgs::msg::OccupancyGrid::SharedPtr& map);
    bool hasMap() const { return map_ != nullptr; }

    // A* from start to goal, both in the map frame. Returns the path as a list of points,
    // or an empty list if the goal can't be reached.
    std::vector<geometry_msgs::msg::Point> plan(const geometry_msgs::msg::Point& start,
                                                const geometry_msgs::msg::Point& goal) const;

  private:
    bool toCell(const geometry_msgs::msg::Point& point, CellIndex& cell) const;
    geometry_msgs::msg::Point toPoint(const CellIndex& cell) const;
    bool inBounds(const CellIndex& cell) const;
    int index(const CellIndex& cell) const;
    int cost(const CellIndex& cell) const;
    bool blocked(const CellIndex& cell) const;
    double heuristic(const CellIndex& a, const CellIndex& b) const;
    bool nearestFreeCell(const CellIndex& from, CellIndex& result) const;

    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid::SharedPtr map_;

    int lethal_cost_ = 50;
    int unknown_cost_ = 10;
    double cost_weight_ = 3.0;
    double goal_search_radius_ = 1.5;
};

}

#endif
