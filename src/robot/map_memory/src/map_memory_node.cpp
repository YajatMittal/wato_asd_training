#include "map_memory_node.hpp"

#include <chrono>
#include <cmath>

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  double resolution = this->declare_parameter("resolution", 0.1);
  double size = this->declare_parameter("size", 40.0);
  double update_period = this->declare_parameter("update_period", 1.0);
  update_distance_ = this->declare_parameter("update_distance", 1.5);

  map_memory_.initMap(resolution, size);

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  // Transient local so the planner (or foxglove) still gets the latest map if it subscribes after we published
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::QoS(1).transient_local());

  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(update_period), std::bind(&MapMemoryNode::updateMap, this));
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  // Hang on to a few, the newest one usually doesn't have odometry after it yet
  recent_costmaps_.push_back(msg);
  while (recent_costmaps_.size() > 5) {
    recent_costmaps_.pop_front();
  }
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  const auto& q = msg->pose.pose.orientation;
  double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  odom_history_.emplace_back(rclcpp::Time(msg->header.stamp),
                             Pose2D{msg->pose.pose.position.x, msg->pose.pose.position.y, yaw});
  // ~3 s of odom at 10 Hz, way more than the lag between a scan and its odometry
  while (odom_history_.size() > 30) {
    odom_history_.pop_front();
  }
  map_frame_ = msg->header.frame_id;
}

bool MapMemoryNode::poseAt(const rclcpp::Time& t, Pose2D& pose) const {
  // Only interpolate, never guess past the ends. The map never forgets a cost, so a costmap
  // placed in the wrong spot would stay wrong forever.
  if (odom_history_.size() < 2 || t < odom_history_.front().first || t > odom_history_.back().first) {
    return false;
  }

  // Find the two samples either side of t
  size_t i = 1;
  while (i < odom_history_.size() - 1 && odom_history_[i].first < t) {
    ++i;
  }
  const auto& [t0, p0] = odom_history_[i - 1];
  const auto& [t1, p1] = odom_history_[i];

  double span = (t1 - t0).seconds();
  double ratio = span > 1e-6 ? (t - t0).seconds() / span : 1.0;
  double dyaw = std::remainder(p1.yaw - p0.yaw, 2.0 * M_PI);  // the short way round

  pose.x = p0.x + ratio * (p1.x - p0.x);
  pose.y = p0.y + ratio * (p1.y - p0.y);
  pose.yaw = p0.yaw + ratio * dyaw;
  return true;
}

void MapMemoryNode::updateMap() {
  // Place the costmap using where the robot was when the scan was taken, not where it is now.
  // Otherwise anything far away gets smeared across the map whenever the robot is turning.
  // Use the newest costmap we have odometry on both sides of.
  nav_msgs::msg::OccupancyGrid::SharedPtr costmap;
  Pose2D pose;
  for (auto it = recent_costmaps_.rbegin(); it != recent_costmaps_.rend(); ++it) {
    if (poseAt(rclcpp::Time((*it)->header.stamp), pose)) {
      costmap = *it;
      break;
    }
  }
  if (!costmap) {
    return;
  }

  // Always fuse the first costmap so the planner has a map straight away,
  // after that only once we've moved far enough for it to be worth it
  double moved = std::hypot(pose.x - last_x_, pose.y - last_y_);
  if (has_fused_ && moved < update_distance_) {
    return;
  }

  map_memory_.fuse(*costmap, pose.x, pose.y, pose.yaw);
  has_fused_ = true;
  last_x_ = pose.x;
  last_y_ = pose.y;

  nav_msgs::msg::OccupancyGrid map = map_memory_.map();
  map.header.stamp = costmap->header.stamp;
  map.header.frame_id = map_frame_;
  map.info.map_load_time = map.header.stamp;
  map_pub_->publish(map);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
