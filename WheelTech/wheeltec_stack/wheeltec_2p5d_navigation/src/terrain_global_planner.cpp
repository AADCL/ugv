#include "wheeltec_2p5d_navigation/terrain_global_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>

#include <costmap_2d/cost_values.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace wheeltec_2p5d_navigation {
namespace {
struct QueueNode {
  std::size_t index;
  double score;
  bool operator<(const QueueNode& other) const { return score > other.score; }
};
}  // namespace

void TerrainGlobalPlanner::initialize(std::string name,
                                      costmap_2d::Costmap2DROS* costmap_ros) {
  if (initialized_) return;
  ros::NodeHandle private_nh("~/" + name);
  std::string terrain_map_yaml;
  private_nh.param<std::string>("terrain_map_yaml", terrain_map_yaml, "");
  if (terrain_map_yaml.empty()) {
    ros::param::get(ros::this_node::getName() + "/terrain_map_yaml",
                    terrain_map_yaml);
  }
  private_nh.param("terrain_weight", terrain_weight_, 4.0);
  private_nh.param("uphill_weight", uphill_weight_, 2.0);
  private_nh.param("downhill_weight", downhill_weight_, 1.0);
  private_nh.param("max_edge_step_m", max_edge_step_m_, 0.08);
  private_nh.param("footprint_radius_m", footprint_radius_m_, 0.32);
  private_nh.param("snap_radius_m", snap_radius_m_, 0.50);
  if (terrain_map_yaml.empty())
    throw std::runtime_error("TerrainGlobalPlanner requires terrain_map_yaml");
  terrain_ = loadTerrainMap(terrain_map_yaml);
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  footprint_radius_cells_ = std::max(0, static_cast<int>(std::ceil(
      footprint_radius_m_ / terrain_.resolution)));
  snap_radius_cells_ = std::max(0, static_cast<int>(std::ceil(
      snap_radius_m_ / terrain_.resolution)));
  plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1, true);
  initialized_ = true;
  ROS_INFO("TerrainGlobalPlanner loaded %ux%u 2.5D cells from %s",
           terrain_.width, terrain_.height, terrain_map_yaml.c_str());
}

bool TerrainGlobalPlanner::cellCost(uint32_t x, uint32_t y, uint8_t* output) const {
  uint8_t maximum = 0;
  for (int dy = -footprint_radius_cells_; dy <= footprint_radius_cells_; ++dy) {
    for (int dx = -footprint_radius_cells_; dx <= footprint_radius_cells_; ++dx) {
      if (dx * dx + dy * dy > footprint_radius_cells_ * footprint_radius_cells_) continue;
      const int nx = static_cast<int>(x) + dx;
      const int ny = static_cast<int>(y) + dy;
      if (nx < 0 || ny < 0 || nx >= static_cast<int>(terrain_.width) ||
          ny >= static_cast<int>(terrain_.height)) return false;
      const uint8_t value = terrain_.cost[terrain_.index(nx, ny)];
      if (value >= costmap_2d::LETHAL_OBSTACLE) return false;
      maximum = std::max(maximum, value);
    }
  }
  double wx = 0.0;
  double wy = 0.0;
  terrain_.mapToWorld(x, y, &wx, &wy);
  unsigned int cx = 0;
  unsigned int cy = 0;
  if (costmap_ && costmap_->worldToMap(wx, wy, cx, cy)) {
    const uint8_t value = costmap_->getCost(cx, cy);
    if (value == costmap_2d::LETHAL_OBSTACLE || value == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      return false;
  }
  *output = maximum;
  return true;
}

bool TerrainGlobalPlanner::nearestValid(uint32_t input_x, uint32_t input_y,
                                        uint32_t* output_x, uint32_t* output_y) const {
  uint8_t ignored = 0;
  if (cellCost(input_x, input_y, &ignored)) {
    *output_x = input_x;
    *output_y = input_y;
    return true;
  }
  for (int radius = 1; radius <= snap_radius_cells_; ++radius) {
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dx = -radius; dx <= radius; ++dx) {
        if (std::max(std::abs(dx), std::abs(dy)) != radius) continue;
        const int x = static_cast<int>(input_x) + dx;
        const int y = static_cast<int>(input_y) + dy;
        if (x < 0 || y < 0 || x >= static_cast<int>(terrain_.width) ||
            y >= static_cast<int>(terrain_.height)) continue;
        if (cellCost(x, y, &ignored)) {
          *output_x = x;
          *output_y = y;
          return true;
        }
      }
    }
  }
  return false;
}

bool TerrainGlobalPlanner::makePlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    std::vector<geometry_msgs::PoseStamped>& plan) {
  plan.clear();
  if (!initialized_) return false;
  if (start.header.frame_id != terrain_.frame_id || goal.header.frame_id != terrain_.frame_id) {
    ROS_ERROR("2.5D planner requires poses in %s (start=%s goal=%s)",
              terrain_.frame_id.c_str(), start.header.frame_id.c_str(), goal.header.frame_id.c_str());
    return false;
  }
  uint32_t sx, sy, gx, gy;
  if (!terrain_.worldToMap(start.pose.position.x, start.pose.position.y, &sx, &sy) ||
      !terrain_.worldToMap(goal.pose.position.x, goal.pose.position.y, &gx, &gy)) {
    ROS_WARN("2.5D plan start or goal lies outside the terrain map");
    return false;
  }
  if (!nearestValid(sx, sy, &sx, &sy) || !nearestValid(gx, gy, &gx, &gy)) {
    ROS_WARN("2.5D plan start or goal has no traversable cell nearby");
    return false;
  }
  const std::size_t count = terrain_.size();
  const std::size_t start_id = terrain_.index(sx, sy);
  const std::size_t goal_id = terrain_.index(gx, gy);
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<double> distance(count, inf);
  std::vector<int64_t> parent(count, -1);
  std::vector<uint8_t> closed(count, 0);
  std::priority_queue<QueueNode> open;
  distance[start_id] = 0.0;
  open.push({start_id, 0.0});
  const int offsets[8][2] = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};
  while (!open.empty()) {
    const std::size_t current = open.top().index;
    open.pop();
    if (closed[current]) continue;
    closed[current] = 1;
    if (current == goal_id) break;
    const int x = static_cast<int>(current % terrain_.width);
    const int y = static_cast<int>(current / terrain_.width);
    for (const auto& offset : offsets) {
      const int nx = x + offset[0];
      const int ny = y + offset[1];
      if (nx < 0 || ny < 0 || nx >= static_cast<int>(terrain_.width) ||
          ny >= static_cast<int>(terrain_.height)) continue;
      uint8_t terrain_cost = 0;
      if (!cellCost(nx, ny, &terrain_cost)) continue;
      const std::size_t next = terrain_.index(nx, ny);
      if (closed[next]) continue;
      const double dz = terrain_.elevation[next] - terrain_.elevation[current];
      if (!std::isfinite(dz)) continue;
      if ((std::isfinite(terrain_.step_height[next]) &&
           terrain_.step_height[next] > max_edge_step_m_) &&
          terrain_.cost[next] >= costmap_2d::LETHAL_OBSTACLE) continue;
      const double horizontal = terrain_.resolution *
          ((offset[0] != 0 && offset[1] != 0) ? std::sqrt(2.0) : 1.0);
      const double terrain_factor = 1.0 + terrain_weight_ * terrain_cost / 252.0;
      const double vertical = dz >= 0.0 ? uphill_weight_ * dz : downhill_weight_ * -dz;
      const double candidate = distance[current] + horizontal * terrain_factor + vertical;
      if (candidate >= distance[next]) continue;
      distance[next] = candidate;
      parent[next] = static_cast<int64_t>(current);
      const double hx = static_cast<double>(nx) - gx;
      const double hy = static_cast<double>(ny) - gy;
      open.push({next, candidate + terrain_.resolution * std::hypot(hx, hy)});
    }
  }
  if (parent[goal_id] < 0 && goal_id != start_id) {
    ROS_WARN("2.5D planner found no traversable path");
    return false;
  }
  std::vector<std::size_t> reversed;
  for (std::size_t id = goal_id;; id = static_cast<std::size_t>(parent[id])) {
    reversed.push_back(id);
    if (id == start_id) break;
    if (parent[id] < 0) return false;
  }
  std::reverse(reversed.begin(), reversed.end());
  plan.reserve(reversed.size());
  const ros::Time stamp = ros::Time::now();
  for (std::size_t i = 0; i < reversed.size(); ++i) {
    const std::size_t id = reversed[i];
    const uint32_t x = id % terrain_.width;
    const uint32_t y = id / terrain_.width;
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = terrain_.frame_id;
    pose.header.stamp = stamp;
    terrain_.mapToWorld(x, y, &pose.pose.position.x, &pose.pose.position.y);
    pose.pose.position.z = terrain_.elevation[id];
    double yaw = 0.0;
    if (i + 1 < reversed.size()) {
      double next_x, next_y;
      terrain_.mapToWorld(reversed[i + 1] % terrain_.width,
                          reversed[i + 1] / terrain_.width, &next_x, &next_y);
      yaw = std::atan2(next_y - pose.pose.position.y, next_x - pose.pose.position.x);
    }
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    pose.pose.orientation = tf2::toMsg(q);
    plan.push_back(pose);
  }
  if (!plan.empty()) plan.back().pose.orientation = goal.pose.orientation;
  nav_msgs::Path path;
  path.header.frame_id = terrain_.frame_id;
  path.header.stamp = stamp;
  path.poses = plan;
  plan_pub_.publish(path);
  ROS_INFO("2.5D plan: %zu poses, terrain cost %.3f", plan.size(), distance[goal_id]);
  return true;
}

}  // namespace wheeltec_2p5d_navigation

PLUGINLIB_EXPORT_CLASS(wheeltec_2p5d_navigation::TerrainGlobalPlanner,
                       nav_core::BaseGlobalPlanner)
