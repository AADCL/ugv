#include "wheeltec_2p5d_navigation/terrain_costmap_layer.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <string>

#include <costmap_2d/cost_values.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace wheeltec_2p5d_navigation {

void TerrainCostmapLayer::onInitialize() {
  ros::NodeHandle private_nh("~/" + name_);
  pnh_ = private_nh;
  std::string terrain_map_yaml;
  private_nh.param("enabled", enabled_, true);
  private_nh.param("unknown_as_lethal", unknown_as_lethal_, false);
  private_nh.param<std::string>("terrain_map_yaml", terrain_map_yaml, "");
  reload_service_ = private_nh.advertiseService(
      "reload_map", &TerrainCostmapLayer::reloadMap, this);
  bool live_mapping = false;
  private_nh.param("live_mapping", live_mapping, false);
  if (live_mapping) {
    current_ = false;
    ROS_INFO("TerrainCostmapLayer waiting for the original mapper's first map");
    return;
  }
  if (terrain_map_yaml.empty())
    throw std::runtime_error("TerrainCostmapLayer requires terrain_map_yaml");
  terrain_ = loadTerrainMap(terrain_map_yaml);
  loaded_ = true;
  current_ = true;
  ROS_INFO("TerrainCostmapLayer loaded %ux%u 2.5D cells", terrain_.width, terrain_.height);
}

bool TerrainCostmapLayer::reloadMap(std_srvs::Trigger::Request&,
                                   std_srvs::Trigger::Response& response) {
  try {
    std::string path;
    pnh_.getParam("terrain_map_yaml", path);
    auto next = loadTerrainMap(path);
    if (next.frame_id != layered_costmap_->getGlobalFrameID())
      throw std::runtime_error("terrain frame does not match global costmap");
    std::lock_guard<std::mutex> lock(map_mutex_);
    terrain_ = std::move(next);
    loaded_ = current_ = true;
    response.success = true;
    response.message = "loaded " + path;
  } catch (const std::exception& error) {
    response.success = false;
    response.message = error.what();
  }
  return true;
}

void TerrainCostmapLayer::updateBounds(double, double, double, double* min_x,
                                       double* min_y, double* max_x, double* max_y) {
  std::lock_guard<std::mutex> lock(map_mutex_);
  if (!enabled_ || !loaded_) return;
  *min_x = std::min(*min_x, terrain_.origin_x);
  *min_y = std::min(*min_y, terrain_.origin_y);
  *max_x = std::max(*max_x, terrain_.origin_x + terrain_.width * terrain_.resolution);
  *max_y = std::max(*max_y, terrain_.origin_y + terrain_.height * terrain_.resolution);
}

void TerrainCostmapLayer::updateCosts(costmap_2d::Costmap2D& master_grid,
                                      int min_i, int min_j, int max_i, int max_j) {
  std::lock_guard<std::mutex> lock(map_mutex_);
  if (!enabled_ || !loaded_) return;
  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(master_grid.getSizeInCellsX()), max_i);
  max_j = std::min(static_cast<int>(master_grid.getSizeInCellsY()), max_j);
  for (int y = min_j; y < max_j; ++y) {
    for (int x = min_i; x < max_i; ++x) {
      double wx, wy;
      master_grid.mapToWorld(x, y, wx, wy);
      uint32_t tx, ty;
      if (!terrain_.worldToMap(wx, wy, &tx, &ty)) continue;
      uint8_t cost = terrain_.cost[terrain_.index(tx, ty)];
      if (cost == costmap_2d::NO_INFORMATION) {
        if (!unknown_as_lethal_) continue;
        cost = costmap_2d::LETHAL_OBSTACLE;
      }
      const uint8_t previous = master_grid.getCost(x, y);
      // Occupancy and unknown-space ownership stays with the static PGM. The
      // terrain layer may only add slope cost to cells already known there.
      if (previous != costmap_2d::NO_INFORMATION && cost > previous)
        master_grid.setCost(x, y, cost);
    }
  }
}

}  // namespace wheeltec_2p5d_navigation

PLUGINLIB_EXPORT_CLASS(wheeltec_2p5d_navigation::TerrainCostmapLayer,
                       costmap_2d::Layer)
