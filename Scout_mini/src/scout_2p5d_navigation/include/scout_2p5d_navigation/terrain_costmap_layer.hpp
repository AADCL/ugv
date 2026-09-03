#pragma once

#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>

#include "scout_2p5d_navigation/terrain_map.hpp"

namespace scout_2p5d_navigation {

class TerrainCostmapLayer : public costmap_2d::Layer {
 public:
  void onInitialize() override;
  void updateBounds(double robot_x, double robot_y, double robot_yaw,
                    double* min_x, double* min_y, double* max_x, double* max_y) override;
  void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j,
                   int max_i, int max_j) override;
 private:
  TerrainMap terrain_;
  bool loaded_ = false;
  bool unknown_as_lethal_ = false;
};

}  // namespace scout_2p5d_navigation
