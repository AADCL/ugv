#pragma once

#include <string>
#include <utility>
#include <vector>

#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include "wheeltec_2p5d_navigation/terrain_map.hpp"

namespace wheeltec_2p5d_navigation {

class TerrainGlobalPlanner : public nav_core::BaseGlobalPlanner {
 public:
  TerrainGlobalPlanner() = default;
  TerrainGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    initialize(std::move(name), costmap_ros);
  }
  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;
  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan) override;

 private:
  bool nearestValid(uint32_t input_x, uint32_t input_y, uint32_t* output_x,
                    uint32_t* output_y) const;
  bool cellCost(uint32_t x, uint32_t y, uint8_t* cost) const;
  bool initialized_ = false;
  TerrainMap terrain_;
  costmap_2d::Costmap2DROS* costmap_ros_ = nullptr;
  costmap_2d::Costmap2D* costmap_ = nullptr;
  ros::Publisher plan_pub_;
  double terrain_weight_ = 4.0;
  double uphill_weight_ = 2.0;
  double downhill_weight_ = 1.0;
  double max_edge_step_m_ = 0.08;
  double footprint_radius_m_ = 0.32;
  double snap_radius_m_ = 0.50;
  int footprint_radius_cells_ = 3;
  int snap_radius_cells_ = 5;
};

}  // namespace wheeltec_2p5d_navigation
