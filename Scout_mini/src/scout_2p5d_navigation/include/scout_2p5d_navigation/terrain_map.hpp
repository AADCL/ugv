#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace scout_2p5d_navigation {

struct TerrainMap {
  std::string frame_id = "map";
  double resolution = 0.10;
  double origin_x = 0.0;
  double origin_y = 0.0;
  uint32_t width = 0;
  uint32_t height = 0;
  std::vector<float> elevation;
  std::vector<float> slope_deg;
  std::vector<float> roughness;
  std::vector<float> step_height;
  std::vector<uint8_t> cost;
  std::vector<uint8_t> confidence;

  std::size_t size() const { return static_cast<std::size_t>(width) * height; }
  bool valid() const;
  bool worldToMap(double wx, double wy, uint32_t* mx, uint32_t* my) const;
  void mapToWorld(uint32_t mx, uint32_t my, double* wx, double* wy) const;
  std::size_t index(uint32_t mx, uint32_t my) const {
    return static_cast<std::size_t>(my) * width + mx;
  }
};

TerrainMap loadTerrainMap(const std::string& yaml_path);
void saveTerrainMap(const TerrainMap& map, const std::string& yaml_path);

}  // namespace scout_2p5d_navigation
