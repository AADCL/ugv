#include "wheeltec_2p5d_navigation/terrain_map.hpp"

#include <fstream>
#include <stdexcept>
#include <type_traits>

#include <boost/filesystem.hpp>
#include <yaml-cpp/yaml.h>

namespace wheeltec_2p5d_navigation {
namespace {

template <typename T>
void writeBinary(const boost::filesystem::path& path, const std::vector<T>& data) {
  static_assert(std::is_trivially_copyable<T>::value, "binary layer type required");
  std::ofstream stream(path.string(), std::ios::binary | std::ios::trunc);
  if (!stream) throw std::runtime_error("Cannot write terrain layer: " + path.string());
  stream.write(reinterpret_cast<const char*>(data.data()),
               static_cast<std::streamsize>(data.size() * sizeof(T)));
  if (!stream) throw std::runtime_error("Incomplete terrain layer write: " + path.string());
}

template <typename T>
std::vector<T> readBinary(const boost::filesystem::path& path, std::size_t count) {
  std::ifstream stream(path.string(), std::ios::binary);
  if (!stream) throw std::runtime_error("Cannot read terrain layer: " + path.string());
  std::vector<T> data(count);
  stream.read(reinterpret_cast<char*>(data.data()),
              static_cast<std::streamsize>(count * sizeof(T)));
  if (stream.gcount() != static_cast<std::streamsize>(count * sizeof(T)))
    throw std::runtime_error("Terrain layer size mismatch: " + path.string());
  return data;
}

boost::filesystem::path layerPath(const boost::filesystem::path& yaml_path,
                                  const YAML::Node& layers,
                                  const std::string& name) {
  if (!layers[name]) throw std::runtime_error("Missing terrain layer: " + name);
  return yaml_path.parent_path() / layers[name].as<std::string>();
}
}  // namespace

bool TerrainMap::valid() const {
  return width > 0 && height > 0 && resolution > 0.0 &&
         elevation.size() == size() && slope_deg.size() == size() &&
         roughness.size() == size() && step_height.size() == size() &&
         cost.size() == size() && confidence.size() == size();
}

bool TerrainMap::worldToMap(double wx, double wy, uint32_t* mx, uint32_t* my) const {
  if (wx < origin_x || wy < origin_y) return false;
  const auto x = static_cast<uint32_t>((wx - origin_x) / resolution);
  const auto y = static_cast<uint32_t>((wy - origin_y) / resolution);
  if (x >= width || y >= height) return false;
  *mx = x;
  *my = y;
  return true;
}

void TerrainMap::mapToWorld(uint32_t mx, uint32_t my, double* wx, double* wy) const {
  *wx = origin_x + (static_cast<double>(mx) + 0.5) * resolution;
  *wy = origin_y + (static_cast<double>(my) + 0.5) * resolution;
}

TerrainMap loadTerrainMap(const std::string& yaml_path_string) {
  const boost::filesystem::path yaml_path(yaml_path_string);
  const YAML::Node root = YAML::LoadFile(yaml_path_string);
  if (!root["format"] || root["format"].as<std::string>() != "wheeltec_terrain_2p5d")
    throw std::runtime_error("Unsupported 2.5D terrain map format");
  TerrainMap map;
  map.frame_id = root["frame_id"].as<std::string>("map");
  map.resolution = root["resolution"].as<double>();
  map.width = root["width"].as<uint32_t>();
  map.height = root["height"].as<uint32_t>();
  map.origin_x = root["origin"][0].as<double>();
  map.origin_y = root["origin"][1].as<double>();
  const YAML::Node layers = root["layers"];
  map.elevation = readBinary<float>(layerPath(yaml_path, layers, "elevation"), map.size());
  map.slope_deg = readBinary<float>(layerPath(yaml_path, layers, "slope_deg"), map.size());
  map.roughness = readBinary<float>(layerPath(yaml_path, layers, "roughness"), map.size());
  map.step_height = readBinary<float>(layerPath(yaml_path, layers, "step_height"), map.size());
  map.cost = readBinary<uint8_t>(layerPath(yaml_path, layers, "cost"), map.size());
  map.confidence = readBinary<uint8_t>(layerPath(yaml_path, layers, "confidence"), map.size());
  if (!map.valid()) throw std::runtime_error("Invalid 2.5D terrain map");
  return map;
}

void saveTerrainMap(const TerrainMap& map, const std::string& yaml_path_string) {
  if (!map.valid()) throw std::runtime_error("Refusing to save invalid 2.5D terrain map");
  const boost::filesystem::path yaml_path(yaml_path_string);
  boost::filesystem::create_directories(yaml_path.parent_path());
  const std::string stem = yaml_path.stem().string();
  const std::string elevation = stem + "_elevation.f32";
  const std::string slope = stem + "_slope_deg.f32";
  const std::string roughness = stem + "_roughness.f32";
  const std::string step = stem + "_step_height.f32";
  const std::string cost = stem + "_cost.u8";
  const std::string confidence = stem + "_confidence.u8";
  writeBinary(yaml_path.parent_path() / elevation, map.elevation);
  writeBinary(yaml_path.parent_path() / slope, map.slope_deg);
  writeBinary(yaml_path.parent_path() / roughness, map.roughness);
  writeBinary(yaml_path.parent_path() / step, map.step_height);
  writeBinary(yaml_path.parent_path() / cost, map.cost);
  writeBinary(yaml_path.parent_path() / confidence, map.confidence);

  YAML::Emitter out;
  out << YAML::BeginMap
      << YAML::Key << "format" << YAML::Value << "wheeltec_terrain_2p5d"
      << YAML::Key << "version" << YAML::Value << 1
      << YAML::Key << "frame_id" << YAML::Value << map.frame_id
      << YAML::Key << "resolution" << YAML::Value << map.resolution
      << YAML::Key << "width" << YAML::Value << map.width
      << YAML::Key << "height" << YAML::Value << map.height
      << YAML::Key << "origin" << YAML::Value << YAML::Flow
      << YAML::BeginSeq << map.origin_x << map.origin_y << 0.0 << YAML::EndSeq
      << YAML::Key << "unknown_cost" << YAML::Value << 255
      << YAML::Key << "lethal_cost" << YAML::Value << 254
      << YAML::Key << "layers" << YAML::Value << YAML::BeginMap
      << YAML::Key << "elevation" << YAML::Value << elevation
      << YAML::Key << "slope_deg" << YAML::Value << slope
      << YAML::Key << "roughness" << YAML::Value << roughness
      << YAML::Key << "step_height" << YAML::Value << step
      << YAML::Key << "cost" << YAML::Value << cost
      << YAML::Key << "confidence" << YAML::Value << confidence
      << YAML::EndMap << YAML::EndMap;
  std::ofstream stream(yaml_path_string, std::ios::trunc);
  if (!stream) throw std::runtime_error("Cannot write terrain metadata: " + yaml_path_string);
  stream << out.c_str() << '\n';
}

}  // namespace wheeltec_2p5d_navigation
