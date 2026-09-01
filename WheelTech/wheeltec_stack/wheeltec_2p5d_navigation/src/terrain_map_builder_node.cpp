#include "wheeltec_2p5d_navigation/terrain_map.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

namespace wt = wheeltec_2p5d_navigation;
namespace {
using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;
constexpr uint8_t kLethal = 254;
constexpr uint8_t kUnknown = 255;

float median(std::vector<float> values) {
  if (values.empty()) return std::numeric_limits<float>::quiet_NaN();
  const std::size_t middle = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + middle, values.end());
  return values[middle];
}

float percentile(std::vector<float> values, double fraction) {
  if (values.empty()) return std::numeric_limits<float>::quiet_NaN();
  fraction = std::max(0.0, std::min(1.0, fraction));
  const std::size_t index = static_cast<std::size_t>(
      std::round(fraction * static_cast<double>(values.size() - 1)));
  std::nth_element(values.begin(), values.begin() + index, values.end());
  return values[index];
}

double clamp01(double value) { return std::max(0.0, std::min(1.0, value)); }
}  // namespace

class TerrainMapBuilder {
 public:
  TerrainMapBuilder() : pnh_("~") {
    pnh_.param<std::string>("ground_pcd", ground_path_, "");
    pnh_.param<std::string>("obstacle_pcd", obstacle_path_, "");
    pnh_.param<std::string>("output_yaml", output_yaml_, "");
    pnh_.param<std::string>("frame_id", map_.frame_id, "map");
    pnh_.param("resolution", map_.resolution, 0.10);
    pnh_.param("padding_m", padding_m_, 0.50);
    pnh_.param("min_points_per_cell", min_points_per_cell_, 1);
    pnh_.param("ground_height_percentile", ground_percentile_, 0.20);
    pnh_.param("max_fill_radius_cells", max_fill_radius_, 4);
    pnh_.param("min_fill_neighbors", min_fill_neighbors_, 3);
    pnh_.param("fit_radius_m", fit_radius_m_, 0.30);
    pnh_.param("smooth_height_delta_m", smooth_height_delta_, 0.12);
    pnh_.param("preferred_slope_deg", preferred_slope_deg_, 4.0);
    pnh_.param("max_slope_deg", max_slope_deg_, 22.0);
    pnh_.param("max_step_height_m", max_step_height_, 0.08);
    pnh_.param("max_roughness_m", max_roughness_, 0.050);
    pnh_.param("obstacle_min_relative_height_m", obstacle_min_relative_height_, 0.08);
    pnh_.param("obstacle_max_relative_height_m", obstacle_max_relative_height_, 1.50);
    pnh_.param("min_obstacle_points_per_cell", min_obstacle_points_, 3);
    pnh_.param("min_lethal_neighbors", min_lethal_neighbors_, 4);
    pnh_.param("slope_cost_weight", slope_weight_, 0.55);
    pnh_.param("roughness_cost_weight", roughness_weight_, 0.20);
    pnh_.param("step_cost_weight", step_weight_, 0.25);
    pnh_.param("obstacle_inflation_m", obstacle_inflation_m_, 0.05);
    if (ground_path_.empty() || obstacle_path_.empty() || output_yaml_.empty())
      throw std::runtime_error("ground_pcd, obstacle_pcd and output_yaml are required");
    if (map_.resolution <= 0.0 || max_slope_deg_ <= preferred_slope_deg_)
      throw std::runtime_error("invalid 2.5D map parameters");
    build();
  }

 private:
  int cell(double x, double y) const {
    const int mx = static_cast<int>(std::floor((x - map_.origin_x) / map_.resolution));
    const int my = static_cast<int>(std::floor((y - map_.origin_y) / map_.resolution));
    if (mx < 0 || my < 0 || mx >= static_cast<int>(map_.width) ||
        my >= static_cast<int>(map_.height)) return -1;
    return static_cast<int>(map_.index(mx, my));
  }

  void build() {
    Cloud ground;
    Cloud obstacles;
    if (pcl::io::loadPCDFile<Point>(ground_path_, ground) < 0)
      throw std::runtime_error("failed to load ground PCD: " + ground_path_);
    if (pcl::io::loadPCDFile<Point>(obstacle_path_, obstacles) < 0)
      throw std::runtime_error("failed to load obstacle PCD: " + obstacle_path_);
    if (ground.empty()) throw std::runtime_error("ground PCD is empty");

    double min_x = std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    const auto bounds = [&](const Cloud& cloud) {
      for (const auto& p : cloud) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y)) continue;
        min_x = std::min(min_x, static_cast<double>(p.x));
        min_y = std::min(min_y, static_cast<double>(p.y));
        max_x = std::max(max_x, static_cast<double>(p.x));
        max_y = std::max(max_y, static_cast<double>(p.y));
      }
    };
    bounds(ground);
    bounds(obstacles);
    map_.origin_x = min_x - padding_m_;
    map_.origin_y = min_y - padding_m_;
    map_.width = std::max(1, static_cast<int>(std::ceil(
        (max_x - min_x + 2.0 * padding_m_) / map_.resolution)));
    map_.height = std::max(1, static_cast<int>(std::ceil(
        (max_y - min_y + 2.0 * padding_m_) / map_.resolution)));
    const std::size_t count = map_.size();
    std::vector<std::vector<float>> samples(count);
    std::vector<std::vector<float>> obstacle_samples(count);
    for (const auto& p : ground) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
      const int id = cell(p.x, p.y);
      if (id >= 0) samples[id].push_back(p.z);
    }
    for (const auto& p : obstacles) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y)) continue;
      const int id = cell(p.x, p.y);
      if (id >= 0 && std::isfinite(p.z)) obstacle_samples[id].push_back(p.z);
    }

    const float nan = std::numeric_limits<float>::quiet_NaN();
    map_.elevation.assign(count, nan);
    map_.slope_deg.assign(count, nan);
    map_.roughness.assign(count, nan);
    map_.step_height.assign(count, nan);
    map_.cost.assign(count, kUnknown);
    map_.confidence.assign(count, 0);
    for (std::size_t i = 0; i < count; ++i) {
      if (static_cast<int>(samples[i].size()) < min_points_per_cell_) continue;
      map_.elevation[i] = percentile(samples[i], ground_percentile_);
      map_.confidence[i] = static_cast<uint8_t>(std::min<std::size_t>(255, samples[i].size() * 32));
    }

    // Fill only small holes. A height gate smooths a ramp without blending across a stair edge.
    for (int pass = 0; pass < max_fill_radius_; ++pass) {
      std::vector<float> next = map_.elevation;
      std::vector<uint8_t> next_confidence = map_.confidence;
      for (uint32_t y = 0; y < map_.height; ++y) {
        for (uint32_t x = 0; x < map_.width; ++x) {
          const std::size_t id = map_.index(x, y);
          if (std::isfinite(map_.elevation[id])) continue;
          std::vector<float> neighbor;
          for (int dy = -1; dy <= 1; ++dy) for (int dx = -1; dx <= 1; ++dx) {
            const int nx = static_cast<int>(x) + dx;
            const int ny = static_cast<int>(y) + dy;
            if (nx < 0 || ny < 0 || nx >= static_cast<int>(map_.width) ||
                ny >= static_cast<int>(map_.height) || (dx == 0 && dy == 0)) continue;
            const float z = map_.elevation[map_.index(nx, ny)];
            if (std::isfinite(z)) neighbor.push_back(z);
          }
          if (static_cast<int>(neighbor.size()) >= min_fill_neighbors_) {
            next[id] = median(neighbor);
            next_confidence[id] = 1;
          }
        }
      }
      map_.elevation.swap(next);
      map_.confidence.swap(next_confidence);
    }

    const int radius = std::max(1, static_cast<int>(std::round(fit_radius_m_ / map_.resolution)));
    for (uint32_t y = 0; y < map_.height; ++y) {
      for (uint32_t x = 0; x < map_.width; ++x) evaluateCell(x, y, radius);
    }

    // Reject isolated lethal classifications caused by sparse returns or pose
    // noise. A stair/ridge forms a continuous band and survives this test;
    // a single unsupported cell remains traversable at maximum soft cost.
    const std::vector<uint8_t> raw_cost = map_.cost;
    for (uint32_t y = 0; y < map_.height; ++y) {
      for (uint32_t x = 0; x < map_.width; ++x) {
        const std::size_t id = map_.index(x, y);
        if (raw_cost[id] != kLethal) continue;
        int lethal_neighbors = 0;
        for (int dy = -1; dy <= 1; ++dy) for (int dx = -1; dx <= 1; ++dx) {
          const int nx = static_cast<int>(x) + dx;
          const int ny = static_cast<int>(y) + dy;
          if (nx < 0 || ny < 0 || nx >= static_cast<int>(map_.width) ||
              ny >= static_cast<int>(map_.height)) continue;
          lethal_neighbors += raw_cost[map_.index(nx, ny)] == kLethal ? 1 : 0;
        }
        if (lethal_neighbors < min_lethal_neighbors_) map_.cost[id] = 252;
      }
    }

    // A nonground return is an obstacle only when it rises above the local
    // ground surface. This rejects ceiling/low outliers and legacy marker
    // points that do not form a persistent vertical object.
    std::vector<uint8_t> obstacles_mask(count, 0);
    for (std::size_t id = 0; id < count; ++id) {
      if (!std::isfinite(map_.elevation[id])) continue;
      int hits = 0;
      for (const float z : obstacle_samples[id]) {
        const double relative = z - map_.elevation[id];
        if (relative >= obstacle_min_relative_height_ &&
            relative <= obstacle_max_relative_height_) ++hits;
      }
      if (hits >= min_obstacle_points_) obstacles_mask[id] = 1;
    }

    const int obstacle_radius = std::max(0, static_cast<int>(std::ceil(
        obstacle_inflation_m_ / map_.resolution)));
    for (uint32_t y = 0; y < map_.height; ++y) {
      for (uint32_t x = 0; x < map_.width; ++x) {
        bool lethal = false;
        for (int dy = -obstacle_radius; dy <= obstacle_radius && !lethal; ++dy) {
          for (int dx = -obstacle_radius; dx <= obstacle_radius; ++dx) {
            if (dx * dx + dy * dy > obstacle_radius * obstacle_radius) continue;
            const int nx = static_cast<int>(x) + dx;
            const int ny = static_cast<int>(y) + dy;
            if (nx >= 0 && ny >= 0 && nx < static_cast<int>(map_.width) &&
                ny < static_cast<int>(map_.height) && obstacles_mask[map_.index(nx, ny)]) {
              lethal = true;
              break;
            }
          }
        }
        if (lethal) map_.cost[map_.index(x, y)] = kLethal;
      }
    }

    wt::saveTerrainMap(map_, output_yaml_);
    const auto known = std::count_if(map_.cost.begin(), map_.cost.end(),
                                     [](uint8_t c) { return c != kUnknown; });
    const auto lethal = std::count(map_.cost.begin(), map_.cost.end(), kLethal);
    ROS_INFO("Saved 2.5D map %ux%u @ %.3fm: known=%zu lethal=%zu -> %s",
             map_.width, map_.height, map_.resolution, known, lethal,
             output_yaml_.c_str());
  }

  void evaluateCell(uint32_t x, uint32_t y, int radius) {
    const std::size_t id = map_.index(x, y);
    const float center = map_.elevation[id];
    if (!std::isfinite(center)) return;
    std::vector<Eigen::Vector3d> rows;
    std::vector<double> z_values;
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dx = -radius; dx <= radius; ++dx) {
        if (dx * dx + dy * dy > radius * radius) continue;
        const int nx = static_cast<int>(x) + dx;
        const int ny = static_cast<int>(y) + dy;
        if (nx < 0 || ny < 0 || nx >= static_cast<int>(map_.width) ||
            ny >= static_cast<int>(map_.height)) continue;
        const float z = map_.elevation[map_.index(nx, ny)];
        if (!std::isfinite(z)) continue;
        const double delta = std::abs(z - center);
        if (delta <= smooth_height_delta_)
          rows.emplace_back(dx * map_.resolution, dy * map_.resolution, 1.0);
        if (delta <= smooth_height_delta_) z_values.push_back(z);
      }
    }
    if (rows.size() < 4) return;
    Eigen::MatrixXd a(rows.size(), 3);
    Eigen::VectorXd b(rows.size());
    for (std::size_t i = 0; i < rows.size(); ++i) {
      a.row(i) = rows[i].transpose();
      b(i) = z_values[i];
    }
    const Eigen::Vector3d plane = a.colPivHouseholderQr().solve(b);
    const double rmse = std::sqrt((a * plane - b).squaredNorm() / rows.size());
    const double slope = std::atan(std::hypot(plane.x(), plane.y())) * 180.0 / M_PI;
    // Step height is the adjacent-cell residual from the fitted local ramp,
    // not the raw height delta. A continuous incline is therefore allowed,
    // while a stair edge remains a discontinuity after plane compensation.
    double max_step = 0.0;
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) continue;
        const int nx = static_cast<int>(x) + dx;
        const int ny = static_cast<int>(y) + dy;
        if (nx < 0 || ny < 0 || nx >= static_cast<int>(map_.width) ||
            ny >= static_cast<int>(map_.height)) continue;
        const float z = map_.elevation[map_.index(nx, ny)];
        if (!std::isfinite(z)) continue;
        const double predicted = plane.x() * dx * map_.resolution +
                                 plane.y() * dy * map_.resolution + plane.z();
        max_step = std::max(max_step, std::abs(z - predicted));
      }
    }
    map_.slope_deg[id] = slope;
    map_.roughness[id] = rmse;
    map_.step_height[id] = max_step;
    if (slope > max_slope_deg_ || max_step > max_step_height_ || rmse > max_roughness_) {
      map_.cost[id] = kLethal;
      return;
    }
    const double slope_score = clamp01((slope - preferred_slope_deg_) /
                                       (max_slope_deg_ - preferred_slope_deg_));
    const double rough_score = clamp01(rmse / max_roughness_);
    const double step_score = clamp01(max_step / max_step_height_);
    const double score = slope_weight_ * slope_score +
                         roughness_weight_ * rough_score + step_weight_ * step_score;
    map_.cost[id] = static_cast<uint8_t>(std::round(252.0 * clamp01(score)));
  }

  ros::NodeHandle pnh_;
  wt::TerrainMap map_;
  std::string ground_path_;
  std::string obstacle_path_;
  std::string output_yaml_;
  double padding_m_ = 0.5;
  int min_points_per_cell_ = 1;
  double ground_percentile_ = 0.20;
  int max_fill_radius_ = 4;
  int min_fill_neighbors_ = 3;
  double fit_radius_m_ = 0.30;
  double smooth_height_delta_ = 0.12;
  double preferred_slope_deg_ = 4.0;
  double max_slope_deg_ = 22.0;
  double max_step_height_ = 0.08;
  double max_roughness_ = 0.050;
  double obstacle_min_relative_height_ = 0.08;
  double obstacle_max_relative_height_ = 1.50;
  int min_obstacle_points_ = 3;
  int min_lethal_neighbors_ = 4;
  double slope_weight_ = 0.55;
  double roughness_weight_ = 0.20;
  double step_weight_ = 0.25;
  double obstacle_inflation_m_ = 0.05;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "wheeltec_terrain_map_builder");
  try {
    TerrainMapBuilder builder;
  } catch (const std::exception& error) {
    ROS_FATAL("2.5D map build failed: %s", error.what());
    return 1;
  }
  return 0;
}
