#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

namespace {

using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;

struct Cell {
  std::vector<float> samples;
  float candidate = std::numeric_limits<float>::quiet_NaN();
  bool connected = false;
};

double percentile(std::vector<float>* values, double fraction) {
  if (values->empty()) return std::numeric_limits<double>::quiet_NaN();
  fraction = std::max(0.0, std::min(1.0, fraction));
  const std::size_t selected = static_cast<std::size_t>(std::round(
      fraction * static_cast<double>(values->size() - 1)));
  std::nth_element(values->begin(), values->begin() + selected, values->end());
  return (*values)[selected];
}

std::vector<std::pair<int, int>> diskOffsets(int radius) {
  std::vector<std::pair<int, int>> offsets;
  for (int dy = -radius; dy <= radius; ++dy) {
    for (int dx = -radius; dx <= radius; ++dx) {
      if (dx * dx + dy * dy <= radius * radius) {
        offsets.emplace_back(dx, dy);
      }
    }
  }
  return offsets;
}

}  // namespace

class TerrainReclassifier {
 public:
  TerrainReclassifier() : pnh_("~") {
    loadParameters();
    loadClouds();
    buildGrid();
    connectFloor();
    classify();
    save();
  }

 private:
  void loadParameters() {
    pnh_.param<std::string>("input_ground_pcd", input_ground_path_, "");
    pnh_.param<std::string>("input_obstacle_pcd", input_obstacle_path_, "");
    pnh_.param<std::string>("output_ground_pcd", output_ground_path_, "");
    pnh_.param<std::string>("output_obstacle_pcd", output_obstacle_path_, "");
    pnh_.param("resolution", resolution_, 0.05);
    pnh_.param("padding_m", padding_m_, 0.50);
    pnh_.param("ground_candidate_percentile", candidate_percentile_, 0.20);
    pnh_.param("seed_x", seed_x_, 0.0);
    pnh_.param("seed_y", seed_y_, 0.0);
    pnh_.param("seed_ground_z", seed_ground_z_, -0.15);
    pnh_.param("seed_radius_m", seed_radius_m_, 1.50);
    pnh_.param("seed_height_tolerance_m", seed_height_tolerance_m_, 0.12);
    pnh_.param("connect_radius_m", connect_radius_m_, 0.50);
    pnh_.param("max_ground_slope_deg", max_ground_slope_deg_, 22.0);
    pnh_.param("max_ground_step_m", max_ground_step_m_, 0.01);
    pnh_.param("ground_reference_radius_m", ground_reference_radius_m_, 0.35);
    pnh_.param("min_obstacle_relative_height_m",
                min_obstacle_relative_height_m_, 0.04);
    pnh_.param("max_obstacle_relative_height_m",
                max_obstacle_relative_height_m_, 1.50);
    pnh_.param("min_connected_ground_cells", min_connected_ground_cells_, 100);

    if (input_ground_path_.empty() || input_obstacle_path_.empty() ||
        output_ground_path_.empty() || output_obstacle_path_.empty()) {
      throw std::runtime_error("Terrain reclassifier PCD paths are required");
    }
    if (resolution_ <= 0.0 || connect_radius_m_ < resolution_ ||
        ground_reference_radius_m_ < resolution_) {
      throw std::runtime_error("Terrain reclassifier grid parameters are invalid");
    }
    if (min_obstacle_relative_height_m_ < 0.0 ||
        max_obstacle_relative_height_m_ <= min_obstacle_relative_height_m_) {
      throw std::runtime_error("Terrain reclassifier height band is invalid");
    }
    connect_offsets_ = diskOffsets(std::max(
        1, static_cast<int>(std::ceil(connect_radius_m_ / resolution_))));
    reference_offsets_ = diskOffsets(std::max(
        1, static_cast<int>(std::ceil(
               ground_reference_radius_m_ / resolution_))));
  }

  void loadClouds() {
    if (pcl::io::loadPCDFile<Point>(input_ground_path_, observed_ground_) < 0 ||
        pcl::io::loadPCDFile<Point>(input_obstacle_path_, observed_obstacles_) < 0) {
      throw std::runtime_error("Failed to load terrain observation PCD files");
    }
    if (observed_ground_.empty() && observed_obstacles_.empty()) {
      throw std::runtime_error("Terrain observation PCD files are empty");
    }
    observed_all_.reserve(observed_ground_.size() + observed_obstacles_.size());
    observed_all_ += observed_ground_;
    observed_all_ += observed_obstacles_;
  }

  bool finite(const Point& point) const {
    return std::isfinite(point.x) && std::isfinite(point.y) &&
           std::isfinite(point.z);
  }

  int index(int x, int y) const { return y * width_ + x; }

  bool inside(int x, int y) const {
    return x >= 0 && y >= 0 && x < width_ && y < height_;
  }

  bool pointCell(const Point& point, int* x, int* y) const {
    *x = static_cast<int>(std::floor((point.x - min_x_) / resolution_));
    *y = static_cast<int>(std::floor((point.y - min_y_) / resolution_));
    return inside(*x, *y);
  }

  void buildGrid() {
    min_x_ = std::numeric_limits<double>::infinity();
    min_y_ = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    for (const Point& point : observed_all_.points) {
      if (!finite(point)) continue;
      min_x_ = std::min(min_x_, static_cast<double>(point.x));
      min_y_ = std::min(min_y_, static_cast<double>(point.y));
      max_x = std::max(max_x, static_cast<double>(point.x));
      max_y = std::max(max_y, static_cast<double>(point.y));
    }
    if (!std::isfinite(min_x_) || !std::isfinite(min_y_)) {
      throw std::runtime_error("Terrain observations contain no finite points");
    }
    min_x_ -= padding_m_;
    min_y_ -= padding_m_;
    max_x += padding_m_;
    max_y += padding_m_;
    width_ = std::max(1, static_cast<int>(std::ceil((max_x - min_x_) / resolution_)));
    height_ = std::max(1, static_cast<int>(std::ceil((max_y - min_y_) / resolution_)));
    cells_.resize(static_cast<std::size_t>(width_) * height_);

    // Patchwork labels can conflict in indoor scenes. The lower envelope of all
    // observations is a better floor candidate: a ceiling is selected only when
    // no lower return exists and is then rejected by seed connectivity.
    for (const Point& point : observed_all_.points) {
      if (!finite(point)) continue;
      int x = 0;
      int y = 0;
      if (!pointCell(point, &x, &y)) continue;
      cells_[index(x, y)].samples.push_back(point.z);
    }
    candidate_cells_ = 0;
    for (Cell& cell : cells_) {
      if (cell.samples.empty()) continue;
      cell.candidate = static_cast<float>(
          percentile(&cell.samples, candidate_percentile_));
      std::vector<float>().swap(cell.samples);
      ++candidate_cells_;
    }
  }

  void connectFloor() {
    std::deque<int> queue;
    const double seed_radius_sq = seed_radius_m_ * seed_radius_m_;
    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < width_; ++x) {
        Cell& cell = cells_[index(x, y)];
        if (!std::isfinite(cell.candidate)) continue;
        const double world_x = min_x_ + (x + 0.5) * resolution_;
        const double world_y = min_y_ + (y + 0.5) * resolution_;
        const double distance_sq = (world_x - seed_x_) * (world_x - seed_x_) +
                                   (world_y - seed_y_) * (world_y - seed_y_);
        if (distance_sq <= seed_radius_sq &&
            std::abs(cell.candidate - seed_ground_z_) <= seed_height_tolerance_m_) {
          cell.connected = true;
          queue.push_back(index(x, y));
        }
      }
    }
    seed_cells_ = queue.size();
    if (queue.empty()) {
      throw std::runtime_error("No floor seed was found near the mapping origin");
    }

    const double slope = std::tan(max_ground_slope_deg_ * M_PI / 180.0);
    while (!queue.empty()) {
      const int current_index = queue.front();
      queue.pop_front();
      const int current_x = current_index % width_;
      const int current_y = current_index / width_;
      const float current_z = cells_[current_index].candidate;
      for (const auto& offset : connect_offsets_) {
        const int dx = offset.first;
        const int dy = offset.second;
        if (dx == 0 && dy == 0) continue;
        const int x = current_x + dx;
        const int y = current_y + dy;
        if (!inside(x, y)) continue;
        Cell& neighbor = cells_[index(x, y)];
        if (neighbor.connected || !std::isfinite(neighbor.candidate)) continue;
        const double distance = resolution_ * std::hypot(dx, dy);
        const double allowed_height = max_ground_step_m_ + slope * distance;
        if (std::abs(neighbor.candidate - current_z) > allowed_height) continue;
        neighbor.connected = true;
        queue.push_back(index(x, y));
      }
    }

    connected_cells_ = 0;
    for (const Cell& cell : cells_) {
      connected_cells_ += cell.connected ? 1 : 0;
    }
    if (connected_cells_ < static_cast<std::size_t>(min_connected_ground_cells_)) {
      throw std::runtime_error("Connected floor is too small; check seed height and map origin");
    }
  }

  bool referenceGround(int x, int y, double* ground_z) const {
    std::vector<float> heights;
    heights.reserve(reference_offsets_.size());
    for (const auto& offset : reference_offsets_) {
      const int nx = x + offset.first;
      const int ny = y + offset.second;
      if (!inside(nx, ny)) continue;
      const Cell& cell = cells_[index(nx, ny)];
      if (cell.connected) heights.push_back(cell.candidate);
    }
    if (heights.empty()) return false;
    const std::size_t middle = heights.size() / 2;
    std::nth_element(heights.begin(), heights.begin() + middle, heights.end());
    *ground_z = heights[middle];
    return true;
  }

  void classify() {
    clean_ground_.reserve(connected_cells_);
    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < width_; ++x) {
        const Cell& cell = cells_[index(x, y)];
        if (!cell.connected) continue;
        Point point;
        point.x = static_cast<float>(min_x_ + (x + 0.5) * resolution_);
        point.y = static_cast<float>(min_y_ + (y + 0.5) * resolution_);
        point.z = cell.candidate;
        point.intensity = 0.0f;
        clean_ground_.push_back(point);
      }
    }

    clean_obstacles_.reserve(observed_all_.size() / 4);
    for (const Point& point : observed_all_.points) {
      if (!finite(point)) continue;
      int x = 0;
      int y = 0;
      if (!pointCell(point, &x, &y)) continue;
      double ground_z = 0.0;
      if (!referenceGround(x, y, &ground_z)) {
        ++points_without_ground_;
        continue;
      }
      const double relative_height = point.z - ground_z;
      if (relative_height < min_obstacle_relative_height_m_) {
        ++points_below_obstacle_band_;
        continue;
      }
      if (relative_height > max_obstacle_relative_height_m_) {
        ++points_above_obstacle_band_;
        continue;
      }
      clean_obstacles_.push_back(point);
    }
    clean_ground_.width = clean_ground_.size();
    clean_ground_.height = 1;
    clean_ground_.is_dense = false;
    clean_obstacles_.width = clean_obstacles_.size();
    clean_obstacles_.height = 1;
    clean_obstacles_.is_dense = false;
  }

  void save() const {
    if (pcl::io::savePCDFileBinary(output_ground_path_, clean_ground_) != 0 ||
        pcl::io::savePCDFileBinary(output_obstacle_path_, clean_obstacles_) != 0) {
      throw std::runtime_error("Failed to save reclassified terrain PCD files");
    }
    ROS_INFO(
        "Terrain reclassified: observed=%zu candidate_cells=%zu seeds=%zu "
        "connected_ground=%zu obstacles=%zu no_ground=%zu below=%zu above=%zu",
        observed_all_.size(), candidate_cells_, seed_cells_, connected_cells_,
        clean_obstacles_.size(), points_without_ground_,
        points_below_obstacle_band_, points_above_obstacle_band_);
    ROS_INFO("Ground output: %s", output_ground_path_.c_str());
    ROS_INFO("Obstacle output: %s", output_obstacle_path_.c_str());
  }

  ros::NodeHandle pnh_;
  std::string input_ground_path_;
  std::string input_obstacle_path_;
  std::string output_ground_path_;
  std::string output_obstacle_path_;
  double resolution_ = 0.05;
  double padding_m_ = 0.50;
  double candidate_percentile_ = 0.20;
  double seed_x_ = 0.0;
  double seed_y_ = 0.0;
  double seed_ground_z_ = -0.15;
  double seed_radius_m_ = 1.50;
  double seed_height_tolerance_m_ = 0.12;
  double connect_radius_m_ = 0.50;
  double max_ground_slope_deg_ = 22.0;
  double max_ground_step_m_ = 0.01;
  double ground_reference_radius_m_ = 0.35;
  double min_obstacle_relative_height_m_ = 0.04;
  double max_obstacle_relative_height_m_ = 1.50;
  int min_connected_ground_cells_ = 100;

  Cloud observed_ground_;
  Cloud observed_obstacles_;
  Cloud observed_all_;
  Cloud clean_ground_;
  Cloud clean_obstacles_;
  std::vector<Cell> cells_;
  std::vector<std::pair<int, int>> connect_offsets_;
  std::vector<std::pair<int, int>> reference_offsets_;
  int width_ = 0;
  int height_ = 0;
  double min_x_ = 0.0;
  double min_y_ = 0.0;
  std::size_t candidate_cells_ = 0;
  std::size_t seed_cells_ = 0;
  std::size_t connected_cells_ = 0;
  std::size_t points_without_ground_ = 0;
  std::size_t points_below_obstacle_band_ = 0;
  std::size_t points_above_obstacle_band_ = 0;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "wheeltec_terrain_reclassify");
  try {
    TerrainReclassifier reclassifier;
  } catch (const std::exception& error) {
    ROS_FATAL("Terrain reclassification failed: %s", error.what());
    return 1;
  }
  return 0;
}
