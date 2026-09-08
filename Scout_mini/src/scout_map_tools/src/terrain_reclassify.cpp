#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/approximate_progressive_morphological_filter.h>
#include <ros/ros.h>

namespace {

using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;

struct Plane {
  double a = 0.0;
  double b = 0.0;
  double c = 0.0;
  double rmse = std::numeric_limits<double>::infinity();
  int inliers = 0;
};

struct Cell {
  std::vector<float> samples;
  float candidate = std::numeric_limits<float>::quiet_NaN();
  bool trusted_seed = false;
  bool valid_candidate = false;
  bool connected = false;
  Plane plane;
  float surface_z = std::numeric_limits<float>::quiet_NaN();
};

struct Observation {
  Eigen::Vector3d row;
  double z = 0.0;
  double base_weight = 1.0;
};

bool finite(const Point& point) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z);
}

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
    loadInput();
    buildTrustedSeeds();
    buildCandidateGrid();
    validateCandidates();
    connectSurface();
    buildSurface();
    filterSurfaceComponents();
    classifyObstacles();
    saveOutputs();
  }

 private:
  void loadParameters() {
    pnh_.param<std::string>("input_pcd", input_path_, "");
    pnh_.param<std::string>("trusted_seed_pcd", trusted_seed_path_, "");
    pnh_.param<std::string>("output_candidate_pcd", output_candidate_path_, "");
    pnh_.param<std::string>("output_ground_pcd", output_ground_path_, "");
    pnh_.param<std::string>("output_obstacle_pcd", output_obstacle_path_, "");
    pnh_.param("cell_size", cell_size_, 0.05);
    pnh_.param("candidate_percentile", candidate_percentile_, 0.05);
    pnh_.param("seed_x", seed_x_, 0.0);
    pnh_.param("seed_y", seed_y_, 0.0);
    pnh_.param("seed_ground_z", seed_ground_z_, -0.15);
    pnh_.param("seed_radius_m", seed_radius_m_, 1.0);
    pnh_.param("seed_height_tolerance_m", seed_height_tolerance_m_, 0.12);
    pnh_.param("trusted_seed_height_tolerance_m",
               trusted_seed_height_tolerance_m_, 0.08);
    pnh_.param("pmf/max_window_size", pmf_max_window_size_, 101);
    pnh_.param("pmf/slope", pmf_slope_, 0.40);
    pnh_.param("pmf/initial_distance_m", pmf_initial_distance_m_, 0.04);
    pnh_.param("pmf/max_distance_m", pmf_max_distance_m_, 0.18);
    pnh_.param("pmf/base", pmf_base_, 2.0);
    pnh_.param("pmf/exponential", pmf_exponential_, true);
    pnh_.param("validation_radius_m", validation_radius_m_, 0.30);
    pnh_.param("validation_min_candidates", validation_min_candidates_, 8);
    pnh_.param("plane_inlier_tolerance_m", plane_inlier_tolerance_m_, 0.05);
    pnh_.param("plane_min_inlier_ratio", plane_min_inlier_ratio_, 0.55);
    pnh_.param("plane_min_spread_m", plane_min_spread_m_, 0.04);
    pnh_.param("candidate_plane_tolerance_m", candidate_plane_tolerance_m_, 0.05);
    pnh_.param("plane_max_rmse_m", plane_max_rmse_m_, 0.035);
    pnh_.param("max_ground_slope_deg", max_ground_slope_deg_, 25.0);
    pnh_.param("connect_radius_m", connect_radius_m_, 0.25);
    pnh_.param("connection_plane_tolerance_m", connection_plane_tolerance_m_, 0.05);
    pnh_.param("connection_max_normal_delta_deg",
               connection_max_normal_delta_deg_, 20.0);
    pnh_.param("max_ground_step_m", max_ground_step_m_, 0.015);
    pnh_.param("surface_fit_radius_m", surface_fit_radius_m_, 0.80);
    pnh_.param("surface_observation_radius_m",
               surface_observation_radius_m_, 0.15);
    pnh_.param("surface_gap_fill_radius_m",
               surface_gap_fill_radius_m_, 0.30);
    pnh_.param("surface_min_candidates", surface_min_candidates_, 4);
    pnh_.param("surface_min_component_cells",
               surface_min_component_cells_, 100);
    pnh_.param("surface_min_component_fraction",
               surface_min_component_fraction_, 0.03);
    pnh_.param("min_obstacle_relative_height_m",
               min_obstacle_relative_height_m_, 0.08);
    pnh_.param("max_obstacle_relative_height_m",
               max_obstacle_relative_height_m_, 1.50);
    pnh_.param("min_connected_ground_cells", min_connected_ground_cells_, 100);

    if (input_path_.empty() || output_ground_path_.empty() ||
        output_obstacle_path_.empty()) {
      throw std::runtime_error("Input, ground and obstacle PCD paths are required");
    }
    if (cell_size_ <= 0.0 || candidate_percentile_ < 0.0 ||
        candidate_percentile_ > 1.0 || seed_radius_m_ < cell_size_ ||
        trusted_seed_height_tolerance_m_ <= 0.0 ||
        pmf_max_window_size_ < 3 || pmf_slope_ < 0.0 ||
        pmf_initial_distance_m_ < 0.0 ||
        pmf_max_distance_m_ < pmf_initial_distance_m_ || pmf_base_ < 1.0 ||
        validation_radius_m_ < cell_size_ || validation_min_candidates_ < 3 ||
        plane_inlier_tolerance_m_ <= 0.0 || plane_min_inlier_ratio_ <= 0.0 ||
        plane_min_inlier_ratio_ > 1.0 || plane_min_spread_m_ <= 0.0 ||
        candidate_plane_tolerance_m_ <= 0.0 || plane_max_rmse_m_ <= 0.0 ||
        max_ground_slope_deg_ <= 0.0 || max_ground_slope_deg_ >= 89.0 ||
        connect_radius_m_ < cell_size_ || connection_plane_tolerance_m_ <= 0.0 ||
        connection_max_normal_delta_deg_ <= 0.0 ||
        connection_max_normal_delta_deg_ >= 90.0 || max_ground_step_m_ < 0.0 ||
        surface_fit_radius_m_ < cell_size_ ||
        surface_observation_radius_m_ < 0.0 ||
        surface_gap_fill_radius_m_ < surface_observation_radius_m_ ||
        surface_min_candidates_ < 3 || surface_min_component_cells_ < 1 ||
        surface_min_component_fraction_ < 0.0 ||
        surface_min_component_fraction_ > 1.0 ||
        min_obstacle_relative_height_m_ < 0.0 ||
        max_obstacle_relative_height_m_ <= min_obstacle_relative_height_m_) {
      throw std::runtime_error("Invalid terrain surface parameters");
    }

    validation_offsets_ = diskOffsets(std::max(
        1, static_cast<int>(std::ceil(validation_radius_m_ / cell_size_))));
    connect_offsets_ = diskOffsets(std::max(
        1, static_cast<int>(std::ceil(connect_radius_m_ / cell_size_))));
    surface_offsets_ = diskOffsets(std::max(
        1, static_cast<int>(std::ceil(surface_fit_radius_m_ / cell_size_))));
    observation_offsets_ = diskOffsets(std::max(
        0, static_cast<int>(std::ceil(
               surface_observation_radius_m_ / cell_size_))));
    gap_fill_offsets_ = diskOffsets(std::max(
        1, static_cast<int>(
               std::ceil(surface_gap_fill_radius_m_ / cell_size_))));
  }

  void loadInput() {
    input_.reset(new Cloud);
    if (pcl::io::loadPCDFile<Point>(input_path_, *input_) < 0 || input_->empty()) {
      throw std::runtime_error("Failed to load input PCD");
    }
    if (!trusted_seed_path_.empty() &&
        pcl::io::loadPCDFile<Point>(trusted_seed_path_, trusted_seeds_) < 0) {
      throw std::runtime_error("Failed to load trusted seed PCD");
    }
  }

  void buildTrustedSeeds() {
    if (!trusted_seeds_.empty()) return;
    pcl::ApproximateProgressiveMorphologicalFilter<Point> pmf;
    pmf.setInputCloud(input_);
    pmf.setCellSize(static_cast<float>(cell_size_));
    pmf.setMaxWindowSize(pmf_max_window_size_);
    pmf.setSlope(static_cast<float>(pmf_slope_));
    pmf.setInitialDistance(static_cast<float>(pmf_initial_distance_m_));
    pmf.setMaxDistance(static_cast<float>(pmf_max_distance_m_));
    pmf.setBase(static_cast<float>(pmf_base_));
    pmf.setExponential(pmf_exponential_);
    std::vector<int> indices;
    pmf.extract(indices);
    trusted_seeds_.reserve(indices.size());
    for (const int point_index : indices) {
      if (point_index < 0 ||
          static_cast<std::size_t>(point_index) >= input_->size()) {
        continue;
      }
      const Point& point = input_->points[point_index];
      if (finite(point)) trusted_seeds_.push_back(point);
    }
    if (trusted_seeds_.empty()) {
      throw std::runtime_error("PMF returned no conservative terrain seeds");
    }
  }

  int index(int x, int y) const { return y * width_ + x; }

  bool inside(int x, int y) const {
    return x >= 0 && y >= 0 && x < width_ && y < height_;
  }

  bool pointCell(const Point& point, int* x, int* y) const {
    *x = static_cast<int>(std::floor((point.x - min_x_) / cell_size_));
    *y = static_cast<int>(std::floor((point.y - min_y_) / cell_size_));
    return inside(*x, *y);
  }

  void buildCandidateGrid() {
    min_x_ = std::numeric_limits<double>::infinity();
    min_y_ = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    for (const Point& point : input_->points) {
      if (!finite(point)) continue;
      min_x_ = std::min(min_x_, static_cast<double>(point.x));
      min_y_ = std::min(min_y_, static_cast<double>(point.y));
      max_x = std::max(max_x, static_cast<double>(point.x));
      max_y = std::max(max_y, static_cast<double>(point.y));
    }
    if (!std::isfinite(min_x_) || !std::isfinite(min_y_)) {
      throw std::runtime_error("Input contains no finite points");
    }
    width_ = std::max(
        1, static_cast<int>(std::floor((max_x - min_x_) / cell_size_)) + 1);
    height_ = std::max(
        1, static_cast<int>(std::floor((max_y - min_y_) / cell_size_)) + 1);
    cells_.resize(static_cast<std::size_t>(width_) * height_);

    for (const Point& point : input_->points) {
      if (!finite(point)) continue;
      int x = 0;
      int y = 0;
      if (!pointCell(point, &x, &y)) continue;
      cells_[index(x, y)].samples.push_back(point.z);
    }
    for (Cell& cell : cells_) {
      if (cell.samples.empty()) continue;
      cell.candidate = static_cast<float>(
          percentile(&cell.samples, candidate_percentile_));
      std::vector<float>().swap(cell.samples);
      ++candidate_cells_;
    }
  }

  bool fitPlane(int center_x, int center_y,
                const std::vector<std::pair<int, int>>& offsets,
                bool connected_only, int minimum_candidates,
                bool require_center, Plane* plane) const {
    std::vector<Observation> observations;
    observations.reserve(offsets.size());
    for (const auto& offset : offsets) {
      const int x = center_x + offset.first;
      const int y = center_y + offset.second;
      if (!inside(x, y)) continue;
      const Cell& cell = cells_[index(x, y)];
      if (!std::isfinite(cell.candidate) ||
          (connected_only && !cell.connected)) {
        continue;
      }
      const double dx = offset.first * cell_size_;
      const double dy = offset.second * cell_size_;
      observations.push_back({Eigen::Vector3d(dx, dy, 1.0), cell.candidate,
                              1.0 / (0.05 + std::hypot(dx, dy))});
    }
    if (static_cast<int>(observations.size()) < minimum_candidates) return false;

    std::vector<double> robust_weight(observations.size(), 1.0);
    Eigen::Vector3d coefficients = Eigen::Vector3d::Zero();
    for (int iteration = 0; iteration < 4; ++iteration) {
      Eigen::Matrix3d normal = Eigen::Matrix3d::Zero();
      Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
      for (std::size_t i = 0; i < observations.size(); ++i) {
        const double weight = observations[i].base_weight * robust_weight[i];
        normal.noalias() += weight * observations[i].row *
                            observations[i].row.transpose();
        rhs.noalias() += weight * observations[i].row * observations[i].z;
      }
      const Eigen::LDLT<Eigen::Matrix3d> decomposition(normal);
      if (decomposition.info() != Eigen::Success) return false;
      coefficients = decomposition.solve(rhs);
      if (!coefficients.allFinite()) return false;
      for (std::size_t i = 0; i < observations.size(); ++i) {
        const double residual = std::abs(
            observations[i].row.dot(coefficients) - observations[i].z);
        robust_weight[i] = residual <= plane_inlier_tolerance_m_
                               ? 1.0
                               : plane_inlier_tolerance_m_ / residual;
      }
    }

    std::vector<std::size_t> inliers;
    inliers.reserve(observations.size());
    for (std::size_t i = 0; i < observations.size(); ++i) {
      if (std::abs(observations[i].row.dot(coefficients) - observations[i].z) <=
          plane_inlier_tolerance_m_) {
        inliers.push_back(i);
      }
    }
    if (static_cast<int>(inliers.size()) < minimum_candidates ||
        static_cast<double>(inliers.size()) / observations.size() <
            plane_min_inlier_ratio_) {
      return false;
    }

    Eigen::Matrix3d normal = Eigen::Matrix3d::Zero();
    Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
    Eigen::Vector2d mean = Eigen::Vector2d::Zero();
    double total_weight = 0.0;
    for (const std::size_t i : inliers) {
      const double weight = observations[i].base_weight;
      normal.noalias() += weight * observations[i].row *
                          observations[i].row.transpose();
      rhs.noalias() += weight * observations[i].row * observations[i].z;
      mean += weight * observations[i].row.head<2>();
      total_weight += weight;
    }
    if (total_weight <= 0.0) return false;
    mean /= total_weight;
    Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
    for (const std::size_t i : inliers) {
      const Eigen::Vector2d centered = observations[i].row.head<2>() - mean;
      covariance.noalias() += observations[i].base_weight * centered *
                              centered.transpose();
    }
    covariance /= total_weight;
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> spread_solver(covariance);
    if (spread_solver.info() != Eigen::Success ||
        spread_solver.eigenvalues().minCoeff() <
            plane_min_spread_m_ * plane_min_spread_m_) {
      return false;
    }

    const Eigen::LDLT<Eigen::Matrix3d> decomposition(normal);
    if (decomposition.info() != Eigen::Success) return false;
    coefficients = decomposition.solve(rhs);
    if (!coefficients.allFinite()) return false;

    const double slope = std::hypot(coefficients.x(), coefficients.y());
    if (slope > std::tan(max_ground_slope_deg_ * M_PI / 180.0)) return false;
    double squared_error = 0.0;
    for (const std::size_t i : inliers) {
      const double residual =
          observations[i].row.dot(coefficients) - observations[i].z;
      squared_error += observations[i].base_weight * residual * residual;
    }
    const double rmse = std::sqrt(squared_error / total_weight);
    if (rmse > plane_max_rmse_m_) return false;

    if (require_center) {
      const Cell& center = cells_[index(center_x, center_y)];
      if (!std::isfinite(center.candidate) ||
          std::abs(center.candidate - coefficients.z()) >
              candidate_plane_tolerance_m_) {
        return false;
      }
    }
    plane->a = coefficients.x();
    plane->b = coefficients.y();
    plane->c = coefficients.z();
    plane->rmse = rmse;
    plane->inliers = static_cast<int>(inliers.size());
    return true;
  }

  void validateCandidates() {
    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < width_; ++x) {
        Cell& cell = cells_[index(x, y)];
        if (!std::isfinite(cell.candidate)) continue;
        Plane plane;
        if (!fitPlane(x, y, validation_offsets_, false,
                      validation_min_candidates_, true, &plane)) {
          continue;
        }
        cell.valid_candidate = true;
        cell.plane = plane;
        ++valid_candidate_cells_;
      }
    }
  }

  bool compatible(const Cell& current, const Cell& neighbor,
                  int dx, int dy) const {
    if (!current.valid_candidate || !neighbor.valid_candidate) return false;
    const double world_dx = dx * cell_size_;
    const double world_dy = dy * cell_size_;
    const double distance = std::hypot(world_dx, world_dy);
    const double allowed_step = max_ground_step_m_ +
        std::tan(max_ground_slope_deg_ * M_PI / 180.0) * distance;
    if (std::abs(neighbor.candidate - current.candidate) > allowed_step) {
      return false;
    }
    const double expected_neighbor = current.plane.a * world_dx +
                                     current.plane.b * world_dy +
                                     current.plane.c;
    const double expected_current = neighbor.plane.a * -world_dx +
                                    neighbor.plane.b * -world_dy +
                                    neighbor.plane.c;
    if (std::abs(neighbor.candidate - expected_neighbor) >
            connection_plane_tolerance_m_ ||
        std::abs(current.candidate - expected_current) >
            connection_plane_tolerance_m_) {
      return false;
    }
    Eigen::Vector3d current_normal(-current.plane.a, -current.plane.b, 1.0);
    Eigen::Vector3d neighbor_normal(-neighbor.plane.a, -neighbor.plane.b, 1.0);
    current_normal.normalize();
    neighbor_normal.normalize();
    const double cosine = std::max(-1.0, std::min(
        1.0, current_normal.dot(neighbor_normal)));
    const double angle = std::acos(cosine) * 180.0 / M_PI;
    return angle <= connection_max_normal_delta_deg_;
  }

  void connectSurface() {
    std::deque<int> queue;
    for (const Point& point : trusted_seeds_.points) {
      if (!finite(point)) continue;
      int x = 0;
      int y = 0;
      if (!pointCell(point, &x, &y)) continue;
      Cell& cell = cells_[index(x, y)];
      if (!std::isfinite(cell.candidate) ||
          std::abs(cell.candidate - point.z) >
              trusted_seed_height_tolerance_m_) {
        continue;
      }
      if (!cell.trusted_seed) {
        cell.trusted_seed = true;
        ++trusted_seed_cells_;
      }
    }
    // The origin seed is a useful frame sanity check when local observations
    // are planar enough.  It must not be the only growth source: a saved PCD
    // contains no sensor trajectory, and a ramp edge or a single sparse scan
    // gap can make the origin cell fail strict local-plane validation.
    const double seed_radius_sq = seed_radius_m_ * seed_radius_m_;
    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < width_; ++x) {
        Cell& cell = cells_[index(x, y)];
        if (!cell.valid_candidate) continue;
        const double world_x = min_x_ + (x + 0.5) * cell_size_;
        const double world_y = min_y_ + (y + 0.5) * cell_size_;
        const double distance_sq = (world_x - seed_x_) * (world_x - seed_x_) +
                                   (world_y - seed_y_) * (world_y - seed_y_);
        if (distance_sq <= seed_radius_sq &&
            std::abs(cell.candidate - seed_ground_z_) <=
                seed_height_tolerance_m_) {
          if (!cell.connected) {
            cell.connected = true;
            queue.push_back(index(x, y));
            ++origin_seed_cells_;
          }
        }
      }
    }
    // PMF selects the locally lowest morphological surface.  All of its cells
    // provide distributed support for the later robust plane fit even when
    // their absolute Z differs greatly from the map origin.  Only locally
    // validated PMF cells are allowed to grow into neighboring candidates.
    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < width_; ++x) {
        Cell& cell = cells_[index(x, y)];
        if (!cell.trusted_seed) continue;
        ++distributed_seed_cells_;
        if (cell.connected) continue;
        cell.connected = true;
        if (cell.valid_candidate) queue.push_back(index(x, y));
      }
    }
    if (origin_seed_cells_ == 0) {
      ROS_WARN("No locally planar origin floor seed; using distributed PMF anchors");
    }
    if (queue.empty()) {
      throw std::runtime_error("No valid distributed terrain seed was found");
    }

    while (!queue.empty()) {
      const int current_index = queue.front();
      queue.pop_front();
      const int current_x = current_index % width_;
      const int current_y = current_index / width_;
      const Cell& current = cells_[current_index];
      for (const auto& offset : connect_offsets_) {
        if (offset.first == 0 && offset.second == 0) continue;
        const int x = current_x + offset.first;
        const int y = current_y + offset.second;
        if (!inside(x, y)) continue;
        Cell& neighbor = cells_[index(x, y)];
        if (neighbor.connected || !neighbor.valid_candidate) continue;
        if (!compatible(current, neighbor, offset.first, offset.second)) continue;
        neighbor.connected = true;
        queue.push_back(index(x, y));
      }
    }

    for (const Cell& cell : cells_) {
      connected_cells_ += cell.connected ? 1 : 0;
      connected_trusted_seed_cells_ +=
          cell.connected && cell.trusted_seed ? 1 : 0;
    }
    if (connected_cells_ < static_cast<std::size_t>(min_connected_ground_cells_)) {
      throw std::runtime_error(
          "Connected terrain is too small; check the seed and plane parameters");
    }
  }

  bool hasOpposingConnectedSupport(int center_x, int center_y) const {
    uint8_t sector_mask = 0;
    for (const auto& offset : gap_fill_offsets_) {
      if (offset.first == 0 && offset.second == 0) continue;
      const int x = center_x + offset.first;
      const int y = center_y + offset.second;
      if (!inside(x, y)) continue;
      const Cell& cell = cells_[index(x, y)];
      if (!cell.connected || !std::isfinite(cell.candidate)) continue;
      const double angle = std::atan2(
          static_cast<double>(offset.second),
          static_cast<double>(offset.first));
      int sector = static_cast<int>(std::floor(
          (angle + M_PI + M_PI / 8.0) / (M_PI / 4.0)));
      sector %= 8;
      if (sector < 0) sector += 8;
      sector_mask |= static_cast<uint8_t>(1u << sector);
    }
    for (int first = 0; first < 8; ++first) {
      if ((sector_mask & (1u << first)) == 0) continue;
      for (int second = first + 1; second < 8; ++second) {
        if ((sector_mask & (1u << second)) == 0) continue;
        const int separation = std::min(second - first, 8 - (second - first));
        if (separation >= 3) return true;
      }
    }
    return false;
  }

  void buildSurface() {
    ground_.reserve(connected_cells_ * 2);
    candidates_.reserve(connected_cells_);
    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < width_; ++x) {
        Cell& cell = cells_[index(x, y)];
        if (cell.connected) {
          Point candidate;
          candidate.x = static_cast<float>(min_x_ + (x + 0.5) * cell_size_);
          candidate.y = static_cast<float>(min_y_ + (y + 0.5) * cell_size_);
          candidate.z = cell.candidate;
          candidate.intensity = 0.0f;
          candidates_.push_back(candidate);
        }

        bool observed_nearby = false;
        for (const auto& offset : observation_offsets_) {
          const int nx = x + offset.first;
          const int ny = y + offset.second;
          if (inside(nx, ny) &&
              std::isfinite(cells_[index(nx, ny)].candidate)) {
            observed_nearby = true;
            break;
          }
        }
        const bool internal_gap =
            !observed_nearby && hasOpposingConnectedSupport(x, y);
        if (!observed_nearby && !internal_gap) continue;

        Plane surface;
        if (!fitPlane(x, y, surface_offsets_, true,
                      surface_min_candidates_, false, &surface)) {
          continue;
        }
        cell.surface_z = static_cast<float>(surface.c);
        Point point;
        point.x = static_cast<float>(min_x_ + (x + 0.5) * cell_size_);
        point.y = static_cast<float>(min_y_ + (y + 0.5) * cell_size_);
        point.z = cell.surface_z;
        point.intensity = 0.0f;
        ground_.push_back(point);
        if (internal_gap) ++gap_filled_surface_cells_;
      }
    }
  }

  void filterSurfaceComponents() {
    std::vector<int> labels(cells_.size(), -1);
    std::vector<std::vector<int>> components;
    const int neighbor_dx[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
    const int neighbor_dy[8] = {-1, -1, -1, 0, 0, 1, 1, 1};

    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < width_; ++x) {
        const int start = index(x, y);
        if (!std::isfinite(cells_[start].surface_z) || labels[start] >= 0) {
          continue;
        }
        const int component_id = static_cast<int>(components.size());
        components.emplace_back();
        std::deque<int> queue;
        queue.push_back(start);
        labels[start] = component_id;
        while (!queue.empty()) {
          const int current = queue.front();
          queue.pop_front();
          components.back().push_back(current);
          const int current_x = current % width_;
          const int current_y = current / width_;
          for (int direction = 0; direction < 8; ++direction) {
            const int neighbor_x = current_x + neighbor_dx[direction];
            const int neighbor_y = current_y + neighbor_dy[direction];
            if (!inside(neighbor_x, neighbor_y)) continue;
            const int neighbor = index(neighbor_x, neighbor_y);
            if (labels[neighbor] >= 0 ||
                !std::isfinite(cells_[neighbor].surface_z)) {
              continue;
            }
            labels[neighbor] = component_id;
            queue.push_back(neighbor);
          }
        }
      }
    }

    surface_component_count_ = components.size();
    if (components.empty()) {
      throw std::runtime_error("No terrain surface survived local-plane fitting");
    }
    std::size_t largest_component_cells = 0;
    for (const auto& component : components) {
      largest_component_cells = std::max(
          largest_component_cells, component.size());
    }
    const std::size_t relative_minimum = static_cast<std::size_t>(std::ceil(
        surface_min_component_fraction_ * largest_component_cells));
    const std::size_t minimum_cells = std::max(
        static_cast<std::size_t>(surface_min_component_cells_),
        relative_minimum);

    for (const auto& component : components) {
      if (component.size() >= minimum_cells) {
        ++kept_surface_component_count_;
        continue;
      }
      removed_surface_cells_ += component.size();
      for (const int cell_index : component) {
        cells_[cell_index].surface_z =
            std::numeric_limits<float>::quiet_NaN();
      }
    }

    ground_.clear();
    ground_.reserve(cells_.size() - removed_surface_cells_);
    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < width_; ++x) {
        const Cell& cell = cells_[index(x, y)];
        if (!std::isfinite(cell.surface_z)) continue;
        Point point;
        point.x = static_cast<float>(min_x_ + (x + 0.5) * cell_size_);
        point.y = static_cast<float>(min_y_ + (y + 0.5) * cell_size_);
        point.z = cell.surface_z;
        point.intensity = 0.0f;
        ground_.push_back(point);
      }
    }
  }

  void classifyObstacles() {
    obstacles_.reserve(input_->size() / 3);
    for (const Point& point : input_->points) {
      if (!finite(point)) continue;
      int x = 0;
      int y = 0;
      if (!pointCell(point, &x, &y)) continue;
      const float ground_z = cells_[index(x, y)].surface_z;
      if (!std::isfinite(ground_z)) {
        ++points_without_surface_;
        continue;
      }
      const double relative_height = point.z - ground_z;
      if (relative_height >= min_obstacle_relative_height_m_ &&
          relative_height <= max_obstacle_relative_height_m_) {
        obstacles_.push_back(point);
      }
    }
  }

  static void prepare(Cloud* cloud) {
    cloud->width = static_cast<uint32_t>(cloud->size());
    cloud->height = 1;
    cloud->is_dense = true;
  }

  void saveOutputs() {
    prepare(&candidates_);
    prepare(&ground_);
    prepare(&obstacles_);
    if ((!output_candidate_path_.empty() &&
         pcl::io::savePCDFileBinary(output_candidate_path_, candidates_) != 0) ||
        pcl::io::savePCDFileBinary(output_ground_path_, ground_) != 0 ||
        pcl::io::savePCDFileBinary(output_obstacle_path_, obstacles_) != 0) {
      throw std::runtime_error("Failed to save terrain surface PCD files");
    }
    ROS_INFO(
        "Terrain surface input=%zu pmf=%zu candidates=%zu valid=%zu "
        "origin_seeds=%zu distributed_seeds=%zu trusted_cells=%zu "
        "trusted_connected=%zu "
        "connected=%zu surface=%zu gap_filled=%zu components=%zu kept=%zu "
        "component_removed=%zu obstacles=%zu no_surface=%zu",
        input_->size(), trusted_seeds_.size(), candidate_cells_,
        valid_candidate_cells_, origin_seed_cells_, distributed_seed_cells_,
        trusted_seed_cells_,
        connected_trusted_seed_cells_,
        connected_cells_, ground_.size(), gap_filled_surface_cells_,
        surface_component_count_, kept_surface_component_count_,
        removed_surface_cells_, obstacles_.size(), points_without_surface_);
    ROS_INFO(
        "Terrain surface cell=%.3f local_radius=%.3f connect_radius=%.3f "
        "fit_radius=%.3f max_slope=%.1fdeg plane_rmse=%.3f",
        cell_size_, validation_radius_m_, connect_radius_m_,
        surface_fit_radius_m_, max_ground_slope_deg_, plane_max_rmse_m_);
  }

  ros::NodeHandle pnh_;
  std::string input_path_;
  std::string trusted_seed_path_;
  std::string output_candidate_path_;
  std::string output_ground_path_;
  std::string output_obstacle_path_;
  double cell_size_ = 0.05;
  double candidate_percentile_ = 0.05;
  double seed_x_ = 0.0;
  double seed_y_ = 0.0;
  double seed_ground_z_ = -0.15;
  double seed_radius_m_ = 1.0;
  double seed_height_tolerance_m_ = 0.12;
  double trusted_seed_height_tolerance_m_ = 0.08;
  int pmf_max_window_size_ = 101;
  double pmf_slope_ = 0.40;
  double pmf_initial_distance_m_ = 0.04;
  double pmf_max_distance_m_ = 0.18;
  double pmf_base_ = 2.0;
  bool pmf_exponential_ = true;
  double validation_radius_m_ = 0.30;
  int validation_min_candidates_ = 8;
  double plane_inlier_tolerance_m_ = 0.05;
  double plane_min_inlier_ratio_ = 0.55;
  double plane_min_spread_m_ = 0.04;
  double candidate_plane_tolerance_m_ = 0.05;
  double plane_max_rmse_m_ = 0.035;
  double max_ground_slope_deg_ = 25.0;
  double connect_radius_m_ = 0.25;
  double connection_plane_tolerance_m_ = 0.05;
  double connection_max_normal_delta_deg_ = 20.0;
  double max_ground_step_m_ = 0.015;
  double surface_fit_radius_m_ = 0.80;
  double surface_observation_radius_m_ = 0.15;
  double surface_gap_fill_radius_m_ = 0.30;
  int surface_min_candidates_ = 4;
  int surface_min_component_cells_ = 100;
  double surface_min_component_fraction_ = 0.03;
  double min_obstacle_relative_height_m_ = 0.08;
  double max_obstacle_relative_height_m_ = 1.50;
  int min_connected_ground_cells_ = 100;

  Cloud::Ptr input_;
  Cloud trusted_seeds_;
  Cloud candidates_;
  Cloud ground_;
  Cloud obstacles_;
  std::vector<Cell> cells_;
  std::vector<std::pair<int, int>> validation_offsets_;
  std::vector<std::pair<int, int>> connect_offsets_;
  std::vector<std::pair<int, int>> surface_offsets_;
  std::vector<std::pair<int, int>> observation_offsets_;
  std::vector<std::pair<int, int>> gap_fill_offsets_;
  int width_ = 0;
  int height_ = 0;
  double min_x_ = 0.0;
  double min_y_ = 0.0;
  std::size_t candidate_cells_ = 0;
  std::size_t valid_candidate_cells_ = 0;
  std::size_t origin_seed_cells_ = 0;
  std::size_t distributed_seed_cells_ = 0;
  std::size_t trusted_seed_cells_ = 0;
  std::size_t connected_trusted_seed_cells_ = 0;
  std::size_t connected_cells_ = 0;
  std::size_t gap_filled_surface_cells_ = 0;
  std::size_t surface_component_count_ = 0;
  std::size_t kept_surface_component_count_ = 0;
  std::size_t removed_surface_cells_ = 0;
  std::size_t points_without_surface_ = 0;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "scout_terrain_reclassify");
  try {
    TerrainReclassifier reclassifier;
  } catch (const std::exception& error) {
    ROS_FATAL("Terrain reclassification failed: %s", error.what());
    std::cerr << "Terrain reclassification failed: " << error.what()
              << std::endl;
    return 1;
  }
  return 0;
}
