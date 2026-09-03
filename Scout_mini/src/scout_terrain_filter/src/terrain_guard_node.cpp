#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <boost/bind/bind.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace {

using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;

double percentile(std::vector<float> values, double fraction) {
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  fraction = std::max(0.0, std::min(1.0, fraction));
  const std::size_t index = static_cast<std::size_t>(
      std::round(fraction * static_cast<double>(values.size() - 1)));
  std::nth_element(values.begin(), values.begin() + index, values.end());
  return values[index];
}

diagnostic_msgs::KeyValue keyValue(const std::string& key,
                                   const std::string& value) {
  diagnostic_msgs::KeyValue output;
  output.key = key;
  output.value = value;
  return output;
}

template <typename T>
std::string numberString(const T value, const int precision = 3) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

}  // namespace

class TerrainGuard {
 public:
  TerrainGuard()
      : nh_(),
        pnh_("~"),
        ground_sub_(nh_, "ground", 4),
        nonground_sub_(nh_, "nonground", 4),
        sync_(SyncPolicy(6), ground_sub_, nonground_sub_) {
    loadParameters();

    width_ = static_cast<int>(std::ceil((max_x_ - min_x_) / resolution_));
    height_ = static_cast<int>(std::ceil((max_y_ - min_y_) / resolution_));
    if (width_ <= 0 || height_ <= 0 || resolution_ <= 0.0) {
      throw std::runtime_error("Invalid terrain grid dimensions");
    }

    cells_.resize(static_cast<std::size_t>(width_ * height_));

    safe_ground_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("safe_ground", 2);
    unsafe_ground_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>("unsafe_ground", 2);
    obstacle_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("obstacles", 2);
    clearing_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("clearing", 2);
    unknown_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("unknown", 2);
    diagnostics_pub_ =
        nh_.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 2);

    if (!metrics_csv_.empty()) {
      metrics_.open(metrics_csv_, std::ios::out | std::ios::trunc);
      if (!metrics_) {
        throw std::runtime_error("Unable to open metrics CSV: " + metrics_csv_);
      }
      metrics_ << "stamp,input,patchwork_ground,patchwork_nonground,safe_ground,"
                  "unsafe_ground,obstacles,unknown,populated_cells,safe_cells,"
                  "unsafe_cells,step_cells,slope_cells,rough_cells,mean_slope_deg,"
                  "max_slope_deg,mean_span,max_span,mean_rmse,max_rmse,"
                  "front_safe_ground,front_obstacles,processing_ms\n";
    }

    sync_.registerCallback(
        boost::bind(&TerrainGuard::callback, this, boost::placeholders::_1,
                    boost::placeholders::_2));

    ROS_INFO("Terrain guard grid %dx%d at %.3f m, slope %.1f deg, step %.3f m",
             width_, height_, resolution_, max_traversable_slope_deg_,
             max_step_height_);
  }

 private:
  struct Cell {
    std::vector<float> z_values;
    double ground_z = std::numeric_limits<double>::quiet_NaN();
    double smooth_z = std::numeric_limits<double>::quiet_NaN();
    double span = 0.0;
    double slope_deg = 0.0;
    double plane_rmse = 0.0;
    bool populated = false;
    bool safe = false;
    bool step_detected = false;
    bool excessive_slope = false;
    bool rough_surface = false;

    void reset() {
      z_values.clear();
      ground_z = std::numeric_limits<double>::quiet_NaN();
      smooth_z = std::numeric_limits<double>::quiet_NaN();
      span = 0.0;
      slope_deg = 0.0;
      plane_rmse = 0.0;
      populated = false;
      safe = false;
      step_detected = false;
      excessive_slope = false;
      rough_surface = false;
    }
  };

  using SyncPolicy = message_filters::sync_policies::ExactTime<
      sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>;

  void loadParameters() {
    pnh_.param("grid/resolution", resolution_, 0.10);
    pnh_.param("grid/min_x", min_x_, -5.0);
    pnh_.param("grid/max_x", max_x_, 5.0);
    pnh_.param("grid/min_y", min_y_, -4.0);
    pnh_.param("grid/max_y", max_y_, 4.0);
    pnh_.param("grid/neighbor_radius_cells", neighbor_radius_cells_, 2);
    pnh_.param("grid/reference_radius_cells", reference_radius_cells_, 3);

    pnh_.param("vehicle/min_horizontal_range", min_horizontal_range_, 0.25);
    pnh_.param("vehicle/max_horizontal_range", max_horizontal_range_, 5.0);
    pnh_.param("vehicle/max_traversable_slope_deg",
                max_traversable_slope_deg_, 15.0);
    pnh_.param("vehicle/max_step_height", max_step_height_, 0.08);

    pnh_.param("surface/min_points_per_cell", min_points_per_cell_, 2);
    pnh_.param("surface/min_plane_cells", min_plane_cells_, 4);
    pnh_.param("surface/min_step_neighbors", min_step_neighbors_, 2);
    pnh_.param("surface/max_vertical_span", max_vertical_span_, 0.10);
    pnh_.param("surface/max_plane_rmse", max_plane_rmse_, 0.045);
    pnh_.param("surface/obstacle_min_relative_height",
                obstacle_min_relative_height_, 0.06);
    pnh_.param("surface/obstacle_max_relative_height",
                obstacle_max_relative_height_, 1.50);
    pnh_.param("surface/unknown_absolute_min_z", unknown_absolute_min_z_, -0.35);
    pnh_.param("surface/unknown_absolute_max_z", unknown_absolute_max_z_, 1.50);

    pnh_.param("output/marking_z", marking_z_, 0.20);
    pnh_.param("output/publish_debug_clouds", publish_debug_clouds_, true);
    pnh_.param<std::string>("output/metrics_csv", metrics_csv_, "");
  }

  int cellIndex(double x, double y) const {
    const int ix = static_cast<int>(std::floor((x - min_x_) / resolution_));
    const int iy = static_cast<int>(std::floor((y - min_y_) / resolution_));
    if (ix < 0 || iy < 0 || ix >= width_ || iy >= height_) {
      return -1;
    }
    return iy * width_ + ix;
  }

  bool inRange(const Point& point) const {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      return false;
    }
    const double range = std::hypot(point.x, point.y);
    return range >= min_horizontal_range_ && range <= max_horizontal_range_ &&
           cellIndex(point.x, point.y) >= 0;
  }

  Eigen::Vector2d cellCenter(int index) const {
    const int ix = index % width_;
    const int iy = index / width_;
    return Eigen::Vector2d(min_x_ + (ix + 0.5) * resolution_,
                           min_y_ + (iy + 0.5) * resolution_);
  }

  void buildGroundGrid(const Cloud& ground) {
    for (Cell& cell : cells_) {
      cell.reset();
    }

    for (const Point& point : ground.points) {
      if (!inRange(point)) {
        continue;
      }
      cells_[cellIndex(point.x, point.y)].z_values.push_back(point.z);
    }

    for (Cell& cell : cells_) {
      if (static_cast<int>(cell.z_values.size()) < min_points_per_cell_) {
        continue;
      }
      const double low = percentile(cell.z_values, 0.20);
      const double median = percentile(cell.z_values, 0.50);
      const double high = percentile(cell.z_values, 0.80);
      cell.ground_z = 0.5 * (low + median);
      cell.smooth_z = cell.ground_z;
      cell.span = high - low;
      cell.populated = true;
    }

    evaluateCells();
  }

  void evaluateCells() {
    const double slope_limit_rad =
        max_traversable_slope_deg_ * M_PI / 180.0;

    for (int index = 0; index < static_cast<int>(cells_.size()); ++index) {
      Cell& cell = cells_[index];
      if (!cell.populated) {
        continue;
      }

      const int cx = index % width_;
      const int cy = index / width_;
      std::vector<Eigen::Vector3d> neighbors;
      int step_neighbor_count = 0;

      for (int dy = -neighbor_radius_cells_; dy <= neighbor_radius_cells_; ++dy) {
        for (int dx = -neighbor_radius_cells_; dx <= neighbor_radius_cells_; ++dx) {
          const int nx = cx + dx;
          const int ny = cy + dy;
          if (nx < 0 || ny < 0 || nx >= width_ || ny >= height_) {
            continue;
          }
          const Cell& neighbor = cells_[ny * width_ + nx];
          if (!neighbor.populated) {
            continue;
          }
          const Eigen::Vector2d center = cellCenter(ny * width_ + nx);
          neighbors.emplace_back(center.x(), center.y(), neighbor.ground_z);

          if (std::abs(dx) <= 1 && std::abs(dy) <= 1 &&
              (dx != 0 || dy != 0)) {
            const double distance =
                resolution_ * std::hypot(static_cast<double>(dx),
                                         static_cast<double>(dy));
            const double dz = std::abs(neighbor.ground_z - cell.ground_z);
            const double grade = std::atan2(dz, distance);
            if (dz > max_step_height_ && grade > slope_limit_rad) {
              ++step_neighbor_count;
            }
          }
        }
      }

      bool plane_valid = false;
      if (static_cast<int>(neighbors.size()) >= min_plane_cells_) {
        Eigen::MatrixXd design(neighbors.size(), 3);
        Eigen::VectorXd heights(neighbors.size());
        for (std::size_t i = 0; i < neighbors.size(); ++i) {
          design(i, 0) = neighbors[i].x();
          design(i, 1) = neighbors[i].y();
          design(i, 2) = 1.0;
          heights(i) = neighbors[i].z();
        }
        const Eigen::Vector3d coefficients =
            design.colPivHouseholderQr().solve(heights);
        const Eigen::VectorXd residual = design * coefficients - heights;
        cell.plane_rmse =
            std::sqrt(residual.squaredNorm() / neighbors.size());
        cell.slope_deg =
            std::atan(std::hypot(coefficients.x(), coefficients.y())) *
            180.0 / M_PI;
        const Eigen::Vector2d center = cellCenter(index);
        cell.smooth_z = coefficients.x() * center.x() +
                        coefficients.y() * center.y() + coefficients.z();
        plane_valid = true;
      }

      const bool excessive_slope =
          plane_valid && cell.slope_deg > max_traversable_slope_deg_;
      const bool rough_surface =
          cell.span > max_vertical_span_ ||
          (plane_valid && cell.plane_rmse > max_plane_rmse_ &&
           cell.span > 0.5 * max_vertical_span_);
      const bool step_detected = step_neighbor_count >= min_step_neighbors_;
      cell.step_detected = step_detected;
      cell.excessive_slope = excessive_slope;
      cell.rough_surface = rough_surface;
      cell.safe = !step_detected && !excessive_slope && !rough_surface;
    }
  }

  bool referenceGround(const Point& point, double* ground_z) const {
    const int index = cellIndex(point.x, point.y);
    if (index < 0) {
      return false;
    }
    const int cx = index % width_;
    const int cy = index / width_;
    double best_distance = std::numeric_limits<double>::infinity();
    bool found = false;

    for (int dy = -reference_radius_cells_; dy <= reference_radius_cells_; ++dy) {
      for (int dx = -reference_radius_cells_; dx <= reference_radius_cells_; ++dx) {
        const int nx = cx + dx;
        const int ny = cy + dy;
        if (nx < 0 || ny < 0 || nx >= width_ || ny >= height_) {
          continue;
        }
        const Cell& cell = cells_[ny * width_ + nx];
        if (!cell.populated || !cell.safe) {
          continue;
        }
        const double distance = std::hypot(static_cast<double>(dx),
                                           static_cast<double>(dy));
        if (distance < best_distance) {
          best_distance = distance;
          *ground_z = cell.smooth_z;
          found = true;
        }
      }
    }
    return found;
  }

  sensor_msgs::PointCloud2 cloudMessage(const Cloud& cloud,
                                        const std_msgs::Header& header) const {
    sensor_msgs::PointCloud2 message;
    pcl::toROSMsg(cloud, message);
    message.header = header;
    return message;
  }

  void callback(const sensor_msgs::PointCloud2ConstPtr& ground_message,
                const sensor_msgs::PointCloud2ConstPtr& nonground_message) {
    const ros::WallTime begin = ros::WallTime::now();

    Cloud ground;
    Cloud nonground;
    pcl::fromROSMsg(*ground_message, ground);
    pcl::fromROSMsg(*nonground_message, nonground);
    buildGroundGrid(ground);

    Cloud safe_ground;
    Cloud unsafe_ground;
    Cloud obstacles;
    Cloud clearing;
    Cloud unknown;
    std::size_t front_safe_ground = 0;
    std::size_t front_obstacles = 0;
    safe_ground.reserve(ground.size());
    unsafe_ground.reserve(ground.size() / 10 + 1);
    obstacles.reserve(nonground.size());
    clearing.reserve(ground.size() + nonground.size());
    unknown.reserve(nonground.size() / 4 + 1);

    for (const Point& point : ground.points) {
      if (!inRange(point)) {
        continue;
      }
      clearing.push_back(point);
      const Cell& cell = cells_[cellIndex(point.x, point.y)];
      if (cell.populated && cell.safe) {
        safe_ground.push_back(point);
        if (point.x >= 0.30 && point.x <= 3.0 && std::abs(point.y) <= 1.0) {
          ++front_safe_ground;
        }
      } else {
        unsafe_ground.push_back(point);
        Point marker = point;
        marker.z = marking_z_;
        obstacles.push_back(marker);
        if (point.x >= 0.30 && point.x <= 3.0 && std::abs(point.y) <= 1.0) {
          ++front_obstacles;
        }
      }
    }

    for (const Point& point : nonground.points) {
      if (!inRange(point)) {
        continue;
      }
      clearing.push_back(point);
      double ground_z = 0.0;
      const bool has_reference = referenceGround(point, &ground_z);
      const bool relative_obstacle =
          has_reference &&
          point.z - ground_z >= obstacle_min_relative_height_ &&
          point.z - ground_z <= obstacle_max_relative_height_;
      const bool conservative_obstacle =
          !has_reference && point.z >= unknown_absolute_min_z_ &&
          point.z <= unknown_absolute_max_z_;

      if (relative_obstacle || conservative_obstacle) {
        Point marker = point;
        marker.z = marking_z_;
        obstacles.push_back(marker);
        if (point.x >= 0.30 && point.x <= 3.0 && std::abs(point.y) <= 1.0) {
          ++front_obstacles;
        }
      } else {
        unknown.push_back(point);
      }
    }

    safe_ground.width = safe_ground.size();
    safe_ground.height = 1;
    unsafe_ground.width = unsafe_ground.size();
    unsafe_ground.height = 1;
    obstacles.width = obstacles.size();
    obstacles.height = 1;
    clearing.width = clearing.size();
    clearing.height = 1;
    unknown.width = unknown.size();
    unknown.height = 1;

    obstacle_pub_.publish(cloudMessage(obstacles, ground_message->header));
    clearing_pub_.publish(cloudMessage(clearing, ground_message->header));
    if (publish_debug_clouds_) {
      safe_ground_pub_.publish(
          cloudMessage(safe_ground, ground_message->header));
      unsafe_ground_pub_.publish(
          cloudMessage(unsafe_ground, ground_message->header));
      unknown_pub_.publish(cloudMessage(unknown, ground_message->header));
    }

    int populated_cells = 0;
    int safe_cells = 0;
    int unsafe_cells = 0;
    int step_cells = 0;
    int slope_cells = 0;
    int rough_cells = 0;
    double slope_sum = 0.0;
    double span_sum = 0.0;
    double rmse_sum = 0.0;
    double max_slope = 0.0;
    double max_span = 0.0;
    double max_rmse = 0.0;
    for (const Cell& cell : cells_) {
      if (!cell.populated) {
        continue;
      }
      ++populated_cells;
      if (cell.safe) {
        ++safe_cells;
      } else {
        ++unsafe_cells;
      }
      step_cells += cell.step_detected ? 1 : 0;
      slope_cells += cell.excessive_slope ? 1 : 0;
      rough_cells += cell.rough_surface ? 1 : 0;
      slope_sum += cell.slope_deg;
      span_sum += cell.span;
      rmse_sum += cell.plane_rmse;
      max_slope = std::max(max_slope, cell.slope_deg);
      max_span = std::max(max_span, cell.span);
      max_rmse = std::max(max_rmse, cell.plane_rmse);
    }
    const double divisor = populated_cells > 0 ? populated_cells : 1;

    const double processing_ms =
        (ros::WallTime::now() - begin).toSec() * 1000.0;
    publishDiagnostics(ground_message->header,
                       ground.size() + nonground.size(), ground.size(),
                       nonground.size(), safe_ground.size(),
                       unsafe_ground.size(), obstacles.size(), unknown.size(),
                       populated_cells, safe_cells, unsafe_cells, processing_ms);

    if (metrics_) {
      metrics_ << std::fixed << std::setprecision(9)
               << ground_message->header.stamp.toSec() << ','
               << ground.size() + nonground.size() << ',' << ground.size() << ','
               << nonground.size() << ',' << safe_ground.size() << ','
               << unsafe_ground.size() << ',' << obstacles.size() << ','
               << unknown.size() << ',' << populated_cells << ',' << safe_cells
               << ',' << unsafe_cells << ',' << step_cells << ',' << slope_cells
               << ',' << rough_cells << ',' << std::setprecision(4)
               << slope_sum / divisor << ',' << max_slope << ','
               << span_sum / divisor << ',' << max_span << ','
               << rmse_sum / divisor << ',' << max_rmse << ','
               << front_safe_ground << ',' << front_obstacles << ','
               << std::setprecision(3)
               << processing_ms << '\n';
      metrics_.flush();
    }

    ROS_INFO_THROTTLE(
        2.0,
        "Terrain guard: input=%zu ground=%zu safe=%zu unsafe=%zu obstacles=%zu "
        "cells=%d/%d time=%.1f ms",
        ground.size() + nonground.size(), ground.size(), safe_ground.size(),
        unsafe_ground.size(), obstacles.size(), safe_cells, unsafe_cells,
        processing_ms);
  }

  void publishDiagnostics(const std_msgs::Header& header, std::size_t input,
                          std::size_t ground, std::size_t nonground,
                          std::size_t safe_ground, std::size_t unsafe_ground,
                          std::size_t obstacles, std::size_t unknown,
                          int populated_cells, int safe_cells, int unsafe_cells,
                          double processing_ms) {
    diagnostic_msgs::DiagnosticArray array;
    array.header = header;
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "scout_terrain_filter";
    status.hardware_id = "livox_mid360";
    status.level = input == 0 ? diagnostic_msgs::DiagnosticStatus::ERROR
                              : diagnostic_msgs::DiagnosticStatus::OK;
    status.message = input == 0 ? "empty point cloud" : "terrain filter active";
    status.values.push_back(keyValue("input_points", numberString(input, 0)));
    status.values.push_back(keyValue("patchwork_ground", numberString(ground, 0)));
    status.values.push_back(
        keyValue("patchwork_nonground", numberString(nonground, 0)));
    status.values.push_back(keyValue("safe_ground", numberString(safe_ground, 0)));
    status.values.push_back(
        keyValue("unsafe_ground", numberString(unsafe_ground, 0)));
    status.values.push_back(keyValue("obstacles", numberString(obstacles, 0)));
    status.values.push_back(keyValue("unknown", numberString(unknown, 0)));
    status.values.push_back(
        keyValue("populated_cells", numberString(populated_cells, 0)));
    status.values.push_back(keyValue("safe_cells", numberString(safe_cells, 0)));
    status.values.push_back(
        keyValue("unsafe_cells", numberString(unsafe_cells, 0)));
    status.values.push_back(
        keyValue("processing_ms", numberString(processing_ms, 3)));
    array.status.push_back(status);
    diagnostics_pub_.publish(array);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> ground_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> nonground_sub_;
  message_filters::Synchronizer<SyncPolicy> sync_;

  ros::Publisher safe_ground_pub_;
  ros::Publisher unsafe_ground_pub_;
  ros::Publisher obstacle_pub_;
  ros::Publisher clearing_pub_;
  ros::Publisher unknown_pub_;
  ros::Publisher diagnostics_pub_;

  double resolution_ = 0.10;
  double min_x_ = -5.0;
  double max_x_ = 5.0;
  double min_y_ = -4.0;
  double max_y_ = 4.0;
  int neighbor_radius_cells_ = 2;
  int reference_radius_cells_ = 3;
  double min_horizontal_range_ = 0.25;
  double max_horizontal_range_ = 5.0;
  double max_traversable_slope_deg_ = 15.0;
  double max_step_height_ = 0.08;
  int min_points_per_cell_ = 2;
  int min_plane_cells_ = 4;
  int min_step_neighbors_ = 2;
  double max_vertical_span_ = 0.10;
  double max_plane_rmse_ = 0.045;
  double obstacle_min_relative_height_ = 0.06;
  double obstacle_max_relative_height_ = 1.50;
  double unknown_absolute_min_z_ = -0.35;
  double unknown_absolute_max_z_ = 1.50;
  double marking_z_ = 0.20;
  bool publish_debug_clouds_ = true;
  std::string metrics_csv_;

  int width_ = 0;
  int height_ = 0;
  std::vector<Cell> cells_;
  std::ofstream metrics_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "scout_terrain_guard");
  try {
    TerrainGuard node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("Terrain guard initialization failed: %s", error.what());
    return 1;
  }
  return 0;
}
