#include <cmath>
#include <cstdint>
#include <string>
#include <unordered_set>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

namespace {

using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;

struct VoxelKey {
  int64_t x;
  int64_t y;
  int64_t z;

  bool operator==(const VoxelKey& rhs) const {
    return x == rhs.x && y == rhs.y && z == rhs.z;
  }
};

struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey& key) const {
    std::size_t seed = std::hash<int64_t>()(key.x);
    seed ^= std::hash<int64_t>()(key.y) + 0x9e3779b9 + (seed << 6) +
            (seed >> 2);
    seed ^= std::hash<int64_t>()(key.z) + 0x9e3779b9 + (seed << 6) +
            (seed >> 2);
    return seed;
  }
};

VoxelKey voxelKey(const Point& point, double voxel_size) {
  return {static_cast<int64_t>(std::floor(point.x / voxel_size)),
          static_cast<int64_t>(std::floor(point.y / voxel_size)),
          static_cast<int64_t>(std::floor(point.z / voxel_size))};
}

bool finite(const Point& point) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z);
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "wheeltec_pcd_static_gate");
  ros::NodeHandle pnh("~");

  std::string authority_path;
  std::string input_path;
  std::string output_path;
  double voxel_size = 0.05;
  pnh.param<std::string>("authority_pcd", authority_path, "");
  pnh.param<std::string>("input_pcd", input_path, "");
  pnh.param<std::string>("output_pcd", output_path, "");
  pnh.param("voxel_size", voxel_size, 0.05);

  if (authority_path.empty() || input_path.empty() || output_path.empty()) {
    ROS_FATAL("authority_pcd, input_pcd and output_pcd are required");
    return 1;
  }
  if (!std::isfinite(voxel_size) || voxel_size <= 0.0) {
    ROS_FATAL("voxel_size must be positive");
    return 1;
  }

  Cloud authority;
  Cloud input;
  if (pcl::io::loadPCDFile<Point>(authority_path, authority) < 0) {
    ROS_FATAL("Failed to load authority PCD: %s", authority_path.c_str());
    return 1;
  }
  if (pcl::io::loadPCDFile<Point>(input_path, input) < 0) {
    ROS_FATAL("Failed to load classified PCD: %s", input_path.c_str());
    return 1;
  }

  std::unordered_set<VoxelKey, VoxelKeyHash> static_voxels;
  static_voxels.reserve(authority.size());
  for (const Point& point : authority) {
    if (finite(point)) {
      static_voxels.insert(voxelKey(point, voxel_size));
    }
  }
  if (static_voxels.empty()) {
    ROS_FATAL("Authority PCD contains no finite static voxels");
    return 1;
  }

  Cloud output;
  output.reserve(input.size());
  std::size_t invalid_points = 0;
  for (const Point& point : input) {
    if (!finite(point)) {
      ++invalid_points;
      continue;
    }
    if (static_voxels.count(voxelKey(point, voxel_size)) != 0) {
      output.push_back(point);
    }
  }
  output.width = static_cast<uint32_t>(output.size());
  output.height = 1;
  output.is_dense = true;

  if (pcl::io::savePCDFileBinary(output_path, output) != 0) {
    ROS_FATAL("Failed to save gated PCD: %s", output_path.c_str());
    return 1;
  }

  ROS_INFO("Static gate authority=%zu voxels voxel_size=%.3f m",
           static_voxels.size(), voxel_size);
  ROS_INFO(
      "Static gate input=%zu kept=%zu rejected=%zu invalid=%zu",
      input.size(), output.size(),
      input.size() - output.size() - invalid_points, invalid_points);
  ROS_INFO("Static gate output: %s", output_path.c_str());
  return 0;
}
