#include "scout_2p5d_navigation/terrain_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/TransformStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>

namespace wt = scout_2p5d_navigation;
namespace {
using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;

float median(std::vector<float>* values) {
  const std::size_t middle = values->size() / 2;
  std::nth_element(values->begin(), values->begin() + middle, values->end());
  return (*values)[middle];
}
}  // namespace

class TerrainRelativeObstacleFilter {
 public:
  TerrainRelativeObstacleFilter()
      : nh_(), pnh_("~"), tf_listener_(tf_buffer_) {
    std::string map_yaml;
    pnh_.param<std::string>("terrain_map_yaml", map_yaml, "");
    pnh_.param("min_range", min_range_, 0.25);
    pnh_.param("max_range", max_range_, 5.0);
    pnh_.param("min_obstacle_relative_height", min_obstacle_height_, 0.08);
    pnh_.param("max_obstacle_relative_height", max_obstacle_height_, 1.50);
    pnh_.param("min_clearing_relative_height", min_clearing_height_, -0.25);
    pnh_.param("max_clearing_relative_height", max_clearing_height_, 2.00);
    pnh_.param("ground_search_radius_cells", search_radius_cells_, 2);
    pnh_.param("min_confidence", min_confidence_, 1);
    pnh_.param("marking_z", marking_z_, 0.20);
    if (map_yaml.empty())
      throw std::runtime_error("terrain_map_yaml is required");
    terrain_ = wt::loadTerrainMap(map_yaml);
    search_radius_cells_ = std::max(0, search_radius_cells_);
    min_confidence_ = std::max(1, std::min(255, min_confidence_));
    cloud_sub_ = nh_.subscribe("cloud", 2,
        &TerrainRelativeObstacleFilter::cloudCallback, this);
    obstacle_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("obstacles", 2);
    clearing_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("clearing", 2);
    ROS_INFO("Terrain-relative obstacle filter loaded %ux%u map at %.3f m",
             terrain_.width, terrain_.height, terrain_.resolution);
  }

 private:
  bool groundHeight(double wx, double wy, float* ground_z) const {
    uint32_t mx = 0;
    uint32_t my = 0;
    if (!terrain_.worldToMap(wx, wy, &mx, &my)) return false;
    std::vector<float> candidates;
    for (int dy = -search_radius_cells_; dy <= search_radius_cells_; ++dy) {
      for (int dx = -search_radius_cells_; dx <= search_radius_cells_; ++dx) {
        if (dx * dx + dy * dy > search_radius_cells_ * search_radius_cells_)
          continue;
        const int nx = static_cast<int>(mx) + dx;
        const int ny = static_cast<int>(my) + dy;
        if (nx < 0 || ny < 0 || nx >= static_cast<int>(terrain_.width) ||
            ny >= static_cast<int>(terrain_.height)) continue;
        const std::size_t id = terrain_.index(nx, ny);
        if (terrain_.confidence[id] < min_confidence_ ||
            !std::isfinite(terrain_.elevation[id])) continue;
        candidates.push_back(terrain_.elevation[id]);
      }
    }
    if (candidates.empty()) return false;
    *ground_z = median(&candidates);
    return true;
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& message) {
    geometry_msgs::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(
          terrain_.frame_id, message->header.frame_id, message->header.stamp,
          ros::Duration(0.10));
    } catch (const tf2::TransformException& error) {
      ROS_WARN_THROTTLE(2.0, "Terrain-relative TF %s <- %s failed: %s",
                        terrain_.frame_id.c_str(), message->header.frame_id.c_str(),
                        error.what());
      return;
    }

    const auto& t = transform.transform.translation;
    const auto& q = transform.transform.rotation;
    Eigen::Quaterniond rotation(q.w, q.x, q.y, q.z);
    if (rotation.norm() < 0.5) return;
    rotation.normalize();
    const Eigen::Vector3d translation(t.x, t.y, t.z);

    Cloud input;
    Cloud obstacles;
    Cloud clearing;
    pcl::fromROSMsg(*message, input);
    obstacles.reserve(input.size() / 10 + 1);
    clearing.reserve(input.size());
    std::size_t no_ground = 0;
    for (const Point& point : input.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
          !std::isfinite(point.z)) continue;
      const double range = std::hypot(point.x, point.y);
      if (range < min_range_ || range > max_range_) continue;
      const Eigen::Vector3d world = rotation * Eigen::Vector3d(
          point.x, point.y, point.z) + translation;
      float ground_z = std::numeric_limits<float>::quiet_NaN();
      if (!groundHeight(world.x(), world.y(), &ground_z)) {
        ++no_ground;
        continue;
      }
      const double relative_height = world.z() - ground_z;
      if (relative_height >= min_clearing_height_ &&
          relative_height <= max_clearing_height_) {
        clearing.push_back(point);
      }
      if (relative_height >= min_obstacle_height_ &&
          relative_height <= max_obstacle_height_) {
        Point marker = point;
        marker.z = marking_z_;
        obstacles.push_back(marker);
      }
    }

    obstacles.width = obstacles.size();
    obstacles.height = 1;
    obstacles.is_dense = false;
    clearing.width = clearing.size();
    clearing.height = 1;
    clearing.is_dense = false;
    sensor_msgs::PointCloud2 obstacle_message;
    sensor_msgs::PointCloud2 clearing_message;
    pcl::toROSMsg(obstacles, obstacle_message);
    pcl::toROSMsg(clearing, clearing_message);
    obstacle_message.header = message->header;
    clearing_message.header = message->header;
    obstacle_pub_.publish(obstacle_message);
    clearing_pub_.publish(clearing_message);
    ROS_INFO_THROTTLE(5.0,
        "Terrain-relative cloud: input=%zu clearing=%zu obstacles=%zu no_ground=%zu",
        input.size(), clearing.size(), obstacles.size(), no_ground);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ros::Subscriber cloud_sub_;
  ros::Publisher obstacle_pub_;
  ros::Publisher clearing_pub_;
  wt::TerrainMap terrain_;
  double min_range_ = 0.25;
  double max_range_ = 5.0;
  double min_obstacle_height_ = 0.08;
  double max_obstacle_height_ = 1.50;
  double min_clearing_height_ = -0.25;
  double max_clearing_height_ = 2.00;
  int search_radius_cells_ = 2;
  int min_confidence_ = 1;
  double marking_z_ = 0.20;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "scout_terrain_relative_obstacle_filter");
  try {
    TerrainRelativeObstacleFilter filter;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("Terrain-relative obstacle filter failed: %s", error.what());
    return 1;
  }
  return 0;
}
