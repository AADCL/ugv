#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>

#include <boost/bind/bind.hpp>
#include <boost/filesystem.hpp>
#include <Eigen/Geometry>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_listener.h>

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
    seed ^= std::hash<int64_t>()(key.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<int64_t>()(key.z) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};

struct Accumulator {
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  double intensity_sum = 0.0;
  uint32_t count = 0;
  uint32_t hit_frames = 0;
  std::size_t last_frame = std::numeric_limits<std::size_t>::max();
  ros::Time first_stamp;
  ros::Time last_stamp;
};
}  // namespace

class TerrainMapAccumulator {
 public:
  TerrainMapAccumulator()
      : nh_(), pnh_("~"), tf_listener_(tf_buffer_), ground_sub_(nh_, "ground", 10),
        obstacle_sub_(nh_, "obstacles", 10), odom_sub_(nh_, "odom", 20),
        sync_(SyncPolicy(30), ground_sub_, obstacle_sub_, odom_sub_) {
    pnh_.param<std::string>("output_ground_path", output_ground_path_,
                            "/tmp/terrain_ground_camera_init.pcd");
    pnh_.param<std::string>("output_obstacle_path", output_obstacle_path_,
                            "/tmp/terrain_obstacles_camera_init.pcd");
    pnh_.param("voxel_size", voxel_size_, 0.10);
    pnh_.param("save_on_shutdown", save_on_shutdown_, true);
    pnh_.param("min_obstacle_frames", min_obstacle_frames_, 5);
    pnh_.param("min_observation_span", min_observation_span_, 1.0);
    int max_voxels = 2000000;
    pnh_.param("max_voxels", max_voxels, 2000000);
    max_voxels_ = static_cast<std::size_t>(std::max(1, max_voxels));
    voxel_size_ = std::max(0.02, voxel_size_);

    sync_.registerCallback(boost::bind(&TerrainMapAccumulator::callback, this,
                                       boost::placeholders::_1,
                                       boost::placeholders::_2,
                                       boost::placeholders::_3));
    save_service_ = pnh_.advertiseService("save_map",
        &TerrainMapAccumulator::saveService, this);
    reset_service_ = pnh_.advertiseService("reset_map",
        &TerrainMapAccumulator::resetService, this);
    ROS_INFO("Terrain map accumulator voxel %.3f m", voxel_size_);
  }

  ~TerrainMapAccumulator() {
    if (save_on_shutdown_ && dirty_) {
      std::string message;
      if (!save(&message)) ROS_ERROR_STREAM(message);
    }
  }

 private:
  using SyncPolicy = message_filters::sync_policies::ExactTime<
      sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, nav_msgs::Odometry>;
  using VoxelMap = std::unordered_map<VoxelKey, Accumulator, VoxelKeyHash>;

  VoxelKey key(const Eigen::Vector3d& point) const {
    return {static_cast<int64_t>(std::floor(point.x() / voxel_size_)),
            static_cast<int64_t>(std::floor(point.y() / voxel_size_)),
            static_cast<int64_t>(std::floor(point.z() / voxel_size_))};
  }

  void insert(const Cloud& cloud, const Eigen::Isometry3d& transform,
              const ros::Time& stamp, VoxelMap* voxels) {
    for (const Point& point : cloud.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
          !std::isfinite(point.z)) continue;
      const Eigen::Vector3d world = transform * Eigen::Vector3d(point.x, point.y, point.z);
      const VoxelKey voxel_key = key(world);
      auto found = voxels->find(voxel_key);
      if (found == voxels->end()) {
        if (voxels->size() >= max_voxels_) continue;
        found = voxels->emplace(voxel_key, Accumulator()).first;
      }
      found->second.sum += world;
      found->second.intensity_sum += point.intensity;
      ++found->second.count;
      if (found->second.last_frame != frames_) {
        found->second.last_frame = frames_;
        ++found->second.hit_frames;
        if (found->second.first_stamp.isZero()) found->second.first_stamp = stamp;
        found->second.last_stamp = stamp;
      }
    }
  }

  void callback(const sensor_msgs::PointCloud2ConstPtr& ground_message,
                const sensor_msgs::PointCloud2ConstPtr& obstacle_message,
                const nav_msgs::OdometryConstPtr& odom) {
    if (ground_message->header.frame_id != obstacle_message->header.frame_id) {
      ROS_WARN_THROTTLE(5.0, "Terrain mapper cloud frame mismatch: ground=%s obstacle=%s",
                        ground_message->header.frame_id.c_str(),
                        obstacle_message->header.frame_id.c_str());
      return;
    }
    const auto& pose = odom->pose.pose;
    Eigen::Quaterniond rotation(pose.orientation.w, pose.orientation.x,
                                pose.orientation.y, pose.orientation.z);
    if (rotation.norm() < 0.5) return;
    rotation.normalize();
    Eigen::Isometry3d world_from_body = Eigen::Isometry3d::Identity();
    world_from_body.linear() = rotation.toRotationMatrix();
    world_from_body.translation() = Eigen::Vector3d(
        pose.position.x, pose.position.y, pose.position.z);

    Eigen::Isometry3d body_from_cloud = Eigen::Isometry3d::Identity();
    if (!odom->child_frame_id.empty() &&
        odom->child_frame_id != ground_message->header.frame_id) {
      try {
        const geometry_msgs::TransformStamped tf = tf_buffer_.lookupTransform(
            odom->child_frame_id, ground_message->header.frame_id,
            ground_message->header.stamp, ros::Duration(0.10));
        const auto& t = tf.transform.translation;
        const auto& q = tf.transform.rotation;
        Eigen::Quaterniond cloud_rotation(q.w, q.x, q.y, q.z);
        if (cloud_rotation.norm() < 0.5) return;
        cloud_rotation.normalize();
        body_from_cloud.linear() = cloud_rotation.toRotationMatrix();
        body_from_cloud.translation() = Eigen::Vector3d(t.x, t.y, t.z);
      } catch (const tf2::TransformException& error) {
        ROS_WARN_THROTTLE(2.0, "Terrain mapper TF %s <- %s failed: %s",
                          odom->child_frame_id.c_str(),
                          ground_message->header.frame_id.c_str(), error.what());
        return;
      }
    }
    const Eigen::Isometry3d transform = world_from_body * body_from_cloud;
    Cloud ground;
    Cloud obstacles;
    pcl::fromROSMsg(*ground_message, ground);
    pcl::fromROSMsg(*obstacle_message, obstacles);
    insert(ground, transform, ground_message->header.stamp, &ground_voxels_);
    insert(obstacles, transform, obstacle_message->header.stamp, &obstacle_voxels_);
    world_frame_ = odom->header.frame_id;
    dirty_ = true;
    ++frames_;
    ROS_INFO_THROTTLE(5.0, "Terrain map: frames=%zu ground=%zu obstacles=%zu",
                      frames_, ground_voxels_.size(), obstacle_voxels_.size());
  }

  Cloud toCloud(const VoxelMap& voxels, bool persistent_only) const {
    Cloud cloud;
    cloud.reserve(voxels.size());
    for (const auto& entry : voxels) {
      if (entry.second.count == 0) continue;
      if (persistent_only &&
          (static_cast<int>(entry.second.hit_frames) < min_obstacle_frames_ ||
           (entry.second.last_stamp - entry.second.first_stamp).toSec() <
               min_observation_span_)) continue;
      const Eigen::Vector3d p = entry.second.sum / entry.second.count;
      Point point;
      point.x = p.x(); point.y = p.y(); point.z = p.z();
      point.intensity = entry.second.intensity_sum / entry.second.count;
      cloud.push_back(point);
    }
    cloud.width = cloud.size();
    cloud.height = 1;
    cloud.is_dense = false;
    return cloud;
  }

  bool ensureParent(const std::string& path, std::string* message) const {
    const boost::filesystem::path parent = boost::filesystem::path(path).parent_path();
    if (parent.empty()) return true;
    boost::system::error_code error;
    boost::filesystem::create_directories(parent, error);
    if (error) {
      *message = "Cannot create output directory: " + error.message();
      return false;
    }
    return true;
  }

  bool save(std::string* message) {
    if (ground_voxels_.empty() && obstacle_voxels_.empty()) {
      *message = "Terrain map is empty";
      return false;
    }
    if (!ensureParent(output_ground_path_, message) ||
        !ensureParent(output_obstacle_path_, message)) return false;
    const Cloud ground = toCloud(ground_voxels_, false);
    const Cloud obstacles = toCloud(obstacle_voxels_, true);
    if (pcl::io::savePCDFileBinary(output_ground_path_, ground) != 0 ||
        pcl::io::savePCDFileBinary(output_obstacle_path_, obstacles) != 0) {
      *message = "Failed to save classified terrain PCD files";
      return false;
    }
    dirty_ = false;
    *message = "Saved terrain map in " + world_frame_ + ": ground=" +
               std::to_string(ground.size()) + " obstacles=" +
               std::to_string(obstacles.size());
    ROS_INFO_STREAM(*message);
    return true;
  }

  bool saveService(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
    response.success = save(&response.message);
    return true;
  }

  bool resetService(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
    ground_voxels_.clear();
    obstacle_voxels_.clear();
    frames_ = 0;
    dirty_ = false;
    ROS_WARN("Terrain classified map reset");
    return true;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> ground_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> obstacle_sub_;
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
  message_filters::Synchronizer<SyncPolicy> sync_;
  ros::ServiceServer save_service_;
  ros::ServiceServer reset_service_;
  VoxelMap ground_voxels_;
  VoxelMap obstacle_voxels_;
  std::string output_ground_path_;
  std::string output_obstacle_path_;
  std::string world_frame_ = "camera_init";
  double voxel_size_ = 0.10;
  std::size_t max_voxels_ = 2000000;
  std::size_t frames_ = 0;
  bool save_on_shutdown_ = true;
  int min_obstacle_frames_ = 5;
  double min_observation_span_ = 1.0;
  bool dirty_ = false;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "scout_terrain_map_accumulator");
  TerrainMapAccumulator node;
  ros::spin();
  return 0;
}
