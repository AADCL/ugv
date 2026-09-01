#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <boost/filesystem.hpp>
#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <pcl/common/point_tests.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Header.h>

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

struct OccupancyVoxelState {
  double log_odds = 0.0;
  uint32_t hit_scans = 0;
  uint32_t miss_scans = 0;
  uint64_t last_hit_scan = std::numeric_limits<uint64_t>::max();
  uint64_t last_miss_scan = std::numeric_limits<uint64_t>::max();
  ros::Time first_hit;
  ros::Time last_hit;
  ros::Time last_seen;
  uint64_t generation = 0;
};

struct MapVoxelState {
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  double intensity_sum = 0.0;
  uint32_t sample_count = 0;
  VoxelKey occupancy_key{0, 0, 0};
  uint64_t occupancy_generation = 0;
};

class PointcloudMapper {
 public:
  PointcloudMapper() : nh_(), pnh_("~") {
    loadParameters();

    static_scan_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>(static_scan_topic_, 1);
    static_map_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>(static_map_topic_, 1, true);
    if (publish_dynamic_) {
      dynamic_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(dynamic_topic_, 1);
    }

    odom_sub_ = nh_.subscribe(input_odom_, 20,
                              &PointcloudMapper::odomCallback, this);
    cloud_sub_ = nh_.subscribe(input_cloud_, 2,
                               &PointcloudMapper::cloudCallback, this);
    save_service_ = pnh_.advertiseService(
        "save_map", &PointcloudMapper::saveMap, this);
    reset_service_ = pnh_.advertiseService(
        "reset_map", &PointcloudMapper::resetMap, this);
    publish_timer_ = nh_.createTimer(
        ros::Duration(map_publish_period_),
        &PointcloudMapper::publishMapTimer, this);
    autosave_timer_ = nh_.createTimer(
        ros::Duration(autosave_period_),
        &PointcloudMapper::autosaveTimer, this);

    ROS_INFO_STREAM("scout_pointcloud_mapper: " << input_cloud_ << " + "
                    << input_odom_ << " -> " << output_path_);
  }

  ~PointcloudMapper() {
    if (save_on_shutdown_ && dirty_) {
      std::string message;
      if (!saveMapToDisk(&message)) {
        ROS_ERROR_STREAM("Final filtered-map save failed: " << message);
      } else {
        ROS_INFO_STREAM(message);
      }
    }
  }

 private:
  void loadParameters() {
    pnh_.param<std::string>("input_cloud", input_cloud_,
                            "/cloud_registered");
    pnh_.param<std::string>("input_odom", input_odom_, "/Odometry");
    pnh_.param<std::string>("static_scan_topic", static_scan_topic_,
                            "/scout/static_scan");
    pnh_.param<std::string>("static_map_topic", static_map_topic_,
                            "/scout/static_map_cloud");
    pnh_.param<std::string>("dynamic_topic", dynamic_topic_,
                            "/scout/dynamic_points");
    pnh_.param<std::string>("output_path", output_path_,
                            "/tmp/filtered_camera_init.pcd");
    pnh_.param("min_range", min_range_, 0.5);
    pnh_.param("max_range", max_range_, 50.0);
    pnh_.param("scan_voxel_size", scan_voxel_size_, 0.10);
    pnh_.param("radius_filter/enable", radius_filter_enable_, true);
    pnh_.param("radius_filter/radius", radius_, 0.20);
    pnh_.param("radius_filter/min_neighbors", min_neighbors_, 2);
    pnh_.param("self_filter/enable", self_filter_enable_, false);
    pnh_.param("self_filter/min_x", self_min_x_, -0.55);
    pnh_.param("self_filter/max_x", self_max_x_, 0.55);
    pnh_.param("self_filter/min_y", self_min_y_, -0.45);
    pnh_.param("self_filter/max_y", self_max_y_, 0.45);
    pnh_.param("self_filter/min_z", self_min_z_, -0.40);
    pnh_.param("self_filter/max_z", self_max_z_, 0.25);
    pnh_.param("dynamic_filter/enable", dynamic_filter_enable_, true);
    pnh_.param("dynamic_filter/voxel_size", temporal_voxel_size_, 0.20);
    pnh_.param("dynamic_filter/hit_probability", hit_probability_, 0.70);
    pnh_.param("dynamic_filter/miss_probability", miss_probability_, 0.40);
    pnh_.param("dynamic_filter/occupied_probability",
               occupied_probability_, 0.72);
    pnh_.param("dynamic_filter/clearing_probability",
               clearing_probability_, 0.35);
    pnh_.param("dynamic_filter/min_hit_scans", min_hit_scans_, 8);
    pnh_.param("dynamic_filter/min_observation_span",
               min_observation_span_, 2.0);
    pnh_.param("dynamic_filter/ray_stride", ray_stride_, 4);
    pnh_.param("dynamic_filter/max_clearing_range",
               max_clearing_range_, 20.0);
    pnh_.param("dynamic_filter/ray_endpoint_margin",
               ray_endpoint_margin_, 0.30);
    pnh_.param("dynamic_filter/candidate_timeout", candidate_timeout_, 3.0);
    pnh_.param("dynamic_filter/cleanup_period", cleanup_period_, 1.0);
    int max_voxels_param = 2000000;
    pnh_.param("dynamic_filter/max_voxels", max_voxels_param, 2000000);
    max_voxels_ = static_cast<std::size_t>(std::max(1, max_voxels_param));
    pnh_.param("map/voxel_size", map_voxel_size_, 0.05);
    int max_map_voxels_param = 5000000;
    pnh_.param("map/max_voxels", max_map_voxels_param, 5000000);
    max_map_voxels_ =
        static_cast<std::size_t>(std::max(1, max_map_voxels_param));
    pnh_.param("map/autosave_period", autosave_period_, 30.0);
    pnh_.param("map/save_on_shutdown", save_on_shutdown_, true);
    pnh_.param("map_publish_period", map_publish_period_, 2.0);
    pnh_.param("publish_dynamic_points", publish_dynamic_, false);
    pnh_.param("max_odom_age", max_odom_age_, 0.20);

    min_range_ = std::max(0.0, min_range_);
    max_range_ = std::max(min_range_, max_range_);
    scan_voxel_size_ = std::max(0.01, scan_voxel_size_);
    temporal_voxel_size_ = std::max(0.01, temporal_voxel_size_);
    map_voxel_size_ = std::max(0.01, map_voxel_size_);
    hit_probability_ = clampProbability(hit_probability_);
    miss_probability_ = clampProbability(miss_probability_);
    occupied_probability_ = clampProbability(occupied_probability_);
    clearing_probability_ = clampProbability(clearing_probability_);
    hit_log_odds_ = probabilityToLogOdds(hit_probability_);
    miss_log_odds_ = probabilityToLogOdds(miss_probability_);
    occupied_log_odds_ = probabilityToLogOdds(occupied_probability_);
    clearing_log_odds_ = probabilityToLogOdds(clearing_probability_);
    min_hit_scans_ = std::max(1, min_hit_scans_);
    min_observation_span_ = std::max(0.0, min_observation_span_);
    ray_stride_ = std::max(1, ray_stride_);
    max_clearing_range_ = std::max(temporal_voxel_size_,
                                   max_clearing_range_);
    ray_endpoint_margin_ = std::max(temporal_voxel_size_,
                                    ray_endpoint_margin_);
    map_publish_period_ = std::max(0.1, map_publish_period_);
    autosave_period_ = std::max(1.0, autosave_period_);

    if (hit_log_odds_ <= 0.0 || miss_log_odds_ >= 0.0) {
      ROS_WARN("Invalid Bayesian probabilities; using hit=0.70, miss=0.40");
      hit_log_odds_ = probabilityToLogOdds(0.70);
      miss_log_odds_ = probabilityToLogOdds(0.40);
    }
  }

  static double clampProbability(double probability) {
    return std::max(0.001, std::min(0.999, probability));
  }

  static double probabilityToLogOdds(double probability) {
    return std::log(probability / (1.0 - probability));
  }

  void odomCallback(const nav_msgs::OdometryConstPtr& msg) {
    latest_odom_ = *msg;
    have_odom_ = true;
  }

  VoxelKey voxelKey(const Point& point, double voxel_size) const {
    return {static_cast<int64_t>(std::floor(point.x / voxel_size)),
            static_cast<int64_t>(std::floor(point.y / voxel_size)),
            static_cast<int64_t>(std::floor(point.z / voxel_size))};
  }

  VoxelKey voxelKey(const Eigen::Vector3d& point, double voxel_size) const {
    return {static_cast<int64_t>(std::floor(point.x() / voxel_size)),
            static_cast<int64_t>(std::floor(point.y() / voxel_size)),
            static_cast<int64_t>(std::floor(point.z() / voxel_size))};
  }

  OccupancyVoxelState& getOrCreateOccupancy(const VoxelKey& key) {
    const auto inserted = temporal_voxels_.emplace(
        key, OccupancyVoxelState());
    OccupancyVoxelState& state = inserted.first->second;
    if (inserted.second) {
      state.generation = next_occupancy_generation_++;
    }
    return state;
  }

  bool isStatic(const OccupancyVoxelState& state) const {
    if (!dynamic_filter_enable_) {
      return true;
    }
    if (state.hit_scans < static_cast<uint32_t>(min_hit_scans_) ||
        state.log_odds < occupied_log_odds_ || state.first_hit.isZero() ||
        state.last_hit.isZero()) {
      return false;
    }
    return (state.last_hit - state.first_hit).toSec() >=
           min_observation_span_;
  }

  void updateHit(const VoxelKey& key, const ros::Time& stamp) {
    OccupancyVoxelState& state = getOrCreateOccupancy(key);
    state.last_seen = stamp;
    if (state.last_hit_scan == scan_sequence_) {
      return;
    }
    state.last_hit_scan = scan_sequence_;
    state.log_odds = std::min(max_log_odds_, state.log_odds + hit_log_odds_);
    ++state.hit_scans;
    if (state.first_hit.isZero()) {
      state.first_hit = stamp;
    }
    state.last_hit = stamp;
  }

  void updateMiss(const VoxelKey& key, const ros::Time& stamp) {
    auto it = temporal_voxels_.find(key);
    if (it == temporal_voxels_.end()) {
      return;
    }
    OccupancyVoxelState& state = it->second;
    if (state.last_miss_scan == scan_sequence_) {
      return;
    }
    state.last_miss_scan = scan_sequence_;
    state.last_seen = stamp;
    state.log_odds = std::max(min_log_odds_,
                              state.log_odds + miss_log_odds_);
    ++state.miss_scans;
    if (state.log_odds <= clearing_log_odds_) {
      temporal_voxels_.erase(it);
      ++bayesian_cleared_voxels_;
      dirty_ = true;
    }
  }

  void updateFreeSpace(
      const Eigen::Vector3d& sensor_position, const Cloud& cloud,
      const std::unordered_set<VoxelKey, VoxelKeyHash>& occupied_this_scan,
      const ros::Time& stamp) {
    if (!dynamic_filter_enable_ || cloud.empty()) {
      return;
    }
    for (std::size_t index = 0; index < cloud.size();
         index += static_cast<std::size_t>(ray_stride_)) {
      const Point& endpoint_point = cloud.points[index];
      const Eigen::Vector3d endpoint(endpoint_point.x, endpoint_point.y,
                                     endpoint_point.z);
      const Eigen::Vector3d delta = endpoint - sensor_position;
      const double full_range = delta.norm();
      if (full_range <= ray_endpoint_margin_) {
        continue;
      }
      const double clear_until = std::min(
          max_clearing_range_, full_range - ray_endpoint_margin_);
      if (clear_until <= temporal_voxel_size_) {
        continue;
      }
      const Eigen::Vector3d direction = delta / full_range;
      const int steps = static_cast<int>(
          std::floor(clear_until / temporal_voxel_size_));
      VoxelKey previous_key{std::numeric_limits<int64_t>::min(), 0, 0};
      for (int step = 1; step <= steps; ++step) {
        const Eigen::Vector3d sample = sensor_position +
            direction * (step * temporal_voxel_size_);
        const VoxelKey key = voxelKey(sample, temporal_voxel_size_);
        if (key == previous_key) {
          continue;
        }
        previous_key = key;
        if (occupied_this_scan.find(key) != occupied_this_scan.end()) {
          continue;
        }
        updateMiss(key, stamp);
      }
    }
  }

  bool getBodyTransform(const ros::Time& cloud_stamp, Eigen::Vector3d* position,
                        Eigen::Quaterniond* orientation) const {
    if (!have_odom_) {
      ROS_WARN_THROTTLE(2.0, "Mapper is waiting for FAST-LIO /Odometry");
      return false;
    }
    const double age = std::fabs((cloud_stamp - latest_odom_.header.stamp).toSec());
    if (age > max_odom_age_) {
      ROS_WARN_THROTTLE(2.0,
                        "Mapper skipped a scan: odometry differs by %.3f s", age);
      return false;
    }
    const auto& pose = latest_odom_.pose.pose;
    *position = Eigen::Vector3d(pose.position.x, pose.position.y,
                                pose.position.z);
    *orientation = Eigen::Quaterniond(
        pose.orientation.w, pose.orientation.x, pose.orientation.y,
        pose.orientation.z);
    if (orientation->norm() < 1e-6) {
      ROS_WARN_THROTTLE(2.0, "Mapper received an invalid odometry quaternion");
      return false;
    }
    orientation->normalize();
    return true;
  }

  Cloud::Ptr prefilter(const Cloud::ConstPtr& input,
                       const Eigen::Vector3d& position,
                       const Eigen::Quaterniond& orientation) const {
    Cloud::Ptr valid(new Cloud);
    valid->reserve(input->size());
    const double min_range_sq = min_range_ * min_range_;
    const double max_range_sq = max_range_ * max_range_;
    const Eigen::Quaterniond world_to_body = orientation.conjugate();

    for (const Point& point : input->points) {
      if (!pcl::isFinite(point)) {
        continue;
      }
      const Eigen::Vector3d world_point(point.x, point.y, point.z);
      const Eigen::Vector3d body_point =
          world_to_body * (world_point - position);
      const double range_sq = body_point.squaredNorm();
      if (range_sq < min_range_sq || range_sq > max_range_sq) {
        continue;
      }
      if (self_filter_enable_ && body_point.x() >= self_min_x_ &&
          body_point.x() <= self_max_x_ && body_point.y() >= self_min_y_ &&
          body_point.y() <= self_max_y_ && body_point.z() >= self_min_z_ &&
          body_point.z() <= self_max_z_) {
        continue;
      }
      valid->push_back(point);
    }

    Cloud::Ptr downsampled(new Cloud);
    pcl::VoxelGrid<Point> voxel;
    voxel.setLeafSize(scan_voxel_size_, scan_voxel_size_, scan_voxel_size_);
    voxel.setInputCloud(valid);
    voxel.filter(*downsampled);

    if (!radius_filter_enable_ || downsampled->empty()) {
      return downsampled;
    }
    Cloud::Ptr filtered(new Cloud);
    pcl::RadiusOutlierRemoval<Point> radius_filter;
    radius_filter.setInputCloud(downsampled);
    radius_filter.setRadiusSearch(radius_);
    radius_filter.setMinNeighborsInRadius(min_neighbors_);
    radius_filter.filter(*filtered);
    return filtered;
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    if (!getBodyTransform(msg->header.stamp, &position, &orientation)) {
      return;
    }
    if (!latest_odom_.header.frame_id.empty() && !msg->header.frame_id.empty() &&
        latest_odom_.header.frame_id != msg->header.frame_id) {
      ROS_ERROR_THROTTLE(
          2.0, "Mapper frame mismatch: cloud='%s', odometry='%s'",
          msg->header.frame_id.c_str(), latest_odom_.header.frame_id.c_str());
      return;
    }

    Cloud::Ptr input(new Cloud);
    pcl::fromROSMsg(*msg, *input);
    Cloud::Ptr filtered = prefilter(input, position, orientation);
    Cloud static_scan;
    Cloud dynamic_scan;
    static_scan.reserve(filtered->size());
    if (publish_dynamic_) {
      dynamic_scan.reserve(filtered->size());
    }
    ++scan_sequence_;

    std::unordered_set<VoxelKey, VoxelKeyHash> occupied_this_scan;
    occupied_this_scan.reserve(filtered->size());
    for (const Point& point : filtered->points) {
      occupied_this_scan.insert(voxelKey(point, temporal_voxel_size_));
    }

    for (const Point& point : filtered->points) {
      const VoxelKey key = voxelKey(point, temporal_voxel_size_);
      if (dynamic_filter_enable_) {
        updateHit(key, msg->header.stamp);
      }
      const OccupancyVoxelState* occupancy = nullptr;
      if (dynamic_filter_enable_) {
        occupancy = &temporal_voxels_.at(key);
      }
      const bool static_point = !dynamic_filter_enable_ ||
                                (occupancy != nullptr && isStatic(*occupancy));
      if (static_point) {
        static_scan.push_back(point);
      } else if (publish_dynamic_) {
        dynamic_scan.push_back(point);
      }

      const VoxelKey map_key = voxelKey(point, map_voxel_size_);
      MapVoxelState& map_state = map_voxels_[map_key];
      const uint64_t generation = dynamic_filter_enable_
                                      ? occupancy->generation
                                      : 1;
      if (map_state.sample_count == 0 ||
          map_state.occupancy_generation != generation) {
        map_state = MapVoxelState();
        map_state.occupancy_key = key;
        map_state.occupancy_generation = generation;
      }
      map_state.sum += Eigen::Vector3d(point.x, point.y, point.z);
      map_state.intensity_sum += point.intensity;
      ++map_state.sample_count;
      dirty_ = true;
    }


    updateFreeSpace(position, *filtered, occupied_this_scan,
                    msg->header.stamp);

    frame_id_ = msg->header.frame_id;
    last_cloud_stamp_ = msg->header.stamp;
    publishCloud(static_scan, static_scan_pub_, msg->header);
    if (publish_dynamic_) {
      publishCloud(dynamic_scan, dynamic_pub_, msg->header);
    }
    maybeCleanup(msg->header.stamp);

    ROS_INFO_THROTTLE(
        5.0,
        "Mapper: input=%zu filtered=%zu static_scan=%zu map=%zu occupancy=%zu "
        "bayes_cleared=%llu",
        input->size(), filtered->size(), static_scan.size(),
        map_voxels_.size(), temporal_voxels_.size(),
        static_cast<unsigned long long>(bayesian_cleared_voxels_));
  }

  void publishCloud(const Cloud& cloud, const ros::Publisher& publisher,
                    const std_msgs::Header& header) const {
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    output.header = header;
    publisher.publish(output);
  }

  Cloud buildMapCloud() {
    Cloud map;
    map.reserve(map_voxels_.size());
    for (auto it = map_voxels_.begin(); it != map_voxels_.end();) {
      const MapVoxelState& state = it->second;
      if (state.sample_count == 0) {
        it = map_voxels_.erase(it);
        continue;
      }
      if (dynamic_filter_enable_) {
        const auto occupancy_it = temporal_voxels_.find(state.occupancy_key);
        if (occupancy_it == temporal_voxels_.end() ||
            occupancy_it->second.generation !=
                state.occupancy_generation) {
          it = map_voxels_.erase(it);
          dirty_ = true;
          continue;
        }
        if (!isStatic(occupancy_it->second)) {
          ++it;
          continue;
        }
      }
      Point point;
      const Eigen::Vector3d mean = state.sum / state.sample_count;
      point.x = static_cast<float>(mean.x());
      point.y = static_cast<float>(mean.y());
      point.z = static_cast<float>(mean.z());
      point.intensity =
          static_cast<float>(state.intensity_sum / state.sample_count);
      map.push_back(point);
      ++it;
    }
    map.width = static_cast<uint32_t>(map.size());
    map.height = 1;
    map.is_dense = true;
    return map;
  }

  void maybeCleanup(const ros::Time& now) {
    if (!last_cleanup_.isZero() &&
        (now - last_cleanup_).toSec() < cleanup_period_ &&
        temporal_voxels_.size() <= max_voxels_) {
      return;
    }
    last_cleanup_ = now;
    for (auto it = temporal_voxels_.begin();
         it != temporal_voxels_.end();) {
      const bool expired = !isStatic(it->second) &&
                           !it->second.last_seen.isZero() &&
                           (now - it->second.last_seen).toSec() >
                               candidate_timeout_;
      if (expired) {
        it = temporal_voxels_.erase(it);
      } else {
        ++it;
      }
    }
    if (temporal_voxels_.size() > max_voxels_) {
      ROS_ERROR_THROTTLE(
          5.0, "Mapper voxel limit exceeded (%zu > %zu); save/reset or raise "
               "dynamic_filter/max_voxels",
          temporal_voxels_.size(), max_voxels_);
    }
    if (map_voxels_.size() > max_map_voxels_) {
      ROS_ERROR_THROTTLE(
          5.0, "Mapper fine-map voxel limit exceeded (%zu > %zu)",
          map_voxels_.size(), max_map_voxels_);
    }
  }

  void publishMapTimer(const ros::TimerEvent&) {
    if (frame_id_.empty()) {
      return;
    }
    const Cloud map = buildMapCloud();
    std_msgs::Header header;
    header.frame_id = frame_id_;
    header.stamp = last_cloud_stamp_;
    publishCloud(map, static_map_pub_, header);
  }

  bool saveMap(std_srvs::Trigger::Request&,
               std_srvs::Trigger::Response& response) {
    response.success = saveMapToDisk(&response.message);
    if (response.success) {
      ROS_INFO_STREAM(response.message);
    }
    return true;
  }

  bool saveMapToDisk(std::string* message) {
    const Cloud map = buildMapCloud();
    if (map.empty()) {
      *message = "No confirmed static map points are available";
      return false;
    }
    try {
      const boost::filesystem::path output(output_path_);
      if (output.has_parent_path()) {
        boost::filesystem::create_directories(output.parent_path());
      }
      if (pcl::io::savePCDFileBinary(output_path_, map) != 0) {
        *message = "PCL failed to write " + output_path_;
        return false;
      }
    } catch (const std::exception& error) {
      *message = error.what();
      return false;
    }
    dirty_ = false;
    *message = "Saved " + std::to_string(map.size()) +
               " fine static-map points to " + output_path_;
    return true;
  }

  void autosaveTimer(const ros::TimerEvent&) {
    if (!dirty_) {
      return;
    }
    std::string message;
    if (!saveMapToDisk(&message)) {
      ROS_ERROR_STREAM_THROTTLE(5.0, "Automatic map save failed: " << message);
    } else {
      ROS_INFO_STREAM(message);
    }
  }

  bool resetMap(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
    temporal_voxels_.clear();
    map_voxels_.clear();
    frame_id_.clear();
    scan_sequence_ = 0;
    bayesian_cleared_voxels_ = 0;
    dirty_ = false;
    ROS_WARN("scout_pointcloud_mapper map was reset");
    return true;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber cloud_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher static_scan_pub_;
  ros::Publisher static_map_pub_;
  ros::Publisher dynamic_pub_;
  ros::ServiceServer save_service_;
  ros::ServiceServer reset_service_;
  ros::Timer publish_timer_;
  ros::Timer autosave_timer_;

  std::string input_cloud_;
  std::string input_odom_;
  std::string static_scan_topic_;
  std::string static_map_topic_;
  std::string dynamic_topic_;
  std::string output_path_;
  std::string frame_id_;
  double min_range_ = 0.5;
  double max_range_ = 50.0;
  double scan_voxel_size_ = 0.10;
  bool radius_filter_enable_ = true;
  double radius_ = 0.20;
  int min_neighbors_ = 2;
  bool self_filter_enable_ = false;
  double self_min_x_ = -0.55;
  double self_max_x_ = 0.55;
  double self_min_y_ = -0.45;
  double self_max_y_ = 0.45;
  double self_min_z_ = -0.40;
  double self_max_z_ = 0.25;
  bool dynamic_filter_enable_ = true;
  double temporal_voxel_size_ = 0.20;
  double hit_probability_ = 0.70;
  double miss_probability_ = 0.40;
  double occupied_probability_ = 0.72;
  double clearing_probability_ = 0.35;
  double hit_log_odds_ = 0.0;
  double miss_log_odds_ = 0.0;
  double occupied_log_odds_ = 0.0;
  double clearing_log_odds_ = 0.0;
  const double min_log_odds_ = -4.0;
  const double max_log_odds_ = 4.0;
  int min_hit_scans_ = 8;
  double min_observation_span_ = 2.0;
  int ray_stride_ = 4;
  double max_clearing_range_ = 20.0;
  double ray_endpoint_margin_ = 0.30;
  double candidate_timeout_ = 3.0;
  double cleanup_period_ = 1.0;
  std::size_t max_voxels_ = 2000000;
  double map_voxel_size_ = 0.05;
  std::size_t max_map_voxels_ = 5000000;
  double autosave_period_ = 30.0;
  bool save_on_shutdown_ = true;
  double map_publish_period_ = 2.0;
  bool publish_dynamic_ = false;
  double max_odom_age_ = 0.20;

  nav_msgs::Odometry latest_odom_;
  bool have_odom_ = false;
  uint64_t scan_sequence_ = 0;
  uint64_t next_occupancy_generation_ = 1;
  uint64_t bayesian_cleared_voxels_ = 0;
  ros::Time last_cleanup_;
  ros::Time last_cloud_stamp_;
  bool dirty_ = false;
  std::unordered_map<VoxelKey, OccupancyVoxelState, VoxelKeyHash>
      temporal_voxels_;
  std::unordered_map<VoxelKey, MapVoxelState, VoxelKeyHash> map_voxels_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "scout_pointcloud_mapper");
  PointcloudMapper mapper;
  ros::spin();
  return 0;
}
