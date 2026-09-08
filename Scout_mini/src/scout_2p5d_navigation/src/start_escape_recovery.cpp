#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <boost/thread/locks.hpp>
#include <base_local_planner/costmap_model.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_core/recovery_behavior.h>
#include <pluginlib/class_list_macros.hpp>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>

namespace scout_2p5d_navigation {

class StartEscapeRecovery : public nav_core::RecoveryBehavior {
 public:
  StartEscapeRecovery() = default;

  void initialize(std::string name, tf2_ros::Buffer*,
                  costmap_2d::Costmap2DROS* global_costmap,
                  costmap_2d::Costmap2DROS* local_costmap) override {
    if (initialized_) {
      ROS_WARN("StartEscapeRecovery was initialized more than once");
      return;
    }
    name_ = std::move(name);
    global_costmap_ = global_costmap;
    local_costmap_ = local_costmap;

    ros::NodeHandle private_nh("~/" + name_);
    private_nh.param("backward_speed", backward_speed_, 0.05);
    private_nh.param("escape_distance", escape_distance_, 0.30);
    private_nh.param("lookahead_distance", lookahead_distance_, 0.10);
    private_nh.param("sample_step", sample_step_, 0.025);
    private_nh.param("control_frequency", control_frequency_, 20.0);
    private_nh.param("max_duration", max_duration_, 8.0);
    private_nh.param("minimum_progress", minimum_progress_, 0.12);
    private_nh.param("stall_timeout", stall_timeout_, 1.5);
    private_nh.param("stall_progress", stall_progress_, 0.02);
    private_nh.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "cmd_vel");
    private_nh.param<std::string>("rear_cloud_topic", rear_cloud_topic_,
                                  "/cloud_registered_terrain");
    private_nh.param<std::string>("rear_cloud_frame", rear_cloud_frame_,
                                  "terrain_sensor");
    private_nh.param("require_rear_observation", require_rear_observation_, true);
    private_nh.param("rear_min_x", rear_min_x_, -1.05);
    private_nh.param("rear_max_x", rear_max_x_, -0.55);
    private_nh.param("rear_half_width", rear_half_width_, 0.36);
    private_nh.param("rear_min_points", rear_min_points_, 20);
    private_nh.param("rear_min_x_span", rear_min_x_span_, 0.20);
    private_nh.param("rear_min_points_per_side", rear_min_points_per_side_, 5);
    private_nh.param("rear_observation_timeout", rear_observation_timeout_, 0.25);

    backward_speed_ = std::max(0.01, std::abs(backward_speed_));
    escape_distance_ = std::max(0.05, escape_distance_);
    lookahead_distance_ = std::max(0.05, lookahead_distance_);
    sample_step_ = std::max(0.01, sample_step_);
    control_frequency_ = std::max(5.0, control_frequency_);
    max_duration_ = std::max(1.0, max_duration_);
    minimum_progress_ = std::max(0.0, minimum_progress_);
    stall_timeout_ = std::max(0.5, stall_timeout_);
    stall_progress_ = std::max(0.005, stall_progress_);

    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
    rear_cloud_sub_ = nh_.subscribe(rear_cloud_topic_, 1,
                                    &StartEscapeRecovery::rearCloudCallback, this);
    initialized_ = true;
    ROS_INFO("StartEscapeRecovery ready: reverse %.2f m at %.2f m/s",
             escape_distance_, backward_speed_);
  }

  void runBehavior() override {
    if (!initialized_ || global_costmap_ == nullptr || local_costmap_ == nullptr) {
      ROS_ERROR("StartEscapeRecovery is not initialized");
      return;
    }

    geometry_msgs::PoseStamped global_pose;
    geometry_msgs::PoseStamped local_pose;
    if (!global_costmap_->getRobotPose(global_pose) ||
        !local_costmap_->getRobotPose(local_pose)) {
      ROS_ERROR("StartEscapeRecovery cannot read the robot pose");
      stopRobot();
      return;
    }

    const double global_yaw = tf2::getYaw(global_pose.pose.orientation);
    const double global_cost = footprintCost(
        global_costmap_, global_pose.pose.position.x,
        global_pose.pose.position.y, global_yaw);
    if (global_cost >= 0.0) {
      ROS_WARN("StartEscapeRecovery skipped: the global start footprint is free");
      stopRobot();
      return;
    }
    // CostmapModel distinguishes lethal collision (-1) from unknown space
    // (-2) and map-boundary failure (-3). Never turn either uncertainty into
    // an automatic motion command.
    if (global_cost != -1.0) {
      ROS_ERROR("StartEscapeRecovery refused: global footprint is unknown or outside the map");
      stopRobot();
      return;
    }

    const double local_yaw = tf2::getYaw(local_pose.pose.orientation);
    if (footprintCost(local_costmap_, local_pose.pose.position.x,
                      local_pose.pose.position.y, local_yaw) < 0.0) {
      ROS_ERROR("StartEscapeRecovery refused: live local sensing reports a collision");
      stopRobot();
      return;
    }
    if (!rearObserved()) {
      ROS_ERROR("StartEscapeRecovery refused: the rear corridor has no fresh "
                "sensor coverage");
      stopRobot();
      return;
    }

    // Preflight the whole reverse corridor using live local obstacles. The
    // global map is deliberately not consulted because it is the suspected
    // stale/false source that this recovery is intended to escape.
    for (double distance = sample_step_; distance <= escape_distance_;
         distance += sample_step_) {
      const double x = local_pose.pose.position.x - distance * std::cos(local_yaw);
      const double y = local_pose.pose.position.y - distance * std::sin(local_yaw);
      if (footprintCost(local_costmap_, x, y, local_yaw) < 0.0) {
        ROS_ERROR("StartEscapeRecovery refused: reverse corridor blocked at %.2f m",
                  distance);
        stopRobot();
        return;
      }
    }

    ROS_WARN("Global start is occupied but the local reverse corridor is clear; "
             "starting guarded reverse escape");
    const double start_x = local_pose.pose.position.x;
    const double start_y = local_pose.pose.position.y;
    const ros::WallTime begin = ros::WallTime::now();
    ros::Rate rate(control_frequency_);
    double progress = 0.0;
    double last_progress = 0.0;
    ros::WallTime last_progress_time = begin;
    bool blocked = false;

    while (ros::ok() && progress < escape_distance_ &&
           (ros::WallTime::now() - begin).toSec() < max_duration_) {
      geometry_msgs::PoseStamped current;
      if (!local_costmap_->getRobotPose(current)) {
        ROS_ERROR("StartEscapeRecovery lost the local pose");
        blocked = true;
        break;
      }
      if (!rearObserved()) {
        ROS_ERROR("StartEscapeRecovery stopped: rear sensor coverage was lost");
        blocked = true;
        break;
      }
      const double yaw = tf2::getYaw(current.pose.orientation);
      const double check_x = current.pose.position.x -
                             lookahead_distance_ * std::cos(yaw);
      const double check_y = current.pose.position.y -
                             lookahead_distance_ * std::sin(yaw);
      if (footprintCost(local_costmap_, check_x, check_y, yaw) < 0.0) {
        ROS_ERROR("StartEscapeRecovery stopped: a live obstacle entered behind");
        blocked = true;
        break;
      }

      const double dx = current.pose.position.x - start_x;
      const double dy = current.pose.position.y - start_y;
      progress = -(dx * std::cos(local_yaw) + dy * std::sin(local_yaw));
      if (progress >= last_progress + stall_progress_) {
        last_progress = progress;
        last_progress_time = ros::WallTime::now();
      } else if ((ros::WallTime::now() - last_progress_time).toSec() >=
                 stall_timeout_) {
        ROS_ERROR("StartEscapeRecovery stopped: chassis made no progress for %.2f s",
                  stall_timeout_);
        blocked = true;
        break;
      }

      geometry_msgs::Twist command;
      command.linear.x = -backward_speed_;
      cmd_vel_pub_.publish(command);
      rate.sleep();
    }
    stopRobot();

    if (!blocked && progress >= minimum_progress_) {
      ROS_WARN("StartEscapeRecovery completed %.2f m; move_base will replan",
               progress);
    } else {
      ROS_ERROR("StartEscapeRecovery made insufficient progress: %.2f m", progress);
    }
  }

 private:
  void rearCloudCallback(const sensor_msgs::PointCloud2ConstPtr& message) {
    const bool frame_matches =
        message->header.frame_id == rear_cloud_frame_ ||
        message->header.frame_id == "/" + rear_cloud_frame_;
    const double stamp_age =
        message->header.stamp.isZero()
            ? std::numeric_limits<double>::infinity()
            : (ros::Time::now() - message->header.stamp).toSec();
    if (!frame_matches || stamp_age < -0.05 ||
        stamp_age > rear_observation_timeout_) {
      ROS_ERROR_THROTTLE(
          2.0, "Rear coverage rejected: frame='%s' expected='%s', stamp age=%.3f s",
          message->header.frame_id.c_str(), rear_cloud_frame_.c_str(), stamp_age);
      return;
    }
    int count = 0;
    int left_count = 0;
    int right_count = 0;
    double min_x = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*message, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*message, "y");
      for (; x != x.end(); ++x, ++y) {
        if (std::isfinite(*x) && std::isfinite(*y) && *x >= rear_min_x_ &&
            *x <= rear_max_x_ && std::abs(*y) <= rear_half_width_) {
          ++count;
          min_x = std::min(min_x, static_cast<double>(*x));
          max_x = std::max(max_x, static_cast<double>(*x));
          if (*y >= 0.0F) ++left_count;
          else ++right_count;
        }
      }
    } catch (const std::runtime_error& error) {
      ROS_ERROR_THROTTLE(2.0, "Rear coverage cloud is invalid: %s", error.what());
      return;
    }
    std::lock_guard<std::mutex> lock(rear_mutex_);
    rear_point_count_ = count;
    rear_distribution_ok_ =
        count >= rear_min_points_ && max_x - min_x >= rear_min_x_span_ &&
        left_count >= rear_min_points_per_side_ &&
        right_count >= rear_min_points_per_side_;
    rear_received_wall_time_ = ros::WallTime::now();
    rear_cloud_stamp_ = message->header.stamp;
  }

  bool rearObserved() const {
    if (!require_rear_observation_) return true;
    std::lock_guard<std::mutex> lock(rear_mutex_);
    const double age = (ros::WallTime::now() - rear_received_wall_time_).toSec();
    const double stamp_age = rear_cloud_stamp_.isZero()
                                 ? std::numeric_limits<double>::infinity()
                                 : (ros::Time::now() - rear_cloud_stamp_).toSec();
    if (rear_received_wall_time_.isZero() || age > rear_observation_timeout_) {
      return false;
    }
    if (stamp_age < -0.05 || stamp_age > rear_observation_timeout_) {
      return false;
    }
    return rear_point_count_ >= rear_min_points_ && rear_distribution_ok_;
  }

  double footprintCost(costmap_2d::Costmap2DROS* costmap_ros, double x,
                       double y, double yaw) const {
    costmap_2d::Costmap2D* costmap = costmap_ros->getCostmap();
    const std::vector<geometry_msgs::Point> footprint =
        costmap_ros->getRobotFootprint();
    if (footprint.size() < 3) {
      ROS_ERROR_THROTTLE(2.0,
                         "StartEscapeRecovery requires a polygon footprint");
      return -3.0;
    }
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> costmap_lock(
        *costmap->getMutex());
    base_local_planner::CostmapModel model(*costmap);
    const double boundary_cost = model.footprintCost(x, y, yaw, footprint);
    if (boundary_cost < 0.0) return boundary_cost;

    const double cosine = std::cos(yaw);
    const double sine = std::sin(yaw);
    std::vector<std::pair<double, double>> polygon;
    polygon.reserve(footprint.size());
    double min_x = std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    for (const geometry_msgs::Point& point : footprint) {
      const double world_x = x + cosine * point.x - sine * point.y;
      const double world_y = y + sine * point.x + cosine * point.y;
      polygon.emplace_back(world_x, world_y);
      min_x = std::min(min_x, world_x);
      min_y = std::min(min_y, world_y);
      max_x = std::max(max_x, world_x);
      max_y = std::max(max_y, world_y);
    }

    unsigned int min_mx = 0;
    unsigned int min_my = 0;
    unsigned int max_mx = 0;
    unsigned int max_my = 0;
    if (!costmap->worldToMap(min_x, min_y, min_mx, min_my) ||
        !costmap->worldToMap(max_x, max_y, max_mx, max_my)) {
      return -3.0;
    }
    for (unsigned int my = min_my; my <= max_my; ++my) {
      for (unsigned int mx = min_mx; mx <= max_mx; ++mx) {
        double world_x = 0.0;
        double world_y = 0.0;
        costmap->mapToWorld(mx, my, world_x, world_y);
        bool inside = false;
        for (std::size_t i = 0, j = polygon.size() - 1;
             i < polygon.size(); j = i++) {
          const bool crosses =
              ((polygon[i].second > world_y) !=
               (polygon[j].second > world_y)) &&
              (world_x < (polygon[j].first - polygon[i].first) *
                                 (world_y - polygon[i].second) /
                                 (polygon[j].second - polygon[i].second) +
                             polygon[i].first);
          if (crosses) inside = !inside;
        }
        if (!inside) continue;
        const unsigned char cost = costmap->getCost(mx, my);
        if (cost == costmap_2d::NO_INFORMATION) return -2.0;
        if (cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) return -1.0;
      }
    }
    return boundary_cost;
  }

  void stopRobot() {
    geometry_msgs::Twist stop;
    ros::Rate rate(control_frequency_);
    for (int i = 0; i < 3 && ros::ok(); ++i) {
      cmd_vel_pub_.publish(stop);
      rate.sleep();
    }
  }

  ros::NodeHandle nh_;
  ros::Publisher cmd_vel_pub_;
  ros::Subscriber rear_cloud_sub_;
  costmap_2d::Costmap2DROS* global_costmap_ = nullptr;
  costmap_2d::Costmap2DROS* local_costmap_ = nullptr;
  bool initialized_ = false;
  std::string name_;
  std::string cmd_vel_topic_ = "cmd_vel";
  std::string rear_cloud_topic_ = "/cloud_registered_terrain";
  std::string rear_cloud_frame_ = "terrain_sensor";
  double backward_speed_ = 0.05;
  double escape_distance_ = 0.30;
  double lookahead_distance_ = 0.10;
  double sample_step_ = 0.025;
  double control_frequency_ = 20.0;
  double max_duration_ = 8.0;
  double minimum_progress_ = 0.12;
  double stall_timeout_ = 1.5;
  double stall_progress_ = 0.02;
  bool require_rear_observation_ = true;
  double rear_min_x_ = -1.05;
  double rear_max_x_ = -0.55;
  double rear_half_width_ = 0.36;
  int rear_min_points_ = 20;
  double rear_min_x_span_ = 0.20;
  int rear_min_points_per_side_ = 5;
  double rear_observation_timeout_ = 0.25;
  mutable std::mutex rear_mutex_;
  int rear_point_count_ = 0;
  bool rear_distribution_ok_ = false;
  ros::WallTime rear_received_wall_time_;
  ros::Time rear_cloud_stamp_;
};

}  // namespace scout_2p5d_navigation

PLUGINLIB_EXPORT_CLASS(scout_2p5d_navigation::StartEscapeRecovery,
                       nav_core::RecoveryBehavior)
