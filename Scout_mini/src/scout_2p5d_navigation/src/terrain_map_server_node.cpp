#include "scout_2p5d_navigation/terrain_map.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

#include <nav_msgs/OccupancyGrid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace wt = scout_2p5d_navigation;

class TerrainMapServer {
 public:
  TerrainMapServer() : pnh_("~") {
    std::string path;
    pnh_.param<std::string>("terrain_map_yaml", path, "");
    pnh_.param("visualization_max_slope_deg", visualization_max_slope_, 20.0);
    if (path.empty()) throw std::runtime_error("terrain_map_yaml is required");
    map_ = wt::loadTerrainMap(path);
    elevation_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
        "/terrain_2p5d/elevation_cloud", 1, true);
    cost_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(
        "/terrain_2p5d/traversability_cost", 1, true);
    slope_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(
        "/terrain_2p5d/slope", 1, true);
    publish();
  }

 private:
  nav_msgs::OccupancyGrid gridBase() const {
    nav_msgs::OccupancyGrid grid;
    grid.header.frame_id = map_.frame_id;
    grid.header.stamp = ros::Time::now();
    grid.info.map_load_time = grid.header.stamp;
    grid.info.resolution = map_.resolution;
    grid.info.width = map_.width;
    grid.info.height = map_.height;
    grid.info.origin.position.x = map_.origin_x;
    grid.info.origin.position.y = map_.origin_y;
    grid.info.origin.orientation.w = 1.0;
    grid.data.resize(map_.size(), -1);
    return grid;
  }

  void publish() {
    nav_msgs::OccupancyGrid cost = gridBase();
    nav_msgs::OccupancyGrid slope = gridBase();
    pcl::PointCloud<pcl::PointXYZI> elevation;
    elevation.header.frame_id = map_.frame_id;
    elevation.reserve(map_.size());
    for (uint32_t y = 0; y < map_.height; ++y) {
      for (uint32_t x = 0; x < map_.width; ++x) {
        const std::size_t id = map_.index(x, y);
        if (map_.cost[id] == 255 || !std::isfinite(map_.elevation[id])) continue;
        cost.data[id] = map_.cost[id] >= 254 ? 100 :
            static_cast<int8_t>(std::round(100.0 * map_.cost[id] / 252.0));
        if (std::isfinite(map_.slope_deg[id]))
          slope.data[id] = static_cast<int8_t>(std::round(100.0 * std::min(
              1.0, static_cast<double>(map_.slope_deg[id]) / visualization_max_slope_)));
        double wx, wy;
        map_.mapToWorld(x, y, &wx, &wy);
        pcl::PointXYZI p;
        p.x = wx;
        p.y = wy;
        p.z = map_.elevation[id];
        p.intensity = map_.cost[id];
        elevation.push_back(p);
      }
    }
    elevation.width = elevation.size();
    elevation.height = 1;
    sensor_msgs::PointCloud2 cloud;
    pcl::toROSMsg(elevation, cloud);
    cloud.header.frame_id = map_.frame_id;
    cloud.header.stamp = ros::Time::now();
    elevation_pub_.publish(cloud);
    cost_pub_.publish(cost);
    slope_pub_.publish(slope);
    ROS_INFO("Published persistent 2.5D terrain topics (%zu elevation cells)", elevation.size());
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  wt::TerrainMap map_;
  ros::Publisher elevation_pub_;
  ros::Publisher cost_pub_;
  ros::Publisher slope_pub_;
  double visualization_max_slope_ = 20.0;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "scout_terrain_map_server");
  try {
    TerrainMapServer server;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("2.5D terrain map server failed: %s", error.what());
    return 1;
  }
  return 0;
}
