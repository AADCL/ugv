// Export the existing obstacle layer BEFORE inflation. No second cloud
// classifier or ray tracer: local control and exploration use the same evidence.
#include <costmap_2d/obstacle_layer.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pluginlib/class_list_macros.h>

namespace wheeltec_exploration {
class ObservedObstacleLayer : public costmap_2d::ObstacleLayer {
 public:
  void onInitialize() override {
    costmap_2d::ObstacleLayer::onInitialize();
    ros::NodeHandle nh("~/" + name_);
    std::string topic;
    nh.param<std::string>("observation_map_topic", topic,
                          "/exploration/observed_local_map");
    publisher_ = nh.advertise<nav_msgs::OccupancyGrid>(topic, 1, true);
    load_time_ = ros::Time::now();
  }

  void updateCosts(costmap_2d::Costmap2D& master, int x0, int y0,
                   int x1, int y1) override {
    costmap_2d::ObstacleLayer::updateCosts(master, x0, y0, x1, y1);
    const auto now = ros::Time::now();
    if (!enabled_ || !isCurrent() || (now-last_publish_).toSec() < .5) return;
    last_publish_ = now;
    nav_msgs::OccupancyGrid grid;
    grid.header.stamp = now;
    grid.header.frame_id = layered_costmap_->getGlobalFrameID();
    grid.info.map_load_time = load_time_;
    grid.info.resolution = getResolution();
    grid.info.width = getSizeInCellsX();
    grid.info.height = getSizeInCellsY();
    grid.info.origin.position.x = getOriginX();
    grid.info.origin.position.y = getOriginY();
    grid.info.origin.orientation.w = 1;
    grid.data.resize(grid.info.width * grid.info.height);
    const auto* cells = getCharMap();
    for (size_t i=0; i<grid.data.size(); ++i)
      grid.data[i] = cells[i] == costmap_2d::FREE_SPACE ? 0 :
          cells[i] == costmap_2d::LETHAL_OBSTACLE ? 100 : -1;
    publisher_.publish(grid);
  }
 private:
  ros::Publisher publisher_;
  ros::Time last_publish_, load_time_;
};
}
PLUGINLIB_EXPORT_CLASS(wheeltec_exploration::ObservedObstacleLayer, costmap_2d::Layer)
