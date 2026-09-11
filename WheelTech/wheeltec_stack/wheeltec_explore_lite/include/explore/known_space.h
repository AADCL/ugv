#pragma once
#include <algorithm>
#include <cmath>
#include <set>
#include <vector>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>
#include <geometry_msgs/Point.h>

namespace explore {
// Check actual occupancy with the physical rectangle, not inflated center
// costs applied to the rectangle a second time. Unknown/outside stay invalid.
inline bool knownFootprint(const costmap_2d::Costmap2D& map, double x, double y,
                           double yaw, double front=.27, double rear=.27,
                           double half_width=.22) {
  const double step=map.getResolution()*.5;
  if (step<=0) return false;
  const double c=std::cos(yaw), s=std::sin(yaw);
  for (double bx=-rear; bx<=front+step*.5; bx+=step)
    for (double by=-half_width; by<=half_width+step*.5; by+=step) {
      unsigned mx,my;
      if (!map.worldToMap(x+c*bx-s*by,y+s*bx+c*by,mx,my)
          || map.getCost(mx,my)>=costmap_2d::LETHAL_OBSTACLE) return false;
    }
  return true;
}

inline std::vector<geometry_msgs::Point> knownApproaches(
    const costmap_2d::Costmap2D& map,
    const std::vector<geometry_msgs::Point>& frontier, double radius=.75) {
  std::set<unsigned> cells;
  const int n=std::ceil(radius/map.getResolution());
  for (const auto& p:frontier) {
    unsigned fx,fy;
    if (!map.worldToMap(p.x,p.y,fx,fy)) continue;
    for (int dy=-n;dy<=n;++dy) for (int dx=-n;dx<=n;++dx) {
      const int x=static_cast<int>(fx)+dx,y=static_cast<int>(fy)+dy;
      if (x<0 || y<0 || x>=static_cast<int>(map.getSizeInCellsX())
          || y>=static_cast<int>(map.getSizeInCellsY())
          || std::hypot(dx,dy)*map.getResolution()>radius) continue;
      if (map.getCost(x,y)<costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
        cells.insert(map.getIndex(x,y));
    }
  }
  std::vector<geometry_msgs::Point> result;
  for (const auto index:cells) {
    unsigned x,y;
    map.indexToCells(index,x,y);
    geometry_msgs::Point p;
    map.mapToWorld(x,y,p.x,p.y);
    result.push_back(p);
  }
  return result;
}
} // namespace explore
