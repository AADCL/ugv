// WheelTech addition. Upstream frontier extraction/scoring remains unchanged.
#pragma once
#include <algorithm>
#include <cmath>
#include <geometry_msgs/Point.h>
#include <vector>

namespace explore {
inline std::vector<geometry_msgs::Point> boundaryCandidates(
    const std::vector<geometry_msgs::Point>& points,
    const geometry_msgs::Point& center, const geometry_msgs::Point& robot,
    double yaw, double minimum_distance) {
  std::vector<std::pair<double, geometry_msgs::Point>> ranked;
  for (const auto& p : points) {
    const double dx = p.x - robot.x, dy = p.y - robot.y;
    const double distance = std::hypot(dx, dy);
    if (!std::isfinite(distance) || distance < minimum_distance) continue;
    const double angle = std::atan2(dy, dx) - yaw;
    const double turn = std::abs(std::atan2(std::sin(angle), std::cos(angle)));
    // Project to an actual frontier cell; modestly prefer a forward approach.
    ranked.emplace_back(std::hypot(p.x-center.x, p.y-center.y) + .15*turn, p);
  }
  std::stable_sort(ranked.begin(), ranked.end(),
      [](const auto& a, const auto& b) { return a.first < b.first; });
  std::vector<geometry_msgs::Point> result;
  for (const auto& pair : ranked) result.push_back(pair.second);
  return result;
}
}  // namespace explore
