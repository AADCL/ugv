/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <explore/explore.h>
#include <explore/boundary_goal.h>
#include <explore/known_space.h>
#include <nav_msgs/GetPlan.h>

#include <thread>

inline static bool operator==(const geometry_msgs::Point& one,
                              const geometry_msgs::Point& two)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist < 0.01;
}

namespace explore
{
Explore::Explore()
  : private_nh_("~")
  , geometry_nh_("~geometry")
  , local_nh_("~local_geometry")
  , tf_listener_(ros::Duration(10.0))
  , costmap_client_(private_nh_, relative_nh_, &tf_listener_)
  , geometry_client_(geometry_nh_, relative_nh_, &tf_listener_)
  , local_client_(local_nh_, relative_nh_, &tf_listener_)
  , move_base_client_("move_base")
  , prev_distance_(0)
  , last_markers_count_(0)
{
  double timeout;
  double min_frontier_size;
  private_nh_.param("planner_frequency", planner_frequency_, 1.0);
  private_nh_.param("progress_timeout", timeout, 30.0);
  progress_timeout_ = ros::Duration(timeout);
  private_nh_.param("visualize", visualize_, false);
  private_nh_.param("potential_scale", potential_scale_, 1e-3);
  private_nh_.param("orientation_scale", orientation_scale_, 0.0);
  private_nh_.param("gain_scale", gain_scale_, 1.0);
  private_nh_.param("min_frontier_size", min_frontier_size, 0.5);
  private_nh_.param("preview_only", preview_only_, false);
  double xy_tolerance = .15;
  relative_nh_.param("/move_base/TebLocalPlannerROS/xy_goal_tolerance", xy_tolerance, .15);
  private_nh_.param("minimum_goal_distance", minimum_goal_distance_, .40);
  minimum_goal_distance_ = std::max(minimum_goal_distance_,
      xy_tolerance + costmap_client_.getCostmap()->getResolution());
  selected_goal_publisher_ = private_nh_.advertise<geometry_msgs::PoseStamped>("selected_goal", 1, true);
  make_plan_client_ = relative_nh_.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");

  search_ = frontier_exploration::FrontierSearch(costmap_client_.getCostmap(),
                                                 potential_scale_, gain_scale_,
                                                 min_frontier_size);

  if (visualize_) {
    marker_array_publisher_ =
        private_nh_.advertise<visualization_msgs::MarkerArray>("frontiers", 10);
  }

  ROS_INFO("Waiting to connect to move_base server");
  move_base_client_.waitForServer();
  ROS_INFO("Connected to move_base server");

  exploring_timer_ =
      relative_nh_.createTimer(ros::Duration(1. / planner_frequency_),
                               [this](const ros::TimerEvent&) { makePlan(); });
}

Explore::~Explore()
{
  stop();
}

void Explore::visualizeFrontiers(
    const std::vector<frontier_exploration::Frontier>& frontiers)
{
  std_msgs::ColorRGBA blue;
  blue.r = 0;
  blue.g = 0;
  blue.b = 1.0;
  blue.a = 1.0;
  std_msgs::ColorRGBA red;
  red.r = 1.0;
  red.g = 0;
  red.b = 0;
  red.a = 1.0;
  std_msgs::ColorRGBA green;
  green.r = 0;
  green.g = 1.0;
  green.b = 0;
  green.a = 1.0;

  ROS_DEBUG("visualising %lu frontiers", frontiers.size());
  visualization_msgs::MarkerArray markers_msg;
  std::vector<visualization_msgs::Marker>& markers = markers_msg.markers;
  visualization_msgs::Marker m;

  m.header.frame_id = costmap_client_.getGlobalFrameID();
  m.header.stamp = ros::Time::now();
  m.ns = "frontiers";
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = 0;
  m.color.g = 0;
  m.color.b = 255;
  m.color.a = 255;
  // lives forever
  m.lifetime = ros::Duration(0);
  m.frame_locked = true;

  // weighted frontiers are always sorted
  double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;

  m.action = visualization_msgs::Marker::ADD;
  size_t id = 0;
  for (auto& frontier : frontiers) {
    m.type = visualization_msgs::Marker::POINTS;
    m.id = int(id);
    m.pose.position = {};
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.points = frontier.points;
    if (goalOnBlacklist(frontier.centroid)) {
      m.color = red;
    } else {
      m.color = blue;
    }
    markers.push_back(m);
    ++id;
    m.type = visualization_msgs::Marker::SPHERE;
    m.id = int(id);
    m.pose.position = frontier.initial;
    // scale frontier according to its cost (costier frontiers will be smaller)
    double scale = std::min(std::abs(min_cost * 0.4 / frontier.cost), 0.5);
    m.scale.x = scale;
    m.scale.y = scale;
    m.scale.z = scale;
    m.points = {};
    m.color = green;
    markers.push_back(m);
    ++id;
  }
  size_t current_markers_count = markers.size();

  // delete previous markers, which are now unused
  m.action = visualization_msgs::Marker::DELETE;
  for (; id < last_markers_count_; ++id) {
    m.id = int(id);
    markers.push_back(m);
  }

  last_markers_count_ = current_markers_count;
  marker_array_publisher_.publish(markers_msg);
}

void Explore::makePlan()
{
  // find frontiers
  auto pose = costmap_client_.getRobotPose();
  const auto& q = pose.orientation;
  if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
      q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w < .5) {
    ROS_WARN_THROTTLE(2.0, "No valid robot pose; not selecting an exploration goal");
    return;
  }
  if (goal_active_) {
    const double distance = std::hypot(prev_goal_.x-pose.position.x, prev_goal_.y-pose.position.y);
    if (distance < prev_distance_ - .02) {
      last_progress_ = ros::Time::now();
      prev_distance_ = distance;
    }
    if (ros::Time::now()-last_progress_ <= progress_timeout_) return;
    frontier_blacklist_.push_back(prev_goal_);
    move_base_client_.cancelGoal();
    goal_active_ = false;
    ROS_WARN("Exploration goal made no progress; waiting for cancellation before selecting another");
    return;
  }
  // get frontiers sorted according to cost
  auto frontiers = search_.searchFrom(pose.position);
  ROS_DEBUG("found %lu frontiers", frontiers.size());
  for (size_t i = 0; i < frontiers.size(); ++i) {
    ROS_DEBUG("frontier %zd cost: %f", i, frontiers[i].cost);
  }

  if (frontiers.empty()) {
    // Map refresh can temporarily remove frontiers. Keep the timer alive;
    // the session supervisor owns the multi-map completion decision.
    return;
  }

  // publish frontiers as visualization markers
  if (visualize_) {
    visualizeFrontiers(frontiers);
  }

  geometry_msgs::Point target_position;
  if (!selectBoundaryGoal(frontiers, pose, target_position)) {
    ROS_WARN_THROTTLE(5.0, "Frontiers exist but no boundary goal outside arrival tolerance has a valid plan; waiting for map update");
    return;
  }

  // send goal to move_base if we have something new to pursue
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.pose.position = target_position;
  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(target_yaw_);
  goal.target_pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.target_pose.header.stamp = ros::Time::now();
  selected_goal_publisher_.publish(goal.target_pose);
  ROS_INFO_THROTTLE(2.0, "Boundary goal (%.3f, %.3f), robot distance %.3f m, preview=%s",
      target_position.x, target_position.y,
      std::hypot(target_position.x-pose.position.x, target_position.y-pose.position.y),
      preview_only_ ? "true" : "false");
  if (preview_only_) return;
  prev_goal_ = target_position;
  prev_distance_ = std::hypot(target_position.x-pose.position.x, target_position.y-pose.position.y);
  last_progress_ = ros::Time::now();
  goal_active_ = true;
  move_base_client_.sendGoal(
      goal, [this, target_position](
                const actionlib::SimpleClientGoalState& status,
                const move_base_msgs::MoveBaseResultConstPtr& result) {
        reachedGoal(status, result, target_position);
      });
}

bool Explore::selectBoundaryGoal(
    const std::vector<frontier_exploration::Frontier>& frontiers,
    const geometry_msgs::Pose& pose, geometry_msgs::Point& target)
{
  const double resolution = costmap_client_.getCostmap()->getResolution();
  auto* geometry = geometry_client_.getCostmap();
  auto* local_map = local_client_.getCostmap();
  if (geometry_client_.getGlobalFrameID() != costmap_client_.getGlobalFrameID()) return false;
  if (!geometry_client_.isFresh(2.5) || !local_client_.isFresh(.75)) return false;
  tf::StampedTransform map_to_local;
  try {
    tf_listener_.lookupTransform(local_client_.getGlobalFrameID(),
        costmap_client_.getGlobalFrameID(), ros::Time(0), map_to_local);
  } catch (const tf::TransformException&) { return false; }
  int calls = 0;
  // Preserve upstream information-gain ranking; choose a supported observation
  // pose on the known side. Do not send the chassis center into unknown cells.
  for (const auto& frontier : frontiers) {
    const auto approaches = knownApproaches(*geometry, frontier.points);
    const auto candidates = boundaryCandidates(approaches, frontier.centroid,
        pose.position, tf::getYaw(pose.orientation), minimum_goal_distance_);
    std::vector<geometry_msgs::Point> tried;
    for (const auto& candidate : candidates) {
      if (goalOnBlacklist(candidate)) continue;
      const double heading=std::atan2(candidate.y-pose.position.y,candidate.x-pose.position.x);
      if (!knownFootprint(*geometry,candidate.x,candidate.y,heading,.43)) continue;
      if (std::any_of(tried.begin(), tried.end(), [&](const auto& p) {
            return std::hypot(p.x-candidate.x,p.y-candidate.y) < 2*resolution;
          })) continue;
      tried.push_back(candidate);
      if (++calls > 12) return false;
      nav_msgs::GetPlan request;
      request.request.start.header.frame_id = costmap_client_.getGlobalFrameID();
      request.request.start.header.stamp = ros::Time::now();
      request.request.start.pose = pose;
      request.request.goal = request.request.start;
      request.request.goal.pose.position = candidate;
      request.request.goal.pose.orientation = tf::createQuaternionMsgFromYaw(
          std::atan2(candidate.y-pose.position.y,candidate.x-pose.position.x));
      request.request.tolerance = 0.0;
      if (!make_plan_client_.call(request) || request.response.plan.poses.empty()) continue;
      const auto& end = request.response.plan.poses.back().pose.position;
      if (std::hypot(end.x-candidate.x,end.y-candidate.y) > resolution ||
          std::hypot(end.x-pose.position.x,end.y-pose.position.y) < minimum_goal_distance_) continue;
      bool executable = true;
      const auto& path = request.response.plan.poses;
      double final_heading=heading;
      for (size_t i=0; i<path.size(); ++i) {
        const auto& p=path[i].pose.position;
        const auto& next=path[std::min(i+1,path.size()-1)].pose.position;
        if (std::hypot(next.x-p.x,next.y-p.y)>1e-6)
          final_heading=std::atan2(next.y-p.y,next.x-p.x);
        if (!knownFootprint(*geometry,p.x,p.y,final_heading)) { executable=false; break; }
        if (std::hypot(p.x-pose.position.x,p.y-pose.position.y) <= .75) {
          const auto lp=map_to_local*tf::Vector3(p.x,p.y,0);
          if (!knownFootprint(*local_map,lp.x(),lp.y(),
                             final_heading+tf::getYaw(map_to_local.getRotation()),.43)) {
            executable=false; break;
          }
        }
      }
      if (!executable || !knownFootprint(*geometry,end.x,end.y,final_heading,.43)) continue;
      target_yaw_=final_heading;
      target = candidate;
      return true;
    }
  }
  return false;
}

bool Explore::goalOnBlacklist(const geometry_msgs::Point& goal)
{
  constexpr static size_t tolerace = 5;
  costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();

  // check if a goal is on the blacklist for goals that we're pursuing
  for (auto& frontier_goal : frontier_blacklist_) {
    double x_diff = fabs(goal.x - frontier_goal.x);
    double y_diff = fabs(goal.y - frontier_goal.y);

    if (x_diff < tolerace * costmap2d->getResolution() &&
        y_diff < tolerace * costmap2d->getResolution())
      return true;
  }
  return false;
}

void Explore::reachedGoal(const actionlib::SimpleClientGoalState& status,
                          const move_base_msgs::MoveBaseResultConstPtr&,
                          const geometry_msgs::Point& frontier_goal)
{
  ROS_DEBUG("Reached goal with status: %s", status.toString().c_str());
  if (!(frontier_goal == prev_goal_)) return;
  goal_active_ = false;
  if (status == actionlib::SimpleClientGoalState::ABORTED) {
    frontier_blacklist_.push_back(frontier_goal);
    ROS_DEBUG("Adding current goal to black list");
  }

  // find new goal immediatelly regardless of planning frequency.
  // execute via timer to prevent dead lock in move_base_client (this is
  // callback for sendGoal, which is called in makePlan). the timer must live
  // until callback is executed.
  oneshot_ = relative_nh_.createTimer(
      ros::Duration(0, 0), [this](const ros::TimerEvent&) { makePlan(); },
      true);
}

void Explore::start()
{
  exploring_timer_.start();
}

void Explore::stop()
{
  if (!preview_only_) move_base_client_.cancelGoal();
  exploring_timer_.stop();
  ROS_INFO("Exploration stopped.");
}

}  // namespace explore

int main(int argc, char** argv)
{
  ros::init(argc, argv, "explore");
  explore::Explore explore;
  ros::spin();

  return 0;
}
