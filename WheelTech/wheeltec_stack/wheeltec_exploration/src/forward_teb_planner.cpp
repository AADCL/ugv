// Exploration-only adapter: align with the path before asking TEB to advance.
// Ordinary navigation continues to load the upstream plugin unchanged.
#include <algorithm>
#include <cmath>
#include <mutex>
#include <nav_core/base_local_planner.h>
#include <nav_msgs/Path.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/String.h>
#include <teb_local_planner/teb_local_planner_ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

namespace wheeltec_exploration {
class ForwardTebPlanner : public nav_core::BaseLocalPlanner {
 public:
  void initialize(std::string name, tf2_ros::Buffer* tf,
                  costmap_2d::Costmap2DROS* costmap) override {
    tf_ = tf;
    costmap_ = costmap;
    ros::NodeHandle nh("~/" + name);
    nh.param("max_vel_theta", max_w_, .40);
    nh.param("acc_lim_theta", acc_w_, .30);
    nh.param("xy_goal_tolerance", goal_tolerance_, .15);
    ros::param::param("/wheeltec_safety/footprint_front", front_, .25);
    ros::param::param("/wheeltec_safety/footprint_rear", rear_, .25);
    ros::param::param("/wheeltec_safety/footprint_half_width", half_width_, .20);
    ros::param::param("/wheeltec_safety/footprint_margin", margin_, .02);
    ros::param::param("/wheeltec_safety/min_forward_clearance", forward_clearance_, .18);
    ros::param::param("/wheeltec_safety/reaction_time", reaction_, .20);
    ros::param::param("/wheeltec_safety/linear_deceleration", decel_v_, .20);
    ros::param::param("/wheeltec_safety/angular_deceleration", decel_w_, .40);
    ros::param::param("/wheeltec_safety/max_prediction_horizon", horizon_, 1.50);
    // Keep the established parameter namespace and local_plan/feedback topics.
    teb_.initialize(name, tf, costmap);
    plan_pub_ = nh.advertise<nav_msgs::Path>("local_plan", 1);
    mode_pub_ = nh.advertise<std_msgs::String>("execution_mode", 1, true);
    last_time_ = ros::WallTime::now();
  }

  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override {
    if (plan.empty()) return false;
    if (plan_.empty() || std::hypot(plan.back().pose.position.x-plan_.back().pose.position.x,
                                  plan.back().pose.position.y-plan_.back().pose.position.y) > .10) {
      aligning_ = false;
      last_w_ = 0;
    }
    plan_ = plan;
    return teb_.setPlan(plan);
  }

  bool isGoalReached() override { return !aligning_ && teb_.isGoalReached(); }

  bool computeVelocityCommands(geometry_msgs::Twist& cmd) override {
    cmd = geometry_msgs::Twist();
    geometry_msgs::PoseStamped robot;
    if (plan_.empty() || !costmap_->getRobotPose(robot) || !costmap_->isCurrent())
      return reject("WAITING_FOR_LOCAL_DATA");
    std::vector<geometry_msgs::PoseStamped> local;
    try {
      for (auto p : plan_) {
        p.header.stamp = ros::Time(0);
        local.push_back(tf_->transform(p, robot.header.frame_id, ros::Duration(.02)));
      }
    } catch (const tf2::TransformException&) { return reject("WAITING_FOR_TF"); }
    const auto& r = robot.pose.position;
    size_t nearest = 0;
    double closest = INFINITY;
    for (size_t i=0; i<local.size(); ++i) {
      const double d = std::hypot(local[i].pose.position.x-r.x, local[i].pose.position.y-r.y);
      if (d < closest) { closest=d; nearest=i; }
    }
    size_t lookahead = nearest;
    while (lookahead+1<local.size() &&
           std::hypot(local[lookahead].pose.position.x-r.x,
                      local[lookahead].pose.position.y-r.y) < .45) ++lookahead;
    const double goal_distance = std::hypot(local.back().pose.position.x-r.x,
                                            local.back().pose.position.y-r.y);
    const double heading = goal_distance <= goal_tolerance_
        ? tf2::getYaw(local.back().pose.orientation)
        : std::atan2(local[lookahead].pose.position.y-r.y, local[lookahead].pose.position.x-r.x);
    const double error = wrap(heading-tf2::getYaw(robot.pose.orientation));
    if (std::abs(error) > 1.05 && goal_distance > goal_tolerance_) aligning_ = true;
    if (aligning_ && std::abs(error) < .20) {
      aligning_ = false;
      teb_.setPlan(plan_);
    }
    const auto now = ros::WallTime::now();
    const double dt = std::max(.01, std::min(.20, (now-last_time_).toSec()));
    last_time_ = now;
    if (aligning_) {
      const double wanted = std::copysign(std::min(max_w_, std::sqrt(2*acc_w_*std::abs(error))), error);
      cmd.angular.z = std::max(last_w_-acc_w_*dt, std::min(last_w_+acc_w_*dt, wanted));
    } else if (!teb_.computeVelocityCommands(cmd)) {
      cmd = geometry_msgs::Twist();
      return reject("TEB_NO_VALID_TRAJECTORY");
    }
    // Reject an unsupported trajectory as a whole. Never turn a reverse arc
    // into a rotation by silently deleting its translational component.
    if (!std::isfinite(cmd.linear.x) || !std::isfinite(cmd.angular.z)
        || cmd.linear.x < -1e-4 || std::abs(cmd.linear.y) > 1e-4) {
      if (std::abs(error) > .20) aligning_ = true;
      cmd = geometry_msgs::Twist();
      return reject("REPLAN_UNSUPPORTED_REVERSE");
    }
    if (cmd.linear.x < 0) cmd.linear.x = 0; // numerical noise below 0.1 mm/s
    if (!safeSweep(robot, cmd)) {
      cmd = geometry_msgs::Twist();
      return reject("LOCAL_STOP_ENVELOPE_BLOCKED");
    }
    last_w_ = cmd.angular.z;
    publishMode(aligning_ ? "ALIGN_TO_PATH" : "TEB_FORWARD");
    if (aligning_) {
      // Publish the actual stationary turn so the existing gate can verify
      // a fresh plan/command handshake and perform its independent veto.
      nav_msgs::Path p;
      p.header = robot.header;
      p.header.stamp = ros::Time::now();
      p.poses.push_back(robot);
      auto end = robot;
      tf2::Quaternion q;
      q.setRPY(0, 0, heading);
      end.pose.orientation = tf2::toMsg(q);
      p.poses.push_back(end);
      plan_pub_.publish(p);
    }
    return true;
  }

 private:
  static double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }
  void publishMode(const std::string& mode) {
    if (mode == mode_) return;
    mode_ = mode;
    std_msgs::String message;
    message.data = mode;
    mode_pub_.publish(message);
  }
  bool reject(const std::string& reason) {
    last_w_ = 0;
    publishMode(reason);
    return false;
  }
  bool safeSweep(const geometry_msgs::PoseStamped& robot, const geometry_msgs::Twist& cmd) {
    auto* map = costmap_->getCostmap();
    std::lock_guard<costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
    const double yaw = tf2::getYaw(robot.pose.orientation);
    const double c = std::cos(yaw), s = std::sin(yaw);
    // The same conservative constant-twist envelope as the independent gate.
    // Braking assumptions/clearances are not weakened for this change.
    const double duration = std::min(horizon_, reaction_ + std::max(
        std::abs(cmd.linear.x)/decel_v_, std::abs(cmd.angular.z)/decel_w_));
    const double step = map->getResolution()*.5;
    for (int i=0; i<=12; ++i) {
      const double t=duration*i/12, a=cmd.angular.z*t;
      const double px=std::abs(cmd.angular.z)<1e-6 ? cmd.linear.x*t : cmd.linear.x/cmd.angular.z*std::sin(a);
      const double py=std::abs(cmd.angular.z)<1e-6 ? 0 : cmd.linear.x/cmd.angular.z*(1-std::cos(a));
      const double ca=std::cos(yaw+a), sa=std::sin(yaw+a);
      const double end=(i==0 && cmd.linear.x>1e-4)
          ? front_+std::max(forward_clearance_, cmd.linear.x*reaction_+cmd.linear.x*cmd.linear.x/(2*decel_v_))
          : front_+margin_;
      // Sample the interior too: footprint-edge checks alone miss an obstacle
      // already inside the footprint. Half-cell spacing includes small legs.
      for (double x=-rear_-margin_; x<=end+step*.5; x+=step)
        for (double y=-half_width_-margin_; y<=half_width_+margin_+step*.5; y+=step) {
          unsigned mx,my;
          if (!map->worldToMap(robot.pose.position.x+c*px-s*py+ca*x-sa*y,
                              robot.pose.position.y+s*px+c*py+sa*x+ca*y,mx,my)
              || map->getCost(mx,my)>=costmap_2d::LETHAL_OBSTACLE) return false;
        }
    }
    return true;
  }
  teb_local_planner::TebLocalPlannerROS teb_;
  tf2_ros::Buffer* tf_ = nullptr;
  costmap_2d::Costmap2DROS* costmap_ = nullptr;
  std::vector<geometry_msgs::PoseStamped> plan_;
  ros::Publisher plan_pub_, mode_pub_;
  ros::WallTime last_time_;
  std::string mode_;
  bool aligning_ = false;
  double last_w_ = 0, max_w_, acc_w_, goal_tolerance_;
  double front_, rear_, half_width_, margin_, forward_clearance_;
  double reaction_, decel_v_, decel_w_, horizon_;
};
} // namespace wheeltec_exploration
PLUGINLIB_EXPORT_CLASS(wheeltec_exploration::ForwardTebPlanner, nav_core::BaseLocalPlanner)
