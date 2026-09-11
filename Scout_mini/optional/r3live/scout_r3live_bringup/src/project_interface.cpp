#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <Eigen/Geometry>
#include <boost/bind/bind.hpp>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

using Matrix = Eigen::Matrix4d;

static double number(const XmlRpc::XmlRpcValue& value) {
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) return static_cast<int>(value);
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) return static_cast<double>(value);
  throw std::runtime_error("Non-numeric geometry parameter");
}

static Matrix geometry(const XmlRpc::XmlRpcValue& value) {
  Matrix t = Matrix::Identity();
  const double deg = std::acos(-1.0) / 180.0;
  const double roll = number(value["roll_deg"]) * deg;
  const double pitch = number(value["pitch_deg"]) * deg;
  const double yaw = number(value["yaw_deg"]) * deg;
  t.block<3, 3>(0, 0) = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
  t.block<3, 1>(0, 3) << number(value["x"]), number(value["y"]), number(value["z"]);
  if (!t.allFinite()) throw std::runtime_error("Non-finite geometry");
  return t;
}

static Matrix flatMatrix(const ros::NodeHandle& node, const std::string& key) {
  std::vector<double> v;
  if (!node.getParam(key, v) || v.size() != 16) throw std::runtime_error("Missing 4x4 " + key);
  Matrix t;
  for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c) t(r, c) = v[r * 4 + c];
  const Eigen::Matrix3d r = t.block<3, 3>(0, 0);
  if (!t.allFinite() || !t.row(3).isApprox(Eigen::RowVector4d(0, 0, 0, 1), 1e-7) ||
      !(r.transpose() * r).isApprox(Eigen::Matrix3d::Identity(), 2e-4) ||
      std::abs(r.determinant() - 1.0) > 2e-4) throw std::runtime_error("Invalid rigid " + key);
  return t;
}

static geometry_msgs::TransformStamped transform(const Matrix& t, const std::string& parent,
                                                 const std::string& child, ros::Time stamp) {
  geometry_msgs::TransformStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = parent;
  msg.child_frame_id = child;
  msg.transform.translation.x = t(0, 3);
  msg.transform.translation.y = t(1, 3);
  msg.transform.translation.z = t(2, 3);
  Eigen::Quaterniond q(t.block<3, 3>(0, 0));
  q.normalize();
  msg.transform.rotation.x = q.x(); msg.transform.rotation.y = q.y();
  msg.transform.rotation.z = q.z(); msg.transform.rotation.w = q.w();
  return msg;
}

static nav_msgs::Odometry odometry(const Matrix& t, const std::string& parent,
                                    const std::string& child, ros::Time stamp) {
  nav_msgs::Odometry msg;
  const auto tr = transform(t, parent, child, stamp);
  msg.header = tr.header; msg.child_frame_id = child;
  msg.pose.pose.position.x = t(0, 3); msg.pose.pose.position.y = t(1, 3);
  msg.pose.pose.position.z = t(2, 3); msg.pose.pose.orientation = tr.transform.rotation;
  // R3LIVE does not publish usable covariance/twist here. Never claim exact zero noise.
  // These pose-only compatibility outputs must not be used as velocity observations.
  for (int i = 0; i < 6; ++i) {
    msg.pose.covariance[i * 7] = 1e6;
    msg.twist.covariance[i * 7] = 1e6;
  }
  return msg;
}

class ProjectInterface {
 public:
  ProjectInterface() : private_("~"), cloud_sub_(node_, "/r3live/cloud_registered", 10),
      odom_sub_(node_, "/r3live/odometry", 10), sync_(cloud_sub_, odom_sub_, 10) {
    XmlRpc::XmlRpcValue g, entries;
    if (!private_.getParam("odom_to_camera_init", g) || !private_.getParam("transforms", entries))
      throw std::runtime_error("Scout geometry is required");
    odom_camera_init_ = geometry(g);
    bool found = false;
    for (int i = 0; i < entries.size(); ++i) {
      const auto& e = entries[i];
      if (static_cast<std::string>(e["parent"]) == "base_link" &&
          static_cast<std::string>(e["child"]) == "body" && static_cast<bool>(e["publish_inverse"])) {
        base_body_ = geometry(e); found = true;
      }
    }
    if (!found) throw std::runtime_error("Expected inverse base_link -> body geometry");
    imu_camera_link_ = flatMatrix(private_, "scout_project_interface/imu_to_camera_link");
    imu_camera_optical_ = flatMatrix(private_, "scout_project_interface/imu_to_camera_optical");
    std::vector<double> il;
    if (!private_.getParam("r3live_lio/lidar_to_imu_translation", il) || il.size() != 3)
      throw std::runtime_error("LiDAR/IMU internal translation is required");
    imu_lidar_ = Matrix::Identity();
    for (int i = 0; i < 3; ++i) imu_lidar_(i, 3) = il[i];
    if (!imu_lidar_.allFinite()) throw std::runtime_error("Invalid LiDAR/IMU translation");
    odom_pub_ = node_.advertise<nav_msgs::Odometry>("/Odometry", 10);
    base_odom_pub_ = node_.advertise<nav_msgs::Odometry>("/fastlio_odom", 10);
    world_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 2);
    body_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/cloud_registered_body", 2);
    base_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/cloud_registered_base", 2);
    sync_.registerCallback(boost::bind(&ProjectInterface::receive, this,
                                     boost::placeholders::_1, boost::placeholders::_2));
    last_receipt_ = ros::WallTime::now();
    watchdog_ = node_.createWallTimer(ros::WallDuration(0.5), &ProjectInterface::watchdog, this);
    ROS_INFO("Scout R3LIVE interface: exact scan/pose stamps; waiting for stationary first pair");
  }

  bool failed() const { return failed_; }

 private:
  void watchdog(const ros::WallTimerEvent&) {
    if ((ros::WallTime::now() - last_receipt_).toSec() > (initialized_ ? 2.0 : 60.0))
      fail("No synchronized scan/pose; restart the complete R3LIVE session");
  }
  void fail(const std::string& reason) {
    failed_ = true;
    ROS_FATAL_STREAM("R3LIVE project interface invalid: " << reason);
    ros::shutdown();
  }
  void cloud(const sensor_msgs::PointCloud2& input, const Matrix& t,
             const std::string& frame, ros::Publisher& pub) {
    if (!pub.getNumSubscribers()) return;
    sensor_msgs::PointCloud2 output = input;
    // Preserve fields, padding, organization and point count; no PCL conversion,
    // filtering or downsampling. Handle organized row padding explicitly.
    uint32_t offsets[3] = {};
    for (const auto& f : input.fields) {
      if (f.name == "x") offsets[0] = f.offset;
      if (f.name == "y") offsets[1] = f.offset;
      if (f.name == "z") offsets[2] = f.offset;
    }
    const Eigen::Matrix3f rotation = t.block<3, 3>(0, 0).cast<float>();
    const Eigen::Vector3f translation = t.block<3, 1>(0, 3).cast<float>();
    for (uint32_t row = 0; row < input.height; ++row) {
      for (uint32_t col = 0; col < input.width; ++col) {
        const size_t start = static_cast<size_t>(row) * input.row_step +
                             static_cast<size_t>(col) * input.point_step;
        Eigen::Vector3f point;
        for (int i = 0; i < 3; ++i) std::memcpy(&point[i], &input.data[start + offsets[i]], 4);
        point = rotation * point + translation;
        for (int i = 0; i < 3; ++i) std::memcpy(&output.data[start + offsets[i]], &point[i], 4);
      }
    }
    output.header.stamp = input.header.stamp;
    output.header.frame_id = frame;
    pub.publish(output);
  }
  void receive(const sensor_msgs::PointCloud2ConstPtr& scan, const nav_msgs::OdometryConstPtr& pose) {
    try {
      if (scan->header.frame_id != "r3live_world" || pose->header.frame_id != "r3live_world" ||
          pose->child_frame_id != "r3live_imu") throw std::runtime_error("Unexpected raw frame");
      const ros::Time stamp = pose->header.stamp;
      if (stamp.isZero() || (initialized_ && stamp <= last_stamp_))
        throw std::runtime_error("Zero, repeated or backwards source stamp");
      // Raw clouds and poses have the SAME lidar_end_time; never use latest TF or approximate sync.
      if (scan->header.stamp != stamp) throw std::runtime_error("Scan/pose time mismatch");
      if (scan->is_bigendian || scan->point_step == 0 ||
          scan->row_step < static_cast<uint64_t>(scan->width) * scan->point_step ||
          scan->data.size() < static_cast<uint64_t>(scan->row_step) * scan->height)
        throw std::runtime_error("Malformed cloud storage");
      for (const std::string name : {"x", "y", "z"}) {
        bool valid = false;
        for (const auto& f : scan->fields)
          if (f.name == name && f.datatype == sensor_msgs::PointField::FLOAT32 && f.count == 1 &&
              static_cast<uint64_t>(f.offset) + 4 <= scan->point_step) valid = true;
        if (!valid) throw std::runtime_error("Missing FLOAT32 coordinate " + name);
      }
      const auto& p = pose->pose.pose.position;
      const auto& r = pose->pose.pose.orientation;
      Eigen::Quaterniond q(r.w, r.x, r.y, r.z);
      if (!q.coeffs().allFinite() || std::abs(q.squaredNorm() - 1.0) > 1e-3)
        throw std::runtime_error("Invalid pose quaternion");
      Matrix world_imu = Matrix::Identity();
      world_imu.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
      world_imu.block<3, 1>(0, 3) << p.x, p.y, p.z;
      if (!world_imu.allFinite()) throw std::runtime_error("Invalid pose position");
      if (!initialized_) {
        // Body is the project's existing IMU-origin convention. Align base_link at
        // first valid sample to odom identity, even if R3LIVE initializes rotated.
        camera_init_world_ = odom_camera_init_.inverse() * base_body_ * world_imu.inverse();
        static_tf_.sendTransform(std::vector<geometry_msgs::TransformStamped>{
          transform(camera_init_world_, "camera_init", "r3live_world", stamp),
          transform(Matrix::Identity(), "body", "r3live_imu", stamp),
          transform(imu_lidar_, "body", "livox_frame", stamp),
          transform(Matrix::Identity(), "livox_frame", "livox", stamp),
          transform(imu_camera_link_, "body", "r3live_camera_link", stamp),
          transform(imu_camera_optical_, "body", "r3live_camera_optical", stamp)});
        initialized_ = true;
        ROS_INFO("Scout R3LIVE interface aligned: first base_link pose is odom identity");
      }
      const Matrix camera_init_body = camera_init_world_ * world_imu;
      const Matrix odom_base = odom_camera_init_ * camera_init_body * base_body_.inverse();
      dynamic_tf_.sendTransform(transform(camera_init_body, "camera_init", "body", stamp));
      odom_pub_.publish(odometry(camera_init_body, "camera_init", "body", stamp));
      base_odom_pub_.publish(odometry(odom_base, "odom", "base_link", stamp));
      cloud(*scan, camera_init_world_, "camera_init", world_pub_);
      const Matrix imu_world = world_imu.inverse();
      cloud(*scan, imu_world, "body", body_pub_);
      cloud(*scan, base_body_ * imu_world, "base_link", base_pub_);
      last_stamp_ = stamp;
      last_receipt_ = ros::WallTime::now();
    } catch (const std::exception& e) { fail(e.what()); }
  }

  ros::NodeHandle node_, private_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
  message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, nav_msgs::Odometry> sync_;
  ros::Publisher odom_pub_, base_odom_pub_, world_pub_, body_pub_, base_pub_;
  tf2_ros::TransformBroadcaster dynamic_tf_;
  tf2_ros::StaticTransformBroadcaster static_tf_;
  Matrix odom_camera_init_, base_body_, camera_init_world_, imu_lidar_;
  Matrix imu_camera_link_, imu_camera_optical_;
  bool initialized_ = false;
  bool failed_ = false;
  ros::Time last_stamp_;
  ros::WallTime last_receipt_;
  ros::WallTimer watchdog_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "scout_r3live_project_interface");
  try { ProjectInterface adapter; ros::spin(); return adapter.failed() ? 1 : 0; }
  catch (const std::exception& e) { ROS_FATAL("%s", e.what()); return 1; }
  return 0;
}
