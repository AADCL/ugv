#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <Eigen/Geometry>

#include <cmath>
#include <stdexcept>
#include <string>

static double deg2rad(double deg)
{
    constexpr double kPi = 3.14159265358979323846;
    return deg * kPi / 180.0;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "scout_pcd_transform");
    ros::NodeHandle pnh("~");

    std::string input_pcd;
    std::string output_pcd;

    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double roll_deg = 0.0;
    double pitch_deg = 0.0;
    double yaw_deg = 0.0;

    pnh.param<std::string>("input_pcd", input_pcd, "");
    pnh.param<std::string>("output_pcd", output_pcd, "");

    pnh.param<double>("x", x, 0.0);
    pnh.param<double>("y", y, 0.0);
    pnh.param<double>("z", z, 0.0);

    pnh.param<double>("roll_deg", roll_deg, 0.0);
    pnh.param<double>("pitch_deg", pitch_deg, 0.0);
    pnh.param<double>("yaw_deg", yaw_deg, 0.0);

    if (input_pcd.empty() || output_pcd.empty())
    {
        ROS_FATAL("input_pcd/output_pcd is empty");
        return 1;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZI>);

    if (pcl::io::loadPCDFile<pcl::PointXYZI>(input_pcd, *cloud) < 0)
    {
        ROS_FATAL("Failed to load PCD: %s", input_pcd.c_str());
        return 1;
    }

    const float roll = static_cast<float>(deg2rad(roll_deg));
    const float pitch = static_cast<float>(deg2rad(pitch_deg));
    const float yaw = static_cast<float>(deg2rad(yaw_deg));

    // R = Rz(yaw) * Ry(pitch) * Rx(roll)
    Eigen::Affine3f tf = Eigen::Affine3f::Identity();
    tf.translation() << static_cast<float>(x),
                        static_cast<float>(y),
                        static_cast<float>(z);

    tf.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
    tf.rotate(Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()));
    tf.rotate(Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()));

    pcl::PointCloud<pcl::PointXYZI> output;
    pcl::transformPointCloud(*cloud, output, tf);

    if (pcl::io::savePCDFileBinary(output_pcd, output) < 0)
    {
        ROS_FATAL("Failed to save PCD: %s", output_pcd.c_str());
        return 1;
    }

    ROS_INFO("Input : %s", input_pcd.c_str());
    ROS_INFO("Output: %s", output_pcd.c_str());
    ROS_INFO("xyz=(%.4f, %.4f, %.4f)", x, y, z);
    ROS_INFO("rpy_deg=(%.3f, %.3f, %.3f)",
             roll_deg, pitch_deg, yaw_deg);
    ROS_INFO("points=%zu", output.size());

    return 0;
}

