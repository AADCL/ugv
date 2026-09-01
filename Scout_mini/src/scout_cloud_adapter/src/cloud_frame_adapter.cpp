#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <string>

class CloudFrameAdapter
{
public:
    CloudFrameAdapter()
        : tf_listener_(tf_buffer_)
    {
        ros::NodeHandle pnh("~");

        pnh.param<std::string>(
            "input_topic",
            input_topic_,
            "/cloud_registered_body");

        pnh.param<std::string>(
            "output_topic",
            output_topic_,
            "/cloud_registered_base");

        pnh.param<std::string>(
            "target_frame",
            target_frame_,
            "base_link");

        pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
            output_topic_,
            2);

        sub_ = nh_.subscribe(
            input_topic_,
            2,
            &CloudFrameAdapter::cloudCallback,
            this);

        ROS_INFO(
            "Cloud adapter: %s -> %s, frame=%s",
            input_topic_.c_str(),
            output_topic_.c_str(),
            target_frame_.c_str());
    }

private:
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        try
        {
            geometry_msgs::TransformStamped transform =
                tf_buffer_.lookupTransform(
                    target_frame_,
                    msg->header.frame_id,
                    msg->header.stamp,
                    ros::Duration(0.1));

            sensor_msgs::PointCloud2 output;

            tf2::doTransform(
                *msg,
                output,
                transform);

            output.header.stamp = msg->header.stamp;
            output.header.frame_id = target_frame_;

            pub_.publish(output);
        }
        catch (const tf2::TransformException& ex)
        {
            ROS_WARN_THROTTLE(
                2.0,
                "Cloud TF failed: %s",
                ex.what());
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::string input_topic_;
    std::string output_topic_;
    std::string target_frame_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "scout_cloud_adapter");

    CloudFrameAdapter node;

    ros::spin();

    return 0;
}

