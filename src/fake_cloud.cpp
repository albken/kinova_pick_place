#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

static const std::string PCD_FILE = "src/kinova_pick_place/data/pcd/cube/cloud_passthrough.pcd";
static const std::string FRAME_ID = "camera_color_frame";

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("pcd_cloud_pub");

    auto pub = node->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/camera/depth/color/points",
        rclcpp::QoS(1).transient_local().reliable());

    tf2_ros::Buffer tf_buffer(node->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);

    while (rclcpp::ok())
    {
        try
        {
            tf_buffer.lookupTransform("base_link", FRAME_ID, tf2::TimePointZero);
            break;
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(node->get_logger(), "Waiting for TF '%s': %s", FRAME_ID.c_str(), ex.what());
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile(PCD_FILE, *cloud) == -1)
    {
        RCLCPP_FATAL(node->get_logger(), "Couldn't read file %s", PCD_FILE.c_str());
        rclcpp::shutdown();
        return -1;
    }

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.frame_id = FRAME_ID;
    while (rclcpp::ok())
    {
        msg.header.stamp = node->now();
        pub->publish(msg);
        rclcpp::spin_some(node);
        rclcpp::sleep_for(std::chrono::milliseconds(500));
    }

    rclcpp::shutdown();
    return 0;
}