// kinova_pick_place/include/color_cloud_detector.hpp
#ifndef COLOR_CLOUD_DETECTOR_HPP
#define COLOR_CLOUD_DETECTOR_HPP

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

class CubeFromCloud
{
public:
    CubeFromCloud(const rclcpp::NodeOptions &options);

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

private:
    void detectionCubeAlgorithm(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void selectedCubeColor(const std_msgs::msg::String::SharedPtr msg);
    void scanRobotReadyPose(const std_msgs::msg::Bool::SharedPtr msg);

    rclcpp::Node::SharedPtr node_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pc_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_color_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_scan_ready_;

    std::string parent_frame_;
    std::string child_frame_;
    std::string cloud_topic_;

    std::string color_cube_;
    std::atomic<bool> ready_to_scan_{false};
};

#endif // COLOR_CLOUD_DETECTOR_HPP