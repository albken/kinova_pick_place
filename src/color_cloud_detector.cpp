// TODO: remove roll and pitch, dont use OBB at all. use only yaw from minAreaRect for clean pick

#include "kinova_pick_place/color_cloud_detector.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>

using pclXYZRGB = pcl::PointCloud<pcl::PointXYZRGB>;

CubeFromCloud::CubeFromCloud(const rclcpp::NodeOptions &options)
    : node_{std::make_shared<rclcpp::Node>("cube_from_cloud", options)}
{
    node_->declare_parameter<std::string>("parent_frame", "base_link");
    node_->declare_parameter<std::string>("child_frame", "cube");
    node_->declare_parameter<std::string>("cloud_topic", "/camera/depth/color/points");

    parent_frame_ = node_->get_parameter("parent_frame").as_string();
    child_frame_ = node_->get_parameter("child_frame").as_string();
    cloud_topic_ = node_->get_parameter("cloud_topic").as_string();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

    sub_scan_ready_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/scan_ready", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg)
        { scanRobotReadyPose(msg); });

    sub_color_ = node_->create_subscription<std_msgs::msg::String>(
        "/cube_color", 10,
        [this](const std_msgs::msg::String::SharedPtr msg)
        { selectedCubeColor(msg); });

    sub_pc_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, rclcpp::SensorDataQoS().keep_last(1),
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        { detectionCubeAlgorithm(msg); });
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
CubeFromCloud::getNodeBaseInterface()
{
    return node_->get_node_base_interface();
}

void CubeFromCloud::scanRobotReadyPose(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data)
    {
        ready_to_scan_.store(true);
        RCLCPP_INFO(node_->get_logger(), "Robot in ready pose. Detection enabled.");
    }
}

void CubeFromCloud::selectedCubeColor(const std_msgs::msg::String::SharedPtr msg)
{
    color_cube_ = msg->data;
    RCLCPP_INFO(node_->get_logger(), "Cube color set to: %s", color_cube_.c_str());
}

void CubeFromCloud::detectionCubeAlgorithm(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    if (!ready_to_scan_.load())
    {
        RCLCPP_WARN(node_->get_logger(), "Not ready to scan yet, skipping cloud.");
        return;
    }
    if (color_cube_.empty())
    {
        RCLCPP_WARN(node_->get_logger(), "Cube color not set yet, skipping cloud.");
        return;
    }
    // pcl::PCDWriter writer;

    pclXYZRGB::Ptr cloud(new pclXYZRGB);
    pcl::fromROSMsg(*msg, *cloud);
    // writer.write<pcl::PointXYZRGB>("cloud_raw.pcd", *cloud, false);

    pclXYZRGB::Ptr cloud_vox(new pclXYZRGB);
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.005f, 0.005f, 0.005f);
    vg.filter(*cloud_vox);
    // writer.write<pcl::PointXYZRGB>("cloud_voxelgrid.pcd", *cloud_vox, false);

    pclXYZRGB::Ptr cloud_filtered(new pclXYZRGB);
    pcl::PassThrough<pcl::PointXYZRGB> p;
    p.setInputCloud(cloud_vox);
    p.setFilterLimits(0.0, 1.0);
    p.setFilterFieldName("z");
    p.filter(*cloud_filtered);
    // writer.write<pcl::PointXYZRGB>("cloud_passthrough.pcd", *cloud_filtered, false);

    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(500);
    seg.setDistanceThreshold(0.01);
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);

    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);

    pclXYZRGB::Ptr cloud_no_table(new pclXYZRGB);
    extract.setNegative(true);
    extract.filter(*cloud_no_table);
    cloud_filtered.swap(cloud_no_table);
    // writer.write<pcl::PointXYZRGB>("cloud_no_table.pcd", *cloud_filtered, false);

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.02);
    ec.setMinClusterSize(50);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);
    if (cluster_indices.empty())
    {
        RCLCPP_WARN(node_->get_logger(), "No clusters found, skipping.");
        return;
    }

    bool found = false;

    for (const auto &cluster : cluster_indices)
    {
        pclXYZRGB::Ptr cloud_cluster(new pclXYZRGB);
        cloud_cluster->reserve(cluster.indices.size());

        double sum_r = 0, sum_g = 0, sum_b = 0;

        for (const auto &idx : cluster.indices)
        {
            const auto &p = (*cloud_filtered)[idx];
            cloud_cluster->push_back(p);

            sum_r += p.r;
            sum_g += p.g;
            sum_b += p.b;
        }

        double n = static_cast<double>(cluster.indices.size());
        double mean_r = sum_r / n;
        double mean_g = sum_g / n;
        double mean_b = sum_b / n;

        bool is_blue = mean_r < 100 && mean_g < 100 && mean_b > 100;
        bool is_red = mean_r > 150 && mean_g < 100 && mean_b < 100;

        if (color_cube_ == "blue")
        {
            if (!is_blue)
                continue;
        }
        else if (color_cube_ == "red")
        {
            if (!is_red)
                continue;
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(), "Cant identify %s cube", color_cube_.c_str());
            continue;
        }

        pcl::MomentOfInertiaEstimation<pcl::PointXYZRGB> moi;
        moi.setInputCloud(cloud_cluster);
        moi.compute();

        pcl::PointXYZRGB min_pt_OBB, max_pt_OBB, position_OBB;
        Eigen::Matrix3f rot_OBB;
        moi.getOBB(min_pt_OBB, max_pt_OBB, position_OBB, rot_OBB);

        float dx = max_pt_OBB.x - min_pt_OBB.x;
        float dy = max_pt_OBB.y - min_pt_OBB.y;
        float dz = max_pt_OBB.z - min_pt_OBB.z;

        RCLCPP_INFO(node_->get_logger(), "OBB dx=%f dy=%f dz=%f", dx, dy, dz);

        float is_xy_cube = std::max(dx, dy) / std::min(dx, dy);

        RCLCPP_INFO(node_->get_logger(), "Ratio XY: %f", is_xy_cube);

        if (is_xy_cube >= 1.7f)
        {
            RCLCPP_WARN(node_->get_logger(), "Ratio XY: %f", is_xy_cube);
            continue;
        }

        float a = coefficients->values[0];
        float b = coefficients->values[1];
        float c = coefficients->values[2];
        float d = coefficients->values[3];

        float num = std::sqrt(a * a + b * b + c * c);

        float h_max = -std::numeric_limits<float>::infinity();
        for (const auto &p : cloud_cluster->points)
        {
            float dist = (a * p.x + b * p.y + c * p.z + d) / num;
            h_max = std::max(h_max, dist);
        }

        RCLCPP_INFO(node_->get_logger(), "h_max: %f", h_max);
        if (std::abs(h_max) > 0.015f)
        {
            RCLCPP_WARN(node_->get_logger(), "h_max: %f", h_max);
            continue;
        }

        float h_min = h_max - 0.008f;

        pclXYZRGB::Ptr cloud_top_lyr(new pclXYZRGB);
        cloud_top_lyr->reserve(cloud_cluster->size());
        for (const auto &p : cloud_cluster->points)
        {
            float h = (a * p.x + b * p.y + c * p.z + d) / num;

            if (h >= h_min && h <= h_max)
                cloud_top_lyr->push_back(p);
        }
        // writer.write<pcl::PointXYZRGB>("cloud_top_lyr.pcd", *cloud_top_lyr, false);

        std::vector<cv::Point2f> pts;
        pts.reserve(cloud_top_lyr->size());
        for (const auto &p : cloud_top_lyr->points)
        {
            cv::Point2f pt(p.x, p.y);
            pts.push_back(pt);
        }
        if (pts.size() < 3)
        {
            RCLCPP_WARN(node_->get_logger(), "Too few points for minAreaRect");
            continue;
        }

        std::vector<cv::Point2f> hull2d;
        cv::convexHull(pts, hull2d);

        double area_hull = std::abs(cv::contourArea(hull2d));
        auto r = cv::minAreaRect(pts);
        double area_rect = r.size.width * r.size.height;

        double fill = area_hull / area_rect;

        RCLCPP_INFO(node_->get_logger(), "Hull fill: %f", fill);
        if (fill < 0.70)
            continue;

        RCLCPP_INFO(node_->get_logger(), "Found cube.");

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud_cluster, centroid);

        geometry_msgs::msg::PoseStamped cube_pose_cam;
        cube_pose_cam.header.frame_id = msg->header.frame_id;
        cube_pose_cam.header.stamp = msg->header.stamp;
        Eigen::Quaternionf q(rot_OBB);
        q.normalize();
        cube_pose_cam.pose.position.x = centroid[0];
        cube_pose_cam.pose.position.y = centroid[1];
        cube_pose_cam.pose.position.z = centroid[2];
        cube_pose_cam.pose.orientation.w = q.w();
        cube_pose_cam.pose.orientation.x = q.x();
        cube_pose_cam.pose.orientation.y = q.y();
        cube_pose_cam.pose.orientation.z = q.z();

        geometry_msgs::msg::PoseStamped cube_pose_parent;
        try
        {
            cube_pose_parent = tf_buffer_->transform(cube_pose_cam, parent_frame_, tf2::durationFromSec(0.2));
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(node_->get_logger(), "TF transform error: %s", e.what());
            continue;
        }

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = msg->header.stamp;
        t.header.frame_id = parent_frame_;
        t.child_frame_id = child_frame_;
        t.transform.translation.x = cube_pose_parent.pose.position.x;
        t.transform.translation.y = cube_pose_parent.pose.position.y;
        t.transform.translation.z = cube_pose_parent.pose.position.z;
        t.transform.rotation = cube_pose_parent.pose.orientation;

        tf_broadcaster_->sendTransform(t);

        found = true;
        break;
    }

    if (!found)
    {
        RCLCPP_WARN(node_->get_logger(), "no cube cluster found");
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<CubeFromCloud>(options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->getNodeBaseInterface());
    executor.spin();

    rclcpp::shutdown();
    return 0;
}