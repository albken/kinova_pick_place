// kinova_pick_place/include/kinova_pick_place.hpp
#ifndef KINOVA_PICK_PLACE_HPP
#define KINOVA_PICK_PLACE_HPP

#include <rclcpp/rclcpp.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>

#include <moveit_msgs/srv/get_planning_scene.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/empty.hpp>

namespace mtc = moveit::task_constructor;

class KinovaPickPlace
{
public:
    KinovaPickPlace(const rclcpp::NodeOptions &options);

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

    void runPickPlaceTask();

private:
    mtc::Task createTaskPickPlace();

    // TODO bool waitForControllers(); : Automate Simulation to 1 launchfile

    bool callClearOctomapService();

    bool callWaitForOctomapService();

    bool moveRobotToReadyPose();

    void publishScanReadyPose();

    bool addCubeFromTF();

    bool runMoveItTaskConstructor();

    mtc::Task task_;

    rclcpp::Node::SharedPtr node_;

    rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr planning_scene_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_octomap_client_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr scan_ready_pub_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    moveit::planning_interface::PlanningSceneInterface psi_;

    std::string arm_group_;
    std::string gripper_group_;
    std::string hand_frame_;
    std::string parent_frame_;
    std::string child_frame_;

    double object_size_;
    double place_x_;
    double place_y_;
    double place_z_;

    std::map<std::string, double> ready_pose_{
        {"joint_1", -3.08037066223177e-05},
        {"joint_2", -0.13650598445271078},
        {"joint_3", -1.7642098609757548},
        {"joint_4", 4.036498956738749e-05},
        {"joint_5", -1.565415112656074},
        {"joint_6", 1.570891537038878},
    };
};

#endif // KINOVA_PICK_PLACE_HPP