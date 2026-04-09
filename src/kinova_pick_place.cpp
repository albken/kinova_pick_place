#include "kinova_pick_place/kinova_pick_place.hpp"

#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#include <tf2/utils.h>
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif
#include <moveit/move_group_interface/move_group_interface.h>

using namespace std::chrono_literals;

KinovaPickPlace::KinovaPickPlace(const rclcpp::NodeOptions &options)
    : node_{std::make_shared<rclcpp::Node>("kinova_pick_place", options)}
{

    planning_scene_client_ = node_->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    clear_octomap_client_ = node_->create_client<std_srvs::srv::Empty>("/clear_octomap");

    scan_ready_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/scan_ready", 10);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    node_->declare_parameter<std::string>("arm_group", "manipulator");
    node_->declare_parameter<std::string>("gripper_group", "gripper");
    node_->declare_parameter<std::string>("hand_frame", "robotiq_85_base_link");
    node_->declare_parameter<std::string>("parent_frame", "base_link");
    node_->declare_parameter<std::string>("child_frame", "cube");

    node_->declare_parameter<double>("object_size", 0.035);
    node_->declare_parameter<double>("place_x", 0.2);
    node_->declare_parameter<double>("place_y", -0.3);
    node_->declare_parameter<double>("place_z", 0.03);

    arm_group_ = node_->get_parameter("arm_group").as_string();
    gripper_group_ = node_->get_parameter("gripper_group").as_string();
    hand_frame_ = node_->get_parameter("hand_frame").as_string();
    parent_frame_ = node_->get_parameter("parent_frame").as_string();
    child_frame_ = node_->get_parameter("child_frame").as_string();

    object_size_ = node_->get_parameter("object_size").as_double();
    place_x_ = node_->get_parameter("place_x").as_double();
    place_y_ = node_->get_parameter("place_y").as_double();
    place_z_ = node_->get_parameter("place_z").as_double();
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr KinovaPickPlace::getNodeBaseInterface()
{
    return node_->get_node_base_interface();
}

bool KinovaPickPlace::addCubeFromTF()
{
    try
    {
        auto tf = tf_buffer_->lookupTransform(parent_frame_, child_frame_,
                                              tf2::TimePointZero, std::chrono::seconds(1000));

        moveit_msgs::msg::CollisionObject object;
        object.header.frame_id = parent_frame_;
        object.id = child_frame_;

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
        primitive.dimensions = {object_size_, object_size_, object_size_};

        geometry_msgs::msg::Pose pose;
        pose.position.x = tf.transform.translation.x;
        pose.position.y = tf.transform.translation.y;
        pose.position.z = tf.transform.translation.z;

        tf2::Quaternion q;
        tf2::fromMsg(tf.transform.rotation, q);

        double yaw = tf2::getYaw(q);

        tf2::Quaternion q_yaw;
        q_yaw.setRPY(0.0, 0.0, yaw);
        q_yaw.normalize();

        pose.orientation = tf2::toMsg(q_yaw);

        object.primitives.push_back(primitive);
        object.primitive_poses.push_back(pose);

        object.operation = object.ADD;

        psi_.applyCollisionObject(object);

        RCLCPP_INFO(node_->get_logger(), "Cube added to the scene at [%f %f %f]", pose.position.x, pose.position.y, pose.position.z);
        return true;
    }
    catch (const tf2::TransformException &e)
    {
        RCLCPP_ERROR(node_->get_logger(), "No TF for frame 'cube' failed with exception: %s", e.what());
        return false;
    }
}

void KinovaPickPlace::publishScanReadyPose()
{
    std_msgs::msg::Bool msg;
    msg.data = true;
    scan_ready_pub_->publish(msg);
    RCLCPP_INFO(node_->get_logger(), "Scan-ready signal published.");
}

bool KinovaPickPlace::moveRobotToReadyPose()
{
    auto arm_move_group_ = moveit::planning_interface::MoveGroupInterface(node_, arm_group_);

    arm_move_group_.setPlanningTime(5.0);
    arm_move_group_.setMaxVelocityScalingFactor(0.2);
    arm_move_group_.setMaxAccelerationScalingFactor(0.2);
    arm_move_group_.setPlannerId("RRTConnectkConfigDefault");

    arm_move_group_.setJointValueTarget(ready_pose_);

    if (arm_move_group_.move() != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "Move to Ready failed");
        return false;
    }
    return true;
}

bool KinovaPickPlace::callWaitForOctomapService()
{
    RCLCPP_INFO(node_->get_logger(), "Waiting for Octomap data...");
    if (!planning_scene_client_->wait_for_service(5s))
    {
        RCLCPP_ERROR(node_->get_logger(), "/get_planning_scene not available.");
        return false;
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);

    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
    {
        auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
        request->components.components = moveit_msgs::msg::PlanningSceneComponents::OCTOMAP;

        auto result = planning_scene_client_->async_send_request(request);

        if (result.wait_for(500ms) == std::future_status::ready)
        {
            const auto response = result.get();
            const auto &octomap = response->scene.world.octomap.octomap;

            if (!octomap.data.empty())
            {
                RCLCPP_INFO(node_->get_logger(), "Octomap received.");
                return true;
            }
        }
        std::this_thread::sleep_for(200ms);
    }
    RCLCPP_WARN(node_->get_logger(), "Timeout waiting for Octomap.");
    return false;
}

bool KinovaPickPlace::callClearOctomapService()
{
    RCLCPP_INFO(node_->get_logger(), "Clearing Octomap...");
    if (!clear_octomap_client_->wait_for_service(5s))
    {
        RCLCPP_ERROR(node_->get_logger(), "clear_octomap service not available.");
        return false;
    }

    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    auto result = clear_octomap_client_->async_send_request(request);

    if (result.wait_for(3s) != std::future_status::ready)
    {
        RCLCPP_ERROR(node_->get_logger(), "Timeout while clearing octomap.");
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "Octomap cleared.");
    return true;
}

void KinovaPickPlace::runPickPlaceTask()
{
    RCLCPP_INFO(node_->get_logger(), "Starting Kinova Pick Place");

    auto ids = psi_.getKnownObjectNames();
    if (!ids.empty())
        psi_.removeCollisionObjects(ids);

    RCLCPP_INFO(node_->get_logger(), "System ready.");

    if (!moveRobotToReadyPose())
        return;

    publishScanReadyPose();

    if (!addCubeFromTF())
        return;

    if (!callClearOctomapService())
        return;

    if (!callWaitForOctomapService())
        return;

    if (!runMoveItTaskConstructor())
        return;

    RCLCPP_INFO(node_->get_logger(), "Kinova Pick Place completed");
}

bool KinovaPickPlace::runMoveItTaskConstructor()
{
    task_ = createTaskPickPlace();

    try
    {
        task_.init();
    }
    catch (mtc::InitStageException &e)
    {
        RCLCPP_ERROR(node_->get_logger(), "Task creation failed: %s", e.what());
        return false;
    }
    if (!task_.plan())
    {
        RCLCPP_ERROR(node_->get_logger(), "Planning failed");
        return false;
    }
    task_.introspection().publishSolution(*task_.solutions().front());

    auto result = task_.execute(*task_.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "Execution failed");
        return false;
    }

    return true;
}

mtc::Task KinovaPickPlace::createTaskPickPlace()
{
    mtc::Task task;
    task.stages()->setName("Kinova Pick Place Task");
    task.loadRobotModel(node_);

    task.setProperty("group", arm_group_);
    task.setProperty("eef", gripper_group_);
    task.setProperty("ik_frame", hand_frame_);

    auto pipeline_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
    pipeline_planner->setPlannerId("RRTConnectkConfigDefault");
    pipeline_planner->setMaxVelocityScalingFactor(0.2);
    pipeline_planner->setMaxAccelerationScalingFactor(0.2);

    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(0.15);
    cartesian_planner->setMaxAccelerationScalingFactor(0.15);
    cartesian_planner->setStepSize(0.002);

    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

    auto current = std::make_unique<mtc::stages::CurrentState>("current");
    auto *current_ptr = current.get();
    task.add(std::move(current));

    auto stage_open_hand =
        std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
    stage_open_hand->setGroup(gripper_group_);
    stage_open_hand->setGoal("Open");
    task.add(std::move(stage_open_hand));

    auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
        "move to pick",
        mtc::stages::Connect::GroupPlannerVector{{arm_group_, pipeline_planner}});
    stage_move_to_pick->setTimeout(10.0);
    stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_pick));

    mtc::Stage *attach_object_stage = nullptr;
    {
        auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
        task.properties().exposeTo(grasp->properties(), {"eef", "group", "ik_frame"});
        grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                              {"eef", "group", "ik_frame"});
        {
            auto stage =
                std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
            stage->properties().set("marker_ns", "approach_object");
            stage->properties().set("link", hand_frame_);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
            stage->setMinMaxDistance(0.05, 0.10);

            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = parent_frame_;
            vec.vector.z = -1.0;

            stage->setDirection(vec);
            grasp->insert(std::move(stage));
        }
        {
            auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
            stage->properties().configureInitFrom(mtc::Stage::PARENT);
            stage->properties().set("marker_ns", "grasp_pose");
            stage->setPreGraspPose("Open");
            stage->setObject(child_frame_);
            stage->setAngleDelta(M_PI / 2.0);
            stage->setMonitoredStage(current_ptr);

            auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
            wrapper->setMaxIKSolutions(32);
            wrapper->setMinSolutionDistance(0.5);

            Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
            Eigen::Matrix3d R = (Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())).toRotationMatrix();
            R = R * Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();

            grasp_frame_transform.linear() = R;
            grasp_frame_transform.translation().z() = 0.15;

            wrapper->setIKFrame(grasp_frame_transform, hand_frame_);

            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
            wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
            grasp->insert(std::move(wrapper));
        }
        {
            auto stage =
                std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (gripper,object)");
            stage->allowCollisions(child_frame_, task.getRobotModel()->getJointModelGroup(gripper_group_)->getLinkModelNamesWithCollisionGeometry(), true);
            grasp->insert(std::move(stage));
        }
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
            stage->setGroup(gripper_group_);
            stage->setGoal("Close");
            grasp->insert(std::move(stage));
        }
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
            stage->attachObject(child_frame_, hand_frame_);
            attach_object_stage = stage.get();
            grasp->insert(std::move(stage));
        }
        {
            auto stage =
                std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
            stage->setMinMaxDistance(0.05, 0.15);
            stage->setIKFrame(hand_frame_);
            stage->properties().set("marker_ns", "lift_object");

            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = parent_frame_;
            vec.vector.z = 1.0;
            stage->setDirection(vec);
            grasp->insert(std::move(stage));
        }

        task.add(std::move(grasp));
    }
    {
        auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
            "move to place",
            mtc::stages::Connect::GroupPlannerVector{{arm_group_, pipeline_planner}});
        stage_move_to_place->setTimeout(5.0);
        stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
        task.add(std::move(stage_move_to_place));
    }
    {
        auto place = std::make_unique<mtc::SerialContainer>("place object");
        task.properties().exposeTo(place->properties(), {"eef", "group", "ik_frame"});
        place->properties().configureInitFrom(mtc::Stage::PARENT,
                                              {"eef", "group", "ik_frame"});
        {
            auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
            stage->properties().configureInitFrom(mtc::Stage::PARENT);
            stage->properties().set("marker_ns", "place_pose");
            stage->setObject(child_frame_);

            Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
            Eigen::Matrix3d R = (Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())).toRotationMatrix();
            R = R * Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();

            grasp_frame_transform.linear() = R;
            grasp_frame_transform.translation().z() = 0.15;

            geometry_msgs::msg::PoseStamped target_pose_msg;
            target_pose_msg.header.frame_id = parent_frame_;
            target_pose_msg.pose.position.x = place_x_;
            target_pose_msg.pose.position.y = place_y_;
            target_pose_msg.pose.position.z = place_z_;
            target_pose_msg.pose.orientation.w = 1.0;
            stage->setPose(target_pose_msg);
            stage->setMonitoredStage(attach_object_stage);

            auto wrapper =
                std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
            wrapper->setMaxIKSolutions(8);
            wrapper->setMinSolutionDistance(1.0);

            wrapper->setIKFrame(grasp_frame_transform, hand_frame_);

            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
            wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
            place->insert(std::move(wrapper));
        }
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
            stage->setGroup(gripper_group_);
            stage->setGoal("Open");
            place->insert(std::move(stage));
        }
        {
            auto stage =
                std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (gripper,object)");
            stage->allowCollisions(child_frame_,
                                   task.getRobotModel()
                                       ->getJointModelGroup(gripper_group_)
                                       ->getLinkModelNamesWithCollisionGeometry(),
                                   false);
            place->insert(std::move(stage));
        }
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
            stage->detachObject(child_frame_, hand_frame_);
            place->insert(std::move(stage));
        }
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
            stage->setMinMaxDistance(0.05, 0.10);
            stage->setIKFrame(hand_frame_);
            stage->properties().set("marker_ns", "retreat");

            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = parent_frame_;
            vec.vector.z = 1.0;
            stage->setDirection(vec);
            place->insert(std::move(stage));
        }
        task.add(std::move(place));
    }
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("return Ready", pipeline_planner);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
        stage->setGoal(ready_pose_);
        stage->setTimeout(5.0);
        task.add(std::move(stage));
    }
    return task;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto mtc_task_node = std::make_shared<KinovaPickPlace>(options);

    rclcpp::executors::MultiThreadedExecutor executor;
    auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]()
                                                     {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface()); });

    mtc_task_node->runPickPlaceTask();

    spin_thread->join();
    rclcpp::shutdown();
    return 0;
}
