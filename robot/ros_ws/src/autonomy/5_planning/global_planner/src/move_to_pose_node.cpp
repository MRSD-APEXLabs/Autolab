// ┌─────────────────────────────────────────────────────────────┐
// │  SUBSCRIBED TOPICS                                          │
// │  /task_planner/task_type         std_msgs/String            │
// │  /joint_states                   sensor_msgs/JointState     │
// │                                                             │
// └─────────────────────────────────────────────────────────────┘

// ── ROS / MoveIt includes ────────────────────────────────────
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include "dummy_cloud.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <moveit/point_cloud_collision_updater/point_cloud_collision_updater.h>
#include <visualization_msgs/msg/marker_array.hpp>
// At top of file add:
#include <moveit_msgs/msg/display_trajectory.hpp>
// ── TF2 includes ─────────────────────────────────────────────
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <geometry_msgs/msg/point_stamped.hpp>
// ── STL includes ─────────────────────────────────────────────
#include <thread>
#include <chrono>
#include <optional>
#include <string>
#include <fstream>
#include <iomanip>

static constexpr double ARM_PLANNING_TIME_SEC = 15.0;
static constexpr int INPUT_TIMEOUT_SEC = 5;
static constexpr int MAX_PLANNING_ATTEMPTS = 100;

double april_tag_x = 0.0;
double april_tag_y = 0.0;
double april_tag_z = 0.0;
static int marker_id = 0;

double wellplate_tag_x = 0.0;
double wellplate_tag_y = 0.0;
double wellplate_tag_z = 0.0;

namespace defaults
{
    static const std::string TASK_TYPE = "Move";

    void publish_marker(rclcpp::Node::SharedPtr node,
                        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub,
                        const geometry_msgs::msg::Point &pt)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = node->now();
        marker.ns = "target_points";
        marker.id = marker_id++;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position = pt;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        marker_pub->publish(marker);
    }

    // Main function to compute pose and visualize point
    geometry_msgs::msg::Pose target_pose_func(
        rclcpp::Node::SharedPtr node,
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub,
        const geometry_msgs::msg::Point &target_position,
        int offset)
    {
        geometry_msgs::msg::Pose p;

        // ── Point in top_camera frame ────────────────────────────
        geometry_msgs::msg::PointStamped cam_point;
        cam_point.header.frame_id = "top_camera";
        cam_point.header.stamp = node->now();
        cam_point.point = target_position;

        tf2_ros::Buffer tf_buffer(node->get_clock());
        tf2_ros::TransformListener tf_listener(tf_buffer, node);

        geometry_msgs::msg::PointStamped world_point;
        try
        {
            world_point = tf_buffer.transform(cam_point, "world",
                                              tf2::durationFromSec(3.0));
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_ERROR(node->get_logger(),
                         "TF transform top_camera→world failed: %s", ex.what());
            world_point.point = cam_point.point;
        }

        // ── Build pose with transformed position ─────────────────
        p.position.x = world_point.point.x;
        p.position.y = world_point.point.y;
        p.position.z = world_point.point.z + offset; // height offset for pre-grasp

        // Orientation: gripper pointing down
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, M_PI);
        q.normalize();
        p.orientation = tf2::toMsg(q);

        // ── Publish marker for RViz ──────────────────────────────
        publish_marker(node, marker_pub, p.position);

        return p;
    }
}

moveit_msgs::msg::Constraints make_ee_down_constraint()
{
    moveit_msgs::msg::OrientationConstraint ocm;
    ocm.link_name = "end_effector_p4_1";
    ocm.header.frame_id = "world";

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, M_PI); // target orientation: RPY=(0,0,π)
    q.normalize();
    ocm.orientation = tf2::toMsg(q);

    ocm.absolute_x_axis_tolerance = 0.2;  // ≈11° tilt tolerance
    ocm.absolute_y_axis_tolerance = 0.2;  // ≈11° tilt tolerance
    ocm.absolute_z_axis_tolerance = M_PI; // spin freely around Z
    ocm.weight = 1.0;

    moveit_msgs::msg::Constraints c;
    c.name = "ee_down";
    c.orientation_constraints.push_back(ocm);
    return c;
}

visualization_msgs::msg::Marker make_path_marker(
    const trajectory_msgs::msg::JointTrajectory &jt,
    moveit::core::RobotStatePtr robot_state,
    const moveit::core::JointModelGroup *jmg,
    const std::string &ee_link,
    const std::string &ns,
    float r, float g, float b,
    rclcpp::Node::SharedPtr node)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = node->now();
    marker.ns = ns;
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.01; // line width in metres
    marker.color = [&]
    { std_msgs::msg::ColorRGBA c; c.r=r; c.g=g; c.b=b; c.a=1.0; return c; }();
    marker.lifetime = rclcpp::Duration::from_seconds(3600);

    for (const auto &pt : jt.points)
    {
        robot_state->setJointGroupPositions(jmg, pt.positions);
        robot_state->updateLinkTransforms();
        const Eigen::Isometry3d &tf = robot_state->getGlobalLinkTransform(ee_link);

        geometry_msgs::msg::Point p;
        p.x = tf.translation().x();
        p.y = tf.translation().y();
        p.z = tf.translation().z();
        marker.points.push_back(p);
    }
    return marker;
}

std::optional<moveit::planning_interface::MoveGroupInterface::Plan> plan_and_publish_rrt(
    const std::string &task_type,
    const geometry_msgs::msg::Pose &target_pose,
    moveit::planning_interface::MoveGroupInterface &arm_group,
    rclcpp::Node::SharedPtr node)
{
    RCLCPP_INFO(node->get_logger(), "[Path planning] Task: %s  target=(%.3f, %.3f, %.3f)",
                task_type.c_str(),
                target_pose.position.x, target_pose.position.y, target_pose.position.z);

    // ── Configure planner ────────────────────────────────────
    arm_group.setPlanningPipelineId("ompl");
    arm_group.setPlannerId("RRTstar");
    arm_group.setPlanningTime(ARM_PLANNING_TIME_SEC);
    arm_group.setNumPlanningAttempts(10);
    arm_group.setPoseReferenceFrame("world");
    arm_group.setPathConstraints(make_ee_down_constraint());
    arm_group.setStartStateToCurrentState();
    arm_group.setPoseTarget(target_pose);
    arm_group.setWorkspace(
        -0.04, -1.5, 0.074,
        1, 1.5, 1.5);
    arm_group.allowReplanning(true);

    // Adjust speed based on task type
    double velocity_scaling = (task_type == "Grasp") ? 0.05 : 0.1;
    arm_group.setMaxVelocityScalingFactor(velocity_scaling);
    arm_group.setMaxAccelerationScalingFactor(velocity_scaling);

    try
    {

        // ── IK sanity-check ───────────────────────────────────
        auto test_state = arm_group.getCurrentState(5.0);
        if (!test_state)
        {
            RCLCPP_ERROR(node->get_logger(), "getCurrentState() returned null — is /joint_states active?");
            return std::nullopt;
        }
        const auto *ik_jmg = test_state->getJointModelGroup("demo_arm_bot");
        if (!test_state->setFromIK(ik_jmg, target_pose, 5.0))
        {
            RCLCPP_ERROR(node->get_logger(),
                         "[Path Planning] IK FAILED for (%.3f, %.3f, %.3f) — aborting.",
                         target_pose.position.x, target_pose.position.y, target_pose.position.z);
            return std::nullopt;
        }

        // ── Plan ──────────────────────────────────────────────
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (arm_group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(node->get_logger(), "[Path Planning] Planning failed.");
            return std::nullopt;
        }

        // ── Publish to RViz (like MoveIt Plan button) ─────────
        auto display_pub = node->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
            "/display_planned_path", 10);

        moveit_msgs::msg::DisplayTrajectory display_msg;
        display_msg.model_id = arm_group.getRobotModel()->getName();
        display_msg.trajectory_start = plan.start_state_;
        display_msg.trajectory.push_back(plan.trajectory_);
        display_pub->publish(display_msg);
        RCLCPP_INFO(node->get_logger(), "[Path Planning] Plan published to RViz.");

        // ── Execute ──────────────────────────────────────────────

        RCLCPP_INFO(node->get_logger(), "[Path Planning] Executing plan...");
        if (arm_group.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(node->get_logger(), "[Path Planning] Execution failed.");
            // pub_status->publish([]{ std_msgs::msg::String s; s.data="RRT_EXEC_FAILED"; return s; }());
            return std::nullopt;
        }

        RCLCPP_INFO(node->get_logger(), "[Path Planning] Execution succeeded. Syncing state...");
        // ── Update start state for next planning stage ─────────
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        arm_group.setStartStateToCurrentState();

        return plan;
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "[Path Planning] Exception: %s", e.what());
        return std::nullopt;
    }
    catch (...)
    {
        RCLCPP_ERROR(node->get_logger(), "[Path Planning] Unknown exception occurred.");
        return std::nullopt;
    }
}

void publish_target_marker(
    rclcpp::Node::SharedPtr node,
    const geometry_msgs::msg::Pose &pose)
{
    auto pub = node->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/target_pose_marker", 10);

    visualization_msgs::msg::MarkerArray marker_array;

    // ── Sphere at target position ────────────────────────────
    visualization_msgs::msg::Marker sphere;
    sphere.header.frame_id = "world";
    sphere.header.stamp = node->now();
    sphere.ns = "target_pose";
    sphere.id = 0;
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.action = visualization_msgs::msg::Marker::ADD;
    sphere.pose = pose;
    sphere.scale.x = 0.05;
    sphere.scale.y = 0.05;
    sphere.scale.z = 0.05;
    sphere.color.r = 1.0f;
    sphere.color.g = 0.0f;
    sphere.color.b = 0.0f;
    sphere.color.a = 1.0f;
    sphere.lifetime = rclcpp::Duration::from_seconds(3600);
    marker_array.markers.push_back(sphere);

    // ── Arrow showing orientation ────────────────────────────
    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = "world";
    arrow.header.stamp = node->now();
    arrow.ns = "target_pose";
    arrow.id = 1;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.pose = pose;
    arrow.scale.x = 0.15; // shaft length
    arrow.scale.y = 0.02; // shaft diameter
    arrow.scale.z = 0.02; // head diameter
    arrow.color.r = 0.0f;
    arrow.color.g = 1.0f;
    arrow.color.b = 0.0f;
    arrow.color.a = 1.0f;
    arrow.lifetime = rclcpp::Duration::from_seconds(3600);
    marker_array.markers.push_back(arrow);

    // Publish a few times to make sure RViz receives it
    for (int i = 0; i < 5; ++i)
    {
        pub->publish(marker_array);
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    RCLCPP_INFO(node->get_logger(),
                "[Marker] Target pose published at (%.3f, %.3f, %.3f)",
                pose.position.x, pose.position.y, pose.position.z);
}

void apriltag_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
    for (const auto &marker : msg->markers)
    {
        // Skip invalid or deleted markers
        if (marker.action != visualization_msgs::msg::Marker::ADD)
            continue;

        april_tag_x = marker.pose.position.x;
        april_tag_y = marker.pose.position.y;
        april_tag_z = marker.pose.position.z;

        // RCLCPP_INFO(this->get_logger(),
        //             "Marker ID: %d | Position -> x: %.3f, y: %.3f, z: %.3f",
        //             marker.id, april_tag_x, y, z);
    }
}

void wellplate_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
    for (const auto &marker : msg->markers)
    {
        // Skip markers that are not being added
        if (marker.action != visualization_msgs::msg::Marker::ADD)
            continue;

        // Process wellplate markers
        if (marker.ns == "wellplates")
        {
            wellplate_tag_x = marker.pose.position.x;
            wellplate_tag_y = marker.pose.position.y;
            wellplate_tag_z = marker.pose.position.z;

            // RCLCPP_INFO(this->get_logger(),
            //             "Wellplate Marker ID: %d | Position -> x: %.3f, y: %.3f, z: %.3f",
            //             marker.id, wellplate_tag_x, wellplate_tag_y, wellplate_tag_z);
        }
    }
}

std::string received_command = "plan_home";

void command_callback(const std_msgs::msg::String::SharedPtr msg)
{
    received_command = msg->data;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("move_to_pose_node");

    // Spin on a background thread so MoveGroupInterface can call services
    // without blocking the main execution thread.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread executor_thread([&executor]()
                                { executor.spin(); });

    // ── MoveIt planning groups ───────────────────────────────
    moveit::planning_interface::MoveGroupInterface arm_group(node, "demo_arm_bot");
    moveit::planning_interface::PlanningSceneInterface psi;

    arm_group.setPoseReferenceFrame("world");
    arm_group.setPlanningTime(ARM_PLANNING_TIME_SEC);


    // ── Live input storage (filled by subscribers below) ────
    std::optional<geometry_msgs::msg::Pose> live_target_pose;
    std::optional<std::string> live_task_type;
    sensor_msgs::msg::JointState::SharedPtr live_joint_state;
    sensor_msgs::msg::PointCloud2::SharedPtr latest_obstacle_cloud;
    // std::string received_command = "";

    // ── Subscribers ──────────────────────────────────────────

    // Task type: "Move" | "Grasp" | "Place"
    auto sub_task_type = node->create_subscription<std_msgs::msg::String>(
        "/task_planner/task_type",
        rclcpp::QoS(10).reliable(),
        [&live_task_type, node](const std_msgs::msg::String::SharedPtr msg)
        {
            live_task_type = msg->data;
            RCLCPP_INFO(node->get_logger(), "Task type received: %s", msg->data.c_str());
        });

    // Joint state — MoveIt reads /joint_states internally, but we also store
    // it here for logging and any custom validation logic.
    auto sub_joint_state = node->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states",
        rclcpp::QoS(10).reliable(),
        [&live_joint_state](const sensor_msgs::msg::JointState::SharedPtr msg)
        {
            live_joint_state = msg;
        });

    auto qos = rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    // April tag
    auto apriltag_subscription = node->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/inspect/apriltags",
        qos,
        apriltag_callback);

    // Wellplates
    auto wellplate_subscription = node->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/inspect/wellplates",
        qos,
        wellplate_callback);

    // 1. Define your custom QoS profile
    rclcpp::QoS pointcloud_qos(1);
    pointcloud_qos.best_effort();
    pointcloud_qos.durability_volatile();

    auto sub_obstacle_cloud = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/zed_pointcloud", // <-- Put your actual topic name here
        pointcloud_qos,    // <-- Inject your custom QoS here
        [&latest_obstacle_cloud](const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
            latest_obstacle_cloud = msg;
        });

    auto command_sub = node->create_subscription<std_msgs::msg::String>(
        "/planning_command", 10, command_callback);


    // If no live obstacle cloud arrived, use the hollow-box fallback
    if (latest_obstacle_cloud)
    {
        RCLCPP_INFO(node->get_logger(), "Live obstacle cloud detected.");
    }
    else
    {
        RCLCPP_WARN(node->get_logger(), "No obstacle cloud — adding hollow fallback obstacle.");
    }

    auto marker_pub = node->create_publisher<visualization_msgs::msg::Marker>(
        "/target_points_marker", 10);

    geometry_msgs::msg::Point target_point;
    target_point.x = 1.0;
    target_point.y = 0.5;
    target_point.z = 0.0;

    rclcpp::Rate rate(1);
    while (rclcpp::ok())
    {

        // rclcpp::spin_some(node);

        if (received_command == "plan_april")
        {

            geometry_msgs::msg::Point apriltag_point;
            apriltag_point.x = april_tag_x;
            apriltag_point.y = april_tag_y;
            apriltag_point.z = april_tag_z;

            // Resolve final task parameters (live > default)
            const geometry_msgs::msg::Pose target_position =
                live_target_pose.value_or(defaults::target_pose_func(node, marker_pub, apriltag_point, 0.35));
            const std::string task_type =
                live_task_type.value_or(defaults::TASK_TYPE);
            publish_target_marker(node, target_position);

            if (!live_target_pose)
                RCLCPP_WARN(node->get_logger(), "Using default target pose.");
            if (!live_task_type)
                RCLCPP_WARN(node->get_logger(), "Using default task type: %s.", task_type.c_str());

            const double PREGRASP_Z_OFFSET = 0.00;
            geometry_msgs::msg::Pose pregrasp_pose = target_position;
            pregrasp_pose.position.z += PREGRASP_Z_OFFSET;

            // ── Stage 1: RRT* → pre-grasp pose (offset above target) ─
            auto rrt_plan_opt = plan_and_publish_rrt(
                task_type, pregrasp_pose,
                arm_group, node);

            if (!rrt_plan_opt.has_value())
            {
                RCLCPP_ERROR(node->get_logger(), "RRT* failed — aborting pipeline.");
            }
            else
            {
                received_command = "idle";
            }
        }
        else if (received_command == "plan_wellplate")
        {

            geometry_msgs::msg::Point wellplate_point;
            wellplate_point.x = wellplate_tag_x;
            wellplate_point.y = wellplate_tag_y;
            wellplate_point.z = wellplate_tag_z;

            // Resolve final task parameters (live > default)
            const geometry_msgs::msg::Pose target_position =
                live_target_pose.value_or(defaults::target_pose_func(node, marker_pub, wellplate_point, 0.35));
            const std::string task_type =
                live_task_type.value_or(defaults::TASK_TYPE);
            publish_target_marker(node, target_position);

            if (!live_target_pose)
                RCLCPP_WARN(node->get_logger(), "Using default target pose.");
            if (!live_task_type)
                RCLCPP_WARN(node->get_logger(), "Using default task type: %s.", task_type.c_str());

            const double PREGRASP_Z_OFFSET = 0.00;
            geometry_msgs::msg::Pose pregrasp_pose = target_position;
            pregrasp_pose.position.z += PREGRASP_Z_OFFSET;

            // ── Stage 1: RRT* → pre-grasp pose (offset above target) ─
            auto rrt_plan_opt = plan_and_publish_rrt(
                task_type, pregrasp_pose,
                arm_group, node);

            if (!rrt_plan_opt.has_value())
            {
                RCLCPP_ERROR(node->get_logger(), "RRT* failed — aborting pipeline.");
            }
            else
            {
                received_command = "idle";
            }
        }
        else if (received_command == "plan_home")
        {
            // TODO: Move Hardcoded values to a config file
            geometry_msgs::msg::Point home_point;
            home_point.x = -0.048;
            home_point.y = 0.443;
            home_point.z = 0.247;

            // Resolve final task parameters (live > default)
            const geometry_msgs::msg::Pose target_position =
                live_target_pose.value_or(defaults::target_pose_func(node, marker_pub, home_point, 0));
            const std::string task_type =
                live_task_type.value_or(defaults::TASK_TYPE);
            publish_target_marker(node, target_position);

            if (!live_target_pose)
                RCLCPP_WARN(node->get_logger(), "Using default target pose.");
            if (!live_task_type)
                RCLCPP_WARN(node->get_logger(), "Using default task type: %s.", task_type.c_str());

            const double PREGRASP_Z_OFFSET = 0.00;
            geometry_msgs::msg::Pose pregrasp_pose = target_position;
            pregrasp_pose.position.z += PREGRASP_Z_OFFSET;

            // ── Stage 1: RRT* → pre-grasp pose (offset above target) ─
            auto rrt_plan_opt = plan_and_publish_rrt(
                task_type, pregrasp_pose,
                arm_group, node);

            if (!rrt_plan_opt.has_value())
            {
                RCLCPP_ERROR(node->get_logger(), "RRT* failed — aborting pipeline.");
            }
            else
            {
                received_command = "idle";
            }
        }
        else if (received_command == "plan_home_offset")
        {
            // TODO: Move Hardcoded values to a config file
            geometry_msgs::msg::Point home_point;
            home_point.x = -0.048;
            home_point.y = 0.443;
            home_point.z = 0.247;

            // Resolve final task parameters (live > default)
            const geometry_msgs::msg::Pose target_position =
                live_target_pose.value_or(defaults::target_pose_func(node, marker_pub, home_point, 0.35));
            const std::string task_type =
                live_task_type.value_or(defaults::TASK_TYPE);
            publish_target_marker(node, target_position);

            if (!live_target_pose)
                RCLCPP_WARN(node->get_logger(), "Using default target pose.");
            if (!live_task_type)
                RCLCPP_WARN(node->get_logger(), "Using default task type: %s.", task_type.c_str());

            const double PREGRASP_Z_OFFSET = 0.00;
            geometry_msgs::msg::Pose pregrasp_pose = target_position;
            pregrasp_pose.position.z += PREGRASP_Z_OFFSET;

            // ── Stage 1: RRT* → pre-grasp pose (offset above target) ─
            auto rrt_plan_opt = plan_and_publish_rrt(
                task_type, pregrasp_pose,
                arm_group, node);

            if (!rrt_plan_opt.has_value())
            {
                RCLCPP_ERROR(node->get_logger(), "RRT* failed — aborting pipeline.");
            }
            else
            {
                received_command = "idle";
            }
        }
        else
        {
            RCLCPP_INFO(node->get_logger(), "Waiting for the 'start_pick' command...");
        }
        rate.sleep();
    }

    // ── Clean shutdown ───────────────────────────────────────
    executor.cancel();
    executor_thread.join();
    rclcpp::shutdown();
    return 0;
}