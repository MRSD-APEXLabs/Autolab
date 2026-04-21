// ┌─────────────────────────────────────────────────────────────┐
// │  SUBSCRIBED TOPICS                                          │
// │  /task_planner/task_type         std_msgs/String            │
// │  /joint_states                   sensor_msgs/JointState     │
// │  /planning_command               std_msgs/String            │
// │                                                             │
// │  PUBLISHED TOPICS                                           │
// │  /planning_state                 std_msgs/String            │
// │    Values: IDLE | PLANNING | EXECUTING | SUCCESS | ERROR    │
// └─────────────────────────────────────────────────────────────┘

// ── ROS / MoveIt includes ────────────────────────────────────
#include <rclcpp/rclcpp.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include "crrt_plan.hpp"   // pulls in all others transitively
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
// ── xArm service includes ─────────────────────────────────────
#include <xarm_msgs/srv/call.hpp>
#include <xarm_msgs/srv/set_int16.hpp>
#include <xarm_msgs/srv/set_int16_by_id.hpp>
// ── STL includes ─────────────────────────────────────────────
#include <thread>
#include <chrono>
#include <mutex>
#include <optional>
#include <string>
#include <fstream>
#include <iomanip>
#include <map>

// Workspace bounds — must match setWorkspace() in move_to_pose_node.cpp
static constexpr double WS_X_MIN = -0.15;
static constexpr double WS_X_MAX =  0.85;
static constexpr double WS_Y_MIN = -0.50;
static constexpr double WS_Y_MAX =  0.50;
static constexpr double WS_Z_MIN =  0.90;
static constexpr double WS_Z_MAX =  1.30;

static constexpr double ARM_PLANNING_TIME_SEC = 15.0;
static constexpr int INPUT_TIMEOUT_SEC = 5;
static constexpr double APRIL_TAG_STALE_SEC = 1.0;
static constexpr int APRIL_TAG_WAIT_TIMEOUT_SEC = 10;
static constexpr int APRIL_TAG_WAIT_STEP_MS = 100;
static constexpr double WELLPLATE_STALE_SEC = 1.0;
static constexpr int WELLPLATE_WAIT_TIMEOUT_SEC = 10;
static constexpr int WELLPLATE_WAIT_STEP_MS = 100;



struct AprilTagEntry {
    geometry_msgs::msg::Point position;
    rclcpp::Time stamp;
};
static std::map<int, AprilTagEntry> april_tags;
static std::mutex april_tags_mutex;
static int marker_id = 0;

struct WellplateEntry {
    geometry_msgs::msg::Point position;
    rclcpp::Time stamp;
};
static std::optional<WellplateEntry> wellplate_detection;
static std::mutex wellplate_mutex;

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
        double offset)
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

// Reactivate xArm controller: clear errors, enable motion, set state to ready (0).
// Call this before any execute() to recover from fault/stopped state.
bool reactivate_xarm(rclcpp::Node::SharedPtr node)
{
    // clean_error
    auto clean_client = node->create_client<xarm_msgs::srv::Call>("/xarm/clean_error");
    if (!clean_client->wait_for_service(std::chrono::seconds(3))) {
        RCLCPP_ERROR(node->get_logger(), "[reactivate_xarm] /xarm/clean_error service unavailable");
        return false;
    }
    auto clean_req = std::make_shared<xarm_msgs::srv::Call::Request>();
    auto clean_fut = clean_client->async_send_request(clean_req);
    if (rclcpp::spin_until_future_complete(node, clean_fut, std::chrono::seconds(5))
        != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "[reactivate_xarm] clean_error call failed");
        return false;
    }
    RCLCPP_INFO(node->get_logger(), "[reactivate_xarm] clean_error ret=%d", clean_fut.get()->ret);

    // motion_enable: id=8 (all joints), data=1 (enable)
    auto enable_client = node->create_client<xarm_msgs::srv::SetInt16ById>("/xarm/motion_enable");
    if (!enable_client->wait_for_service(std::chrono::seconds(3))) {
        RCLCPP_ERROR(node->get_logger(), "[reactivate_xarm] /xarm/motion_enable service unavailable");
        return false;
    }
    auto enable_req = std::make_shared<xarm_msgs::srv::SetInt16ById::Request>();
    enable_req->id = 8;
    enable_req->data = 1;
    auto enable_fut = enable_client->async_send_request(enable_req);
    if (rclcpp::spin_until_future_complete(node, enable_fut, std::chrono::seconds(5))
        != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "[reactivate_xarm] motion_enable call failed");
        return false;
    }
    RCLCPP_INFO(node->get_logger(), "[reactivate_xarm] motion_enable ret=%d", enable_fut.get()->ret);

    // set_state: data=0 (SPORT / ready)
    auto state_client = node->create_client<xarm_msgs::srv::SetInt16>("/xarm/set_state");
    if (!state_client->wait_for_service(std::chrono::seconds(3))) {
        RCLCPP_ERROR(node->get_logger(), "[reactivate_xarm] /xarm/set_state service unavailable");
        return false;
    }
    auto state_req = std::make_shared<xarm_msgs::srv::SetInt16::Request>();
    state_req->data = 0;
    auto state_fut = state_client->async_send_request(state_req);
    if (rclcpp::spin_until_future_complete(node, state_fut, std::chrono::seconds(5))
        != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "[reactivate_xarm] set_state call failed");
        return false;
    }
    RCLCPP_INFO(node->get_logger(), "[reactivate_xarm] set_state(0) ret=%d", state_fut.get()->ret);

    return true;
}

std::optional<moveit::planning_interface::MoveGroupInterface::Plan> plan_and_publish_rrt(
    const std::string &task_type,
    const geometry_msgs::msg::Pose &target_pose,
    moveit::planning_interface::MoveGroupInterface &arm_group,
    rclcpp::Node::SharedPtr node,
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub)
{
    RCLCPP_INFO(node->get_logger(), "[Path planning] Task: %s  target=(%.3f, %.3f, %.3f)",
                task_type.c_str(),
                target_pose.position.x, target_pose.position.y, target_pose.position.z);

    // ── Configure planner ────────────────────────────────────
    arm_group.setPlanningPipelineId("ompl");
    arm_group.setPlannerId("RRTConnect");
    arm_group.setPlanningTime(ARM_PLANNING_TIME_SEC);           // match GUI exactly
    arm_group.setNumPlanningAttempts(20);     // match GUI exactly
    arm_group.setPoseReferenceFrame("world");
    arm_group.setPathConstraints(make_ee_down_constraint());
    arm_group.setStartStateToCurrentState();
    arm_group.setWorkspace(-0.15, -0.50, 0.90,
                            0.85,  0.50, 1.30);

    // ── Solve IK once, then plan to joint goal (same as GUI) ──
    auto current_state = arm_group.getCurrentState(5.0);
    if (!current_state)
    {
        RCLCPP_ERROR(node->get_logger(), "getCurrentState() failed");
        return std::nullopt;
    }
    const auto* jmg = current_state->getJointModelGroup("demo_arm_bot");
    if (!current_state->setFromIK(jmg, target_pose, 1.0))
    {
        RCLCPP_ERROR(node->get_logger(),
                    "[Path Planning] IK FAILED for (%.3f, %.3f, %.3f) — aborting.",
                    target_pose.position.x, target_pose.position.y, target_pose.position.z);
        return std::nullopt;
    }
    std::vector<double> joint_values;
    current_state->copyJointGroupPositions(jmg, joint_values);
    arm_group.setJointValueTarget(joint_values);  // ← key change
    
    RCLCPP_INFO(node->get_logger(),
        "Constraints: %zu",
        arm_group.getPathConstraints().orientation_constraints.size());
        arm_group.allowReplanning(true);

    // Adjust speed based on task type
    double velocity_scaling = (task_type == "Grasp") ? 0.05 : 0.1;
    arm_group.setMaxVelocityScalingFactor(velocity_scaling);
    arm_group.setMaxAccelerationScalingFactor(velocity_scaling);

    try
    {
        // ── Plan ──────────────────────────────────────────────
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto crrt_opt = prm_plan(arm_group, node, joint_values);
        if (!crrt_opt)
        {
            RCLCPP_ERROR(node->get_logger(), "[Path Planning] CRRT planning failed.");
            return std::nullopt;
        }
        plan = std::move(*crrt_opt);

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
        {
            std_msgs::msg::String s;
            s.data = "EXECUTING";
            state_pub->publish(s);
        }
        RCLCPP_INFO(node->get_logger(), "[Path Planning] Reactivating xArm controller...");
        if (!reactivate_xarm(node)) {
            RCLCPP_ERROR(node->get_logger(), "[Path Planning] xArm reactivation failed — aborting execution.");
            return std::nullopt;
        }
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


static std::mutex cmd_mutex;
static std::string received_command = "idle";

void command_callback(const std_msgs::msg::String::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(cmd_mutex);
    received_command = msg->data;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("move_to_pose_node");

    // Spin on a background thread so MoveGroupInterface can call services
    // without blocking the main execution thread.
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 2);
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
        [&node](const visualization_msgs::msg::MarkerArray::SharedPtr msg)
        {
            std::lock_guard<std::mutex> lock(april_tags_mutex);
            for (const auto &marker : msg->markers)
            {
                if (marker.action != visualization_msgs::msg::Marker::ADD)
                    continue;
                april_tags[marker.id] = {marker.pose.position, node->now()};
            }
        });

    // Wellplates
    auto wellplate_subscription = node->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/inspect/wellplates",
        qos,
        [&node](const visualization_msgs::msg::MarkerArray::SharedPtr msg)
        {
            std::lock_guard<std::mutex> lock(wellplate_mutex);
            for (const auto &marker : msg->markers)
            {
                if (marker.action != visualization_msgs::msg::Marker::ADD)
                    continue;
                if (marker.ns == "wellplates")
                    wellplate_detection = WellplateEntry{marker.pose.position, node->now()};
            }
        });

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

    auto planning_state_pub = node->create_publisher<std_msgs::msg::String>("/planning_state", 10);

    auto publish_state = [&](const std::string& state) {
        std_msgs::msg::String s;
        s.data = state;
        planning_state_pub->publish(s);
    };

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


    auto finish_plan = [&](const std::optional<moveit::planning_interface::MoveGroupInterface::Plan>& opt) {
        if (opt.has_value()) {
            publish_state("SUCCESS");
        } else {
            RCLCPP_ERROR(node->get_logger(), "RRT* failed — aborting pipeline.");
            publish_state("ERROR");
        }
        std::lock_guard<std::mutex> lock(cmd_mutex);
        received_command = "idle";
    };

    rclcpp::Rate rate(1);
    while (rclcpp::ok())
    {
        std::string cmd;
        {
            std::lock_guard<std::mutex> lock(cmd_mutex);
            cmd = received_command;
        }

        RCLCPP_INFO(node->get_logger(), "Received command: %s", cmd.c_str());

        if (cmd.rfind("plan_april_", 0) == 0)
        {
            // Parse tag ID from "plan_april_<id>"
            const std::string april_prefix = "plan_april_";
            const std::string id_str = cmd.substr(april_prefix.size());
            if (id_str.empty()) {
                RCLCPP_ERROR(node->get_logger(), "plan_april_ command has no tag ID: '%s'", cmd.c_str());
                finish_plan(std::nullopt);
                continue;
            }
            int tag_id = 0;
            try {
                std::size_t pos = 0;
                tag_id = std::stoi(id_str, &pos);
                if (pos != id_str.size()) throw std::invalid_argument("trailing chars");
            } catch (...) {
                RCLCPP_ERROR(node->get_logger(), "Could not parse tag ID from command: %s", cmd.c_str());
                finish_plan(std::nullopt);
                continue;
            }

            // Wait up to APRIL_TAG_WAIT_TIMEOUT_SEC for a fresh tag reading
            geometry_msgs::msg::Point apriltag_point;
            bool found = false;
            for (int elapsed_ms = 0;
                 elapsed_ms < APRIL_TAG_WAIT_TIMEOUT_SEC * 1000;
                 elapsed_ms += APRIL_TAG_WAIT_STEP_MS)
            {
                {
                    const rclcpp::Time now = node->now();
                    std::lock_guard<std::mutex> lock(april_tags_mutex);
                    auto it = april_tags.find(tag_id);
                    if (it != april_tags.end() &&
                        (now - it->second.stamp).seconds() <= APRIL_TAG_STALE_SEC)
                    {
                        apriltag_point = it->second.position;
                        found = true;
                    }
                }
                if (found) break;
                rclcpp::sleep_for(std::chrono::milliseconds(APRIL_TAG_WAIT_STEP_MS));
            }

            if (!found)
            {
                RCLCPP_ERROR(node->get_logger(),
                             "April tag ID %d not found or stale after %ds — aborting.",
                             tag_id, APRIL_TAG_WAIT_TIMEOUT_SEC);
                finish_plan(std::nullopt);
                continue;
            }

            // Resolve final task parameters (live > default)
            const geometry_msgs::msg::Pose target_position =
                live_target_pose.value_or(defaults::target_pose_func(node, marker_pub, apriltag_point, 0.25));
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

            publish_state("PLANNING");
            auto rrt_plan_opt = plan_and_publish_rrt(
                task_type, pregrasp_pose,
                arm_group, node, planning_state_pub);

            finish_plan(rrt_plan_opt);
        }
        else if (cmd == "plan_wellplate")
        {
            // Wait up to WELLPLATE_WAIT_TIMEOUT_SEC for a fresh detection
            geometry_msgs::msg::Point wellplate_point;
            bool found = false;
            for (int elapsed_ms = 0;
                 elapsed_ms < WELLPLATE_WAIT_TIMEOUT_SEC * 1000;
                 elapsed_ms += WELLPLATE_WAIT_STEP_MS)
            {
                {
                    const rclcpp::Time now = node->now();
                    std::lock_guard<std::mutex> lock(wellplate_mutex);
                    if (wellplate_detection.has_value() &&
                        (now - wellplate_detection->stamp).seconds() <= WELLPLATE_STALE_SEC)
                    {
                        wellplate_point = wellplate_detection->position;
                        found = true;
                    }
                }
                if (found) break;
                rclcpp::sleep_for(std::chrono::milliseconds(WELLPLATE_WAIT_STEP_MS));
            }

            if (!found)
            {
                RCLCPP_ERROR(node->get_logger(),
                             "Wellplate not found or stale after %ds — aborting.",
                             WELLPLATE_WAIT_TIMEOUT_SEC);
                finish_plan(std::nullopt);
                continue;
            }

            // Resolve final task parameters (live > default)
            const geometry_msgs::msg::Pose target_position =
                live_target_pose.value_or(defaults::target_pose_func(node, marker_pub, wellplate_point, 0.25));
            const std::string task_type =
                live_task_type.value_or(defaults::TASK_TYPE);

            if (!live_target_pose)
                RCLCPP_WARN(node->get_logger(), "Using default target pose.");
            if (!live_task_type)
                RCLCPP_WARN(node->get_logger(), "Using default task type: %s.", task_type.c_str());

            geometry_msgs::msg::Pose pregrasp_pose = target_position;

            const double PREGRASP_Z_OFFSET = 0.00;
            pregrasp_pose.position.z += PREGRASP_Z_OFFSET;

            const double PREGRASP_Y_OFFSET = 0.1;
            pregrasp_pose.position.y += PREGRASP_Y_OFFSET;

            const double PREGRASP_X_OFFSET = 0.1;
            pregrasp_pose.position.x -= PREGRASP_X_OFFSET;


            publish_target_marker(node, pregrasp_pose);

            publish_state("PLANNING");
            auto rrt_plan_opt = plan_and_publish_rrt(
                task_type, pregrasp_pose,
                arm_group, node, planning_state_pub);

            finish_plan(rrt_plan_opt);
        }
        else if (cmd == "plan_home")
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

            publish_state("PLANNING");
            auto rrt_plan_opt = plan_and_publish_rrt(
                task_type, pregrasp_pose,
                arm_group, node, planning_state_pub);

            finish_plan(rrt_plan_opt);
        }
        else if (cmd == "plan_home_offset")
        {
            const JointVec home_offset_joints = {1.6284, -0.3927, -0.5812, 0.0, 0.9738, 1.6284};

            publish_state("PLANNING");
            auto rrt_plan_opt = prm_plan(arm_group, node, home_offset_joints);

            if (rrt_plan_opt) {
                publish_state("EXECUTING");
                if (arm_group.execute(*rrt_plan_opt) != moveit::core::MoveItErrorCode::SUCCESS) {
                    finish_plan(std::nullopt);
                    continue;
                }
            }
            finish_plan(rrt_plan_opt);
        }
        else
        {
            publish_state("IDLE");
        }
        rate.sleep();
    }

    // ── Clean shutdown ───────────────────────────────────────
    executor.cancel();
    executor_thread.join();
    rclcpp::shutdown();
    return 0;
}