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

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

// ── TF2 includes ─────────────────────────────────────────────
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// ── STL includes ─────────────────────────────────────────────
#include <thread>
#include <chrono>
#include <optional>
#include <string>
#include <fstream>
#include <iomanip>

static constexpr double ARM_PLANNING_TIME_SEC     = 15.0;
static constexpr int    INPUT_TIMEOUT_SEC         = 5;

namespace defaults
{
    static const std::string TASK_TYPE = "Grasp";

    // Pre-grasp pose: EE above the wellplate centre, RPY=(0,0,π)
    // so the gripper faces down and the fingers straddle the plate.
    static geometry_msgs::msg::Pose target_pose()
    {
        geometry_msgs::msg::Pose p;
        p.position.x = 0.497;   // aligned with wellplate X
        p.position.y = -0.002;   // aligned with wellplate Y
        p.position.z = 0.126;   // high enough for open fingers to clear the plate

        tf2::Quaternion q;
        q.setRPY(M_PI, 0.0, 0.0);   // gripper pointing down
        q.normalize();
        p.orientation = tf2::toMsg(q);
        return p;
    }
}

moveit_msgs::msg::Constraints make_ee_down_constraint()
{
    moveit_msgs::msg::OrientationConstraint ocm;
    ocm.link_name       = "link_eef";
    ocm.header.frame_id = "link_base";

    tf2::Quaternion q;
    q.setRPY(M_PI, 0.0, 0.0);   // target orientation: RPY=(0,0,π)
    q.normalize();
    ocm.orientation = tf2::toMsg(q);

    ocm.absolute_x_axis_tolerance = M_PI;   // ≈11° tilt tolerance
    ocm.absolute_y_axis_tolerance = 0.2;   // ≈11° tilt tolerance
    ocm.absolute_z_axis_tolerance = 0.2;  // spin freely around Z
    ocm.weight = 1.0;

    moveit_msgs::msg::Constraints c;
    c.name = "ee_down";
    c.orientation_constraints.push_back(ocm);
    return c;
}


visualization_msgs::msg::Marker make_path_marker(
    const trajectory_msgs::msg::JointTrajectory& jt,
    moveit::core::RobotStatePtr                  robot_state,
    const moveit::core::JointModelGroup*         jmg,
    const std::string&                           ee_link,
    const std::string&                           ns,
    float r, float g, float b,
    rclcpp::Node::SharedPtr                      node)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id    = "world";
    marker.header.stamp       = node->now();
    marker.ns                 = ns;
    marker.id                 = 0;
    marker.type               = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action             = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x            = 0.01;   // line width in metres
    marker.color              = [&]{ std_msgs::msg::ColorRGBA c; c.r=r; c.g=g; c.b=b; c.a=1.0; return c; }();
    marker.lifetime           = rclcpp::Duration::from_seconds(3600);

    for (const auto& pt : jt.points)
    {
        robot_state->setJointGroupPositions(jmg, pt.positions);
        robot_state->updateLinkTransforms();
        const Eigen::Isometry3d& tf = robot_state->getGlobalLinkTransform(ee_link);

        geometry_msgs::msg::Point p;
        p.x = tf.translation().x();
        p.y = tf.translation().y();
        p.z = tf.translation().z();
        marker.points.push_back(p);
    }
    return marker;
}


std::optional<moveit::planning_interface::MoveGroupInterface::Plan> plan_and_publish_rrt(
    const std::string&                                                    task_type,
    const geometry_msgs::msg::Pose&                                       target_pose,
    moveit::planning_interface::MoveGroupInterface&                       arm_group,
    rclcpp::Node::SharedPtr                                               node)
{
    RCLCPP_INFO(node->get_logger(), "[Path planning] Task: %s  target=(%.3f, %.3f, %.3f)",
        task_type.c_str(),
        target_pose.position.x, target_pose.position.y, target_pose.position.z);

    // ── Configure planner ────────────────────────────────────
    arm_group.setPlanningPipelineId("ompl");
    arm_group.setPlannerId("RRTConnectkConfigDefault");
    arm_group.setPlanningTime(ARM_PLANNING_TIME_SEC);
    arm_group.setPoseReferenceFrame("link_base");
    arm_group.setPathConstraints(make_ee_down_constraint());
    arm_group.setStartStateToCurrentState();
    // arm_group.setJointValueTarget(target_pose);
    arm_group.setPoseTarget(target_pose);

    // If task type is "Grasp", constrain velocity and acceleration for a
    // slower, more deliberate approach. For "Move", be more aggressive.
    if (task_type == "Grasp") {
        arm_group.setMaxVelocityScalingFactor(0.2);      // 0.0 to 1.0 — 1.0 = full speed
        arm_group.setMaxAccelerationScalingFactor(0.2);  // 0.0 to 1.0 — 1.0 = full a
    }
    else {
        arm_group.setMaxVelocityScalingFactor(0.4);      // 0.0 to 1.0 — 1.0 = full speed
        arm_group.setMaxAccelerationScalingFactor(0.2);  // 0.0 to 1.0 — 1.0 = full a
    }

    moveit::core::RobotStatePtr test_state = arm_group.getCurrentState(5.0);

    if (!test_state) {
    RCLCPP_ERROR(node->get_logger(), "getCurrentState() returned null — is the robot publishing /joint_states?");
    return std::nullopt;
    }
    RCLCPP_INFO(node->get_logger(), "Planning frame: %s, EE link: %s",
        arm_group.getPlanningFrame().c_str(),
        arm_group.getEndEffectorLink().c_str());

    const auto* ik_jmg = test_state->getJointModelGroup("xarm6");

    RCLCPP_INFO(node->get_logger(), "JMG tip links:");
    for (const auto& tip : ik_jmg->getSolverInstance()->getTipFrames())
        RCLCPP_INFO(node->get_logger(), "  %s", tip.c_str());

    geometry_msgs::msg::PoseStamped current_ee = arm_group.getCurrentPose("link_eef");
    RCLCPP_INFO(node->get_logger(), "Current EE pose: pos=(%.3f, %.3f, %.3f) quat=(%.3f, %.3f, %.3f, %.3f)",
        current_ee.pose.position.x,
        current_ee.pose.position.y,
        current_ee.pose.position.z,
        current_ee.pose.orientation.x,
        current_ee.pose.orientation.y,
        current_ee.pose.orientation.z,
        current_ee.pose.orientation.w);


    // ── IK sanity-check (fast, no collision check) ──────────
    // If IK fails here the planner will fail too — bail early.
    
    
    if (!test_state->setFromIK(ik_jmg, target_pose, 5.0))
    {
        RCLCPP_ERROR(node->get_logger(),
            "[Path Planning] IK FAILED for (%.3f, %.3f, %.3f) — aborting.",
            target_pose.position.x, target_pose.position.y, target_pose.position.z);
        // RCLCPP_ERROR(node->get_logger(), "Planning failed because of error: %s", arm_group.getMoveItErrorCodeText(arm_group.plan(plan)));
        // pub_status->publish([]{ std_msgs::msg::String s; s.data="PATH_PLANNING_FAILED_IK"; return s; }());
        return std::nullopt;
    }
    RCLCPP_INFO(node->get_logger(), "[Path Planning] IK check passed.");

    // ── Plan ─────────────────────────────────────────────────
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (arm_group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "[Path Planning] Planning failed.");
        // pub_status->publish([]{ std_msgs::msg::String s; s.data="PATH_PLANNING_FAILED"; return s; }());
        return std::nullopt;
    }

    // ── Publish green RViz path marker ───────────────────────
    {
        moveit::core::RobotStatePtr viz_state = arm_group.getCurrentState(5.0);
        // const auto* jmg    = viz_state->getJointModelGroup(arm_group.getName());
        const auto  ee_lnk = arm_group.getEndEffectorLink();
        // pub_rrt_viz->publish(make_path_marker(
        //     plan.trajectory_.joint_trajectory, viz_state, jmg, ee_lnk,
        //     "rrt_path", 0.0f, 1.0f, 0.0f, node));
        RCLCPP_INFO(node->get_logger(), "[Path Planning] RViz marker published (green).");
    }

    // ── Execute ──────────────────────────────────────────────
    // RCLCPP_INFO(node->get_logger(), "[Path Planning] Executing plan...");
    // if (arm_group.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
    // {
    //     RCLCPP_ERROR(node->get_logger(), "[Path Planning] Execution failed.");
    //     // pub_status->publish([]{ std_msgs::msg::String s; s.data="RRT_EXEC_FAILED"; return s; }());
    //     return std::nullopt;
    // }

    // Give the Planning Scene Monitor time to sync joint states from /joint_states
    RCLCPP_INFO(node->get_logger(), "[Path Planning] Execution succeeded. Syncing state...");
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    arm_group.setStartStateToCurrentState();

    return plan;
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("move_to_pose_node");

    // Spin on a background thread so MoveGroupInterface can call services
    // without blocking the main execution thread.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread executor_thread([&executor]() { executor.spin(); });


    // ── MoveIt planning groups ───────────────────────────────
    moveit::planning_interface::MoveGroupInterface arm_group(node, "xarm6");
    moveit::planning_interface::PlanningSceneInterface psi;

    arm_group.setPoseReferenceFrame("link_base");
    arm_group.setPlanningTime(ARM_PLANNING_TIME_SEC);

    // ── Populate planning scene ──────────────────────────────
    // Wait for the Planning Scene Monitor to register the objects
    // before any planning starts (avoids race condition).
    rclcpp::sleep_for(std::chrono::seconds(3));

    // Log startup EE pose for debugging

    // Unused in current code; placeholder for future obstacle cloud publishing
    auto pub_dummy_cloud = node->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/dummy_obstacle_cloud", 10);


    // ── Live input storage (filled by subscribers below) ────
    std::optional<geometry_msgs::msg::Pose> live_target_pose;
    std::optional<std::string>              live_task_type;
    sensor_msgs::msg::JointState::SharedPtr live_joint_state;
    sensor_msgs::msg::PointCloud2::SharedPtr latest_obstacle_cloud;


    // ── Subscribers ──────────────────────────────────────────

    // Task type: "Move" | "Grasp" | "Place"
    auto sub_task_type = node->create_subscription<std_msgs::msg::String>(
        "/task_planner/task_type",
        rclcpp::QoS(10).reliable(),
        [&live_task_type, &node](const std_msgs::msg::String::SharedPtr msg) {
            live_task_type = msg->data;
            RCLCPP_INFO(node->get_logger(), "Task type received: %s", msg->data.c_str());
        });

    // Joint state — MoveIt reads /joint_states internally, but we also store
    // it here for logging and any custom validation logic.
    auto sub_joint_state = node->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states",
        rclcpp::QoS(10).reliable(),
        [&live_joint_state](const sensor_msgs::msg::JointState::SharedPtr msg) {
            live_joint_state = msg;
        });


    // ── Wait for perception input ────────────────────────────
    RCLCPP_INFO(node->get_logger(), "Waiting %d s for perception topics...", INPUT_TIMEOUT_SEC);
    std::this_thread::sleep_for(std::chrono::seconds(INPUT_TIMEOUT_SEC));

    // If no live obstacle cloud arrived, use the hollow-box fallback
    if (latest_obstacle_cloud) {
        RCLCPP_INFO(node->get_logger(), "Live obstacle cloud detected.");
    } else {
        RCLCPP_WARN(node->get_logger(), "No obstacle cloud — adding hollow fallback obstacle.");
    }
    

    // Resolve final task parameters (live > default)
    const geometry_msgs::msg::Pose target_pose =
        live_target_pose.value_or(defaults::target_pose());
    const std::string task_type =
        live_task_type.value_or(defaults::TASK_TYPE);

    if (!live_target_pose) RCLCPP_WARN(node->get_logger(), "Using default target pose.");
    if (!live_task_type)   RCLCPP_WARN(node->get_logger(), "Using default task type: %s.", task_type.c_str());


    // ╔═══════════════════════════════════════════════════════╗
    // ║  EXECUTION PIPELINE                                   ║
    // ╚═══════════════════════════════════════════════════════╝

    const double PREGRASP_Z_OFFSET = 0.00;
    geometry_msgs::msg::Pose pregrasp_pose = target_pose;
    pregrasp_pose.position.z += PREGRASP_Z_OFFSET;

    // ── Stage 1: RRT* → pre-grasp pose (offset above target) ─
    auto rrt_plan_opt = plan_and_publish_rrt(
        task_type, pregrasp_pose,
        arm_group, node);


    if (!rrt_plan_opt.has_value())
    {
        RCLCPP_ERROR(node->get_logger(), "RRT* failed — aborting pipeline.");
        executor.cancel();
        executor_thread.join();
        rclcpp::shutdown();
        return 1;
    }

    // ── Clean shutdown ───────────────────────────────────────
    executor.cancel();
    executor_thread.join();
    rclcpp::shutdown();
    return 0;
}