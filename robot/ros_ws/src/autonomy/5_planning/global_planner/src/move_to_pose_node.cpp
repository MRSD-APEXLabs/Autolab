// ┌─────────────────────────────────────────────────────────────┐
// │  SUBSCRIBED TOPICS                                          │
// │  /task_planner/task_type         std_msgs/String            │
// │  /joint_states                   sensor_msgs/JointState     │
// │  /planning_command               std_msgs/String            │
// │    plan_* commands; "stop" ends the motion (see below)      │
// │                                                             │
// │  PUBLISHED TOPICS                                           │
// │  /planning_state                 std_msgs/String            │
// │    Values: IDLE | PLANNING | EXECUTING | SUCCESS | ERROR    │
// └─────────────────────────────────────────────────────────────┘

// ── ROS / MoveIt includes ────────────────────────────────────
#include <rclcpp/rclcpp.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include "crrt_parallel.hpp"  // pulls in crrt_rrtstar.hpp, crrt_plan.hpp and all others transitively
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <unordered_set>
#include <array>
#include <std_srvs/srv/empty.hpp>   // TEMP: cloud refresh
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
#include <atomic>
#include <fstream>
#include <iomanip>
#include <map>
#include <functional>

// Workspace bounds — must match setWorkspace() in move_to_pose_node.cpp
static constexpr double WS_X_MIN = -0.15;
static constexpr double WS_X_MAX =  0.85;
static constexpr double WS_Y_MIN = -0.50;
static constexpr double WS_Y_MAX =  0.50;
static constexpr double WS_Z_MIN =  0.90;
static constexpr double WS_Z_MAX =  1.30;

// /planning_command "stop" (manipulation_executive's cancel) sets this until the next command arrives. It ends the
// trajectory that is executing (MoveGroupInterface::stop) and keeps the current command's plan from being executed.
static std::atomic<bool> stop_requested{false};

static constexpr double ARM_PLANNING_TIME_SEC = 15.0;
static constexpr int INPUT_TIMEOUT_SEC = 5;
static constexpr double APRIL_TAG_STALE_SEC = 1.0;
static constexpr int APRIL_TAG_WAIT_TIMEOUT_SEC = 10;
static constexpr int APRIL_TAG_WAIT_STEP_MS = 100;
static constexpr double WELLPLATE_STALE_SEC = 1.0;
static constexpr int WELLPLATE_WAIT_TIMEOUT_SEC = 10;
static constexpr int WELLPLATE_WAIT_STEP_MS = 100;
static constexpr int CLOUD_REFRESH_TIMEOUT_SEC = 3;   // TEMP: cloud refresh
static constexpr double CLOUD_INFLATION_RADIUS = 0.02;   // m: every obstacle point also fills points this far around it
// The inflated cloud (~1.5 MB) goes to move_group best-effort over UDP only (fastdds_lan.xml has no shared
// memory); a lost fragment loses the whole cloud (measured: ~1 in 4 at 1.4 MB). So it is re-sent at this period
// until the planner has the octomap built from it.
static constexpr int CLOUD_RESEND_MS = 500;
// end_effector_p4_1 world yaw at the plan_april_<id> / plan_wellplate goals: the default (pi) turned 90 deg
// about world z, i.e. xArm TCP yaw +90 deg in the base frame, where the OT-2/shaker ACT policies start.
static constexpr double APPROACH_YAW = -M_PI / 2.0;

// ── Fallbacks ────────────────────────────────────────────────
// A plan command reports ERROR only after every fallback below has also failed; each fallback that
// is tried is logged as a WARN. None of them relaxes a check: every path is still collision-checked
// against the octomap and the ee-down constraint, and a failed execution is never retried.
static constexpr int    CLOUD_FALLBACK_TIMEOUT_SEC  = 10;    // second wait for a fresh cloud / the planner's octomap
static constexpr int    DETECTION_FALLBACK_WAIT_SEC = 10;    // second wait for a fresh tag / plate reading...
static constexpr double DETECTION_REUSE_MAX_AGE_SEC = 2.0;   // ...then the last one, if taken at most this long before the command
static constexpr double TF_FALLBACK_TIMEOUT_SEC     = 10.0;  // second wait for top_camera -> world
static constexpr double IK_TIMEOUT_SEC              = 1.0;
static constexpr double IK_FALLBACK_TIMEOUT_SEC     = 5.0;   // second IK try: reachable but hard poses
static constexpr int    FALLBACK_LONG_ATTEMPTS      = 2;     // searches after the normal attempts fail...
static constexpr double FALLBACK_LONG_TIME_SEC      = 10.0;  // ...each with this budget (normal: crrt_cfg::RRTSTAR_TIME_SEC)
// Last fallback, pose goals only: the goal moved by these offsets (world frame, m), one search each.
static const std::array<std::array<double, 3>, 6> GOAL_NUDGES = {{
    {0.0, 0.0, 0.02}, {0.0, 0.0, 0.04},                                          // up
    {0.02, 0.0, 0.0}, {-0.02, 0.0, 0.0}, {0.0, 0.02, 0.0}, {0.0, -0.02, 0.0}}};  // around
// The arm this close to the goal (joint space, rad) already counts as there: SUCCESS without moving.
static constexpr double ALREADY_THERE_RAD = 0.01;
// plan_home_offset's joint goal. The arm there is out of the base camera's view.
static const JointVec HOME_OFFSET_JOINTS = {1.6284, -0.3927, -0.5812, 0.0, 0.9738, 1.6284};
// plan_wellplate takes a fresh cloud only with the arm this close to HOME_OFFSET_JOINTS (joint space, rad);
// elsewhere (e.g. a retried pick, arm still at the OT-2) the arm would be in the cloud, so the map is reused.
static constexpr double FRESH_CLOUD_HOME_RAD = 0.05;



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
        double offset,
        double yaw = M_PI)
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
            RCLCPP_WARN(node->get_logger(),
                        "[Fallback] TF transform top_camera→world failed (%s); waiting up to %.0f s more.",
                        ex.what(), TF_FALLBACK_TIMEOUT_SEC);
            // A camera-frame point is never used as a world point: if this also fails, the
            // exception reaches the command handler, which reports ERROR.
            world_point = tf_buffer.transform(cam_point, "world",
                                              tf2::durationFromSec(TF_FALLBACK_TIMEOUT_SEC));
        }

        // ── Build pose with transformed position ─────────────────
        p.position.x = world_point.point.x;
        p.position.y = world_point.point.y;
        p.position.z = world_point.point.z + offset; // height offset for pre-grasp

        // Orientation: gripper pointing down, turned to `yaw` about world z
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
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

// Ensure `controller_name` is active. Returns true if it is active on return
// (including when it was already active and no switch was needed).
bool set_controller_active(
    const std::shared_ptr<rclcpp::Node>& node,
    const std::string& controller_name)
{
    // ── Check the current state first ─────────────────────────
    // The controller manager only accepts an activation request for a
    // controller that is currently *inactive*. Asking it to activate an
    // already-active controller is treated as a failed transition, and under
    // STRICT strictness that aborts the entire switch ("Aborting, no
    // controller is switched!"). So query the state and skip the no-op.
    auto list_client = node->create_client<controller_manager_msgs::srv::ListControllers>(
        "/controller_manager/list_controllers");

    if (list_client->wait_for_service(std::chrono::seconds(5)))
    {
        auto list_req = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
        auto list_fut = list_client->async_send_request(list_req);

        if (rclcpp::spin_until_future_complete(node, list_fut, std::chrono::seconds(5)) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            bool known = false;
            // get() moves the response shared_ptr out of the future and returns
            // it by value. Iterating list_fut.get()->controller directly would
            // free the response at the end of that expression, leaving the loop
            // walking freed memory, so hold the pointer for the loop's lifetime.
            auto list_resp = list_fut.get();
            for (const auto& c : list_resp->controller)
            {
                if (c.name != controller_name)
                    continue;
                known = true;

                if (c.state == "active")
                {
                    RCLCPP_INFO(node->get_logger(),
                                "Controller '%s' is already active — no switch needed.",
                                controller_name.c_str());
                    return true;
                }
                if (c.state != "inactive")
                {
                    // unconfigured / finalized — activation is impossible from here.
                    RCLCPP_WARN(node->get_logger(),
                                 "Controller '%s' is in state '%s'; it must be 'inactive' "
                                 "before it can be activated.",
                                 controller_name.c_str(), c.state.c_str());
                    return false;
                }
                break;  // inactive → fall through and activate it
            }

            if (!known)
            {
                RCLCPP_WARN(node->get_logger(),
                             "Controller '%s' is not loaded in the controller manager.",
                             controller_name.c_str());
                return false;
            }
        }
        else
        {
            RCLCPP_WARN(node->get_logger(),
                        "list_controllers call failed — attempting the switch anyway.");
        }
    }
    else
    {
        RCLCPP_WARN(node->get_logger(),
                    "/controller_manager/list_controllers unavailable — "
                    "attempting the switch anyway.");
    }

    // ── Activate ──────────────────────────────────────────────
    auto client = node->create_client<controller_manager_msgs::srv::SwitchController>(
        "/controller_manager/switch_controller");

    if (!client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_WARN(node->get_logger(),
                     "/controller_manager/switch_controller unavailable");
        return false;
    }

    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    request->activate_controllers.push_back(controller_name);

    // BEST_EFFORT only triggers a transition when the controller is not already
    // in the target state, so a redundant activation is a no-op instead of an
    // abort. Belt-and-braces with the state check above, which also races
    // against another node switching controllers between the two calls.
    request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
    request->activate_asap = true;   // 'start_asap' is the deprecated spelling
    request->timeout = rclcpp::Duration::from_seconds(5.0);

    auto future = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(10)) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_WARN(node->get_logger(), "switch_controller call failed");
        return false;
    }

    auto response = future.get();
    if (!response->ok) {
       //  Fixed: remove response->message.c_str()
        RCLCPP_WARN(node->get_logger(), "Failed to activate controller '%s'.",
                    controller_name.c_str());
        return false;
    }

    RCLCPP_INFO(node->get_logger(), "Controller '%s' activated.", controller_name.c_str());
    return true;
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

// ── Planner selection ─────────────────────────────────────────
// Change PLANNER to switch the planner at every call site.
enum class Planner { RRT_CONNECT, INFORMED_RRTSTAR, PRM };
static constexpr Planner PLANNER = Planner::INFORMED_RRTSTAR;// RRT_CONNECT or INFORMED_RRTSTAR / PRM
static constexpr int MAX_PLANNING_ATTEMPTS = 3;  // total attempts, including the initial search

using MotionPlan = moveit::planning_interface::MoveGroupInterface::Plan;

// time_sec: the search budget of each attempt (Informed RRT* only; the other planners keep their own).
std::optional<MotionPlan> run_planner(
    moveit::planning_interface::MoveGroupInterface &arm_group,
    rclcpp::Node::SharedPtr node,
    const std::vector<double> &q_goal,
    int attempts = MAX_PLANNING_ATTEMPTS,
    double time_sec = crrt_cfg::RRTSTAR_TIME_SEC)
{
    // Retry only the search. Publishing and execution remain outside this loop so a failed
    // execution can never trigger another movement through the planning retry mechanism.
    for (int attempt = 1; attempt <= attempts && rclcpp::ok(); ++attempt) {
        RCLCPP_INFO(node->get_logger(), "[Path Planning] Attempt %d/%d",
                    attempt, attempts);
        auto plan = [&]() {
            switch (PLANNER) {
                case Planner::INFORMED_RRTSTAR: return irrtstar_plan(arm_group, node, q_goal, time_sec);
                case Planner::PRM:              return prm_plan(arm_group, node, q_goal);
                case Planner::RRT_CONNECT:
                default:                        return crrt_plan(arm_group, node, q_goal);
            }
        }();
        if (plan) {
            return plan;
        }
        if (attempt < attempts && rclcpp::ok()) {
            RCLCPP_WARN(node->get_logger(),
                        "[Path Planning] Attempt %d/%d failed; replanning.",
                        attempt, attempts);
        }
    }
    return std::nullopt;
}

// How one command searches.
struct PlanOptions {
    bool ignore_wellplate_link = false;    // plan_wellplate: see search()
    std::function<void()> refresh_cloud;   // set for the commands that refresh the cloud before planning
};

// `attempts` searches of `time_sec` each to the joint goal q_goal. With margin > 0 the path also keeps
// that clearance from the obstacles, except straight above the start and the goal (crrt_clearance).
static std::optional<MotionPlan> search(
    moveit::planning_interface::MoveGroupInterface &arm_group,
    rclcpp::Node::SharedPtr node,
    const JointVec &q_goal, int attempts, double time_sec, const PlanOptions &opts,
    double margin = 0.0)
{
    std::optional<crrt_internal::ScopedSceneFreeze> scene_freeze;
    if (opts.ignore_wellplate_link) {
        // plan_wellplate is only sent for a pick, so the gripper is empty: wellplate_link (the held
        // plate modelled on Finger_left_2) is not there and may overlap the octomap.
        scene_freeze.emplace(node);
        crrt_internal::frozen_scene()->getAllowedCollisionMatrixNonConst().setEntry("wellplate_link", "<octomap>", true);
    }
    std::optional<crrt_clearance::Scope> clearance;
    if (margin > 0.0) {
        auto current_state = arm_group.getCurrentState(5.0);
        const auto* jmg = current_state ? current_state->getJointModelGroup(crrt_cfg::GROUP_NAME) : nullptr;
        if (!jmg) {
            RCLCPP_WARN(node->get_logger(), "[Fallback] No current state; skipping the safety-margin search.");
            return std::nullopt;
        }
        current_state->updateLinkTransforms();
        moveit::core::RobotState goal_state(*current_state);
        goal_state.setJointGroupPositions(jmg, q_goal);
        goal_state.updateLinkTransforms();
        clearance.emplace(margin, std::vector<Eigen::Vector3d>{
            current_state->getGlobalLinkTransform(crrt_cfg::EE_LINK).translation(),
            goal_state.getGlobalLinkTransform(crrt_cfg::EE_LINK).translation()});
    }
    arm_group.setJointValueTarget(q_goal);
    return run_planner(arm_group, node, q_goal, attempts, time_sec);
}

// IK for target_pose, seeded from the current state. With `fallback`, a failure is diagnosed and
// retried with IK_FALLBACK_TIMEOUT_SEC.
static std::optional<JointVec> solve_ik(
    moveit::planning_interface::MoveGroupInterface &arm_group,
    rclcpp::Node::SharedPtr node,
    const geometry_msgs::msg::Pose &target_pose, bool fallback)
{
    auto current_state = arm_group.getCurrentState(5.0);
    if (!current_state) {
        RCLCPP_WARN(node->get_logger(), "[Fallback] getCurrentState() failed; trying once more.");
        current_state = arm_group.getCurrentState(5.0);
    }
    if (!current_state) {
        RCLCPP_WARN(node->get_logger(), "getCurrentState() failed again.");
        return std::nullopt;
    }
    const auto* jmg = current_state->getJointModelGroup("demo_arm_bot");
    if (!jmg) {
        RCLCPP_WARN(node->get_logger(), "Joint model group 'demo_arm_bot' not found.");
        return std::nullopt;
    }
    moveit::core::RobotState ik_state(*current_state);
    bool solved = ik_state.setFromIK(jmg, target_pose, IK_TIMEOUT_SEC);
    if (!solved && fallback)
    {
        // ── Diagnose why ──────────────────────────────────────
        const auto &q = target_pose.orientation;
        const double qn = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
        if (std::abs(qn - 1.0) > 1e-3)
            RCLCPP_WARN(node->get_logger(),
                "[IK diag] BAD ORIENTATION: quaternion (%.3f, %.3f, %.3f, %.3f) has norm %.3f, not 1.",
                q.x, q.y, q.z, q.w, qn);

        const auto &p = target_pose.position;
        if (p.x < -0.15 || p.x > 0.85 || p.y < -0.50 || p.y > 0.50 || p.z < 0.90 || p.z > 1.30)
            RCLCPP_WARN(node->get_logger(),
                "[IK diag] OUTSIDE PLANNER WORKSPACE: (%.3f, %.3f, %.3f) not in x[-0.15,0.85] y[-0.50,0.50] z[0.90,1.30].",
                p.x, p.y, p.z);

        RCLCPP_WARN(node->get_logger(),
            "[Fallback] IK failed for (%.3f, %.3f, %.3f) in %.0f s; retrying with %.0f s.",
            p.x, p.y, p.z, IK_TIMEOUT_SEC, IK_FALLBACK_TIMEOUT_SEC);
        ik_state = *current_state;
        solved = ik_state.setFromIK(jmg, target_pose, IK_FALLBACK_TIMEOUT_SEC);
        if (solved)
            RCLCPP_WARN(node->get_logger(),
                "[IK diag] IK succeeded with %.0f s: the pose is reachable but hard (near a singularity / joint limit).",
                IK_FALLBACK_TIMEOUT_SEC);
        else
            RCLCPP_WARN(node->get_logger(),
                "[IK diag] UNREACHABLE: no solution even with %.0f s. The pose is out of reach, or the "
                "orientation is not achievable by the arm (or it is in the wrong frame — IK uses the "
                "model root frame, currently expected to be 'world').", IK_FALLBACK_TIMEOUT_SEC);
    }
    if (!solved) {
        if (!fallback)
            RCLCPP_WARN(node->get_logger(), "IK failed for (%.3f, %.3f, %.3f).",
                        target_pose.position.x, target_pose.position.y, target_pose.position.z);
        return std::nullopt;
    }
    JointVec joint_values;
    ik_state.copyJointGroupPositions(jmg, joint_values);
    return joint_values;
}

// The fallback ladder, one search after another (planners other than Informed RRT*; see find_plan).
// The goal is q_goal (plan_home_offset), or the IK of pose_goal. In order:
//   0. SAFETY_MARGIN_ATTEMPTS searches that keep SAFETY_MARGIN_M from the obstacles
//   1. MAX_PLANNING_ATTEMPTS normal searches to the goal
//   2. a fresh obstacle cloud (commands with opts.refresh_cloud), then FALLBACK_LONG_ATTEMPTS
//      searches of FALLBACK_LONG_TIME_SEC each
//   3. pose goals only: one normal search to the goal moved by each of GOAL_NUDGES
// A start or goal in collision fails at once, without a search, so those cases stay quick.
static std::optional<MotionPlan> find_plan_sequential(
    moveit::planning_interface::MoveGroupInterface &arm_group,
    rclcpp::Node::SharedPtr node,
    std::optional<JointVec> q_goal,
    const std::optional<geometry_msgs::msg::Pose> &pose_goal,
    const PlanOptions &opts)
{
    if (pose_goal)
        q_goal = solve_ik(arm_group, node, *pose_goal, true);

    if (q_goal) {
        RCLCPP_INFO(node->get_logger(), "[Path Planning] Searching with a %.0f mm safety margin around the obstacles.",
                    crrt_cfg::SAFETY_MARGIN_M * 1000.0);
        if (auto plan = search(arm_group, node, *q_goal, crrt_cfg::SAFETY_MARGIN_ATTEMPTS, crrt_cfg::RRTSTAR_TIME_SEC,
                               opts, crrt_cfg::SAFETY_MARGIN_M))
            return plan;
        if (rclcpp::ok())
            RCLCPP_WARN(node->get_logger(), "[Fallback] No path keeps the %.0f mm safety margin; planning without it.",
                        crrt_cfg::SAFETY_MARGIN_M * 1000.0);
        if (auto plan = search(arm_group, node, *q_goal, MAX_PLANNING_ATTEMPTS, crrt_cfg::RRTSTAR_TIME_SEC, opts))
            return plan;
        if (opts.refresh_cloud && rclcpp::ok()) {
            RCLCPP_WARN(node->get_logger(), "[Fallback] Planning failed; taking a fresh obstacle cloud.");
            opts.refresh_cloud();
        }
        if (rclcpp::ok()) {
            RCLCPP_WARN(node->get_logger(), "[Fallback] Trying %d longer searches (%.0f s each).",
                        FALLBACK_LONG_ATTEMPTS, FALLBACK_LONG_TIME_SEC);
            if (auto plan = search(arm_group, node, *q_goal, FALLBACK_LONG_ATTEMPTS, FALLBACK_LONG_TIME_SEC, opts))
                return plan;
        }
    }
    if (!pose_goal)
        return std::nullopt;

    for (const auto &d : GOAL_NUDGES) {
        if (!rclcpp::ok()) break;
        geometry_msgs::msg::Pose nudged = *pose_goal;
        nudged.position.x += d[0];
        nudged.position.y += d[1];
        nudged.position.z += d[2];
        RCLCPP_WARN(node->get_logger(),
            "[Fallback] Trying the goal moved by (%+.0f, %+.0f, %+.0f) mm: (%.3f, %.3f, %.3f).",
            d[0] * 1000, d[1] * 1000, d[2] * 1000, nudged.position.x, nudged.position.y, nudged.position.z);
        const auto q = solve_ik(arm_group, node, nudged, false);
        if (!q) continue;
        if (auto plan = search(arm_group, node, *q, 1, crrt_cfg::RRTSTAR_TIME_SEC, opts)) {
            RCLCPP_WARN(node->get_logger(),
                "[Fallback] Planned to the moved goal, (%+.0f, %+.0f, %+.0f) mm from the requested one.",
                d[0] * 1000, d[1] * 1000, d[2] * 1000);
            return plan;
        }
    }
    return std::nullopt;
}

// One stage of find_plan: the jobs searched at once (crrt_parallel::plan), each search at most time_sec.
static crrt_parallel::Result search_parallel(
    moveit::planning_interface::MoveGroupInterface &arm_group,
    rclcpp::Node::SharedPtr node,
    const std::vector<crrt_parallel::Job> &jobs, double time_sec, const PlanOptions &opts)
{
    std::optional<crrt_internal::ScopedSceneFreeze> scene_freeze;
    if (opts.ignore_wellplate_link) {
        // plan_wellplate is only sent for a pick, so the gripper is empty: wellplate_link (the held
        // plate modelled on Finger_left_2) is not there and may overlap the octomap.
        scene_freeze.emplace(node);
        crrt_internal::frozen_scene()->getAllowedCollisionMatrixNonConst().setEntry("wellplate_link", "<octomap>", true);
    }
    return crrt_parallel::plan(arm_group, node, jobs, time_sec);
}

// The plan for a command: the goal is q_goal (plan_home_offset), or the IK of pose_goal. An empty
// trajectory means the arm is already there (ALREADY_THERE_RAD). Every stage below runs its searches at
// once, and takes the straight path or a kept path (an earlier execution, either direction) without
// searching when it passes (crrt_parallel.hpp). The stages, in order:
//   1. SAFETY_MARGIN_ATTEMPTS searches that keep SAFETY_MARGIN_M from the obstacles, with
//      MAX_PLANNING_ATTEMPTS without it; a margin path found within PREFER_WAIT_SEC is preferred
//   2. pose goals whose goal is invalid (or has no IK): one search to the goal moved by each of GOAL_NUDGES
//   3. a fresh obstacle cloud (commands with opts.refresh_cloud), then FALLBACK_LONG_ATTEMPTS searches of
//      FALLBACK_LONG_TIME_SEC to the goal and, for pose goals, one to each moved goal (unless stage 2 tried
//      them on this same cloud); the goal itself is preferred
// A start or goal in collision fails its stage at once, without a search.
static std::optional<MotionPlan> find_plan(
    moveit::planning_interface::MoveGroupInterface &arm_group,
    rclcpp::Node::SharedPtr node,
    std::optional<JointVec> q_goal,
    const std::optional<geometry_msgs::msg::Pose> &pose_goal,
    const PlanOptions &opts)
{
    crrt_internal::last_finished_path().clear();
    if (pose_goal)
        q_goal = solve_ik(arm_group, node, *pose_goal, true);

    if (q_goal) {
        if (auto current_state = arm_group.getCurrentState(5.0)) {
            JointVec q_now;
            current_state->copyJointGroupPositions(crrt_cfg::GROUP_NAME, q_now);
            const double d = joint_dist(q_now, *q_goal);
            if (d < ALREADY_THERE_RAD) {
                RCLCPP_INFO(node->get_logger(), "[Path Planning] Already at the goal (%.3f rad away); not moving.", d);
                MotionPlan no_motion;   // empty trajectory
                return no_motion;
            }
        }
    }
    if (PLANNER != Planner::INFORMED_RRTSTAR)
        return find_plan_sequential(arm_group, node, q_goal, pose_goal, opts);

    // The moved goals (pose goals only), their IK solved when first needed.
    std::optional<std::vector<crrt_parallel::Job>> nudged;
    auto nudges = [&]() -> const std::vector<crrt_parallel::Job> & {
        if (!nudged) {
            nudged.emplace();
            for (const auto &d : GOAL_NUDGES) {
                geometry_msgs::msg::Pose p = *pose_goal;
                p.position.x += d[0];
                p.position.y += d[1];
                p.position.z += d[2];
                char label[96];
                std::snprintf(label, sizeof label, "the goal moved by (%+.0f, %+.0f, %+.0f) mm",
                              d[0] * 1000, d[1] * 1000, d[2] * 1000);
                if (const auto q = solve_ik(arm_group, node, p, false))
                    nudged->push_back({*q, 0.0, 1, label});
            }
        }
        return *nudged;
    };
    auto planned = [&](const crrt_parallel::Result &r, const std::vector<crrt_parallel::Job> &jobs, size_t first_moved) {
        if (r.plan && r.job >= first_moved)
            RCLCPP_WARN(node->get_logger(), "[Fallback] Planned to %s, not the requested goal.", jobs[r.job].label.c_str());
        return r.plan;
    };

    bool goal_invalid = !q_goal;   // no IK for the goal
    if (q_goal) {
        RCLCPP_INFO(node->get_logger(), "[Path Planning] Searching with a %.0f mm safety margin around the obstacles "
                    "(%d searches) and without it (%d), at once.", crrt_cfg::SAFETY_MARGIN_M * 1000.0,
                    crrt_cfg::SAFETY_MARGIN_ATTEMPTS, MAX_PLANNING_ATTEMPTS);
        const std::vector<crrt_parallel::Job> jobs = {
            {*q_goal, crrt_cfg::SAFETY_MARGIN_M, crrt_cfg::SAFETY_MARGIN_ATTEMPTS, "the goal, with the safety margin"},
            {*q_goal, 0.0, MAX_PLANNING_ATTEMPTS, "the goal, without the safety margin"}};
        const auto r = search_parallel(arm_group, node, jobs, crrt_cfg::RRTSTAR_TIME_SEC, opts);
        if (r.plan && r.job == 1)
            RCLCPP_WARN(node->get_logger(), "[Fallback] No path keeping the %.0f mm safety margin in time; planned without it.",
                        crrt_cfg::SAFETY_MARGIN_M * 1000.0);
        if (r.plan)
            return r.plan;
        goal_invalid = !r.start_invalid && !r.any_goal_valid;
    }

    bool nudges_tried = false;
    if (pose_goal && goal_invalid && rclcpp::ok() && !nudges().empty()) {
        RCLCPP_WARN(node->get_logger(), "[Fallback] The goal is invalid; trying %zu moved goals at once.", nudges().size());
        nudges_tried = true;
        if (auto plan = planned(search_parallel(arm_group, node, nudges(), crrt_cfg::RRTSTAR_TIME_SEC, opts), nudges(), 0))
            return plan;
    }

    bool fresh_cloud = false;
    if (q_goal && opts.refresh_cloud && rclcpp::ok()) {
        RCLCPP_WARN(node->get_logger(), "[Fallback] Planning failed; taking a fresh obstacle cloud.");
        opts.refresh_cloud();
        fresh_cloud = true;
    }
    if (!rclcpp::ok())
        return std::nullopt;
    std::vector<crrt_parallel::Job> jobs;
    if (q_goal)
        jobs.push_back({*q_goal, 0.0, FALLBACK_LONG_ATTEMPTS, "the goal"});
    const size_t first_moved = jobs.size();
    if (pose_goal && (fresh_cloud || !nudges_tried))
        jobs.insert(jobs.end(), nudges().begin(), nudges().end());
    if (jobs.empty())
        return std::nullopt;
    RCLCPP_WARN(node->get_logger(), "[Fallback] Trying longer searches (%.0f s): %d to the goal, %zu to moved goals, at once.",
                FALLBACK_LONG_TIME_SEC, q_goal ? FALLBACK_LONG_ATTEMPTS : 0, jobs.size() - first_moved);
    return planned(search_parallel(arm_group, node, jobs, FALLBACK_LONG_TIME_SEC, opts), jobs, first_moved);
}

std::optional<MotionPlan> plan_and_publish_rrt(
    const std::string &task_type,
    const geometry_msgs::msg::Pose &target_pose,
    moveit::planning_interface::MoveGroupInterface &arm_group,
    rclcpp::Node::SharedPtr node,
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub,
    const PlanOptions &opts = {})
{
    RCLCPP_INFO(node->get_logger(), "[Path planning] Task: %s  target=(%.3f, %.3f, %.3f)",
                task_type.c_str(),
                target_pose.position.x, target_pose.position.y, target_pose.position.z);

    // ── Configure planner ────────────────────────────────────
    arm_group.setPlanningPipelineId("ompl");
    arm_group.setPlannerId("RRTConnect"); // Dummy — we override the planner with our own CRRT implementation
    arm_group.setPlanningTime(ARM_PLANNING_TIME_SEC);           // match GUI exactly
    arm_group.setNumPlanningAttempts(20);     // match GUI exactly
    arm_group.setPoseReferenceFrame("world");
    arm_group.setPathConstraints(make_ee_down_constraint());
    arm_group.setStartStateToCurrentState();
    arm_group.setWorkspace(-0.15, -0.50, 0.90,
                            0.85,  0.50, 1.30);

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
        // ── Plan (IK, then the searches, with the fallbacks) ───
        moveit::planning_interface::MoveGroupInterface::Plan plan; // 
        auto crrt_opt = find_plan(arm_group, node, std::nullopt, target_pose, opts);
        if (!crrt_opt)
        {
            RCLCPP_WARN(node->get_logger(), "[Path Planning] Planning failed, fallbacks included.");
            return std::nullopt;
        }
        plan = std::move(*crrt_opt);
        if (plan.trajectory_.joint_trajectory.points.empty())
        {
            RCLCPP_INFO(node->get_logger(), "[Path Planning] Already at the goal; nothing to execute.");
            return plan;
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
        if (stop_requested)
        {
            RCLCPP_WARN(node->get_logger(), "[Path Planning] Stop requested; not executing the plan.");
            return std::nullopt;
        }
        {
            std_msgs::msg::String s;
            s.data = "EXECUTING";
            state_pub->publish(s);
        }
        RCLCPP_INFO(node->get_logger(), "[Path Planning] Reactivating xArm controller...");
//        if (!reactivate_xarm(node)) {
//            RCLCPP_ERROR(node->get_logger(), "[Path Planning] xArm reactivation failed — aborting execution.");
//            return std::nullopt;
//        }
        RCLCPP_INFO(node->get_logger(), "[Path Planning] Executing plan...");
        if (arm_group.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
        {
            // Not retried: the arm stopped partway, possibly because it hit something.
            RCLCPP_WARN(node->get_logger(), "[Path Planning] Execution failed (not retried).");
            // pub_status->publish([]{ std_msgs::msg::String s; s.data="RRT_EXEC_FAILED"; return s; }());
            return std::nullopt;
        }

        RCLCPP_INFO(node->get_logger(), "[Path Planning] Execution succeeded. Syncing state...");
        crrt_parallel::remember(crrt_internal::last_finished_path());
        // ── Update start state for next planning stage ─────────
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        arm_group.setStartStateToCurrentState();

        return plan;
    }
    catch (const std::exception &e)
    {
        RCLCPP_WARN(node->get_logger(), "[Path Planning] Exception: %s", e.what());
        return std::nullopt;
    }
    catch (...)
    {
        RCLCPP_WARN(node->get_logger(), "[Path Planning] Unknown exception occurred.");
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


// Inflate the obstacle cloud by `radius`: each point is kept and also copied `radius` away in all
// 26 axis and diagonal directions, so the octomap fills the voxels next to every obstacle too.
// One point per radius/2 cube is kept, which bounds the size of the published cloud.
static sensor_msgs::msg::PointCloud2 inflate_cloud(const sensor_msgs::msg::PointCloud2& in, double radius)
{
    const double cell = radius / 2.0;
    std::unordered_set<int64_t> filled;
    std::vector<std::array<float, 3>> pts;
    auto add = [&](float x, float y, float z) {
        const int64_t ix = static_cast<int64_t>(std::floor(x / cell)) & 0x1FFFFF;
        const int64_t iy = static_cast<int64_t>(std::floor(y / cell)) & 0x1FFFFF;
        const int64_t iz = static_cast<int64_t>(std::floor(z / cell)) & 0x1FFFFF;
        if (filled.insert((ix << 42) | (iy << 21) | iz).second)
            pts.push_back({x, y, z});
    };

    for (sensor_msgs::PointCloud2ConstIterator<float> x(in, "x"), y(in, "y"), z(in, "z"); x != x.end(); ++x, ++y, ++z)
        if (std::isfinite(*x) && std::isfinite(*y) && std::isfinite(*z))
            add(*x, *y, *z);

    const size_t n_obstacle = pts.size();
    for (size_t i = 0; i < n_obstacle; ++i) {
        const auto p = pts[i];
        for (int dx = -1; dx <= 1; ++dx)
            for (int dy = -1; dy <= 1; ++dy)
                for (int dz = -1; dz <= 1; ++dz) {
                    if (dx == 0 && dy == 0 && dz == 0) continue;
                    const double s = radius / std::sqrt(dx * dx + dy * dy + dz * dz);
                    add(p[0] + dx * s, p[1] + dy * s, p[2] + dz * s);
                }
    }

    sensor_msgs::msg::PointCloud2 out;
    out.header = in.header;
    sensor_msgs::PointCloud2Modifier mod(out);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(pts.size());
    out.is_dense = true;
    sensor_msgs::PointCloud2Iterator<float> ox(out, "x"), oy(out, "y"), oz(out, "z");
    for (const auto& p : pts) {
        *ox = p[0]; *oy = p[1]; *oz = p[2];
        ++ox; ++oy; ++oz;
    }
    return out;
}

static std::mutex cmd_mutex;
static std::string received_command = "idle";

void command_callback(const std_msgs::msg::String::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(cmd_mutex);
    received_command = msg->data;
    stop_requested = false;   // a stop only applies to the commands before it
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
            std::atomic_store(&latest_obstacle_cloud, msg);
        });

    // move_group's octomap reads this topic (sensors_3d.yaml). TEMP: plan_wellplate (arm at home_offset) and
    // plan_april_<id> clear the octomap and re-freeze a fresh cloud; other plans reuse it.
    auto frozen_cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/zed_pointcloud_frozen", rclcpp::SensorDataQoS());
    bool cloud_frozen = false;
    auto clear_octomap_client = node->create_client<std_srvs::srv::Empty>("/clear_octomap");

    auto command_sub = node->create_subscription<std_msgs::msg::String>(
        "/planning_command", 10, [&arm_group, &node](const std_msgs::msg::String::SharedPtr msg) {
            if (msg->data != "stop")
                return command_callback(msg);
            stop_requested = true;
            arm_group.stop();   // move_group ends the trajectory that is executing, if any
            RCLCPP_WARN(node->get_logger(), "Stop requested: the executing trajectory is stopped and the current "
                        "command's plan will not be executed.");
        });

    auto planning_state_pub = node->create_publisher<std_msgs::msg::String>("/planning_state", 10);

    auto publish_state = [&](const std::string& state) {
        std_msgs::msg::String s;
        s.data = state;
        try {
            planning_state_pub->publish(s);
        } catch (const std::exception &e) {
            RCLCPP_WARN(node->get_logger(), "Could not publish /planning_state %s: %s", state.c_str(), e.what());
        }
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


    // The only place a command fails. ERROR stops the executive; the arm is not moved further.
    auto finish_plan = [&](const std::optional<MotionPlan>& opt,
                           const std::string& why = "see the warnings above") {
        if (opt.has_value()) {
            publish_state("SUCCESS");
        } else {
            RCLCPP_ERROR(node->get_logger(), "Planner failed (%s) — aborting pipeline.", why.c_str());
            publish_state("ERROR");
        }
        std::lock_guard<std::mutex> lock(cmd_mutex);
        received_command = "idle";
    };

    // Wait up to timeout_sec for a /zed_pointcloud message (drop the cached one first to get a new one).
    auto wait_for_cloud = [&](int timeout_sec) {
        sensor_msgs::msg::PointCloud2::SharedPtr cloud;
        for (int elapsed_ms = 0; elapsed_ms < timeout_sec * 1000 && !cloud && rclcpp::ok(); elapsed_ms += 100)
        {
            rclcpp::sleep_for(std::chrono::milliseconds(100));
            cloud = std::atomic_load(&latest_obstacle_cloud);
        }
        return cloud;
    };

    // Publish `cloud` (inflated) for move_group's octomap, re-sending it every CLOUD_RESEND_MS. The planner
    // has its own scene copy (crrt_psm_node), and each octomap it receives replaces the <octomap> object, so
    // wait up to timeout_sec for a new, non-null one (a /clear_octomap arriving late only removes it).
    auto publish_cloud_to_planner = [&](const sensor_msgs::msg::PointCloud2& cloud, int timeout_sec) {
        auto planner_psm = crrt_internal::get_psm(node).psm;
        auto planner_octomap = [&] {
            planning_scene_monitor::LockedPlanningSceneRO ls(planner_psm);
            return ls->getWorld()->getObject("<octomap>");
        };
        const auto old_map = planner_octomap();
        const auto inflated = inflate_cloud(cloud, CLOUD_INFLATION_RADIUS);
        int sends = 0;
        for (int elapsed_ms = 0; elapsed_ms < timeout_sec * 1000 && rclcpp::ok(); elapsed_ms += 100)
        {
            if (elapsed_ms % CLOUD_RESEND_MS == 0)
            {
                frozen_cloud_pub->publish(inflated);
                ++sends;
            }
            rclcpp::sleep_for(std::chrono::milliseconds(100));
            const auto map = planner_octomap();
            if (map && map != old_map)
            {
                RCLCPP_INFO(node->get_logger(), "Refreshed obstacle cloud (%u -> %u points, inflated %.0f mm, sent %d x); planner has the new octomap.",
                            cloud.width * cloud.height, inflated.width, CLOUD_INFLATION_RADIUS * 1000.0, sends);
                return true;
            }
        }
        return false;
    };

    // Fallback for plan_wellplate / plan_april_<id> when planning fails: one more fresh cloud. It is
    // added to the octomap; the octomap is not cleared here.
    auto refresh_cloud_fallback = [&]() {
        std::atomic_store(&latest_obstacle_cloud, sensor_msgs::msg::PointCloud2::SharedPtr{});
        const auto cloud = wait_for_cloud(CLOUD_FALLBACK_TIMEOUT_SEC);
        if (!cloud)
            RCLCPP_WARN(node->get_logger(), "[Fallback] No fresh /zed_pointcloud within %d s; keeping the current octomap.",
                        CLOUD_FALLBACK_TIMEOUT_SEC);
        else if (!publish_cloud_to_planner(*cloud, CLOUD_FALLBACK_TIMEOUT_SEC))
            RCLCPP_WARN(node->get_logger(), "[Fallback] Planner did not receive the new octomap within %d s; planning with the one it has.",
                        CLOUD_FALLBACK_TIMEOUT_SEC);
    };

    // A tag / plate position for this command: a reading at most stale_sec old within wait_sec. Then the
    // fallbacks: such a reading within DETECTION_FALLBACK_WAIT_SEC more, and last the latest reading if
    // it was taken at most DETECTION_REUSE_MAX_AGE_SEC before the command (an older one may predate the
    // base's last move).
    using Reading = std::optional<std::pair<geometry_msgs::msg::Point, rclcpp::Time>>;
    auto wait_for_reading = [&](const std::string& what, const std::function<Reading()>& latest,
                                double stale_sec, int wait_sec, int step_ms,
                                const rclcpp::Time& cmd_time) -> std::optional<geometry_msgs::msg::Point> {
        auto wait_fresh = [&](int timeout_sec) -> std::optional<geometry_msgs::msg::Point> {
            for (int elapsed_ms = 0; elapsed_ms < timeout_sec * 1000 && rclcpp::ok(); elapsed_ms += step_ms)
            {
                const rclcpp::Time now = node->now();
                if (const auto r = latest(); r && (now - r->second).seconds() <= stale_sec)
                    return r->first;
                rclcpp::sleep_for(std::chrono::milliseconds(step_ms));
            }
            return std::nullopt;
        };
        if (const auto p = wait_fresh(wait_sec))
            return p;
        RCLCPP_WARN(node->get_logger(), "[Fallback] No fresh %s reading in %d s; waiting up to %d s more.",
                    what.c_str(), wait_sec, DETECTION_FALLBACK_WAIT_SEC);
        if (const auto p = wait_fresh(DETECTION_FALLBACK_WAIT_SEC))
            return p;
        if (const auto r = latest())
        {
            const double before_cmd = (cmd_time - r->second).seconds();
            if (before_cmd <= DETECTION_REUSE_MAX_AGE_SEC)
            {
                RCLCPP_WARN(node->get_logger(), "[Fallback] Using the last %s reading, taken %.1f s before the command.",
                            what.c_str(), before_cmd);
                return r->first;
            }
            RCLCPP_WARN(node->get_logger(), "[Fallback] Not reusing the last %s reading: taken %.1f s before the command (limit %.1f s).",
                        what.c_str(), before_cmd, DETECTION_REUSE_MAX_AGE_SEC);
        }
        return std::nullopt;
    };

    // 20 Hz: a new command starts within 50 ms (was up to 1 s). IDLE is still published once a second.
    rclcpp::Rate rate(20);
    auto last_idle_publish = std::chrono::steady_clock::now() - std::chrono::seconds(1);
    while (rclcpp::ok())
    {
        std::string cmd;
        {
            std::lock_guard<std::mutex> lock(cmd_mutex);
            cmd = received_command;
        }
        const rclcpp::Time cmd_time = node->now();

        // Nothing in here may end the node: an exception fails only this command (ERROR).
        try
        {
            // RCLCPP_INFO(node->get_logger(), "Received command: %s", cmd.c_str());   // printed "idle" every second
            if (cmd != "idle")
            {
                RCLCPP_INFO(node->get_logger(), "Received command: %s", cmd.c_str());
                auto node = rclcpp::Node::make_shared("controller_switch_node");
                if (!set_controller_active(node, "xarm6_traj_controller"))
                {
                    RCLCPP_WARN(node->get_logger(), "[Fallback] xarm6_traj_controller not active; trying once more.");
                    set_controller_active(node, "xarm6_traj_controller");
                }
            }

            if (cmd != "idle" && stop_requested)
            {
                finish_plan(std::nullopt, "stop requested");
                continue;
            }

            // Everything below reads the planner's own scene copy.
            if (cmd.rfind("plan_", 0) == 0)
            {
                const auto planner_psm = crrt_internal::get_psm(node).psm;
                if (!planner_psm || !planner_psm->getPlanningScene())
                {
                    finish_plan(std::nullopt, "the planner's planning scene monitor did not start; see the [CRRT] warnings");
                    continue;
                }
            }

            // plan_wellplate takes a fresh cloud (here and as its fallback) only with the arm at home_offset, out of
            // the camera's view; elsewhere it plans with the current map.
            bool wellplate_fresh_cloud = false;
            if (cmd == "plan_wellplate")
            {
                JointVec q_now;
                if (auto current_state = arm_group.getCurrentState(5.0))
                    current_state->copyJointGroupPositions(crrt_cfg::GROUP_NAME, q_now);
                if (q_now.size() != HOME_OFFSET_JOINTS.size())
                    RCLCPP_WARN(node->get_logger(), "No current arm state; reusing the current octomap (no fresh cloud).");
                else if (const double d = joint_dist(q_now, HOME_OFFSET_JOINTS); d > FRESH_CLOUD_HOME_RAD)
                    RCLCPP_WARN(node->get_logger(), "Arm %.2f rad from home_offset, maybe in the camera's view; reusing the "
                                "current octomap (no fresh cloud).", d);
                else
                    wellplate_fresh_cloud = true;
            }

            // TEMP: fresh cloud for plan_wellplate (arm at home_offset) / plan_april_<id>. Assumes the arm is out
            // of the camera's view.
            if ((cmd == "plan_wellplate" && wellplate_fresh_cloud) || cmd.rfind("plan_april_", 0) == 0)
            {
                // Drop the cached cloud and wait for one that arrives after this command.
                std::atomic_store(&latest_obstacle_cloud, sensor_msgs::msg::PointCloud2::SharedPtr{});
                auto cloud = wait_for_cloud(CLOUD_REFRESH_TIMEOUT_SEC);
                if (!cloud)
                {
                    RCLCPP_WARN(node->get_logger(), "[Fallback] No fresh /zed_pointcloud within %d s; waiting up to %d s more.",
                                CLOUD_REFRESH_TIMEOUT_SEC, CLOUD_FALLBACK_TIMEOUT_SEC);
                    cloud = wait_for_cloud(CLOUD_FALLBACK_TIMEOUT_SEC);
                }
                if (!cloud)
                {
                    finish_plan(std::nullopt, "no fresh /zed_pointcloud within " +
                                std::to_string(CLOUD_REFRESH_TIMEOUT_SEC + CLOUD_FALLBACK_TIMEOUT_SEC) + " s");
                    continue;
                }
                // Empty the octomap so voxels from the old cloud don't stay.
                auto fut = clear_octomap_client->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
                if (fut.wait_for(std::chrono::seconds(2)) != std::future_status::ready)
                    RCLCPP_WARN(node->get_logger(), "/clear_octomap did not answer; old voxels may remain.");

                if (!publish_cloud_to_planner(*cloud, CLOUD_REFRESH_TIMEOUT_SEC))
                {
                    RCLCPP_WARN(node->get_logger(), "[Fallback] Planner did not receive the fresh octomap within %d s; "
                                "re-sending it for up to %d s more.", CLOUD_REFRESH_TIMEOUT_SEC, CLOUD_FALLBACK_TIMEOUT_SEC);
                    if (!publish_cloud_to_planner(*cloud, CLOUD_FALLBACK_TIMEOUT_SEC))
                    {
                        finish_plan(std::nullopt, "the planner did not receive the fresh octomap");
                        continue;
                    }
                }
                cloud_frozen = true;
            }
            else if (!cloud_frozen && cmd.rfind("plan_", 0) == 0)
            {
                if (auto cloud = std::atomic_load(&latest_obstacle_cloud))
                {
                    const auto inflated = inflate_cloud(*cloud, CLOUD_INFLATION_RADIUS);
                    frozen_cloud_pub->publish(inflated);
                    cloud_frozen = true;
                    RCLCPP_INFO(node->get_logger(), "Froze obstacle cloud for the octomap (%u -> %u points, inflated %.0f mm).",
                                cloud->width * cloud->height, inflated.width, CLOUD_INFLATION_RADIUS * 1000.0);
                    // Give move_group time to fold it into the octomap before planning.
                    rclcpp::sleep_for(std::chrono::seconds(1));
                }
            }


            if (cmd.rfind("plan_april_", 0) == 0)
            {
                // Parse tag ID from "plan_april_<id>"
                const std::string april_prefix = "plan_april_";
                const std::string id_str = cmd.substr(april_prefix.size());
                if (id_str.empty()) {
                    finish_plan(std::nullopt, "plan_april_ command has no tag ID: '" + cmd + "'");
                    continue;
                }
                int tag_id = 0;
                try {
                    std::size_t pos = 0;
                    tag_id = std::stoi(id_str, &pos);
                    if (pos != id_str.size()) throw std::invalid_argument("trailing chars");
                } catch (...) {
                    finish_plan(std::nullopt, "could not parse the tag ID from command: " + cmd);
                    continue;
                }

                const auto apriltag_point = wait_for_reading(
                    "April tag " + std::to_string(tag_id),
                    [&]() -> Reading {
                        std::lock_guard<std::mutex> lock(april_tags_mutex);
                        const auto it = april_tags.find(tag_id);
                        if (it == april_tags.end()) return std::nullopt;
                        return std::make_pair(it->second.position, it->second.stamp);
                    },
                    APRIL_TAG_STALE_SEC, APRIL_TAG_WAIT_TIMEOUT_SEC, APRIL_TAG_WAIT_STEP_MS, cmd_time);
                if (!apriltag_point)
                {
                    finish_plan(std::nullopt, "April tag ID " + std::to_string(tag_id) + " not found or stale");
                    continue;
                }

                // Resolve final task parameters (live > default)
                const geometry_msgs::msg::Pose target_position =
                    live_target_pose.value_or(defaults::target_pose_func(node, marker_pub, *apriltag_point, 0.25, APPROACH_YAW));
                const std::string task_type =
                    live_task_type.value_or(defaults::TASK_TYPE);
                publish_target_marker(node, target_position);

                if (!live_target_pose)
                    RCLCPP_WARN(node->get_logger(), "Using default target pose.");
                if (!live_task_type)
                    RCLCPP_WARN(node->get_logger(), "Using default task type: %s.", task_type.c_str());

                const double PREGRASP_Z_OFFSET = 0.03;
                geometry_msgs::msg::Pose pregrasp_pose = target_position;
                pregrasp_pose.position.z += PREGRASP_Z_OFFSET;

                publish_state("PLANNING");
                PlanOptions opts;
                opts.refresh_cloud = refresh_cloud_fallback;
                auto rrt_plan_opt = plan_and_publish_rrt(
                    task_type, pregrasp_pose,
                    arm_group, node, planning_state_pub, opts);

                finish_plan(rrt_plan_opt);
            }
            else if (cmd == "plan_wellplate")
            {
                const auto wellplate_point = wait_for_reading(
                    "wellplate",
                    [&]() -> Reading {
                        std::lock_guard<std::mutex> lock(wellplate_mutex);
                        if (!wellplate_detection.has_value()) return std::nullopt;
                        return std::make_pair(wellplate_detection->position, wellplate_detection->stamp);
                    },
                    WELLPLATE_STALE_SEC, WELLPLATE_WAIT_TIMEOUT_SEC, WELLPLATE_WAIT_STEP_MS, cmd_time);
                if (!wellplate_point)
                {
                    finish_plan(std::nullopt, "wellplate not found or stale");
                    continue;
                }

                // Resolve final task parameters (live > default)
                const geometry_msgs::msg::Pose target_position =
                    live_target_pose.value_or(defaults::target_pose_func(node, marker_pub, *wellplate_point, 0.25, APPROACH_YAW));
                const std::string task_type =
                    live_task_type.value_or(defaults::TASK_TYPE);

                if (!live_target_pose)
                    RCLCPP_WARN(node->get_logger(), "Using default target pose.");
                if (!live_task_type)
                    RCLCPP_WARN(node->get_logger(), "Using default task type: %s.", task_type.c_str());

                geometry_msgs::msg::Pose pregrasp_pose = target_position;

                // 40 mm above the earlier 0.03: back at this hover after the grasp (visual servo return_to_start), the
                // held plate (wellplate_link) clears the voxels of where it stood, so plan_home_offset can start
                const double PREGRASP_Z_OFFSET = 0.07;
                pregrasp_pose.position.z += PREGRASP_Z_OFFSET;

                const double PREGRASP_Y_OFFSET = 0.0; //  0.1;
                pregrasp_pose.position.y += PREGRASP_Y_OFFSET;

                const double PREGRASP_X_OFFSET = 0.0; //  0.1;
                pregrasp_pose.position.x -= PREGRASP_X_OFFSET;


                publish_target_marker(node, pregrasp_pose);

                publish_state("PLANNING");
                PlanOptions opts;
                opts.ignore_wellplate_link = true;   // the gripper is empty for a pick; see search()
                if (wellplate_fresh_cloud)
                    opts.refresh_cloud = refresh_cloud_fallback;
                auto rrt_plan_opt = plan_and_publish_rrt(
                    task_type, pregrasp_pose,
                    arm_group, node, planning_state_pub, opts);

                finish_plan(rrt_plan_opt);
            }
            else if (cmd == "plan_home")
            {
                // TODO: Move Hardcoded values to a config file
                geometry_msgs::msg::Point home_point;
                home_point.x = -0.048;
                home_point.y = 0.443;
                home_point.z = 0.302;  // 0.247 + 0.055 force sensor

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
                publish_state("PLANNING");
                // A joint goal: the fallbacks are the longer searches (no goal nudges, no cloud refresh).
                auto rrt_plan_opt = find_plan(arm_group, node, HOME_OFFSET_JOINTS, std::nullopt, PlanOptions{});

                if (rrt_plan_opt && !rrt_plan_opt->trajectory_.joint_trajectory.points.empty()) {
                    if (stop_requested) {
                        finish_plan(std::nullopt, "stop requested; the plan was not executed");
                        continue;
                    }
                    publish_state("EXECUTING");
                    if (arm_group.execute(*rrt_plan_opt) != moveit::core::MoveItErrorCode::SUCCESS) {
                        finish_plan(std::nullopt, "execution failed (not retried)");
                        continue;
                    }
                    crrt_parallel::remember(crrt_internal::last_finished_path());
                }
                finish_plan(rrt_plan_opt);
            }
            else if (cmd != "idle")
            {
                // Clear it: it would otherwise be re-read (and logged, and a controller switch node made) 20x a second.
                finish_plan(std::nullopt, "unknown command '" + cmd + "'");
            }
            else if (std::chrono::steady_clock::now() - last_idle_publish >= std::chrono::seconds(1))
            {
                publish_state("IDLE");
                last_idle_publish = std::chrono::steady_clock::now();
            }
        }
        catch (const std::exception &e)
        {
            if (cmd != "idle")
                finish_plan(std::nullopt, std::string("exception: ") + e.what());
            else
                RCLCPP_WARN(node->get_logger(), "Exception while idle: %s", e.what());
        }
        catch (...)
        {
            if (cmd != "idle")
                finish_plan(std::nullopt, "unknown exception");
            else
                RCLCPP_WARN(node->get_logger(), "Unknown exception while idle.");
        }
        rate.sleep();
    }

    // ── Clean shutdown ───────────────────────────────────────
    executor.cancel();
    executor_thread.join();
    rclcpp::shutdown();
    return 0;
}
