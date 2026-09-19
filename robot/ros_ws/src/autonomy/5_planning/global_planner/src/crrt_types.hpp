// crrt_types.hpp
// Config, internal types, and PlanningSceneMonitor singleton.
// Include this first — all other crrt_*.hpp depend on it.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// Add this include at top of crrt_types.hpp
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <vector>
#include <optional>
#include <random>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <thread>
#include <string>
#include <algorithm>
#include <limits>
#include <queue>
#include <fstream>
#include <nlohmann/json.hpp>

// ─────────────────────────────────────────────────────────────
//  CONFIG
// ─────────────────────────────────────────────────────────────
namespace crrt_cfg {

static const std::string GROUP_NAME = "demo_arm_bot";
static const std::string EE_LINK    = "end_effector_p4_1";

// ee_down constraint — must match make_ee_down_constraint() in move_to_pose_node.cpp
static constexpr double EE_ROLL_TOL  = 0.2;   // rad (~11.5°)
static constexpr double EE_PITCH_TOL = 0.2;   // rad

// RRT-Connect tuning
static constexpr int    MAX_ITER     = 5000;
static constexpr double MAX_TIME_SEC = 12.0;
// Budget for each leg of the two-stage midpoint fallback, which runs only
// after MAX_TIME_SEC has already been spent. Worst case is MAX_TIME_SEC +
// 2 * STAGE_TIME_SEC before crrt_plan() gives up.
static constexpr double STAGE_TIME_SEC = 5.0;
static constexpr double STEP_SIZE    = 0.05;  // rad per step
static constexpr double GOAL_BIAS    = 0.3;  // 10 % samples toward goal
static constexpr double CONNECT_TOL  = 0.02;  // rad — trees considered joined
// Random-pair shortcut passes run on the raw RRT-Connect path (and on each
// leg of the midpoint fallback) before time parameterization. Each pass
// validates the straight segment at STEP_SIZE resolution, so cost scales
// with path length; 200 matches what prm_plan uses.
static constexpr int    SHORTCUT_ITERS = 200;

// Informed RRT* (irrtstar_plan) — anytime: keeps improving the first solution
// until RRTSTAR_TIME_SEC, then hands the best path to the same post-processing
// as crrt_plan.
static constexpr double RRTSTAR_TIME_SEC = 5.0;
static constexpr int    RRTSTAR_MAX_ITER = 20000;
static constexpr double RRTSTAR_ETA      = 0.5;   // max extension per step (rad)
static constexpr double RRTSTAR_GAMMA    = 3.0;   // near-neighbour radius scale

// Workspace bounds — must match setWorkspace() in move_to_pose_node.cpp
static constexpr double WS_X_MIN = -0.15;
static constexpr double WS_X_MAX =  0.85;
static constexpr double WS_Y_MIN = -0.50;
static constexpr double WS_Y_MAX =  0.50;
static constexpr double WS_Z_MIN =  0.90;
static constexpr double WS_Z_MAX =  1.30;

// Manual waypoint local sampling
static constexpr int    LOCAL_SAMPLES_PER_WP = 5;   // samples per manual waypoint
static constexpr double LOCAL_SIGMA          = 0.10; // Gaussian std dev in rad (~8.5°)

} // namespace crrt_cfg

// ─────────────────────────────────────────────────────────────
//  TYPES
// ─────────────────────────────────────────────────────────────
using JointVec = std::vector<double>;

struct RRTNode {
    JointVec q;
    int      parent = -1;
};

struct PRMNode {
    JointVec         q;
    std::vector<int> neighbors;
};

struct ManualWaypoint {
    int         index;
    JointVec    q;
    std::string note;
};

// ─────────────────────────────────────────────────────────────
//  FILE PATHS
// ─────────────────────────────────────────────────────────────
static const std::string PRM_ROADMAP_FILE      = "/home/robot/AutoLab/robot/prm_roadmap.bin";
static const std::string MANUAL_WAYPOINTS_FILE = "/home/robot/AutoLab/robot/manual_waypoints_0.json";

// ─────────────────────────────────────────────────────────────
//  PLANNING SCENE MONITOR  (singleton)
//  Lives on its own node + MultiThreadedExecutor so it never
//  interferes with your executor in main.cpp.
// ─────────────────────────────────────────────────────────────
namespace crrt_internal {

struct PSMHolder {
    rclcpp::Node::SharedPtr node;
    std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> psm;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec;
    std::thread spin_thread;
    bool ready = false;

    ~PSMHolder() {
        if (exec) exec->cancel();
        if (spin_thread.joinable()) spin_thread.join();
    }
};

static PSMHolder& get_psm(rclcpp::Node::SharedPtr node)
{
    static PSMHolder holder;
    static std::once_flag flag;

    std::call_once(flag, [&]()
    {
        RCLCPP_INFO(node->get_logger(), "[CRRT] Initialising PlanningSceneMonitor...");

        auto psm_node = rclcpp::Node::make_shared(
            "crrt_psm_node",
            rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

        holder.psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
            psm_node, "robot_description");

        if (!holder.psm->getPlanningScene()) {
            RCLCPP_ERROR(node->get_logger(),
                "[CRRT] PSM failed to init — check robot_description param.");
            return;
        }

        holder.psm->startSceneMonitor("/monitored_planning_scene");

        holder.exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        holder.exec->add_node(psm_node);
        holder.node = psm_node;
        holder.spin_thread = std::thread([&]() { holder.exec->spin(); });

        RCLCPP_INFO(node->get_logger(), "[CRRT] Waiting for planning scene (up to 5 s)...");
        rclcpp::Rate poll(20);
        auto deadline = node->now() + rclcpp::Duration::from_seconds(5.0);
        while (rclcpp::ok() && node->now() < deadline) {
            {
                planning_scene_monitor::LockedPlanningSceneRO ls(holder.psm);
                if (ls->getRobotModel() && !ls->getRobotModel()->getName().empty()) {
                    holder.ready = true;
                    break;
                }
            }
            poll.sleep();
        }

        if (holder.ready)
            RCLCPP_INFO(node->get_logger(), "[CRRT] PSM ready.");
        else {
            RCLCPP_WARN(node->get_logger(),
                "[CRRT] PSM not ready after 5 s — collision may be incomplete.");
            holder.ready = true;
        }
    });

    return holder;
}

} // namespace crrt_internal
