// constrained_rrt.hpp
// ROS2 Humble
//
// Collision: PlanningSceneMonitor -> checkCollision()
//   Full STL mesh + octomap (from ZED via move_group) + self-collision
//
// Constraint: ee_down — matches make_ee_down_constraint() in your cpp
// Algorithm:  RRT-Connect in joint space + shortcut smoothing
//
// USAGE in your cpp:
//   #include "constrained_rrt.hpp"
//   ...
//   // joint_values already exists from:
//   //   current_state->copyJointGroupPositions(jmg, joint_values);
//   //   arm_group.setJointValueTarget(joint_values);
//   //
//   // Replace the arm_group.plan(plan) block with:
//   auto crrt_opt = crrt_plan(arm_group, node, joint_values);
//   if (!crrt_opt) { return std::nullopt; }
//   plan = std::move(*crrt_opt);

#pragma once

// ── MoveIt / ROS ─────────────────────────────────────────────
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
// ── STL ──────────────────────────────────────────────────────
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
// ─────────────────────────────────────────────────────────────
//  CONFIG
// ─────────────────────────────────────────────────────────────
namespace crrt_cfg {

static const std::string GROUP_NAME = "demo_arm_bot";
static const std::string EE_LINK    = "end_effector_p4_1";

// Must match make_ee_down_constraint() in your cpp
static constexpr double EE_ROLL_TOL  = 0.2;   // rad
static constexpr double EE_PITCH_TOL = 0.2;   // rad

// RRT tuning
static constexpr int    MAX_ITER     = 20000;
static constexpr double MAX_TIME_SEC = 12.0;
static constexpr double STEP_SIZE    = 0.02;  // rad per step
static constexpr double GOAL_BIAS    = 0.10;  // 10% samples toward goal
static constexpr double CONNECT_TOL  = 0.02;  // rad — trees considered joined

} // namespace crrt_cfg

// ─────────────────────────────────────────────────────────────
//  PLANNING SCENE MONITOR  (singleton)
//
//  Lives on its own node + MultiThreadedExecutor so it never
//  interferes with your SingleThreadedExecutor in main.cpp
// ─────────────────────────────────────────────────────────────
namespace crrt_internal {

struct PSMHolder {
    rclcpp::Node::SharedPtr node; 
    std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> psm;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec; // Add this
    std::thread spin_thread;
    bool ready = false; // Add this

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
        RCLCPP_INFO(node->get_logger(),
            "[CRRT] Initialising PlanningSceneMonitor...");

        // Dedicated node — keeps PSM callbacks off your executor
        auto psm_node = rclcpp::Node::make_shared(
            "crrt_psm_node",
            rclcpp::NodeOptions()
                .automatically_declare_parameters_from_overrides(true));

        holder.psm =
            std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
                psm_node, "robot_description");

        if (!holder.psm->getPlanningScene()) {
            RCLCPP_ERROR(node->get_logger(),
                "[CRRT] PSM failed to init — check robot_description param.");
            return;
        }

        // Subscribe to move_group's full scene (meshes + octomap)
        holder.psm->startSceneMonitor("/monitored_planning_scene");

        // Spin on dedicated thread
        holder.exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        holder.exec->add_node(psm_node);
        holder.node = psm_node; // Ensure the holder knows which node to use

        holder.spin_thread = std::thread([&]() { 
            holder.exec->spin(); 
        });

        // Wait up to 5 s for first scene
        RCLCPP_INFO(node->get_logger(),
            "[CRRT] Waiting for planning scene (up to 5 s)...");
        rclcpp::Rate poll(20);
        auto deadline = node->now() + rclcpp::Duration::from_seconds(5.0);
        while (rclcpp::ok() && node->now() < deadline) {
            {
                planning_scene_monitor::LockedPlanningSceneRO ls(holder.psm);
                if (ls->getRobotModel() &&
                    !ls->getRobotModel()->getName().empty()) {
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

// ─────────────────────────────────────────────────────────────
//  INTERNAL TYPES
// ─────────────────────────────────────────────────────────────
using JointVec = std::vector<double>;

struct RRTNode {
    JointVec q;
    int      parent = -1;
};

// ─────────────────────────────────────────────────────────────
//  CONSTRAINT CHECK  (ee_down)
//  Cheap — always run before collision check
// ─────────────────────────────────────────────────────────────
static bool satisfies_ee_down(moveit::core::RobotState& rs)
{
    rs.updateLinkTransforms();
    const Eigen::Matrix3d R =
        rs.getGlobalLinkTransform(crrt_cfg::EE_LINK).rotation();

    double roll  = std::atan2( R(2,1), R(2,2));
    double pitch = std::atan2(-R(2,0),
                   std::sqrt(R(2,1)*R(2,1) + R(2,2)*R(2,2)));

    return (std::abs(roll)  < crrt_cfg::EE_ROLL_TOL &&
            std::abs(pitch) < crrt_cfg::EE_PITCH_TOL);
}

// ─────────────────────────────────────────────────────────────
//  VALIDITY CHECK
//  Takes a pre-snapshotted scene — NO lock acquired here.
//  1. ee_down constraint (cheap, first)
//  2. checkCollision on snapshot (full mesh + octomap + self)
// ─────────────────────────────────────────────────────────────
static bool is_valid(const JointVec& q,
                     moveit::core::RobotState& rs,
                     const moveit::core::JointModelGroup* jmg,
                     const planning_scene::PlanningScenePtr& scene)
{
    rs.setJointGroupPositions(jmg, q);
    rs.updateLinkTransforms();

    // 1. Constraint check — cheap, reject early
    if (!satisfies_ee_down(rs)) return false;

    // 2. Full collision check on snapshot (no lock needed)
    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult  res;
    req.group_name = crrt_cfg::GROUP_NAME;
    req.contacts   = false; // yes/no only — faster
    scene->checkCollision(req, res, rs);

    return !res.collision;
}

// ─────────────────────────────────────────────────────────────
//  RRT HELPERS
// ─────────────────────────────────────────────────────────────
static double joint_dist(const JointVec& a, const JointVec& b)
{
    double d = 0;
    for (size_t i = 0; i < a.size(); ++i)
        d += (a[i] - b[i]) * (a[i] - b[i]);
    return std::sqrt(d);
}

static JointVec steer(const JointVec& from, const JointVec& to, double step)
{
    double d = joint_dist(from, to);
    if (d <= step) return to;
    JointVec out(from.size());
    double r = step / d;
    for (size_t i = 0; i < from.size(); ++i)
        out[i] = from[i] + r * (to[i] - from[i]);
    return out;
}

static int nearest(const std::vector<RRTNode>& tree, const JointVec& q)
{
    int    best   = 0;
    double best_d = std::numeric_limits<double>::max();
    for (int i = 0; i < (int)tree.size(); ++i) {
        double d = joint_dist(tree[i].q, q);
        if (d < best_d) { best_d = d; best = i; }
    }
    return best;
}

// Try to extend tree one step toward q_target.
// Returns index of new node, or -1 if invalid.

static bool project_to_ee_down(moveit::core::RobotState& rs,
                               const moveit::core::JointModelGroup* jmg)
{
    Eigen::Vector3d pos_before = rs.getGlobalLinkTransform(crrt_cfg::EE_LINK).translation();

    geometry_msgs::msg::Pose projected_pose;
    projected_pose.position.x = pos_before.x();
    projected_pose.position.y = pos_before.y();
    projected_pose.position.z = pos_before.z();
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, M_PI);
    projected_pose.orientation = tf2::toMsg(q);

    if (!rs.setFromIK(jmg, projected_pose, 0.01)) return false;

    // Reject if IK drifted the EE position more than half a step
    rs.updateLinkTransforms();
    Eigen::Vector3d pos_after = rs.getGlobalLinkTransform(crrt_cfg::EE_LINK).translation();
    return (pos_after - pos_before).norm() < (crrt_cfg::STEP_SIZE * 0.5);
}


static bool project_to_ee_down_sample(moveit::core::RobotState& rs,
                                       const moveit::core::JointModelGroup* jmg)
{
    // No drift check — just project orientation, any valid IK solution is fine
    geometry_msgs::msg::Pose projected_pose;
    rs.updateLinkTransforms();
    const Eigen::Vector3d pos = rs.getGlobalLinkTransform(crrt_cfg::EE_LINK).translation();
    projected_pose.position.x = pos.x();
    projected_pose.position.y = pos.y();
    projected_pose.position.z = pos.z();
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, M_PI);
    projected_pose.orientation = tf2::toMsg(q);
    return rs.setFromIK(jmg, projected_pose, 0.005);  // more time, no drift check
}

static int extend_greedy_with_projection(std::vector<RRTNode>& tree,
                                         const JointVec& q_target,
                                         moveit::core::RobotState& rs,
                                         const moveit::core::JointModelGroup* jmg,
                                         const planning_scene::PlanningScenePtr& scene)
{
    int last_added_idx = -1;
    int ni = nearest(tree, q_target);
    JointVec q_curr = tree[ni].q;

    // Greedy loop: keep stepping until we hit an obstacle or reach target
    while (joint_dist(q_curr, q_target) > 1e-6) 
    {
        JointVec q_steered = steer(q_curr, q_target, crrt_cfg::STEP_SIZE);
        
        // 1. Move robot to steered position
        rs.setJointGroupPositions(jmg, q_steered);

        // 2. PROJECT: Force end-effector to point down
        // If projection fails (joint limits/singularities), stop this branch
        if (!project_to_ee_down(rs, jmg)) {
            break; 
        }

        // 3. Get the new projected joint values
        JointVec q_projected;
        rs.copyJointGroupPositions(jmg, q_projected);

        // 4. COLLISION CHECK: Full scene check (Octomap + STL)
        collision_detection::CollisionRequest req;
        collision_detection::CollisionResult res;
        req.group_name = crrt_cfg::GROUP_NAME;
        scene->checkCollision(req, res, rs);

        if (res.collision) {
            break; // Hit a wall
        }

        // 5. Add to tree and continue
        tree.push_back({q_projected, (last_added_idx == -1) ? ni : last_added_idx});
        last_added_idx = (int)tree.size() - 1;
        q_curr = q_projected;

        // Safety break if we aren't making progress
        if (joint_dist(q_curr, q_target) < 0.001) break;
    }

    return last_added_idx;
}
// Walk parent pointers to extract path from root to idx
static std::vector<JointVec> extract_path(const std::vector<RRTNode>& tree,
                                           int idx)
{
    std::vector<JointVec> path;
    while (idx >= 0) {
        path.push_back(tree[idx].q);
        idx = tree[idx].parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

// Shortcut smoothing: try to skip intermediate waypoints
static std::vector<JointVec> smooth_path(const std::vector<JointVec>& raw,
                                          moveit::core::RobotState& rs,
                                          const moveit::core::JointModelGroup* jmg,
                                          const planning_scene::PlanningScenePtr& scene)
{
    if (raw.size() <= 2) return raw;
    std::vector<JointVec> path = raw;
    bool changed = true;
    int  passes  = 0;
    while (changed && passes++ < 10) {
        changed = false;
        for (size_t i = 0; i + 2 < path.size(); ) {
            bool     ok  = true;
            JointVec cur = path[i];
            while (joint_dist(cur, path[i+2]) > crrt_cfg::STEP_SIZE) {
                cur = steer(cur, path[i+2], crrt_cfg::STEP_SIZE);
                if (!is_valid(cur, rs, jmg, scene)) { ok = false; break; }
            }
            if (ok && is_valid(path[i+2], rs, jmg, scene)) {
                path.erase(path.begin() + i + 1);
                changed = true;
            } else {
                ++i;
            }
        }
    }
    return path;
}

// ─────────────────────────────────────────────────────────────
//  BUILD MoveGroupInterface::Plan FROM WAYPOINTS
// ─────────────────────────────────────────────────────────────
static moveit::planning_interface::MoveGroupInterface::Plan
build_plan(const std::vector<JointVec>& waypoints,
           const std::vector<std::string>& joint_names,
           moveit::planning_interface::MoveGroupInterface& arm_group)
{
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    auto cs = arm_group.getCurrentState(5.0);
    moveit::core::robotStateToRobotStateMsg(*cs, plan.start_state_);

    trajectory_msgs::msg::JointTrajectory jt;
    jt.joint_names = joint_names;

    const double MAX_JOINT_VEL = 3.14; // rad/s from your URDF limits
    double vel_scale = 0.1;
    if (vel_scale < 1e-6) vel_scale = 0.1;

    double t = 0.0;
    for (size_t i = 0; i < waypoints.size(); ++i) {
        trajectory_msgs::msg::JointTrajectoryPoint pt;
        pt.positions     = waypoints[i];
        pt.velocities.assign(waypoints[i].size(), 0.0);
        pt.accelerations.assign(waypoints[i].size(), 0.0);

        if (i > 0) {
            double max_delta = 0.0;
            for (size_t j = 0; j < waypoints[i].size(); ++j)
                max_delta = std::max(max_delta,
                    std::abs(waypoints[i][j] - waypoints[i-1][j]));
            t += max_delta / (MAX_JOINT_VEL * vel_scale);
        }
        pt.time_from_start = rclcpp::Duration::from_seconds(t);
        jt.points.push_back(pt);
    }

    moveit_msgs::msg::RobotTrajectory rt;
    rt.joint_trajectory = jt;
    plan.trajectory_    = rt;
    return plan;
}

// ─────────────────────────────────────────────────────────────
//  PRM — ROADMAP FILE PATH
// ─────────────────────────────────────────────────────────────
static const std::string PRM_ROADMAP_FILE = "/home/robot/AutoLab/robot/prm_roadmap.bin";

// ─────────────────────────────────────────────────────────────
//  PRM NODE
// ─────────────────────────────────────────────────────────────
struct PRMNode {
    JointVec q;
    std::vector<int> neighbors;
};

// ─────────────────────────────────────────────────────────────
//  1. BUILD PRM ROADMAP
//  Samples N valid configurations across the workspace,
//  connects each to its K nearest valid neighbors,
//  and saves to file.
// ─────────────────────────────────────────────────────────────

void build_prm_roadmap(
    moveit::planning_interface::MoveGroupInterface& arm_group,
    rclcpp::Node::SharedPtr node,
    int num_samples = 5000,
    int k_neighbors = 10)
{
    auto& psm_holder = crrt_internal::get_psm(node);
    auto robot_model = arm_group.getRobotModel();
    const auto* jmg  = robot_model->getJointModelGroup(crrt_cfg::GROUP_NAME);

    planning_scene::PlanningScenePtr scene_snapshot;
    {
        planning_scene_monitor::LockedPlanningSceneRO ls(psm_holder.psm);
        scene_snapshot = planning_scene::PlanningScene::clone(
            psm_holder.psm->getPlanningScene());
    }
    

    moveit::core::RobotState rs(robot_model);
    rs.setToDefaultValues();

    // ── Joint limits ──────────────────────────────────────────
    std::vector<std::pair<double,double>> limits;
    for (const auto* j : jmg->getActiveJointModels()) {
        const auto& bnd = j->getVariableBounds()[0];
        limits.push_back({bnd.min_position_, bnd.max_position_});
    }

    std::mt19937 rng(42);  // fixed seed for reproducibility
    std::vector<PRMNode> nodes;
    nodes.reserve(num_samples);

    RCLCPP_INFO(node->get_logger(),
        "[PRM] Sampling %d valid configurations...", num_samples);

    int attempts = 0;
    int fail_ik = 0, fail_collision = 0, fail_constraint = 0;

    while ((int)nodes.size() < num_samples && attempts < num_samples * 50) {
        ++attempts;

        JointVec q(limits.size());
        for (size_t i = 0; i < limits.size(); ++i) {
            std::uniform_real_distribution<double> d(
                limits[i].first, limits[i].second);
            q[i] = d(rng);
        }

        moveit::core::RobotState rs_sample(robot_model);
        rs_sample.setToDefaultValues();
        rs_sample.setJointGroupPositions(jmg, q);
        rs_sample.updateLinkTransforms();

        if (!project_to_ee_down_sample(rs_sample, jmg)) { ++fail_ik; continue; }

        JointVec q_projected;
        rs_sample.copyJointGroupPositions(jmg, q_projected);

        // Split is_valid into two parts to see which fails
        rs.setJointGroupPositions(jmg, q_projected);
        rs.updateLinkTransforms();
        if (!satisfies_ee_down(rs)) { ++fail_constraint; continue; }

        collision_detection::CollisionRequest req;
        collision_detection::CollisionResult  res;
        req.group_name = crrt_cfg::GROUP_NAME;
        req.contacts   = false;
        scene_snapshot->checkCollision(req, res, rs);
        if (res.collision) { ++fail_collision; continue; }

        nodes.push_back({q_projected, {}});

        if (attempts % 1000 == 0)
            RCLCPP_INFO(node->get_logger(),
                "[PRM] %zu valid | %d attempts | IK fail: %d | constraint fail: %d | collision fail: %d",
                nodes.size(), attempts, fail_ik, fail_constraint, fail_collision);
    }
    
    
    

    RCLCPP_INFO(node->get_logger(),
        "[PRM] Sampling done: %zu valid / %d attempts | IK: %d | constraint: %d | collision: %d",
        nodes.size(), attempts, fail_ik, fail_constraint, fail_collision);

    // After main sampling loop in build_prm_roadmap, add:
    RCLCPP_INFO(node->get_logger(), "[PRM] Bridge sampling for narrow passages...");
    int bridge_attempts = 0;
    int bridge_added = 0;
    while (bridge_attempts < 50000) {
        ++bridge_attempts;

        // Sample two random configs
        JointVec q1(limits.size()), q2(limits.size());
        for (size_t i = 0; i < limits.size(); ++i) {
            std::uniform_real_distribution<double> d(limits[i].first, limits[i].second);
            q1[i] = d(rng);
            q2[i] = d(rng);
        }

        // Project both onto constraint manifold
        moveit::core::RobotState rs1(robot_model), rs2(robot_model);
        rs1.setToDefaultValues(); rs1.setJointGroupPositions(jmg, q1); rs1.updateLinkTransforms();
        rs2.setToDefaultValues(); rs2.setJointGroupPositions(jmg, q2); rs2.updateLinkTransforms();
        if (!project_to_ee_down_sample(rs1, jmg)) continue;
        if (!project_to_ee_down_sample(rs2, jmg)) continue;
        rs1.copyJointGroupPositions(jmg, q1);
        rs2.copyJointGroupPositions(jmg, q2);

        // Both must be in collision
        moveit::core::RobotState rs_check(robot_model);
        rs_check.setToDefaultValues();

        rs_check.setJointGroupPositions(jmg, q1); rs_check.updateLinkTransforms();
        collision_detection::CollisionRequest req; collision_detection::CollisionResult res;
        req.group_name = crrt_cfg::GROUP_NAME; req.contacts = false;
        scene_snapshot->checkCollision(req, res, rs_check);
        if (!res.collision) continue;  // q1 must be IN collision

        res.clear();
        rs_check.setJointGroupPositions(jmg, q2); rs_check.updateLinkTransforms();
        scene_snapshot->checkCollision(req, res, rs_check);
        if (!res.collision) continue;  // q2 must be IN collision

        // Midpoint must be FREE — this is the narrow passage node
        JointVec q_mid(limits.size());
        for (size_t i = 0; i < limits.size(); ++i)
            q_mid[i] = (q1[i] + q2[i]) * 0.5;

        moveit::core::RobotState rs_mid(robot_model);
        rs_mid.setToDefaultValues();
        rs_mid.setJointGroupPositions(jmg, q_mid);
        rs_mid.updateLinkTransforms();
        if (!project_to_ee_down_sample(rs_mid, jmg)) continue;
        rs_mid.copyJointGroupPositions(jmg, q_mid);

        if (is_valid(q_mid, rs_check, jmg, scene_snapshot)) {
            nodes.push_back({q_mid, {}});
            ++bridge_added;
        }
    }
    RCLCPP_INFO(node->get_logger(), 
        "[PRM] Bridge sampling: %d nodes added from %d attempts",
        bridge_added, bridge_attempts);
    // ── Connect K nearest neighbors ───────────────────────────
    RCLCPP_INFO(node->get_logger(), "[PRM] Connecting neighbors...");
    int edge_ok_count = 0, edge_fail_count = 0;

    for (int i = 0; i < (int)nodes.size(); ++i) {
        moveit::core::RobotState rs_edge(robot_model);
        rs_edge.setToDefaultValues();

        std::vector<std::pair<double,int>> dists;
        for (int j = 0; j < (int)nodes.size(); ++j) {
            if (i == j) continue;
            dists.push_back({joint_dist(nodes[i].q, nodes[j].q), j});
        }
        std::sort(dists.begin(), dists.end());

        int connected = 0;
        for (auto& [d, j] : dists) {
            if (connected >= k_neighbors) break;

            bool edge_ok = true;
            JointVec cur = nodes[i].q;
            while (joint_dist(cur, nodes[j].q) > crrt_cfg::STEP_SIZE) {
                cur = steer(cur, nodes[j].q, crrt_cfg::STEP_SIZE);
                if (!is_valid(cur, rs_edge, jmg, scene_snapshot)) {
                    edge_ok = false;
                    break;
                }
            }
            if (edge_ok) {
                nodes[i].neighbors.push_back(j);
                nodes[j].neighbors.push_back(i);
                ++connected;
                ++edge_ok_count;
            } else {
                ++edge_fail_count;
            }
        }

        if (i == 0)
            RCLCPP_INFO(node->get_logger(),
                "[PRM] Node 0: connected=%d, ok=%d, fail=%d, nearest_dist=%.4f",
                connected, edge_ok_count, edge_fail_count, dists[0].first);

        if (i % 500 == 0)
            RCLCPP_INFO(node->get_logger(),
                "[PRM] Connected %d/%zu nodes...", i, nodes.size());
    }

    // ── Connectivity report ───────────────────────────────────
    int total_edges = 0;
    for (const auto& n : nodes) total_edges += n.neighbors.size();
    RCLCPP_INFO(node->get_logger(),
        "[PRM] Roadmap connectivity: %d total edges, avg %.1f per node",
        total_edges / 2, (double)total_edges / nodes.size());

    // ── Save to file ──────────────────────────────────────────
    std::ofstream f(PRM_ROADMAP_FILE, std::ios::binary);
    if (!f) {
        RCLCPP_ERROR(node->get_logger(),
            "[PRM] Failed to open %s for writing.", PRM_ROADMAP_FILE.c_str());
        return;
    }

    size_t n = nodes.size();
    size_t dof = limits.size();
    f.write(reinterpret_cast<const char*>(&n),   sizeof(n));
    f.write(reinterpret_cast<const char*>(&dof), sizeof(dof));
    for (const auto& node_prm : nodes) {
        f.write(reinterpret_cast<const char*>(node_prm.q.data()),
                dof * sizeof(double));
        size_t nn = node_prm.neighbors.size();
        f.write(reinterpret_cast<const char*>(&nn), sizeof(nn));
        f.write(reinterpret_cast<const char*>(node_prm.neighbors.data()),
                nn * sizeof(int));
    }

    RCLCPP_INFO(node->get_logger(),
        "[PRM] Roadmap saved: %zu nodes → %s", n, PRM_ROADMAP_FILE.c_str());
}

// ─────────────────────────────────────────────────────────────
//  LOAD PRM ROADMAP FROM FILE
// ─────────────────────────────────────────────────────────────
static std::vector<PRMNode> load_prm_roadmap(
    rclcpp::Node::SharedPtr node,
    size_t expected_dof)
{
    std::ifstream f(PRM_ROADMAP_FILE, std::ios::binary);
    if (!f) {
        RCLCPP_WARN(node->get_logger(),
            "[PRM] Roadmap file not found: %s", PRM_ROADMAP_FILE.c_str());
        return {};
    }

    size_t n, dof;
    f.read(reinterpret_cast<char*>(&n),   sizeof(n));
    f.read(reinterpret_cast<char*>(&dof), sizeof(dof));

    if (dof != expected_dof) {
        RCLCPP_ERROR(node->get_logger(),
            "[PRM] DOF mismatch: file has %zu, robot has %zu — rebuild roadmap.",
            dof, expected_dof);
        return {};
    }

    std::vector<PRMNode> nodes(n);
    for (auto& node_prm : nodes) {
        node_prm.q.resize(dof);
        f.read(reinterpret_cast<char*>(node_prm.q.data()), dof * sizeof(double));
        size_t nn;
        f.read(reinterpret_cast<char*>(&nn), sizeof(nn));
        node_prm.neighbors.resize(nn);
        f.read(reinterpret_cast<char*>(node_prm.neighbors.data()), nn * sizeof(int));
    }

    RCLCPP_INFO(node->get_logger(),
        "[PRM] Loaded roadmap: %zu nodes from %s", n, PRM_ROADMAP_FILE.c_str());
    return nodes;
}

// ─────────────────────────────────────────────────────────────
//  2. PRM QUERY
//  Connects q_start and q_goal into the roadmap,
//  runs Dijkstra, returns a plan.
// ─────────────────────────────────────────────────────────────
std::optional<moveit::planning_interface::MoveGroupInterface::Plan>
prm_plan(
    moveit::planning_interface::MoveGroupInterface& arm_group,
    rclcpp::Node::SharedPtr node,
    const std::vector<double>& q_goal,
    bool force_rebuild = false)
{
    auto t0 = std::chrono::steady_clock::now();

    auto& psm_holder = crrt_internal::get_psm(node);
    auto robot_model = arm_group.getRobotModel();
    const auto* jmg  = robot_model->getJointModelGroup(crrt_cfg::GROUP_NAME);
    if (!jmg) return std::nullopt;

    const auto& joint_names = jmg->getVariableNames();
    const size_t dof = joint_names.size();

    planning_scene::PlanningScenePtr scene_snapshot;
    {
        planning_scene_monitor::LockedPlanningSceneRO ls(psm_holder.psm);
        scene_snapshot = planning_scene::PlanningScene::clone(
            psm_holder.psm->getPlanningScene());
    }

    moveit::core::RobotState rs(robot_model);
    rs.setToDefaultValues();

    // ── Get start ─────────────────────────────────────────────
    auto current_state = arm_group.getCurrentState(5.0);
    if (!current_state) return std::nullopt;
    JointVec q_start;
    current_state->copyJointGroupPositions(jmg, q_start);

    // ── Validate start and goal ───────────────────────────────
    if (!is_valid(q_start, rs, jmg, scene_snapshot)) {
        RCLCPP_ERROR(node->get_logger(), "[PRM] Start state is invalid.");
        return std::nullopt;
    }
    if (!is_valid(q_goal, rs, jmg, scene_snapshot)) {
        RCLCPP_ERROR(node->get_logger(), "[PRM] Goal state is invalid.");
        return std::nullopt;
    }

    // ── Load or build roadmap ─────────────────────────────────
    if (force_rebuild) {
        RCLCPP_INFO(node->get_logger(), "[PRM] Force rebuilding roadmap...");
        build_prm_roadmap(arm_group, node);
    }

    std::vector<PRMNode> roadmap = load_prm_roadmap(node, dof);
    if (roadmap.empty()) {
        RCLCPP_INFO(node->get_logger(),
            "[PRM] No roadmap found — building now...");
        build_prm_roadmap(arm_group, node);
        roadmap = load_prm_roadmap(node, dof);
        if (roadmap.empty()) {
            RCLCPP_ERROR(node->get_logger(), "[PRM] Roadmap build failed.");
            return std::nullopt;
        }
    }

    // ── Hook start and goal into roadmap as virtual nodes ─────
    // Indices: roadmap nodes are 0..N-1, start = N, goal = N+1
    const int N        = (int)roadmap.size();
    const int START_ID = N;
    const int GOAL_ID  = N + 1;

    roadmap.push_back({q_start, {}});
    roadmap.push_back({q_goal,  {}});

    const int K_HOOK = 10;  // connect start/goal to K nearest roadmap nodes

    for (int hook_id : {START_ID, GOAL_ID}) {
        std::vector<std::pair<double,int>> dists;
        for (int i = 0; i < N; ++i)
            dists.push_back({joint_dist(roadmap[hook_id].q, roadmap[i].q), i});
        std::sort(dists.begin(), dists.end());

        for (auto& [d, i] : dists) {
            if ((int)roadmap[hook_id].neighbors.size() >= K_HOOK) break;

            bool edge_ok = true;
            JointVec cur = roadmap[hook_id].q;
            while (joint_dist(cur, roadmap[i].q) > crrt_cfg::STEP_SIZE) {
                cur = steer(cur, roadmap[i].q, crrt_cfg::STEP_SIZE);
                if (!is_valid(cur, rs, jmg, scene_snapshot)) {
                    edge_ok = false;
                    break;
                }
            }
            if (edge_ok) {
                roadmap[hook_id].neighbors.push_back(i);
                roadmap[i].neighbors.push_back(hook_id);
            }
        }

        RCLCPP_INFO(node->get_logger(),
            "[PRM] %s hooked to %zu roadmap nodes.",
            (hook_id == START_ID ? "Start" : "Goal"),
            roadmap[hook_id].neighbors.size());
    }

    if (roadmap[START_ID].neighbors.empty() || roadmap[GOAL_ID].neighbors.empty()) {
        RCLCPP_ERROR(node->get_logger(),
            "[PRM] Could not connect start/goal to roadmap — aborting.");
        return std::nullopt;
    }

    // ── Dijkstra ──────────────────────────────────────────────
    const int total = (int)roadmap.size();
    std::vector<double> dist(total, std::numeric_limits<double>::max());
    std::vector<int>    prev(total, -1);

    using PQPair = std::pair<double, int>;
    std::priority_queue<PQPair, std::vector<PQPair>, std::greater<PQPair>> pq;

    dist[START_ID] = 0.0;
    pq.push({0.0, START_ID});

    while (!pq.empty()) {
        auto [d, u] = pq.top(); pq.pop();
        if (d > dist[u]) continue;
        if (u == GOAL_ID) break;

        for (int v : roadmap[u].neighbors) {
            double nd = dist[u] + joint_dist(roadmap[u].q, roadmap[v].q);
            if (nd < dist[v]) {
                dist[v] = nd;
                prev[v] = u;
                pq.push({nd, v});
            }
        }
    }

    if (dist[GOAL_ID] == std::numeric_limits<double>::max()) {
        RCLCPP_ERROR(node->get_logger(),
            "[PRM] Dijkstra found no path — roadmap may need rebuilding.");
        return std::nullopt;
    }

    // ── Extract path ──────────────────────────────────────────
    std::vector<JointVec> full_path;
    for (int cur = GOAL_ID; cur != -1; cur = prev[cur])
        full_path.push_back(roadmap[cur].q);
    std::reverse(full_path.begin(), full_path.end());

    // ── Final collision validation ────────────────────────────
    moveit::core::RobotState rs_check(robot_model);
    rs_check.setToDefaultValues();
    for (const auto& q : full_path) {
        if (!is_valid(q, rs_check, jmg, scene_snapshot)) {
            RCLCPP_ERROR(node->get_logger(),
                "[PRM] Final path contains colliding waypoint — aborting.");
            return std::nullopt;
        }
    }

    double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t0).count();
    RCLCPP_INFO(node->get_logger(),
        "[PRM] Path found: %zu waypoints in %.2f s.", full_path.size(), elapsed);

    auto plan = build_plan(
        full_path,
        std::vector<std::string>(joint_names.begin(), joint_names.end()),
        arm_group);
    plan.planning_time_ = elapsed;
    return plan;
}


// ─────────────────────────────────────────────────────────────
//  PUBLIC API
//
//  q_goal = joint_values from your cpp (already computed via IK)
// ─────────────────────────────────────────────────────────────
std::optional<moveit::planning_interface::MoveGroupInterface::Plan>
crrt_plan(
    moveit::planning_interface::MoveGroupInterface& arm_group,
    rclcpp::Node::SharedPtr node,
    const std::vector<double>& q_goal)
{
    auto t0 = std::chrono::steady_clock::now();

    // ── 1. Init PSM (no-op after first call) ──────────────────
    auto& psm_holder = crrt_internal::get_psm(node);

    // ── 2. Robot model + JMG ──────────────────────────────────
    auto robot_model = arm_group.getRobotModel();
    const auto* jmg  = robot_model->getJointModelGroup(crrt_cfg::GROUP_NAME);
    if (!jmg) {
        RCLCPP_ERROR(node->get_logger(),
            "[CRRT] JMG '%s' not found.", crrt_cfg::GROUP_NAME.c_str());
        return std::nullopt;
    }
    const auto& joint_names = jmg->getVariableNames();

    // ── 3. Start state ────────────────────────────────────────
    auto current_state = arm_group.getCurrentState(5.0);
    if (!current_state) {
        RCLCPP_ERROR(node->get_logger(), "[CRRT] getCurrentState() failed.");
        return std::nullopt;
    }
    JointVec q_start;
    current_state->copyJointGroupPositions(jmg, q_start);

    // ── 4. Snapshot planning scene ONCE ───────────────────────
    //  Done before any validity checks so no lock is held
    //  during the RRT hot loop.
    planning_scene::PlanningScenePtr scene_snapshot;
    {
        planning_scene_monitor::LockedPlanningSceneRO ls(psm_holder.psm);
        scene_snapshot = planning_scene::PlanningScene::clone(
            psm_holder.psm->getPlanningScene());
    }
    // ── 5. Shared robot state for all validity checks ─────────
    moveit::core::RobotState rs(robot_model);
    rs.setToDefaultValues();

    // ── 6. Validate goal ──────────────────────────────────────
    if (!is_valid(q_goal, rs, jmg, scene_snapshot)) {
        RCLCPP_ERROR(node->get_logger(),
            "[CRRT] Goal violates constraint or is in collision — aborting.");
        return std::nullopt;
    }

    // ── 7. Joint limits for random sampling ───────────────────
    std::vector<std::pair<double,double>> limits;
    for (const auto* j : jmg->getActiveJointModels()) {
        const auto& bnd = j->getVariableBounds()[0];
        limits.push_back({bnd.min_position_, bnd.max_position_});
    }

    // ── 8. RRT-Connect ────────────────────────────────────────
    RCLCPP_INFO(node->get_logger(),
        "[CRRT] RRT-Connect started (max %d iter / %.1f s)...",
        crrt_cfg::MAX_ITER, crrt_cfg::MAX_TIME_SEC);

    std::vector<RRTNode> tree_a = {{ q_start, -1 }};
    std::vector<RRTNode> tree_b = {{ q_goal,  -1 }};

    std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<double> unit(0.0, 1.0);

    int  connect_a_idx = -1;
    int  connect_b_idx = -1;
    bool connected     = false;

    for (int iter = 0; iter < crrt_cfg::MAX_ITER && !connected; ++iter)
    {
        // Timeout check every 100 iters
        if (iter % 100 == 0) {
            double elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t0).count();
            if (elapsed > crrt_cfg::MAX_TIME_SEC) {
                RCLCPP_WARN(node->get_logger(),
                    "[CRRT] Timeout at iter %d (%.1f s).", iter, elapsed);
                break;
            }
        }

        // Sample — goal bias toward tree_b root
        JointVec q_rand(limits.size());
        if (unit(rng) < crrt_cfg::GOAL_BIAS) {
            q_rand = tree_b[0].q;
        } else {
            for (size_t j = 0; j < limits.size(); ++j) {
                std::uniform_real_distribution<double> d(
                    limits[j].first, limits[j].second);
                q_rand[j] = d(rng);
            }
        }

        // 1. Extend Tree A greedily toward the random sample
        int new_a = extend_greedy_with_projection(tree_a, q_rand, rs, jmg, scene_snapshot);
        if (new_a < 0) continue;

        // 2. Get the specific joint configuration Tree A just reached
        JointVec q_reached_by_a = tree_a[new_a].q;

        // 3. Attempt to connect Tree B greedily to that EXACT point
        // No while(true) loop needed here; the greedy function handles the "tunneling"
        int new_b = extend_greedy_with_projection(tree_b, q_reached_by_a, rs, jmg, scene_snapshot);

        // 4. Check if they actually met
        // Replace the swap + connected block at the bottom of the RRT loop:
        if (new_b >= 0 && joint_dist(tree_b[new_b].q, q_reached_by_a) < crrt_cfg::CONNECT_TOL) {
            connect_a_idx = new_a;
            connect_b_idx = new_b;
            connected = true;
            break;  // don't swap after connecting
        }
        std::swap(tree_a, tree_b);
        // After swap, the indices are now in the swapped trees — track this:
        // std::swap(connect_a_idx, connect_b_idx);  // REMOVE THIS LINE
    }

    if (!connected) {
        RCLCPP_ERROR(node->get_logger(),
            "[CRRT] Failed to find a valid path.");
        return std::nullopt;
    }

    // ── 9. Extract and join paths from both trees ─────────────
    auto path_a = extract_path(tree_a, connect_a_idx);
    auto path_b = extract_path(tree_b, connect_b_idx);
    std::reverse(path_b.begin(), path_b.end()); // tree_b: goal→node, flip it

    // Identify which tree is the "true" start tree
    std::vector<JointVec> full_path;
    if (joint_dist(tree_a[0].q, q_start) < 0.001) {
        // tree_a is Start, tree_b is Goal
        auto path_start = extract_path(tree_a, connect_a_idx);
        auto path_goal  = extract_path(tree_b, connect_b_idx);
        std::reverse(path_goal.begin(), path_goal.end()); 
        full_path = path_start;
        full_path.insert(full_path.end(), path_goal.begin(), path_goal.end());
    } else {
        // tree_b is Start, tree_a is Goal
        auto path_start = extract_path(tree_b, connect_b_idx);
        auto path_goal  = extract_path(tree_a, connect_a_idx);
        std::reverse(path_goal.begin(), path_goal.end());
        full_path = path_start;
        full_path.insert(full_path.end(), path_goal.begin(), path_goal.end());
    }
    moveit::core::RobotState rs_check(robot_model);
    rs_check.setToDefaultValues();
    for (const auto& q : full_path) {
        if (!is_valid(q, rs_check, jmg, scene_snapshot)) {
            RCLCPP_ERROR(node->get_logger(),
                "[CRRT] Final path contains invalid/colliding waypoint — aborting.");
            return std::nullopt;
        }
    }

    RCLCPP_INFO(node->get_logger(),
        "[CRRT] Raw path: %zu waypoints. Smoothing...", full_path.size());

    // ── 10. Shortcut smoothing ────────────────────────────────
    // auto smoothed = smooth_path(full_path, rs, jmg, scene_snapshot);
    // RCLCPP_INFO(node->get_logger(),
    //     "[CRRT] Smoothed to %zu waypoints.", smoothed.size());
    // std::vector<JointVec> interpolated_path;
    // for (size_t i = 0; i < full_path.size() - 1; ++i) {
    //     interpolated_path.push_back(full_path[i]);
        
    //     // Fill the gap between waypoints
    //     double dist = joint_dist(full_path[i], full_path[i+1]);
    //     int steps = std::max(1, (int)(dist / crrt_cfg::STEP_SIZE));
        
    //     for (int s = 1; s < steps; ++s) {
    //         interpolated_path.push_back(steer(full_path[i], full_path[i+1], s * crrt_cfg::STEP_SIZE));
    //     }
    // }
    // interpolated_path.push_back(full_path.back());

    // Now pass 'interpolated_path' to build_plan instead of 'full_path'
    // ── 11. Build and return plan ─────────────────────────────
    auto plan = build_plan(
        full_path,
        std::vector<std::string>(joint_names.begin(), joint_names.end()),
        arm_group);

    double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t0).count();
    plan.planning_time_ = elapsed;

    RCLCPP_INFO(node->get_logger(), "[CRRT] Done in %.2f s.", elapsed);
    return plan;
}