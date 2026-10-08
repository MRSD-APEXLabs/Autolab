// crrt_validity.hpp
// Constraint checking, collision checking, and RRT/PRM helper functions.
// Depends on: crrt_types.hpp

#pragma once
#include "crrt_types.hpp"

// ─────────────────────────────────────────────────────────────
//  ee_down CONSTRAINT CHECK
//  Cheap — always run before collision check.
// ─────────────────────────────────────────────────────────────
static bool satisfies_ee_down(moveit::core::RobotState& rs)
{
    rs.updateLinkTransforms();
    const Eigen::Matrix3d R =
        rs.getGlobalLinkTransform(crrt_cfg::EE_LINK).rotation();

    double roll  = std::atan2( R(2,1), R(2,2));
    double pitch = std::atan2(-R(2,0), std::sqrt(R(2,1)*R(2,1) + R(2,2)*R(2,2)));

    return (std::abs(roll)  < crrt_cfg::EE_ROLL_TOL &&
            std::abs(pitch) < crrt_cfg::EE_PITCH_TOL);
}

// ─────────────────────────────────────────────────────────────
//  FULL VALIDITY CHECK  (constraint + collision)
//  Uses a pre-snapshotted scene — no lock acquired here.
// ─────────────────────────────────────────────────────────────
static bool is_valid(const JointVec& q,
                     moveit::core::RobotState& rs,
                     const moveit::core::JointModelGroup* jmg,
                     const planning_scene::PlanningScenePtr& scene)
{
    rs.setJointGroupPositions(jmg, q);
    rs.updateLinkTransforms();

    if (!satisfies_ee_down(rs)) return false;

    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult  res;
    req.group_name = crrt_cfg::GROUP_NAME;
    req.contacts   = false;
    scene->checkCollision(req, res, rs);

    return !res.collision;
}

// ─────────────────────────────────────────────────────────────
//  COLLISION-ONLY CHECK  (no constraint)
//  Used for start state validation and edge checking.
// ─────────────────────────────────────────────────────────────
static bool is_collision_free(const JointVec& q,
                               moveit::core::RobotState& rs,
                               const moveit::core::JointModelGroup* jmg,
                               const planning_scene::PlanningScenePtr& scene)
{
    rs.setJointGroupPositions(jmg, q);
    rs.updateLinkTransforms();
    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult  res;
    req.group_name = crrt_cfg::GROUP_NAME;
    req.contacts   = false;
    scene->checkCollision(req, res, rs);
    return !res.collision;
}

// ─────────────────────────────────────────────────────────────
//  GEOMETRY HELPERS
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

static double halton(int index, int base)
{
    double result = 0.0, f = 1.0;
    while (index > 0) {
        f /= base;
        result += f * (index % base);
        index /= base;
    }
    return result;
}

// ─────────────────────────────────────────────────────────────
//  IK PROJECTION HELPERS
// ─────────────────────────────────────────────────────────────

// Strict: projects EE to down orientation AND checks position didn't drift.
// Used during RRT extend steps.
static bool project_to_ee_down(moveit::core::RobotState& rs,
                                const moveit::core::JointModelGroup* jmg)
{
    Eigen::Vector3d pos_before = rs.getGlobalLinkTransform(crrt_cfg::EE_LINK).translation();

    geometry_msgs::msg::Pose pose;
    pose.position.x = pos_before.x();
    pose.position.y = pos_before.y();
    pose.position.z = pos_before.z();
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, M_PI);
    pose.orientation = tf2::toMsg(q);

    if (!rs.setFromIK(jmg, pose, 0.01)) return false;

    rs.updateLinkTransforms();
    Eigen::Vector3d pos_after = rs.getGlobalLinkTransform(crrt_cfg::EE_LINK).translation();
    return (pos_after - pos_before).norm() < (crrt_cfg::STEP_SIZE * 0.5);
}

// Loose: projects EE orientation only, no drift check.
// Used during sampling where any valid IK solution is acceptable.
static bool project_to_ee_down_sample(moveit::core::RobotState& rs,
                                       const moveit::core::JointModelGroup* jmg)
{
    rs.updateLinkTransforms();
    const Eigen::Vector3d pos = rs.getGlobalLinkTransform(crrt_cfg::EE_LINK).translation();

    geometry_msgs::msg::Pose pose;
    pose.position.x = pos.x();
    pose.position.y = pos.y();
    pose.position.z = pos.z();
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, M_PI);
    pose.orientation = tf2::toMsg(q);

    return rs.setFromIK(jmg, pose, 0.005);
}

// ─────────────────────────────────────────────────────────────
//  RRT EXTEND  (greedy + ee_down projection)
//  Walks from nearest tree node toward q_target one step at a
//  time, projecting to ee_down and checking collision at each
//  step. Returns index of last added node, or -1 if blocked.
// ─────────────────────────────────────────────────────────────
static int extend_greedy_with_projection(std::vector<RRTNode>& tree,
                                          const JointVec& q_target,
                                          moveit::core::RobotState& rs,
                                          const moveit::core::JointModelGroup* jmg,
                                          const planning_scene::PlanningScenePtr& scene)
{
    int last_added_idx = -1;
    int ni = nearest(tree, q_target);
    JointVec q_curr = tree[ni].q;
    double d_curr = joint_dist(q_curr, q_target);

    // A straight march needs d/STEP_SIZE steps; allow slack for projection
    // jitter but never walk unbounded.
    const int max_steps = static_cast<int>(d_curr / crrt_cfg::STEP_SIZE) + 20;

    for (int step = 0; step < max_steps && d_curr > 1e-6; ++step)
    {
        JointVec q_steered = steer(q_curr, q_target, crrt_cfg::STEP_SIZE);
        rs.setJointGroupPositions(jmg, q_steered);

        if (!project_to_ee_down(rs, jmg)) break;

        JointVec q_projected;
        rs.copyJointGroupPositions(jmg, q_projected);

        // The ee_down projection only preserves end-effector position; in
        // joint space it can pull the sample back as far as steer() advanced
        // it. If the step made no net progress the manifold blocks this
        // direction, so stop instead of marching in place forever.
        double d_next = joint_dist(q_projected, q_target);
        if (d_next >= d_curr) break;

        collision_detection::CollisionRequest req;
        collision_detection::CollisionResult  res;
        req.group_name = crrt_cfg::GROUP_NAME;
        scene->checkCollision(req, res, rs);
        if (res.collision) break;

        tree.push_back({q_projected, (last_added_idx == -1) ? ni : last_added_idx});
        last_added_idx = (int)tree.size() - 1;
        q_curr = q_projected;
        d_curr = d_next;

        if (d_curr < 0.001) break;
    }

    return last_added_idx;
}

// ─────────────────────────────────────────────────────────────
//  PATH HELPERS
// ─────────────────────────────────────────────────────────────
static std::vector<JointVec> extract_path(const std::vector<RRTNode>& tree, int idx)
{
    std::vector<JointVec> path;
    while (idx >= 0) {
        path.push_back(tree[idx].q);
        idx = tree[idx].parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

static std::vector<JointVec> densify_path(
    const std::vector<JointVec>& path,
    double max_step = 0.01)
{
    if (path.size() < 2) return path;
    std::vector<JointVec> dense;
    dense.push_back(path[0]);

    for (size_t i = 0; i + 1 < path.size(); ++i) {
        double d = joint_dist(path[i], path[i+1]);
        int steps = std::max(1, (int)std::ceil(d / max_step));
        for (int s = 1; s <= steps; ++s) {
            double t = (double)s / steps;
            JointVec q(path[i].size());
            for (size_t j = 0; j < path[i].size(); ++j)
                q[j] = path[i][j] + t * (path[i+1][j] - path[i][j]);
            dense.push_back(q);
        }
    }
    return dense;
}

static std::vector<JointVec> smooth_path(
    const std::vector<JointVec>& raw,
    moveit::core::RobotState& rs,
    const moveit::core::JointModelGroup* jmg,
    const planning_scene::PlanningScenePtr& scene)
{
    if (raw.size() <= 2) return raw;
    std::vector<JointVec> path = raw;
    bool changed = true;
    int passes = 0;

    while (changed && passes++ < 10) {
        changed = false;
        for (size_t i = 0; i + 2 < path.size(); ) {
            bool     ok  = true;
            JointVec cur = path[i];

            // Walk from path[i] toward path[i+2] with projection at each step
            while (joint_dist(cur, path[i+2]) > crrt_cfg::STEP_SIZE) {
                JointVec q_steered = steer(cur, path[i+2], crrt_cfg::STEP_SIZE);

                rs.setJointGroupPositions(jmg, q_steered);

                // ← KEY: project onto constraint manifold before checking
                if (!project_to_ee_down(rs, jmg)) { ok = false; break; }

                JointVec q_proj;
                rs.copyJointGroupPositions(jmg, q_proj);

                // Collision check on projected state
                collision_detection::CollisionRequest req;
                collision_detection::CollisionResult  res;
                req.group_name = crrt_cfg::GROUP_NAME;
                req.contacts   = false;
                scene->checkCollision(req, res, rs);
                if (res.collision) { ok = false; break; }

                cur = q_proj;
            }

            // Also validate the endpoint itself
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
//  BUILD MOVEIT PLAN FROM WAYPOINT LIST
// ─────────────────────────────────────────────────────────────
static moveit::planning_interface::MoveGroupInterface::Plan
build_plan(const std::vector<JointVec>& waypoints,
           const std::vector<std::string>& joint_names,
           moveit::planning_interface::MoveGroupInterface& arm_group)
{
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto cs = arm_group.getCurrentState(5.0);
    moveit::core::robotStateToRobotStateMsg(*cs, plan.start_state_);

    // Build a RobotTrajectory with positions only, uniform dt placeholder
    auto robot_model = arm_group.getRobotModel();
    const auto* jmg  = robot_model->getJointModelGroup(crrt_cfg::GROUP_NAME);

    robot_trajectory::RobotTrajectory rt(robot_model, crrt_cfg::GROUP_NAME);

    moveit::core::RobotState rs(*cs);
    for (size_t i = 0; i < waypoints.size(); ++i) {
        rs.setJointGroupPositions(jmg, waypoints[i]);
        rs.update();
        // dt=0.1 is just a placeholder — TOTG will overwrite all timestamps
        rt.addSuffixWayPoint(rs, (i == 0) ? 0.0 : 0.1);
    }

    // Retime with TOTG — this is where smoothness actually comes from
    trajectory_processing::TimeOptimalTrajectoryGeneration totg(
        0.01,  // path tolerance (smaller = more faithful to waypoints)
        0.05   // resample_dt (output waypoint spacing in seconds)
    );

    const double vel_scale   = 0.08;  // was 0.15
    const double accel_scale = 0.05;  // was 0.10

    if (!totg.computeTimeStamps(rt, vel_scale, accel_scale)) {
        RCLCPP_ERROR(rclcpp::get_logger("build_plan"),
            "[build_plan] TOTG retiming failed — falling back to naive timing.");
        // fallback: build naive timed plan rather than returning empty
        trajectory_msgs::msg::JointTrajectory jt;
        jt.joint_names = joint_names;
        double t = 0.0;
        for (size_t i = 0; i < waypoints.size(); ++i) {
            trajectory_msgs::msg::JointTrajectoryPoint pt;
            pt.positions = waypoints[i];
            pt.velocities.resize(waypoints[i].size(), 0.0);
            pt.accelerations.resize(waypoints[i].size(), 0.0);
            if (i > 0) t += 0.05;
            pt.time_from_start = rclcpp::Duration::from_seconds(t);
            jt.points.push_back(pt);
        }
        moveit_msgs::msg::RobotTrajectory rt_msg;
        rt_msg.joint_trajectory = jt;
        plan.trajectory_ = rt_msg;
        return plan;
    }

    moveit_msgs::msg::RobotTrajectory rt_msg;
    rt.getRobotTrajectoryMsg(rt_msg);
    plan.trajectory_ = rt_msg;
    return plan;
}