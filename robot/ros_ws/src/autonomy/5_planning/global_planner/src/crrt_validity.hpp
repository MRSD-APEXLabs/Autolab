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
//  SAFETY MARGIN
//  While a crrt_clearance::Scope is alive, is_valid() also requires every link the arm moves to be
//  `margin` m clear of the world (the octomap), unless the end effector is in the approach column
//  above the start or the goal (crrt_cfg::APPROACH_*). There only the plain collision check applies,
//  so poses close to a surface stay reachable, entered and left from above.
//  is_valid() and the IRRT* steer also keep the bottom of the plate link above floor_z (crrt_cfg::PLATE_*),
//  approach column included.
// ─────────────────────────────────────────────────────────────
namespace crrt_clearance {

struct State {
    double margin = 0.0;                                       // 0: no margin
    std::vector<Eigen::Vector3d> anchors;                      // end-effector positions of the start and goal
    const planning_scene::PlanningScene* source = nullptr;     // the scene `padded` was made from
    planning_scene::PlanningScenePtr padded;                   // child of `source`, arm links padded
    const moveit::core::LinkModel* plate = nullptr;            // crrt_cfg::PLATE_LINK; null: no floor
    double floor_z = 0.0;                                      // m, world: lowest the plate's bottom may go
};

// Lowest world z of `link`'s collision geometry (its box in the link frame); rs's link transforms up to date.
static double bottom_z(const moveit::core::RobotState& rs, const moveit::core::LinkModel* link)
{
    const Eigen::Isometry3d& T = rs.getGlobalLinkTransform(link);
    const Eigen::Vector3d half = link->getShapeExtentsAtOrigin() / 2.0;
    return (T * link->getCenteredBoundingBoxOffset()).z() - T.rotation().row(2).cwiseAbs().dot(half);
}

// Per thread: the parallel search (crrt_parallel.hpp) gives each search thread a copy, padded scene made beforehand.
static State& state() { static thread_local State s; return s; }

// Make s.padded, the child of `scene` with every link the arm moves padded by s.margin.
static void pad(State& s, const moveit::core::JointModelGroup* jmg, const planning_scene::PlanningScenePtr& scene)
{
    s.padded = scene->diff();
    std::map<std::string, double> padding;
    for (const auto* link : jmg->getUpdatedLinkModelsWithGeometry())
        padding[link->getName()] = s.margin;
    s.padded->getCollisionEnvNonConst()->setLinkPadding(padding);
    s.source = scene.get();
}

struct Scope {
    Scope(double margin, std::vector<Eigen::Vector3d> anchors)
    {
        state() = State{margin, std::move(anchors), nullptr, nullptr};
    }
    explicit Scope(State s) { state() = std::move(s); }
    ~Scope() { state() = State{}; }
    Scope(const Scope&) = delete;
    Scope& operator=(const Scope&) = delete;
};

static bool in_approach_column(const Eigen::Vector3d& ee)
{
    for (const auto& a : state().anchors) {
        const double up = ee.z() - a.z();
        if (std::hypot(ee.x() - a.x(), ee.y() - a.y()) <= crrt_cfg::APPROACH_RADIUS_M &&
            up >= -crrt_cfg::APPROACH_BELOW_M && up <= crrt_cfg::APPROACH_HEIGHT_M)
            return true;
    }
    return false;
}

// Whether rs (link transforms up to date) has the plate below the floor.
static bool below_floor(const moveit::core::RobotState& rs)
{
    const auto& s = state();
    return s.plate && bottom_z(rs, s.plate) < s.floor_z;
}

// Whether rs (link transforms up to date) keeps the margin. A padded child of `scene` is made once per scene.
static bool clear(const moveit::core::RobotState& rs,
                  const moveit::core::JointModelGroup* jmg,
                  const planning_scene::PlanningScenePtr& scene)
{
    auto& s = state();
    if (s.margin <= 0.0 || in_approach_column(rs.getGlobalLinkTransform(crrt_cfg::EE_LINK).translation()))
        return true;
    if (!s.padded || s.source != scene.get())
        pad(s, jmg, scene);
    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult  res;
    req.group_name = crrt_cfg::GROUP_NAME;
    s.padded->getCollisionEnv()->checkRobotCollision(req, res, rs, scene->getAllowedCollisionMatrix());
    return !res.collision;
}

} // namespace crrt_clearance

// ─────────────────────────────────────────────────────────────
//  FULL VALIDITY CHECK  (constraint + collision [+ plate floor + safety margin])
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
    if (crrt_clearance::below_floor(rs)) return false;

    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult  res;
    req.group_name = crrt_cfg::GROUP_NAME;
    req.contacts   = false;
    scene->checkCollision(req, res, rs);

    return !res.collision && crrt_clearance::clear(rs, jmg, scene);
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
//  IK SOLVER PER THREAD
//  The group's KDL solver is not safe to share between threads: its random restarts write a member
//  RobotState. Each thread of the parallel search (crrt_parallel.hpp) sets its own solver here;
//  unset, it is the group's own solver, through RobotState::setFromIK.
// ─────────────────────────────────────────────────────────────
namespace crrt_ik {

static thread_local const kinematics::KinematicsBase* solver = nullptr;

// RobotState::setFromIK(jmg, pose, timeout) for one pose of the solver's tip, with this thread's solver.
static bool set_from_ik(moveit::core::RobotState& rs,
                        const moveit::core::JointModelGroup* jmg,
                        const geometry_msgs::msg::Pose& pose, double timeout)
{
    if (!solver) return rs.setFromIK(jmg, pose, timeout);

    // The pose is in the model frame; the solver wants its base frame.
    Eigen::Isometry3d target = Eigen::Translation3d(pose.position.x, pose.position.y, pose.position.z) *
        Eigen::Quaterniond(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    std::string base = solver->getBaseFrame();
    if (!base.empty() && base[0] == '/') base = base.substr(1);
    if (!moveit::core::Transforms::sameFrame(base, rs.getRobotModel()->getModelFrame())) {
        const auto* base_link = rs.getLinkModel(base);
        if (!base_link) return false;
        target = rs.getGlobalLinkTransform(base_link).inverse() * target;
    }
    if (timeout < std::numeric_limits<double>::epsilon()) timeout = jmg->getDefaultIKTimeout();

    const auto& bij = jmg->getKinematicsSolverJointBijection();
    std::vector<double> values;
    rs.copyJointGroupPositions(jmg, values);
    std::vector<double> seed(bij.size());
    for (size_t i = 0; i < bij.size(); ++i) seed[i] = values[bij[i]];

    geometry_msgs::msg::Pose query;
    query.position.x = target.translation().x();
    query.position.y = target.translation().y();
    query.position.z = target.translation().z();
    const Eigen::Quaterniond q(target.rotation());
    query.orientation.x = q.x();
    query.orientation.y = q.y();
    query.orientation.z = q.z();
    query.orientation.w = q.w();

    std::vector<double> ik_sol;
    moveit_msgs::msg::MoveItErrorCodes error;
    if (!solver->searchPositionIK(query, seed, timeout, std::vector<double>(), ik_sol, error))
        return false;
    std::vector<double> solution(bij.size());
    for (size_t i = 0; i < bij.size(); ++i) solution[bij[i]] = ik_sol[i];
    rs.setJointGroupPositions(jmg, solution);
    return true;
}

} // namespace crrt_ik

// ─────────────────────────────────────────────────────────────
//  IK PROJECTION HELPERS
// ─────────────────────────────────────────────────────────────

// Strict: projects EE to down orientation AND checks position didn't drift.
// Used during RRT extend steps.
static bool project_to_ee_down(moveit::core::RobotState& rs,
                                const moveit::core::JointModelGroup* jmg)
{
    const Eigen::Isometry3d ee_before = rs.getGlobalLinkTransform(crrt_cfg::EE_LINK);
    Eigen::Vector3d pos_before = ee_before.translation();

    geometry_msgs::msg::Pose pose;
    pose.position.x = pos_before.x();
    pose.position.y = pos_before.y();
    pose.position.z = pos_before.z();
    // Level roll and pitch but keep the step's own yaw: ee_down leaves the wrist free about z, and
    // forcing yaw = pi here kept every tree node at pi, so a goal at another yaw (plan_april_<id> /
    // plan_wellplate use APPROACH_YAW) was never within reach of the tree.
    const Eigen::Matrix3d R = ee_before.rotation();
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, std::atan2(R(1, 0), R(0, 0)));
    pose.orientation = tf2::toMsg(q);

    if (!crrt_ik::set_from_ik(rs, jmg, pose, 0.01)) return false;

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

    return crrt_ik::set_from_ik(rs, jmg, pose, 0.005);
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

    // 3x the previous 0.08 / 0.05: velocity x3 and acceleration x9 (= 3^2) make TOTG's trajectory exactly
    // 3x faster (1/3 of the duration, same path). Peak: 0.75 rad/s (43 deg/s) per arm joint, 0.45 rad/s^2.
    const double vel_scale   = 0.24;  // was 0.08 (before that 0.15)
    const double accel_scale = 0.45;  // was 0.05 (before that 0.10)

    if (!totg.computeTimeStamps(rt, vel_scale, accel_scale)) {
        RCLCPP_WARN(rclcpp::get_logger("build_plan"),
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