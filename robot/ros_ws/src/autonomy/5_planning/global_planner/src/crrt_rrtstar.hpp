// crrt_rrtstar.hpp
// Informed RRT* on the ee_down constraint manifold. Same pre-checks as
// crrt_plan (scene snapshot, start collision-only, goal full validity),
// same projection step when growing the tree, same midpoint fallback when
// no solution is found, and the same post-processing via crrt_finish_path.
// Anytime: after the first solution it keeps sampling inside the informed
// ellipsoid and rewiring until RRTSTAR_TIME_SEC, then returns the best path.
// Depends on: crrt_plan.hpp

#pragma once
#include "crrt_plan.hpp"
#include <Eigen/SVD>

struct RRTStarNode {
    JointVec         q;
    int              parent = -1;
    double           cost   = 0.0;   // path length from the root (joint_dist)
    std::vector<int> children;
};

// ─────────────────────────────────────────────────────────────
//  INFORMED SAMPLER — uniform inside the prolate hyperspheroid with foci
//  q_start / q_goal and transverse diameter c_best (Gammell et al. 2014).
//  Falls back to uniform-in-limits until a first solution exists.
// ─────────────────────────────────────────────────────────────
struct InformedSampler {
    int             n;
    double          c_min;
    Eigen::VectorXd center;
    Eigen::MatrixXd C;   // rotation taking e1 onto the start→goal axis

    InformedSampler(const JointVec& s, const JointVec& g)
    {
        n = (int)s.size();
        Eigen::VectorXd a(n), b(n);
        for (int i = 0; i < n; ++i) { a[i] = s[i]; b[i] = g[i]; }
        center = 0.5 * (a + b);
        c_min  = (b - a).norm();
        C = Eigen::MatrixXd::Identity(n, n);
        if (c_min > 1e-9) {
            Eigen::VectorXd a1 = (b - a) / c_min;
            Eigen::VectorXd e1 = Eigen::VectorXd::Zero(n); e1[0] = 1.0;
            Eigen::JacobiSVD<Eigen::MatrixXd> svd(a1 * e1.transpose(),
                                                  Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::VectorXd d = Eigen::VectorXd::Ones(n);
            d[n - 1] = svd.matrixU().determinant() * svd.matrixV().determinant();
            C = svd.matrixU() * d.asDiagonal() * svd.matrixV().transpose();
        }
    }

    JointVec sample(double c_best, std::mt19937& rng,
                    const std::vector<std::pair<double,double>>& limits)
    {
        std::uniform_real_distribution<double> unit(0.0, 1.0);
        if (!std::isfinite(c_best)) {
            JointVec q(n);
            for (int j = 0; j < n; ++j)
                q[j] = std::uniform_real_distribution<double>(limits[j].first, limits[j].second)(rng);
            return q;
        }
        std::normal_distribution<double> gauss(0.0, 1.0);
        Eigen::VectorXd r(n);
        r[0] = 0.5 * c_best;
        const double r_conj = 0.5 * std::sqrt(std::max(0.0, c_best * c_best - c_min * c_min));
        for (int i = 1; i < n; ++i) r[i] = r_conj;

        // Rejection-sample against joint limits; the ellipsoid can poke outside.
        for (int attempt = 0; attempt < 20; ++attempt) {
            Eigen::VectorXd x(n);
            for (int i = 0; i < n; ++i) x[i] = gauss(rng);
            x.normalize();
            x *= std::pow(unit(rng), 1.0 / n);          // uniform in the unit ball
            Eigen::VectorXd q = C * (r.asDiagonal() * x) + center;
            bool ok = true;
            for (int j = 0; j < n; ++j)
                if (q[j] < limits[j].first || q[j] > limits[j].second) { ok = false; break; }
            if (ok) return JointVec(q.data(), q.data() + n);
        }
        JointVec q(n);
        for (int j = 0; j < n; ++j)
            q[j] = std::uniform_real_distribution<double>(limits[j].first, limits[j].second)(rng);
        return q;
    }
};

// ─────────────────────────────────────────────────────────────
//  HELPERS
// ─────────────────────────────────────────────────────────────

// Walk from q_from toward q_to, at most eta in joint space, projecting each
// STEP_SIZE step onto ee_down and collision-checking it — the same march as
// extend_greedy_with_projection, but returns one configuration instead of
// appending to a tree.
static std::optional<JointVec> constrained_steer(
    const JointVec& q_from, const JointVec& q_to, double eta,
    moveit::core::RobotState& rs,
    const moveit::core::JointModelGroup* jmg,
    const planning_scene::PlanningScenePtr& scene)
{
    JointVec q_curr = q_from;
    double   d_curr = joint_dist(q_curr, q_to);
    bool     moved  = false;
    const int max_steps = static_cast<int>(std::min(eta, d_curr) / crrt_cfg::STEP_SIZE) + 20;

    for (int step = 0; step < max_steps && d_curr > 1e-6; ++step) {
        if (joint_dist(q_from, q_curr) >= eta) break;

        JointVec q_steered = steer(q_curr, q_to, crrt_cfg::STEP_SIZE);
        rs.setJointGroupPositions(jmg, q_steered);
        if (!project_to_ee_down(rs, jmg)) break;

        JointVec q_proj;
        rs.copyJointGroupPositions(jmg, q_proj);
        double d_next = joint_dist(q_proj, q_to);
        if (d_next >= d_curr) break;                       // manifold blocks this direction

        collision_detection::CollisionRequest req;
        collision_detection::CollisionResult  res;
        req.group_name = crrt_cfg::GROUP_NAME;
        scene->checkCollision(req, res, rs);
        if (res.collision) break;

        q_curr = q_proj;
        d_curr = d_next;
        moved  = true;
        if (d_curr < 0.001) break;
    }
    return moved ? std::optional<JointVec>(q_curr) : std::nullopt;
}

// Straight joint-space segment valid at STEP_SIZE resolution — the same
// criterion shortcut() uses, so tree edges are exactly what gets executed.
static bool edge_valid(const JointVec& a, const JointVec& b,
                       moveit::core::RobotState& rs,
                       const moveit::core::JointModelGroup* jmg,
                       const planning_scene::PlanningScenePtr& scene)
{
    JointVec cur = a;
    while (joint_dist(cur, b) > crrt_cfg::STEP_SIZE) {
        cur = steer(cur, b, crrt_cfg::STEP_SIZE);
        if (!is_valid(cur, rs, jmg, scene)) return false;
    }
    return true;
}

static int nearest_star(const std::vector<RRTStarNode>& tree, const JointVec& q)
{
    int best = 0; double best_d = std::numeric_limits<double>::max();
    for (int i = 0; i < (int)tree.size(); ++i) {
        double d = joint_dist(tree[i].q, q);
        if (d < best_d) { best_d = d; best = i; }
    }
    return best;
}

// After a rewire, every descendant's cost shifts by the same delta.
static void propagate_cost(std::vector<RRTStarNode>& tree, int root, double delta)
{
    std::vector<int> stack = {root};
    while (!stack.empty()) {
        int i = stack.back(); stack.pop_back();
        tree[i].cost += delta;
        for (int c : tree[i].children) stack.push_back(c);
    }
}

static void reparent(std::vector<RRTStarNode>& tree, int child, int new_parent, double new_cost)
{
    int old = tree[child].parent;
    if (old >= 0) {
        auto& ch = tree[old].children;
        ch.erase(std::remove(ch.begin(), ch.end(), child), ch.end());
    }
    const double delta = new_cost - tree[child].cost;
    tree[child].parent = new_parent;
    tree[new_parent].children.push_back(child);
    propagate_cost(tree, child, delta);
}

// ─────────────────────────────────────────────────────────────
//  INFORMED RRT*
// ─────────────────────────────────────────────────────────────
std::optional<moveit::planning_interface::MoveGroupInterface::Plan>
irrtstar_plan(
    moveit::planning_interface::MoveGroupInterface& arm_group,
    rclcpp::Node::SharedPtr node,
    const std::vector<double>& q_goal)
{
    auto t0 = std::chrono::steady_clock::now();

    auto robot_model = arm_group.getRobotModel();
    const auto* jmg  = robot_model->getJointModelGroup(crrt_cfg::GROUP_NAME);
    if (!jmg) {
        RCLCPP_ERROR(node->get_logger(),
            "[IRRT*] JMG '%s' not found.", crrt_cfg::GROUP_NAME.c_str());
        return std::nullopt;
    }
    const auto& joint_names = jmg->getVariableNames();

    auto current_state = arm_group.getCurrentState(5.0);
    if (!current_state) {
        RCLCPP_ERROR(node->get_logger(), "[IRRT*] getCurrentState() failed.");
        return std::nullopt;
    }
    JointVec q_start;
    current_state->copyJointGroupPositions(jmg, q_start);

    // One frozen, padded world for the whole planning command.
    crrt_internal::ScopedSceneFreeze scene_freeze(node);
    planning_scene::PlanningScenePtr scene_snapshot =
        crrt_internal::snapshot_scene(node);

    moveit::core::RobotState rs(robot_model);
    rs.setToDefaultValues();

    // Start: collision only — arm may not satisfy ee_down at rest
    if (!is_collision_free(q_start, rs, jmg, scene_snapshot)) {
        RCLCPP_ERROR(node->get_logger(), "[IRRT*] Start state is in collision.");
        return std::nullopt;
    }
    // Goal: full validation including ee_down
    if (!is_valid(q_goal, rs, jmg, scene_snapshot)) {
        RCLCPP_ERROR(node->get_logger(),
            "[IRRT*] Goal violates constraint or is in collision — aborting.");
        return std::nullopt;
    }

    std::vector<std::pair<double,double>> limits;
    for (const auto* j : jmg->getActiveJointModels()) {
        const auto& bnd = j->getVariableBounds()[0];
        limits.push_back({bnd.min_position_, bnd.max_position_});
    }
    const int dof = (int)limits.size();

    auto& viz = crrt_viz::get(node, robot_model, jmg);
    viz.begin();

    RCLCPP_INFO(node->get_logger(),
        "[IRRT*] Informed RRT* started (max %d iter / %.1f s, eta %.2f)...",
        crrt_cfg::RRTSTAR_MAX_ITER, crrt_cfg::RRTSTAR_TIME_SEC, crrt_cfg::RRTSTAR_ETA);

    std::vector<RRTStarNode> tree;
    tree.push_back({q_start, -1, 0.0, {}});

    std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    InformedSampler sampler(q_start, q_goal);

    int    goal_idx = -1;
    double c_best   = std::numeric_limits<double>::infinity();
    int    iter     = 0;

    for (; iter < crrt_cfg::RRTSTAR_MAX_ITER; ++iter)
    {
        if (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count()
            > crrt_cfg::RRTSTAR_TIME_SEC)
            break;

        // ── sample ──
        JointVec q_rand = (goal_idx < 0 && unit(rng) < crrt_cfg::GOAL_BIAS)
                        ? q_goal
                        : sampler.sample(c_best, rng, limits);

        // ── steer (projected) ──
        const int ni = nearest_star(tree, q_rand);
        auto q_new_opt = constrained_steer(tree[ni].q, q_rand, crrt_cfg::RRTSTAR_ETA,
                                           rs, jmg, scene_snapshot);
        if (!q_new_opt) continue;
        const JointVec& q_new = *q_new_opt;
        if (!edge_valid(tree[ni].q, q_new, rs, jmg, scene_snapshot)) continue;

        // ── near set ──
        const int    n = (int)tree.size();
        const double r = std::min(crrt_cfg::RRTSTAR_ETA,
                                  crrt_cfg::RRTSTAR_GAMMA * std::pow(std::log(n + 1.0) / (n + 1.0), 1.0 / dof));
        std::vector<int> near;
        for (int j = 0; j < n; ++j)
            if (joint_dist(tree[j].q, q_new) <= r) near.push_back(j);

        // ── choose parent ──
        int    best_parent = ni;
        double best_cost   = tree[ni].cost + joint_dist(tree[ni].q, q_new);
        for (int j : near) {
            if (j == ni) continue;
            double c = tree[j].cost + joint_dist(tree[j].q, q_new);
            if (c < best_cost && edge_valid(tree[j].q, q_new, rs, jmg, scene_snapshot)) {
                best_cost = c; best_parent = j;
            }
        }
        const int new_idx = n;
        tree.push_back({q_new, best_parent, best_cost, {}});
        tree[best_parent].children.push_back(new_idx);

        // ── rewire ──
        for (int j : near) {
            if (j == best_parent) continue;
            double c = best_cost + joint_dist(q_new, tree[j].q);
            if (c < tree[j].cost && edge_valid(q_new, tree[j].q, rs, jmg, scene_snapshot))
                reparent(tree, j, new_idx, c);
        }

        // ── try the goal ──
        const double dg = joint_dist(q_new, q_goal);
        if (dg <= crrt_cfg::RRTSTAR_ETA && edge_valid(q_new, q_goal, rs, jmg, scene_snapshot)) {
            const double cg = best_cost + dg;
            if (goal_idx < 0) {
                goal_idx = (int)tree.size();
                tree.push_back({q_goal, new_idx, cg, {}});
                tree[new_idx].children.push_back(goal_idx);
                RCLCPP_INFO(node->get_logger(),
                    "[IRRT*] First solution at iter %d: cost %.3f (%.2f s) — refining...",
                    iter, cg,
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
            } else if (cg < tree[goal_idx].cost) {
                reparent(tree, goal_idx, new_idx, cg);
            }
        }
        if (goal_idx >= 0 && tree[goal_idx].cost < c_best - 1e-9) {
            c_best = tree[goal_idx].cost;
            RCLCPP_INFO(node->get_logger(), "[IRRT*] Improved: cost %.3f at iter %d", c_best, iter);
        }
    }

    if (goal_idx < 0) {
        RCLCPP_WARN(node->get_logger(),
            "[IRRT*] No solution after %d iters — trying midpoint fallback...", iter);
        static const JointVec q_mid = {0.0611, -0.0977, -0.2164, -0.0436, 0.3159, 0.1012};

        RCLCPP_INFO(node->get_logger(), "[IRRT*] Stage 1: start -> midpoint...");
        auto plan1 = crrt_plan_from_to(arm_group, node, q_start, q_mid);
        if (!plan1) {
            RCLCPP_ERROR(node->get_logger(), "[IRRT*] Midpoint Stage 1 failed.");
            return std::nullopt;
        }
        RCLCPP_INFO(node->get_logger(), "[IRRT*] Stage 2: midpoint -> goal...");
        auto plan2 = crrt_plan_from_to(arm_group, node, q_mid, q_goal);
        if (!plan2) {
            RCLCPP_ERROR(node->get_logger(), "[IRRT*] Midpoint Stage 2 failed.");
            return std::nullopt;
        }
        auto& traj1 = plan1->trajectory_.joint_trajectory;
        auto& traj2 = plan2->trajectory_.joint_trajectory;
        double t_offset =
            traj1.points.back().time_from_start.sec +
            traj1.points.back().time_from_start.nanosec * 1e-9;
        for (auto& pt : traj2.points) {
            double t_pt = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9;
            pt.time_from_start = rclcpp::Duration::from_seconds(t_offset + t_pt);
            traj1.points.push_back(pt);
        }
        plan1->planning_time_ = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();
        return plan1;
    }

    RCLCPP_INFO(node->get_logger(),
        "[IRRT*] Search done: %d iters, %zu nodes, best cost %.3f (direct %.3f).",
        iter, tree.size(), tree[goal_idx].cost, sampler.c_min);

    std::vector<JointVec> full_path;
    for (int i = goal_idx; i >= 0; i = tree[i].parent) full_path.push_back(tree[i].q);
    std::reverse(full_path.begin(), full_path.end());

    return crrt_finish_path(std::move(full_path), q_start, q_goal, arm_group, node,
                            robot_model, jmg, joint_names, scene_snapshot, viz, t0);
}
