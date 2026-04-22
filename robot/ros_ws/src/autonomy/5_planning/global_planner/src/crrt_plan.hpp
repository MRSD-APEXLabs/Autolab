// crrt_plan.hpp
// Public planning API: prm_plan (PRM + Dijkstra) and crrt_plan (RRT-Connect).
// Depends on: crrt_types.hpp, crrt_validity.hpp, crrt_prm.hpp
//
// USAGE in move_to_pose_node.cpp:
//   #include "crrt_plan.hpp"   // pulls in all crrt_*.hpp transitively
//
//   // PRM (primary):
//   auto opt = prm_plan(arm_group, node, joint_values);
//
//   // RRT-Connect (fallback):
//   auto opt = crrt_plan(arm_group, node, joint_values);

#pragma once
#include "crrt_prm.hpp"
static const JointVec best_mid = {0.0611, -0.0977, -0.2164, -0.0436, 0.3159, 0.1012};
static std::optional<moveit::planning_interface::MoveGroupInterface::Plan>
prm_plan_from_to(
    moveit::planning_interface::MoveGroupInterface& arm_group,
    rclcpp::Node::SharedPtr node,
    const JointVec& q_from,
    const JointVec& q_to)
{
    auto& psm_holder = crrt_internal::get_psm(node);
    auto robot_model = arm_group.getRobotModel();
    const auto* jmg  = robot_model->getJointModelGroup(crrt_cfg::GROUP_NAME);
    if (!jmg) return std::nullopt;

    const auto& joint_names = jmg->getVariableNames();

    planning_scene::PlanningScenePtr scene_snapshot;
    {
        planning_scene_monitor::LockedPlanningSceneRO ls(psm_holder.psm);
        scene_snapshot = planning_scene::PlanningScene::clone(
            psm_holder.psm->getPlanningScene());
    }

    moveit::core::RobotState rs(robot_model);
    rs.setToDefaultValues();

    if (!is_collision_free(q_from, rs, jmg, scene_snapshot)) {
        RCLCPP_ERROR(node->get_logger(), "[PRM] from state is in collision.");
        return std::nullopt;
    }
    if (!is_valid(q_to, rs, jmg, scene_snapshot)) {
        RCLCPP_ERROR(node->get_logger(), "[PRM] to state is invalid.");
        return std::nullopt;
    }

    std::vector<PRMNode> roadmap = load_prm_roadmap(node, joint_names.size());
    if (roadmap.empty()) return std::nullopt;

    const int N        = (int)roadmap.size();
    const int START_ID = N;
    const int GOAL_ID  = N + 1;
    roadmap.push_back({q_from, {}});
    roadmap.push_back({q_to,   {}});

    const int K_HOOK = 50;
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
                if (!is_valid(cur, rs, jmg, scene_snapshot)) { edge_ok = false; break; }
            }
            if (edge_ok) {
                roadmap[hook_id].neighbors.push_back(i);
                roadmap[i].neighbors.push_back(hook_id);
            }
        }
    }

    if (roadmap[START_ID].neighbors.empty() || roadmap[GOAL_ID].neighbors.empty())
        return std::nullopt;

    const int total = (int)roadmap.size();
    std::vector<double> dist(total, std::numeric_limits<double>::max());
    std::vector<int>    prev(total, -1);
    using PQPair = std::pair<double,int>;
    std::priority_queue<PQPair, std::vector<PQPair>, std::greater<PQPair>> pq;
    dist[START_ID] = 0.0;
    pq.push({0.0, START_ID});
    while (!pq.empty()) {
        auto [d, u] = pq.top(); pq.pop();
        if (d > dist[u]) continue;
        if (u == GOAL_ID) break;
        for (int v : roadmap[u].neighbors) {
            double nd = dist[u] + joint_dist(roadmap[u].q, roadmap[v].q);
            if (nd < dist[v]) { dist[v] = nd; prev[v] = u; pq.push({nd, v}); }
        }
    }

    if (dist[GOAL_ID] == std::numeric_limits<double>::max()) return std::nullopt;

    std::vector<JointVec> full_path;
    for (int cur = GOAL_ID; cur != -1; cur = prev[cur])
        full_path.push_back(roadmap[cur].q);

    // Normalize joint wrapping across the path
    for (size_t i = 1; i < full_path.size(); ++i) {
        for (size_t j = 0; j < full_path[i].size(); ++j) {
            double diff = full_path[i][j] - full_path[i-1][j];
            while (diff >  M_PI) diff -= 2*M_PI;
            while (diff < -M_PI) diff += 2*M_PI;
            full_path[i][j] = full_path[i-1][j] + diff;
        }
    }
    std::reverse(full_path.begin(), full_path.end());
    full_path = densify_path(full_path, 0.01);

    return build_plan(
        full_path,
        std::vector<std::string>(joint_names.begin(), joint_names.end()),
        arm_group);
}


// ─────────────────────────────────────────────────────────────
//  PRM QUERY
//  1. Validates start (collision only) and goal (full)
//  2. Loads roadmap, builds if missing or force_rebuild=true
//  3. Hooks start/goal into roadmap as virtual nodes
//  4. Runs Dijkstra
//  5. Validates final path and returns a MoveIt plan
// ─────────────────────────────────────────────────────────────
// Helper lambda you can reuse anywhere
// Add this near the top of crrt_plan.hpp, alongside prm_plan_from_to

static void shortcut(
    std::vector<JointVec>& path,
    moveit::core::RobotState& rs_sc,
    const moveit::core::JointModelGroup* jmg,
    const planning_scene::PlanningScenePtr& scene_snapshot,
    int iters = 200)
{
    std::mt19937 rng_sc(42);
    for (int i = 0; i < iters; ++i) {
        if (path.size() < 3) break;
        std::uniform_int_distribution<int> d(0, path.size() - 1);
        int a = d(rng_sc), b = d(rng_sc);
        if (a > b) std::swap(a, b);
        if (b - a < 2) continue;
        bool ok = true;
        JointVec cur = path[a];
        while (joint_dist(cur, path[b]) > crrt_cfg::STEP_SIZE) {
            cur = steer(cur, path[b], crrt_cfg::STEP_SIZE);
            if (!is_valid(cur, rs_sc, jmg, scene_snapshot)) { ok = false; break; }
        }
        if (ok) path.erase(path.begin() + a + 1, path.begin() + b);
    }
}

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

    auto current_state = arm_group.getCurrentState(5.0);
    if (!current_state) return std::nullopt;
    JointVec q_start;
    current_state->copyJointGroupPositions(jmg, q_start);

    // Start: collision only — arm may not satisfy ee_down at rest
    if (!is_collision_free(q_start, rs, jmg, scene_snapshot)) {
        RCLCPP_ERROR(node->get_logger(), "[PRM] Start state is in collision.");
        return std::nullopt;
    }
    // Goal: full validation including ee_down
    if (!is_valid(q_goal, rs, jmg, scene_snapshot)) {
        RCLCPP_ERROR(node->get_logger(),
            "[PRM] Goal state is invalid (collision or ee_down violated).");
        return std::nullopt;
    }

    if (force_rebuild) {
        RCLCPP_INFO(node->get_logger(), "[PRM] Force rebuilding roadmap...");
        build_prm_roadmap(arm_group, node);
    }

    std::vector<PRMNode> roadmap = load_prm_roadmap(node, dof);
    if (roadmap.empty()) {
        RCLCPP_INFO(node->get_logger(), "[PRM] No roadmap found — building now...");
        build_prm_roadmap(arm_group, node);
        roadmap = load_prm_roadmap(node, dof);
        if (roadmap.empty()) {
            RCLCPP_ERROR(node->get_logger(), "[PRM] Roadmap build failed.");
            return std::nullopt;
        }
    }

    // Hook start and goal into roadmap as virtual nodes
    const int N        = (int)roadmap.size();
    const int START_ID = N;
    const int GOAL_ID  = N + 1;
    roadmap.push_back({q_start, {}});
    roadmap.push_back({q_goal,  {}});

    const int K_HOOK = 50;

    for (int hook_id : {START_ID, GOAL_ID}) {
        std::vector<std::pair<double,int>> dists;
        for (int i = 0; i < N; ++i)
            dists.push_back({joint_dist(roadmap[hook_id].q, roadmap[i].q), i});
        std::sort(dists.begin(), dists.end());

        // Start uses collision-only edges; goal uses full validity edges
        bool is_start = (hook_id == START_ID);

        for (auto& [d, i] : dists) {
            if ((int)roadmap[hook_id].neighbors.size() >= K_HOOK) break;

            bool edge_ok = true;
            JointVec cur = roadmap[hook_id].q;
            while (joint_dist(cur, roadmap[i].q) > crrt_cfg::STEP_SIZE) {
                cur = steer(cur, roadmap[i].q, crrt_cfg::STEP_SIZE);
                bool ok = is_collision_free(cur, rs, jmg, scene_snapshot);
                if (!ok) { edge_ok = false; break; }
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

    // Dijkstra
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
        RCLCPP_WARN(node->get_logger(),
            "[PRM] Dijkstra found no direct path — trying hardcoded midpoint fallback...");



        RCLCPP_INFO(node->get_logger(), "[PRM] Stage 1: start -> midpoint...");
        auto plan1 = prm_plan_from_to(arm_group, node, q_start, best_mid);
        if (!plan1) {
            RCLCPP_ERROR(node->get_logger(), "[PRM] Stage 1 failed.");
            return std::nullopt;
        }

        RCLCPP_INFO(node->get_logger(), "[PRM] Stage 2: midpoint -> goal...");
        auto plan2 = prm_plan_from_to(arm_group, node, best_mid, q_goal);
        if (!plan2) {
            RCLCPP_ERROR(node->get_logger(), "[PRM] Stage 2 failed.");
            return std::nullopt;
        }

        // Stitch trajectories
        // Smooth and restitch
        std::vector<JointVec> stitched_wps;
        for (const auto& pt : plan1->trajectory_.joint_trajectory.points)
            stitched_wps.push_back(pt.positions);
        for (const auto& pt : plan2->trajectory_.joint_trajectory.points)
            stitched_wps.push_back(pt.positions);

        // moveit::core::RobotState rs_smooth(robot_model);
        // rs_smooth.setToDefaultValues();
        // stitched_wps = smooth_path(stitched_wps, rs_smooth, jmg, scene_snapshot);
        // stitched_wps = densify_path(stitched_wps, 0.01);

        auto stitched_plan = build_plan(
            stitched_wps,
            std::vector<std::string>(joint_names.begin(), joint_names.end()),
            arm_group);
        stitched_plan.planning_time_ = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();
        return stitched_plan;
    }
    

    std::vector<JointVec> full_path;
    for (int cur = GOAL_ID; cur != -1; cur = prev[cur])
        full_path.push_back(roadmap[cur].q);
    std::reverse(full_path.begin(), full_path.end());

    // Normalize joint wrapping across the path
    for (size_t i = 1; i < full_path.size(); ++i) {
        for (size_t j = 0; j < full_path[i].size(); ++j) {
            double diff = full_path[i][j] - full_path[i-1][j];
            while (diff >  M_PI) diff -= 2*M_PI;
            while (diff < -M_PI) diff += 2*M_PI;
            full_path[i][j] = full_path[i-1][j] + diff;
        }
    }


    // moveit::core::RobotState rs_smooth(robot_model);
    // rs_smooth.setToDefaultValues();
    // full_path = smooth_path(full_path, rs_smooth, jmg, scene_snapshot);
    moveit::core::RobotState rs_check(robot_model);
    rs_check.setToDefaultValues();
    std::mt19937 rng(42);
    for (int i = 0; i < 200; ++i) {
        if (full_path.size() < 3) break;
        std::uniform_int_distribution<int> dist(0, full_path.size()-1);
        int a = dist(rng), b = dist(rng);
        if (a > b) std::swap(a, b);
        if (b - a < 2) continue;
        
        bool ok = true;
        JointVec cur = full_path[a];
        while (joint_dist(cur, full_path[b]) > crrt_cfg::STEP_SIZE) {
            cur = steer(cur, full_path[b], crrt_cfg::STEP_SIZE);
            if (!is_valid(cur, rs_check, jmg, scene_snapshot)) { ok = false; break; }
        }
        if (ok) full_path.erase(full_path.begin()+a+1, full_path.begin()+b);
    }


    
    int wp_idx = 0;
    for (const auto& q : full_path) {
        rs_check.setJointGroupPositions(jmg, q);
        rs_check.updateLinkTransforms();

        // Check constraint separately
        if (!satisfies_ee_down(rs_check)) {
            RCLCPP_ERROR(node->get_logger(),
                "[PRM] Waypoint %d FAILED ee_down constraint.", wp_idx);
            return std::nullopt;
        }

        // Check collision with contact info
        collision_detection::CollisionRequest req;
        collision_detection::CollisionResult  res;
        req.group_name = crrt_cfg::GROUP_NAME;
        req.contacts   = true;
        req.max_contacts = 5;
        scene_snapshot->checkCollision(req, res, rs_check);
        if (res.collision) {
            std::string pairs;
            bool has_octomap = false;
            for (const auto& [key, _] : res.contacts) {
                pairs += key.first + " <-> " + key.second + "  ";
                if (key.first.find("octomap") != std::string::npos ||
                    key.second.find("octomap") != std::string::npos)
                    has_octomap = true;
            }
            RCLCPP_ERROR(node->get_logger(),
                "[PRM] Waypoint %d FAILED collision  octomap=%s  pairs: %s",
                wp_idx, has_octomap ? "YES" : "NO", pairs.c_str());
            RCLCPP_ERROR(node->get_logger(),
                    "[PRM] Waypoint %d angles: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
                    wp_idx,
                    q[0], q[1], q[2], q[3], q[4], q[5]);
            RCLCPP_WARN(node->get_logger(),
                    "[PRM] Path validation failed — trying midpoint fallback...");
                auto plan1 = prm_plan_from_to(arm_group, node, q_start, best_mid);
                if (!plan1) { RCLCPP_ERROR(node->get_logger(), "[PRM] Midpoint Stage 1 failed."); return std::nullopt; }
                auto plan2 = prm_plan_from_to(arm_group, node, best_mid, q_goal);
                if (!plan2) { RCLCPP_ERROR(node->get_logger(), "[PRM] Midpoint Stage 2 failed."); return std::nullopt; }
                std::vector<JointVec> stitched;
                for (const auto& pt : plan1->trajectory_.joint_trajectory.points)
                    stitched.push_back(pt.positions);
                for (const auto& pt : plan2->trajectory_.joint_trajectory.points)
                    stitched.push_back(pt.positions);
                moveit::core::RobotState rs_short(robot_model);
                rs_short.setToDefaultValues();
                std::mt19937 rng_short(42);
                for (int i = 0; i < 200; ++i) {
                    if (stitched.size() < 3) break;
                    std::uniform_int_distribution<int> dist_s(0, stitched.size()-1);
                    int a = dist_s(rng_short), b = dist_s(rng_short);
                    if (a > b) std::swap(a, b);
                    if (b - a < 2) continue;
                    bool ok = true;
                    JointVec cur = stitched[a];
                    while (joint_dist(cur, stitched[b]) > crrt_cfg::STEP_SIZE) {
                        cur = steer(cur, stitched[b], crrt_cfg::STEP_SIZE);
                        if (!is_valid(cur, rs_short, jmg, scene_snapshot)) { ok = false; break; }
                    }
                    if (ok) stitched.erase(stitched.begin()+a+1, stitched.begin()+b);
                }
                return build_plan(
                    stitched,
                    std::vector<std::string>(joint_names.begin(), joint_names.end()),
                    arm_group);
        }
        ++wp_idx;
    }

    double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t0).count();
    RCLCPP_INFO(node->get_logger(),
        "[PRM] Path found: %zu waypoints in %.2f s.", full_path.size(), elapsed);
    // moveit::core::RobotState rs_smooth(robot_model);
    // rs_smooth.setToDefaultValues();
    // full_path = smooth_path(full_path, rs_smooth, jmg, scene_snapshot);
    // RCLCPP_INFO(node->get_logger(), "[CRRT] Smoothed path: %zu waypoints.", full_path.size());
    // full_path = densify_path(full_path, 0.01);
    // After building full_path, before build_plan
    
    double path_length = 0.0;
    for (size_t i = 1; i < full_path.size(); ++i)
        path_length += joint_dist(full_path[i], full_path[i-1]);

    double direct_dist = joint_dist(q_start, q_goal);
    double ratio = path_length / (direct_dist + 1e-6);

    RCLCPP_INFO(node->get_logger(),
        "[PRM] Path length: %.3f  Direct: %.3f  Ratio: %.2f",
        path_length, direct_dist, ratio);

    if (ratio > 3.0) {  // tune this threshold
        RCLCPP_WARN(node->get_logger(),
            "[PRM] Path looks suspicious (ratio %.2f) — trying midpoint fallback...");
        
        auto plan1 = prm_plan_from_to(arm_group, node, q_start, best_mid);
        if (!plan1) return std::nullopt;
        auto plan2 = prm_plan_from_to(arm_group, node, best_mid, q_goal);
        if (!plan2) return std::nullopt;

        // Extract segment waypoints
        std::vector<JointVec> seg1, seg2;
        for (const auto& pt : plan1->trajectory_.joint_trajectory.points)
            seg1.push_back(pt.positions);
        for (const auto& pt : plan2->trajectory_.joint_trajectory.points)
            seg2.push_back(pt.positions);

        // Shortcut each segment independently
        moveit::core::RobotState rs_sc(robot_model);
        rs_sc.setToDefaultValues();
        shortcut(seg1, rs_sc, jmg, scene_snapshot, 200);
        rs_sc.setToDefaultValues();  // reset between segments
        shortcut(seg2, rs_sc, jmg, scene_snapshot, 200);
        // Then stitch the already-shortened segments
        std::vector<JointVec> stitched;
        stitched.insert(stitched.end(), seg1.begin(), seg1.end());
        stitched.insert(stitched.end(), seg2.begin(), seg2.end());

        return build_plan(stitched, std::vector<std::string>(joint_names.begin(), joint_names.end()), arm_group);
    }

    auto plan = build_plan(
        full_path,
        std::vector<std::string>(joint_names.begin(), joint_names.end()),
        arm_group);
    plan.planning_time_ = elapsed;
    return plan;
}

// ─────────────────────────────────────────────────────────────
//  RRT-CONNECT  (fallback / alternative to PRM)
//  Grows two trees simultaneously from start and goal,
//  projecting each step onto the ee_down constraint manifold.
// ─────────────────────────────────────────────────────────────
std::vector<JointVec> load_manual_library() {
    std::vector<JointVec> library;
    try {
        std::ifstream f(MANUAL_WAYPOINTS_FILE);
        if (!f.is_open()) return library;
        nlohmann::json data = nlohmann::json::parse(f);
        for (auto& item : data) {
            library.push_back(item["q"].get<std::vector<double>>());
        }
    } catch (...) {}
    return library;
}












static std::optional<moveit::planning_interface::MoveGroupInterface::Plan>
crrt_plan_from_to(
    moveit::planning_interface::MoveGroupInterface& arm_group,
    rclcpp::Node::SharedPtr node,
    const JointVec& q_from,
    const JointVec& q_goal)
{
    auto t0 = std::chrono::steady_clock::now();

    auto& psm_holder = crrt_internal::get_psm(node);
    auto robot_model = arm_group.getRobotModel();
    const auto* jmg  = robot_model->getJointModelGroup(crrt_cfg::GROUP_NAME);
    if (!jmg) return std::nullopt;
    const auto& joint_names = jmg->getVariableNames();

    planning_scene::PlanningScenePtr scene_snapshot;
    {
        planning_scene_monitor::LockedPlanningSceneRO ls(psm_holder.psm);
        scene_snapshot = planning_scene::PlanningScene::clone(
            psm_holder.psm->getPlanningScene());
    }

    moveit::core::RobotState rs(robot_model);
    rs.setToDefaultValues();

    if (!is_valid(q_goal, rs, jmg, scene_snapshot)) {
        RCLCPP_ERROR(node->get_logger(), "[CRRT] from_to: goal invalid.");
        return std::nullopt;
    }

    std::vector<std::pair<double,double>> limits;
    for (const auto* j : jmg->getActiveJointModels()) {
        const auto& bnd = j->getVariableBounds()[0];
        limits.push_back({bnd.min_position_, bnd.max_position_});
    }

    std::vector<RRTNode> tree_a = {{ q_from, -1 }};
    std::vector<RRTNode> tree_b = {{ q_goal, -1 }};

    std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    static auto library = load_manual_library();

    int connect_a_idx = -1, connect_b_idx = -1;
    bool connected = false;

    for (int iter = 0; iter < crrt_cfg::MAX_ITER && !connected; ++iter) {
        if (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() > 5.0)
            break;

        JointVec q_rand(limits.size());
        double r = unit(rng);
        if (r < crrt_cfg::GOAL_BIAS) {
            q_rand = tree_b[0].q;
        } else if (r < (crrt_cfg::GOAL_BIAS + 0.15) && !library.empty()) {
            std::uniform_int_distribution<int> dist_lib(0, library.size() - 1);
            q_rand = library[dist_lib(rng)];
        } else {
            for (size_t j = 0; j < limits.size(); ++j) {
                std::uniform_real_distribution<double> d(limits[j].first, limits[j].second);
                q_rand[j] = d(rng);
            }
        }

        int new_a = extend_greedy_with_projection(tree_a, q_rand, rs, jmg, scene_snapshot);
        if (new_a < 0) continue;
        JointVec q_reached = tree_a[new_a].q;
        int new_b = extend_greedy_with_projection(tree_b, q_reached, rs, jmg, scene_snapshot);

        if (new_b >= 0 && joint_dist(tree_b[new_b].q, q_reached) < crrt_cfg::CONNECT_TOL) {
            connect_a_idx = new_a;
            connect_b_idx = new_b;
            connected = true;
            break;
        }
        std::swap(tree_a, tree_b);
    }

    if (!connected) return std::nullopt;

    std::vector<JointVec> full_path;
    if (joint_dist(tree_a[0].q, q_from) < 0.001) {
        auto path_start = extract_path(tree_a, connect_a_idx);
        auto path_goal  = extract_path(tree_b, connect_b_idx);
        std::reverse(path_goal.begin(), path_goal.end());
        full_path = path_start;
        full_path.insert(full_path.end(), path_goal.begin(), path_goal.end());
    } else {
        auto path_start = extract_path(tree_b, connect_b_idx);
        auto path_goal  = extract_path(tree_a, connect_a_idx);
        std::reverse(path_goal.begin(), path_goal.end());
        full_path = path_start;
        full_path.insert(full_path.end(), path_goal.begin(), path_goal.end());
    }

    moveit::core::RobotState rs_check(robot_model);
    rs_check.setToDefaultValues();
    for (const auto& q : full_path)
        if (!is_valid(q, rs_check, jmg, scene_snapshot)) return std::nullopt;

    // full_path = densify_path(full_path, 0.01);
    auto plan = build_plan(
        full_path,
        std::vector<std::string>(joint_names.begin(), joint_names.end()),
        arm_group);
    plan.planning_time_ = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t0).count();
    return plan;
}







std::optional<moveit::planning_interface::MoveGroupInterface::Plan>
crrt_plan(
    moveit::planning_interface::MoveGroupInterface& arm_group,
    rclcpp::Node::SharedPtr node,
    const std::vector<double>& q_goal)
{
    auto t0 = std::chrono::steady_clock::now();

    auto& psm_holder = crrt_internal::get_psm(node);
    auto robot_model = arm_group.getRobotModel();
    const auto* jmg  = robot_model->getJointModelGroup(crrt_cfg::GROUP_NAME);
    if (!jmg) {
        RCLCPP_ERROR(node->get_logger(),
            "[CRRT] JMG '%s' not found.", crrt_cfg::GROUP_NAME.c_str());
        return std::nullopt;
    }
    const auto& joint_names = jmg->getVariableNames();

    auto current_state = arm_group.getCurrentState(5.0);
    if (!current_state) {
        RCLCPP_ERROR(node->get_logger(), "[CRRT] getCurrentState() failed.");
        return std::nullopt;
    }
    JointVec q_start;
    current_state->copyJointGroupPositions(jmg, q_start);

    planning_scene::PlanningScenePtr scene_snapshot;
    {
        planning_scene_monitor::LockedPlanningSceneRO ls(psm_holder.psm);
        scene_snapshot = planning_scene::PlanningScene::clone(
            psm_holder.psm->getPlanningScene());
    }

    moveit::core::RobotState rs(robot_model);
    rs.setToDefaultValues();

    if (!is_valid(q_goal, rs, jmg, scene_snapshot)) {
        RCLCPP_ERROR(node->get_logger(),
            "[CRRT] Goal violates constraint or is in collision — aborting.");
        return std::nullopt;
    }

    std::vector<std::pair<double,double>> limits;
    for (const auto* j : jmg->getActiveJointModels()) {
        const auto& bnd = j->getVariableBounds()[0];
        limits.push_back({bnd.min_position_, bnd.max_position_});
    }

    RCLCPP_INFO(node->get_logger(),
        "[CRRT] RRT-Connect started (max %d iter / %.1f s)...",
        crrt_cfg::MAX_ITER, crrt_cfg::MAX_TIME_SEC);

    std::vector<RRTNode> tree_a = {{ q_start, -1 }};
    std::vector<RRTNode> tree_b = {{ q_goal,  -1 }};

    std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<double> unit(0.0, 1.0);

    int  connect_a_idx = -1, connect_b_idx = -1;
    bool connected = false;
    static auto library = load_manual_library(); 
    for (int iter = 0; iter < crrt_cfg::MAX_ITER && !connected; ++iter)
    {
                // NEW: 5s timeout
        if (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() > 5.0) {
            RCLCPP_WARN(node->get_logger(), "[CRRT] 5s timeout — trying midpoint fallback...");
            break;
        }
        
        if (iter % 10 == 0) {
            RCLCPP_INFO(node->get_logger(), "[CRRT] Step 1: Iter %d", iter);
        }    

        JointVec q_rand(limits.size());
        double r = unit(rng);

        if (r < crrt_cfg::GOAL_BIAS) {
            q_rand = tree_b[0].q;
        } 
        else if (r < (crrt_cfg::GOAL_BIAS + 0.15) && !library.empty()) {
            std::uniform_int_distribution<int> dist_lib(0, library.size() - 1);
            q_rand = library[dist_lib(rng)];
        } 
        else {
            for (size_t j = 0; j < limits.size(); ++j) {
                std::uniform_real_distribution<double> d(limits[j].first, limits[j].second);
                q_rand[j] = d(rng);
            }
        }
        if (iter % 10 == 0) RCLCPP_INFO(node->get_logger(), "[CRRT] Step 2: Sampled");

        int new_a = extend_greedy_with_projection(tree_a, q_rand, rs, jmg, scene_snapshot);
        if (new_a < 0) continue;
        if (iter % 10 == 0) RCLCPP_INFO(node->get_logger(), "[CRRT] Step 3: Extended A");
        JointVec q_reached = tree_a[new_a].q;
        int new_b = extend_greedy_with_projection(tree_b, q_reached, rs, jmg, scene_snapshot);

        if (new_b >= 0 && joint_dist(tree_b[new_b].q, q_reached) < crrt_cfg::CONNECT_TOL) {
            connect_a_idx = new_a;
            connect_b_idx = new_b;
            connected = true;
            break;
        }
        std::swap(tree_a, tree_b);
    }

    if (!connected) {
        // NEW: midpoint fallback
        static const JointVec q_mid = {0.0611, -0.0977, -0.2164, -0.0436, 0.3159, 0.1012};

        RCLCPP_INFO(node->get_logger(), "[CRRT] Stage 1: start -> midpoint...");
        auto plan1 = crrt_plan_from_to(arm_group, node, q_start, q_mid);
        if (!plan1) {
            RCLCPP_ERROR(node->get_logger(), "[CRRT] Midpoint Stage 1 failed.");
            return std::nullopt;
        }

        RCLCPP_INFO(node->get_logger(), "[CRRT] Stage 2: midpoint -> goal...");
        auto plan2 = crrt_plan_from_to(arm_group, node, q_mid, q_goal);
        if (!plan2) {
            RCLCPP_ERROR(node->get_logger(), "[CRRT] Midpoint Stage 2 failed.");
            return std::nullopt;
        }

        // Stitch
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

    std::vector<JointVec> full_path;
    if (joint_dist(tree_a[0].q, q_start) < 0.001) {
        auto path_start = extract_path(tree_a, connect_a_idx);
        auto path_goal  = extract_path(tree_b, connect_b_idx);
        std::reverse(path_goal.begin(), path_goal.end());
        full_path = path_start;
        full_path.insert(full_path.end(), path_goal.begin(), path_goal.end());
    } else {
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
                "[CRRT] Final path contains invalid waypoint — aborting.");
            return std::nullopt;
        }
    }

    RCLCPP_INFO(node->get_logger(), "[CRRT] Raw path: %zu waypoints.", full_path.size());
    // full_path = densify_path(full_path, 0.01);
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
