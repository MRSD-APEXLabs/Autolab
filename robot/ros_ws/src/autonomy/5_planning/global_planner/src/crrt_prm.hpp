// crrt_prm.hpp
// PRM roadmap: build (with manual waypoint injection + local sampling),
// load, and save.
// Depends on: crrt_types.hpp, crrt_validity.hpp

#pragma once
#include "crrt_validity.hpp"

// ─────────────────────────────────────────────────────────────
//  LOAD MANUAL WAYPOINTS FROM JSON
// ─────────────────────────────────────────────────────────────
static std::vector<ManualWaypoint> load_manual_waypoints(
    rclcpp::Node::SharedPtr node,
    size_t expected_dof)
{
    std::ifstream f(MANUAL_WAYPOINTS_FILE);
    if (!f) {
        RCLCPP_WARN(node->get_logger(),
            "[PRM] Manual waypoints file not found: %s", MANUAL_WAYPOINTS_FILE.c_str());
        return {};
    }

    nlohmann::json j;
    try { f >> j; }
    catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(),
            "[PRM] Failed to parse manual waypoints JSON: %s", e.what());
        return {};
    }

    std::vector<ManualWaypoint> waypoints;
    for (const auto& wp : j) {
        ManualWaypoint mw;
        mw.index = wp.value("index", -1);
        mw.note  = wp.value("note",  "");

        auto arr = wp["angles_rad"].get<std::vector<double>>();
        // Slice to expected_dof — JSON may have 16 values if recorded with extra joints
        if (arr.size() < expected_dof) {
            RCLCPP_WARN(node->get_logger(),
                "[PRM] Manual wp #%d has only %zu joints (need %zu) — skipping.",
                mw.index, arr.size(), expected_dof);
            continue;
        }
        mw.q = JointVec(arr.begin(), arr.begin() + expected_dof);
        waypoints.push_back(mw);
    }

    RCLCPP_INFO(node->get_logger(),
        "[PRM] Loaded %zu manual waypoints from %s",
        waypoints.size(), MANUAL_WAYPOINTS_FILE.c_str());
    return waypoints;
}

// ─────────────────────────────────────────────────────────────
//  LOAD PRM ROADMAP FROM BINARY FILE
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
    for (auto& np : nodes) {
        np.q.resize(dof);
        f.read(reinterpret_cast<char*>(np.q.data()), dof * sizeof(double));
        size_t nn;
        f.read(reinterpret_cast<char*>(&nn), sizeof(nn));
        np.neighbors.resize(nn);
        f.read(reinterpret_cast<char*>(np.neighbors.data()), nn * sizeof(int));
    }

    RCLCPP_INFO(node->get_logger(),
        "[PRM] Loaded roadmap: %zu nodes from %s", n, PRM_ROADMAP_FILE.c_str());
    return nodes;
}

// ─────────────────────────────────────────────────────────────
//  BUILD PRM ROADMAP
//
//  Phase 1   — Inject manual waypoints with detailed failure stats
//  Phase 1.5 — Gaussian local sampling around each valid manual waypoint
//  Phase 2   — Global Halton sampling across workspace
//  Phase 3   — Connect K nearest neighbors (collision-free edges)
//  Phase 4   — Save to binary file
// ─────────────────────────────────────────────────────────────
void build_prm_roadmap(
    moveit::planning_interface::MoveGroupInterface& arm_group,
    rclcpp::Node::SharedPtr node,
    int num_samples = 500,
    int k_neighbors = 10)
{
    auto robot_model = arm_group.getRobotModel();
    const auto* jmg  = robot_model->getJointModelGroup(crrt_cfg::GROUP_NAME);

    auto& psm_holder = crrt_internal::get_psm(node);
    RCLCPP_INFO(node->get_logger(), "[PRM] Waiting for octomap...");
    rclcpp::Rate wait_rate(2);
    for (int i = 0; i < 30; ++i) {
        {
            planning_scene_monitor::LockedPlanningSceneRO ls(psm_holder.psm);
            if (ls->getWorld()->getObject("<octomap>")) {
                RCLCPP_INFO(node->get_logger(), "[PRM] Octomap confirmed.");
                break;
            }
        }
        if (i == 29)
            RCLCPP_WARN(node->get_logger(), "[PRM] Octomap not found after 15s.");
        wait_rate.sleep();
    }

    // One frozen, padded world for the whole planning command.
    crrt_internal::ScopedSceneFreeze scene_freeze(node);
    planning_scene::PlanningScenePtr scene_snapshot =
        crrt_internal::snapshot_scene(node);

    moveit::core::RobotState rs(robot_model);
    rs.setToDefaultValues();

    std::vector<std::pair<double,double>> limits;
    for (const auto* j : jmg->getActiveJointModels()) {
        const auto& bnd = j->getVariableBounds()[0];
        limits.push_back({bnd.min_position_, bnd.max_position_});
    }
    const size_t dof = limits.size();

    std::vector<PRMNode> nodes;


    // ═════════════════════════════════════════════════════════
    //  PHASE 1 — INJECT MANUAL WAYPOINTS
    // ═════════════════════════════════════════════════════════
    auto manual_waypoints = load_manual_waypoints(node, dof);

    nodes.reserve(manual_waypoints.size() 
              + manual_waypoints.size() * crrt_cfg::LOCAL_SAMPLES_PER_WP 
              + num_samples 
              + 500);
    int mw_ok = 0, mw_fail_jlimits = 0, mw_fail_constraint = 0;
    int mw_fail_self = 0, mw_fail_octomap = 0, mw_fail_urdf = 0, mw_fail_dup = 0;

    RCLCPP_INFO(node->get_logger(),
        "[PRM] ── Phase 1: Injecting %zu manual waypoints ──",
        manual_waypoints.size());

    for (const auto& mw : manual_waypoints) {

        // Joint limits
        bool jlimit_ok = true;
        for (size_t i = 0; i < dof; ++i) {
            if (mw.q[i] < limits[i].first || mw.q[i] > limits[i].second) {
                jlimit_ok = false; break;
            }
        }
        if (!jlimit_ok) {
            RCLCPP_WARN(node->get_logger(),
                "[PRM] Manual wp #%d FAILED joint limits  note='%s'",
                mw.index, mw.note.c_str());
            ++mw_fail_jlimits; continue;
        }

        // ee_down constraint
        rs.setJointGroupPositions(jmg, mw.q);
        rs.updateLinkTransforms();
        if (!satisfies_ee_down(rs)) {
            const Eigen::Matrix3d R =
                rs.getGlobalLinkTransform(crrt_cfg::EE_LINK).rotation();
            double roll  = std::atan2( R(2,1), R(2,2));
            double pitch = std::atan2(-R(2,0),
                           std::sqrt(R(2,1)*R(2,1) + R(2,2)*R(2,2)));
            RCLCPP_WARN(node->get_logger(),
                "[PRM] Manual wp #%d FAILED ee_down  roll=%.2f°  pitch=%.2f°  "
                "tol=%.2f°  note='%s'",
                mw.index, roll*180/M_PI, pitch*180/M_PI,
                crrt_cfg::EE_ROLL_TOL*180/M_PI, mw.note.c_str());
            ++mw_fail_constraint; continue;
        }

        // Self-collision
        {
            collision_detection::CollisionRequest req;
            collision_detection::CollisionResult  res;
            req.group_name = crrt_cfg::GROUP_NAME;
            req.contacts = true; req.max_contacts = 5;
            scene_snapshot->checkSelfCollision(req, res, rs);
            if (res.collision) {
                std::string pairs;
                for (const auto& [key, _] : res.contacts)
                    pairs += key.first + " <-> " + key.second + "  ";
                RCLCPP_WARN(node->get_logger(),
                    "[PRM] Manual wp #%d FAILED self-collision  %s  note='%s'",
                    mw.index, pairs.c_str(), mw.note.c_str());
                ++mw_fail_self; continue;
            }
        }

        // Scene collision (URDF mesh + octomap)
        {
            collision_detection::CollisionRequest req;
            collision_detection::CollisionResult  res;
            req.group_name = crrt_cfg::GROUP_NAME;
            req.contacts = true; req.max_contacts = 5;
            scene_snapshot->checkCollision(req, res, rs);
            if (res.collision) {
                bool has_octomap = false, has_urdf = false;
                std::string pairs;
                for (const auto& [key, _] : res.contacts) {
                    pairs += key.first + " <-> " + key.second + "  ";
                    if (key.first.find("octomap") != std::string::npos ||
                        key.second.find("octomap") != std::string::npos)
                        has_octomap = true;
                    else
                        has_urdf = true;
                }
                if (has_octomap) {
                    RCLCPP_WARN(node->get_logger(),
                        "[PRM] Manual wp #%d FAILED octomap  %s  note='%s'",
                        mw.index, pairs.c_str(), mw.note.c_str());
                    ++mw_fail_octomap;
                }
                if (has_urdf) {
                    RCLCPP_WARN(node->get_logger(),
                        "[PRM] Manual wp #%d FAILED URDF mesh  %s  note='%s'",
                        mw.index, pairs.c_str(), mw.note.c_str());
                    ++mw_fail_urdf;
                }
                continue;
            }
        }

        // Duplicate check
        bool is_dup = false;
        for (const auto& existing : nodes)
            if (joint_dist(mw.q, existing.q) < crrt_cfg::CONNECT_TOL) { is_dup = true; break; }
        if (is_dup) { ++mw_fail_dup; continue; }

        PRMNode n; n.q = mw.q; nodes.push_back(std::move(n));
        ++mw_ok;
    }

    RCLCPP_INFO(node->get_logger(), "[PRM] Manual waypoints summary ──────────────");
    RCLCPP_INFO(node->get_logger(), "[PRM]   Total loaded      : %zu", manual_waypoints.size());
    RCLCPP_INFO(node->get_logger(), "[PRM]   OK added to map   : %d", mw_ok);
    RCLCPP_INFO(node->get_logger(), "[PRM]   FAIL joint limits : %d", mw_fail_jlimits);
    RCLCPP_INFO(node->get_logger(), "[PRM]   FAIL ee_down      : %d", mw_fail_constraint);
    RCLCPP_INFO(node->get_logger(), "[PRM]   FAIL self-coll.   : %d", mw_fail_self);
    RCLCPP_INFO(node->get_logger(), "[PRM]   FAIL octomap      : %d", mw_fail_octomap);
    RCLCPP_INFO(node->get_logger(), "[PRM]   FAIL URDF mesh    : %d", mw_fail_urdf);
    RCLCPP_INFO(node->get_logger(), "[PRM]   SKIP duplicate    : %d", mw_fail_dup);
    RCLCPP_INFO(node->get_logger(), "[PRM] ─────────────────────────────────────");

    // ═════════════════════════════════════════════════════════
    //  PHASE 1.5 — LOCAL GAUSSIAN SAMPLING AROUND MANUAL WAYPOINTS
    //  Densifies the roadmap in regions you actually care about.
    // ═════════════════════════════════════════════════════════
    const int manual_node_count = mw_ok;

    RCLCPP_INFO(node->get_logger(),
        "[PRM] ── Phase 1.5: Local sampling around %d manual waypoints "
        "(%d samples each, sigma=%.2f rad) ──",
        manual_node_count, crrt_cfg::LOCAL_SAMPLES_PER_WP, crrt_cfg::LOCAL_SIGMA);

    std::mt19937 local_rng(123);
    std::normal_distribution<double> gauss(0.0, crrt_cfg::LOCAL_SIGMA);
    int local_added = 0, local_fail_limits = 0, local_fail_constraint = 0, local_fail_collision = 0;

    for (int mi = 0; mi < manual_node_count; ++mi) {
        const JointVec center = nodes[mi].q;
        int added_for_this = 0;

        for (int s = 0;
             s < crrt_cfg::LOCAL_SAMPLES_PER_WP * 10 &&
             added_for_this < crrt_cfg::LOCAL_SAMPLES_PER_WP; ++s)
        {
            rs.setToDefaultValues(); 
            JointVec q_local(dof);
            bool in_limits = true;
            for (size_t i = 0; i < dof; ++i) {
                q_local[i] = center[i] + gauss(local_rng);
                if (q_local[i] < limits[i].first || q_local[i] > limits[i].second) {
                    in_limits = false; break;
                }
            }
            if (!in_limits) { ++local_fail_limits; continue; }

            rs.setJointGroupPositions(jmg, q_local);
            rs.updateLinkTransforms();
            if (!satisfies_ee_down(rs)) {
                if (!project_to_ee_down_sample(rs, jmg)) { ++local_fail_constraint; continue; }
                rs.copyJointGroupPositions(jmg, q_local);  // update q_local to projected version
                rs.updateLinkTransforms();  
}

            collision_detection::CollisionRequest req;
            collision_detection::CollisionResult  res;
            req.group_name = crrt_cfg::GROUP_NAME; req.contacts = false;
            scene_snapshot->checkCollision(req, res, rs);
            if (res.collision) { ++local_fail_collision; continue; }

            PRMNode n; n.q = q_local; nodes.push_back(std::move(n));
            ++local_added; ++added_for_this;
        }
    }

    RCLCPP_INFO(node->get_logger(),
        "[PRM] Local sampling: %d added | %d constraint fail | "
        "%d collision fail | %d limits fail",
        local_added, local_fail_constraint, local_fail_collision, local_fail_limits);

    // ═════════════════════════════════════════════════════════
    //  PHASE 2 — GLOBAL HALTON SAMPLING
    // ═════════════════════════════════════════════════════════
    RCLCPP_INFO(node->get_logger(),
        "[PRM] ── Phase 2: Global sampling %d configurations ──", num_samples);

    int attempts = 0, fail_ik = 0, fail_collision = 0;
    int random_target = (int)nodes.size() + num_samples;

    tf2::Quaternion ee_quat;
    // Add this before the while loop in Phase 2
    std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<double> yaw_dist(-M_PI, M_PI);

    // Replace the Phase 2 while loop:
    while ((int)nodes.size() < random_target && attempts < num_samples * 10) {
        ++attempts;

        geometry_msgs::msg::Pose sample_pose;

        if (manual_node_count > 0 && std::uniform_real_distribution<double>(0,1)(rng) < 0.80) {
            // 80% — Gaussian around a random manual waypoint in joint space
            std::uniform_int_distribution<int> pick(0, manual_node_count - 1);
            const JointVec& center = nodes[pick(rng)].q;

            JointVec q_local(dof);
            bool in_limits = true;
            for (size_t i = 0; i < dof; ++i) {
                q_local[i] = center[i] + gauss(local_rng);
                if (q_local[i] < limits[i].first || q_local[i] > limits[i].second) {
                    in_limits = false; break;
                }
            }
            if (!in_limits) { ++fail_ik; continue; }

            moveit::core::RobotState rs_sample(robot_model);
            rs_sample.setToDefaultValues();
            rs_sample.setJointGroupPositions(jmg, q_local);
            rs_sample.updateLinkTransforms();

            // Project to ee_down
            if (!project_to_ee_down_sample(rs_sample, jmg)) { ++fail_ik; continue; }
            rs_sample.updateLinkTransforms();
            if (!satisfies_ee_down(rs_sample)) { ++fail_ik; continue; }

            JointVec q;
            rs_sample.copyJointGroupPositions(jmg, q);

            collision_detection::CollisionRequest req;
            collision_detection::CollisionResult  res;
            req.group_name = crrt_cfg::GROUP_NAME; req.contacts = false;
            scene_snapshot->checkCollision(req, res, rs_sample);
            if (res.collision) { ++fail_collision; continue; }

            PRMNode pn; pn.q = q; nodes.push_back(std::move(pn));

        } else {
            // 20% — Halton for global coverage
            ee_quat.setRPY(0.0, 0.0, yaw_dist(rng));
            sample_pose.position.x = crrt_cfg::WS_X_MIN + halton(attempts, 2) * (crrt_cfg::WS_X_MAX - crrt_cfg::WS_X_MIN);
            sample_pose.position.y = crrt_cfg::WS_Y_MIN + halton(attempts, 3) * (crrt_cfg::WS_Y_MAX - crrt_cfg::WS_Y_MIN);
            sample_pose.position.z = crrt_cfg::WS_Z_MIN + halton(attempts, 5) * (crrt_cfg::WS_Z_MAX - crrt_cfg::WS_Z_MIN);
            sample_pose.orientation = tf2::toMsg(ee_quat);

            moveit::core::RobotState rs_sample(robot_model);
            rs_sample.setToDefaultValues();
            if (!rs_sample.setFromIK(jmg, sample_pose, 0.005)) { ++fail_ik; continue; }

            JointVec q;
            rs_sample.copyJointGroupPositions(jmg, q);
            rs_sample.updateLinkTransforms();

            if (!satisfies_ee_down(rs_sample)) { ++fail_ik; continue; }

            collision_detection::CollisionRequest req;
            collision_detection::CollisionResult  res;
            req.group_name = crrt_cfg::GROUP_NAME; req.contacts = false;
            scene_snapshot->checkCollision(req, res, rs_sample);
            if (res.collision) { ++fail_collision; continue; }

            PRMNode pn; pn.q = q; nodes.push_back(std::move(pn));
        }

        if (attempts % 500 == 0)
            RCLCPP_INFO(node->get_logger(),
                "[PRM] %zu valid | %d attempts | IK fail: %d | col fail: %d",
                nodes.size(), attempts, fail_ik, fail_collision);
    }
    RCLCPP_INFO(node->get_logger(),
        "[PRM] Sampling complete: %zu total nodes "
        "(%d manual + %d local + %d random)",
        nodes.size(), mw_ok, local_added, (int)nodes.size() - mw_ok - local_added);

    // ═════════════════════════════════════════════════════════
    //  PHASE 3 — CONNECT K NEAREST NEIGHBORS
    // ═════════════════════════════════════════════════════════
    RCLCPP_INFO(node->get_logger(), "[PRM] ── Phase 3: Connecting neighbors ──");

    
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
                if (!is_valid(cur, rs_edge, jmg, scene_snapshot))  {
                    edge_ok = false; break;
                }
            }
            if (edge_ok) {
                nodes[i].neighbors.push_back(j);
                nodes[j].neighbors.push_back(i);
                ++connected;
            }
        }

        if (i % 500 == 0)
            RCLCPP_INFO(node->get_logger(), "[PRM] Connected %d/%zu nodes...", i, nodes.size());
    }

    int total_edges = 0;
    for (const auto& n : nodes) total_edges += n.neighbors.size();
    RCLCPP_INFO(node->get_logger(),
        "[PRM] Roadmap: %zu nodes, %d edges, avg %.1f per node",
        nodes.size(), total_edges / 2, (double)total_edges / nodes.size());

    // ═════════════════════════════════════════════════════════
    //  PHASE 4 — SAVE TO FILE
    // ═════════════════════════════════════════════════════════
    std::ofstream file(PRM_ROADMAP_FILE, std::ios::binary);
    if (!file) {
        RCLCPP_ERROR(node->get_logger(),
            "[PRM] Failed to open %s for writing.", PRM_ROADMAP_FILE.c_str());
        return;
    }

    size_t n = nodes.size();
    file.write(reinterpret_cast<const char*>(&n),   sizeof(n));
    file.write(reinterpret_cast<const char*>(&dof), sizeof(dof));
    for (const auto& np : nodes) {
        file.write(reinterpret_cast<const char*>(np.q.data()), dof * sizeof(double));
        size_t nn = np.neighbors.size();
        file.write(reinterpret_cast<const char*>(&nn), sizeof(nn));
        file.write(reinterpret_cast<const char*>(np.neighbors.data()), nn * sizeof(int));
    }

    RCLCPP_INFO(node->get_logger(),
        "[PRM] Saved: %zu nodes (%d manual, %d local, %d random) → %s",
        n, mw_ok, local_added, (int)n - mw_ok - local_added, PRM_ROADMAP_FILE.c_str());
}

void inject_waypoints_into_roadmap(
    moveit::planning_interface::MoveGroupInterface& arm_group,
    rclcpp::Node::SharedPtr node,
    const std::vector<JointVec>& new_waypoints,
    int k_neighbors = 15)
{
    auto robot_model = arm_group.getRobotModel();
    const auto* jmg  = robot_model->getJointModelGroup(crrt_cfg::GROUP_NAME);
    const size_t dof = jmg->getVariableNames().size();

    // Load existing roadmap
    std::vector<PRMNode> roadmap = load_prm_roadmap(node, dof);
    if (roadmap.empty()) {
        RCLCPP_ERROR(node->get_logger(), "[INJECT] No roadmap to inject into.");
        return;
    }

    // Get scene snapshot for edge validation
    // One frozen, padded world for the whole planning command.
    crrt_internal::ScopedSceneFreeze scene_freeze(node);
    planning_scene::PlanningScenePtr scene_snapshot =
        crrt_internal::snapshot_scene(node);

    moveit::core::RobotState rs(robot_model);
    rs.setToDefaultValues();

    for (const auto& q_new : new_waypoints)
    {
        // You said you trust these satisfy constraints — skip constraint check
        // Just do a quick collision check to be safe
        rs.setJointGroupPositions(jmg, q_new);
        rs.updateLinkTransforms();
        collision_detection::CollisionRequest req;
        collision_detection::CollisionResult  res;
        req.group_name = crrt_cfg::GROUP_NAME;
        scene_snapshot->checkCollision(req, res, rs);
        if (res.collision) {
            RCLCPP_WARN(node->get_logger(), "[INJECT] New waypoint in collision — skipping.");
            continue;
        }

        const int new_idx = (int)roadmap.size();
        PRMNode new_node; new_node.q = q_new;

        // Connect to K nearest existing nodes
        std::vector<std::pair<double,int>> dists;
        for (int i = 0; i < (int)roadmap.size(); ++i)
            dists.push_back({joint_dist(q_new, roadmap[i].q), i});
        std::sort(dists.begin(), dists.end());

        for (auto& [d, i] : dists) {
            if ((int)new_node.neighbors.size() >= k_neighbors) break;
            bool edge_ok = true;
            JointVec cur = q_new;
            while (joint_dist(cur, roadmap[i].q) > crrt_cfg::STEP_SIZE) {
                cur = steer(cur, roadmap[i].q, crrt_cfg::STEP_SIZE);
                if (!is_valid(cur, rs, jmg, scene_snapshot)) { edge_ok = false; break; }
            }
            if (edge_ok) {
                new_node.neighbors.push_back(i);
                roadmap[i].neighbors.push_back(new_idx);
            }
        }

        RCLCPP_INFO(node->get_logger(),
            "[INJECT] Added node %d with %zu edges.",
            new_idx, new_node.neighbors.size());
        roadmap.push_back(std::move(new_node));
    }

    // Save back to same file
    std::ofstream file(PRM_ROADMAP_FILE, std::ios::binary);
    size_t n = roadmap.size(), dof_out = dof;
    file.write(reinterpret_cast<const char*>(&n),       sizeof(n));
    file.write(reinterpret_cast<const char*>(&dof_out), sizeof(dof_out));
    for (const auto& np : roadmap) {
        file.write(reinterpret_cast<const char*>(np.q.data()), dof * sizeof(double));
        size_t nn = np.neighbors.size();
        file.write(reinterpret_cast<const char*>(&nn), sizeof(nn));
        file.write(reinterpret_cast<const char*>(np.neighbors.data()), nn * sizeof(int));
    }

    RCLCPP_INFO(node->get_logger(),
        "[INJECT] Roadmap updated: %zu total nodes saved.", roadmap.size());
}