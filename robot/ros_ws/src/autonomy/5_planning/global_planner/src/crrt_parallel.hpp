// crrt_parallel.hpp
// The planning stages of move_to_pose_node's find_plan, made quicker:
//   - Several searches at once: each Job (a goal, with or without the safety margin) gets its own
//     threads, and each thread grows its own Informed RRT* tree (own seed, RobotState and IK solver).
//     The first path a thread finds is taken as is (no refinement), then shortcut by crrt_finish_path.
//   - Jobs are listed best first. Once any job has a path, the better ones still searching get
//     PREFER_WAIT_SEC more; then the best path found is taken.
//   - Before searching, a job takes the straight start->goal segment, or a path kept from an earlier
//     execution (forward or reversed, its ends moved onto this start and goal), when that passes the
//     same checks as a searched path in the current scene (obstacles, held plate, margin).
// Depends on: crrt_rrtstar.hpp

#pragma once
#include "crrt_rrtstar.hpp"
#include <atomic>
#include <condition_variable>
#include <deque>

namespace crrt_parallel {

static constexpr double PREFER_WAIT_SEC = 1.0;   // see above
static constexpr int    CACHE_SIZE      = 8;     // executed paths kept
static constexpr double CACHE_MATCH_RAD = 0.3;   // a kept path is reused when its ends are this close (joint space)
                                                 // to the new start and goal

struct Job {
    JointVec    q_goal;
    double      margin  = 0.0;   // > 0: the path also keeps this clearance from the obstacles (crrt_clearance)
    int         threads = 1;     // searches at once for this job
    std::string label;           // for the log
};

struct Result {
    std::optional<moveit::planning_interface::MoveGroupInterface::Plan> plan;
    size_t job = 0;                 // the job the plan reaches
    bool   start_invalid  = false;  // the start is in collision: nothing was searched
    bool   any_goal_valid = false;  // false: every job's goal is invalid, nothing was searched
};

// ── Paths kept from earlier executions ───────────────────────
static std::deque<std::vector<JointVec>>& cache()
{
    static std::deque<std::vector<JointVec>> c;
    return c;
}

// Keep `path`, the waypoints of a plan that has executed, in place of a kept path between the same two places.
static void remember(const std::vector<JointVec>& path)
{
    if (path.size() < 2) return;
    auto near = [](const JointVec& a, const JointVec& b) { return joint_dist(a, b) <= CACHE_MATCH_RAD; };
    auto& c = cache();
    c.erase(std::remove_if(c.begin(), c.end(), [&](const std::vector<JointVec>& p) {
                return (near(p.front(), path.front()) && near(p.back(), path.back())) ||
                       (near(p.front(), path.back()) && near(p.back(), path.front()));
            }), c.end());
    c.push_front(path);
    while ((int)c.size() > CACHE_SIZE) c.pop_back();
}

// Kept paths, newest first, from near q_start to near q_goal in either direction, with their ends
// replaced by q_start and q_goal. `reversed`: the path is a kept one run backwards.
struct KeptPath { std::vector<JointVec> path; bool reversed; };
static std::vector<KeptPath> kept_paths(const JointVec& q_start, const JointVec& q_goal)
{
    std::vector<KeptPath> out;
    for (const auto& p : cache()) {
        for (bool reversed : {false, true}) {
            const JointVec& a = reversed ? p.back() : p.front();
            const JointVec& b = reversed ? p.front() : p.back();
            if (joint_dist(a, q_start) > CACHE_MATCH_RAD || joint_dist(b, q_goal) > CACHE_MATCH_RAD) continue;
            std::vector<JointVec> q(p);
            if (reversed) std::reverse(q.begin(), q.end());
            q.front() = q_start;
            q.back()  = q_goal;
            out.push_back({std::move(q), reversed});
            break;
        }
    }
    return out;
}

// ── Checks ───────────────────────────────────────────────────
// Whether a joint turns more than pi between two waypoints. crrt_finish_path would wrap such a path, and
// the path executed would not be the one checked, so it is not reused.
static bool wraps(const std::vector<JointVec>& path)
{
    for (size_t i = 1; i < path.size(); ++i)
        for (size_t j = 0; j < path[i].size(); ++j)
            if (std::abs(path[i][j] - path[i - 1][j]) > M_PI) return true;
    return false;
}

// The checks a searched path passes: every waypoint after the start valid, every segment valid at STEP_SIZE.
static bool path_valid(const std::vector<JointVec>& path,
                       moveit::core::RobotState& rs,
                       const moveit::core::JointModelGroup* jmg,
                       const planning_scene::PlanningScenePtr& scene)
{
    for (size_t i = 1; i < path.size(); ++i)
        if (!is_valid(path[i], rs, jmg, scene) || !edge_valid(path[i - 1], path[i], rs, jmg, scene))
            return false;
    return true;
}

// Whether the straight segment q_start -> q_goal passes edge_valid's checks; `why` says where and why not.
static bool straight_path(const JointVec& q_start, const JointVec& q_goal,
                          moveit::core::RobotState& rs,
                          const moveit::core::JointModelGroup* jmg,
                          const planning_scene::PlanningScenePtr& scene,
                          std::string& why)
{
    if (wraps({q_start, q_goal})) {
        why = "not used: a joint turns more than 180 deg";
        return false;
    }
    const double d = joint_dist(q_start, q_goal);
    JointVec cur = q_start;
    while (joint_dist(cur, q_goal) > crrt_cfg::STEP_SIZE) {
        cur = steer(cur, q_goal, crrt_cfg::STEP_SIZE);
        if (is_valid(cur, rs, jmg, scene)) continue;
        const int pct = (int)std::lround(100.0 * (1.0 - joint_dist(cur, q_goal) / d));
        if (!satisfies_ee_down(rs)) {
            why = "invalid at " + std::to_string(pct) + "%: ee_down";
        } else {
            collision_detection::CollisionRequest req;
            collision_detection::CollisionResult  res;
            req.group_name = crrt_cfg::GROUP_NAME;
            req.contacts = true;
            req.max_contacts = 10;
            req.max_contacts_per_pair = 1;
            scene->checkCollision(req, res, rs);
            why = "invalid at " + std::to_string(pct) + "%: " +
                  (res.collision ? "collision" : crrt_clearance::below_floor(rs) ? "plate below its floor"
                                                                                  : "within the safety margin");
            for (const auto& [key, _] : res.contacts) why += " [" + key.first + " <-> " + key.second + "]";
        }
        return false;
    }
    why = "VALID";
    return true;
}

// ── IK solvers for the search threads ────────────────────────
// The group's own solver is not safe to share (crrt_ik). These are made once, from the group's solver
// allocator, and kept for the life of the node.
static const std::vector<kinematics::KinematicsBasePtr>& solvers(const moveit::core::JointModelGroup* jmg, size_t n)
{
    static std::vector<kinematics::KinematicsBasePtr> pool;
    const auto& alloc = jmg->getGroupKinematics().first.allocator_;
    while (alloc && pool.size() < n) {
        auto s = alloc(jmg);
        if (!s) break;
        pool.push_back(std::move(s));
    }
    return pool;
}

// ── The search ───────────────────────────────────────────────
// Plans from the current state to the best job it can, each search thread at most time_sec. Uses the
// frozen scene if the caller froze one (ScopedSceneFreeze), else freezes its own.
static Result plan(moveit::planning_interface::MoveGroupInterface& arm_group,
                   rclcpp::Node::SharedPtr node,
                   const std::vector<Job>& jobs, double time_sec)
{
    using Clock = std::chrono::steady_clock;
    Result result;
    const auto t0  = Clock::now();
    const auto log = node->get_logger();
    const int max_iter = static_cast<int>(
        crrt_cfg::RRTSTAR_MAX_ITER * std::max(1.0, time_sec / crrt_cfg::RRTSTAR_TIME_SEC));

    auto robot_model = arm_group.getRobotModel();
    const auto* jmg  = robot_model->getJointModelGroup(crrt_cfg::GROUP_NAME);
    if (!jmg) {
        RCLCPP_WARN(log, "[IRRT*] JMG '%s' not found.", crrt_cfg::GROUP_NAME.c_str());
        return result;
    }
    const auto& joint_names = jmg->getVariableNames();

    auto current_state = arm_group.getCurrentState(5.0);
    if (!current_state) {
        RCLCPP_WARN(log, "[IRRT*] getCurrentState() failed.");
        return result;
    }
    JointVec q_start;
    current_state->copyJointGroupPositions(jmg, q_start);

    // One frozen world for the whole search.
    crrt_internal::ScopedSceneFreeze scene_freeze(node);
    const planning_scene::PlanningScenePtr scene = crrt_internal::snapshot_scene(node);

    moveit::core::RobotState rs(robot_model);
    rs.setToDefaultValues();

    // Start: collision only — arm may not satisfy ee_down at rest
    if (!is_collision_free(q_start, rs, jmg, scene)) {
        RCLCPP_WARN(log, "[IRRT*] Start state is in collision.");
        result.start_invalid = true;
        return result;
    }

    std::vector<std::pair<double,double>> limits;
    for (const auto* j : jmg->getActiveJointModels()) {
        const auto& bnd = j->getVariableBounds()[0];
        limits.push_back({bnd.min_position_, bnd.max_position_});
    }

    auto ee_position = [&](const JointVec& q) {
        rs.setJointGroupPositions(jmg, q);
        rs.updateLinkTransforms();
        return Eigen::Vector3d(rs.getGlobalLinkTransform(crrt_cfg::EE_LINK).translation());
    };
    const moveit::core::LinkModel* plate = robot_model->getLinkModel(crrt_cfg::PLATE_LINK);
    auto plate_bottom = [&](const JointVec& q) {
        rs.setJointGroupPositions(jmg, q);
        rs.updateLinkTransforms();
        return crrt_clearance::bottom_z(rs, plate);
    };
    if (!plate)
        RCLCPP_WARN(log, "[IRRT*] No link '%s' in the robot model: no plate floor.", crrt_cfg::PLATE_LINK.c_str());
    {
        rs.setJointGroupPositions(jmg, q_start);
        rs.updateLinkTransforms();
        const Eigen::Matrix3d R = rs.getGlobalLinkTransform(crrt_cfg::EE_LINK).rotation();
        RCLCPP_INFO(log, "[IRRT*] Start EE roll %.1f pitch %.1f yaw %.1f deg (tol %.1f).",
            std::atan2(R(2,1), R(2,2)) * 180 / M_PI,
            std::atan2(-R(2,0), std::hypot(R(2,1), R(2,2))) * 180 / M_PI,
            std::atan2(R(1,0), R(0,0)) * 180 / M_PI, crrt_cfg::EE_ROLL_TOL * 180 / M_PI);
    }

    auto& viz = crrt_viz::get(node, robot_model, jmg);
    viz.begin();

    // Per job, best first: the goal check, the clearance it is searched with, and a path without searching.
    // Once a job has such a path, the jobs after it would never be taken, so they are left out.
    const size_t n = jobs.size();
    std::vector<bool> alive(n, false);
    std::vector<crrt_clearance::State> clearance(n);
    std::vector<std::vector<JointVec>> found(n);
    std::vector<std::string> found_how(n);
    std::vector<bool> kept_path(n, false);   // found[i] is a kept path: already shortcut
    std::optional<size_t> instant;
    for (size_t i = 0; i < n && !instant; ++i) {
        const Job& job = jobs[i];
        if (!is_valid(job.q_goal, rs, jmg, scene)) {
            rs.setJointGroupPositions(jmg, job.q_goal);
            rs.updateLinkTransforms();
            collision_detection::CollisionRequest req;
            collision_detection::CollisionResult  res;
            req.group_name = crrt_cfg::GROUP_NAME;
            req.contacts = true;
            req.max_contacts = 10;
            req.max_contacts_per_pair = 1;
            scene->checkCollision(req, res, rs);
            std::string pairs;
            for (const auto& [key, _] : res.contacts) pairs += key.first + " <-> " + key.second + "  ";
            RCLCPP_WARN(log, "[IRRT*] Goal invalid (%s) — ee_down=%s, collision=%s %s", job.label.c_str(),
                satisfies_ee_down(rs) ? "ok" : "FAIL", res.collision ? "YES:" : "NO", pairs.c_str());
            continue;
        }
        alive[i] = true;
        result.any_goal_valid = true;
        if (job.margin > 0.0) {
            clearance[i] = crrt_clearance::State{job.margin, {ee_position(q_start), ee_position(job.q_goal)},
                                                 nullptr, nullptr};
            crrt_clearance::pad(clearance[i], jmg, scene);
        }
        std::string floor_note;
        if (plate) {
            const double z_start = plate_bottom(q_start), z_goal = plate_bottom(job.q_goal);
            clearance[i].plate   = plate;
            clearance[i].floor_z = std::min(z_start, z_goal) - crrt_cfg::PLATE_FLOOR_BELOW_M;
            char note[96];
            std::snprintf(note, sizeof note, "; plate bottom %.3f -> %.3f m, floor %.3f m",
                          z_start, z_goal, clearance[i].floor_z);
            floor_note = note;
        }

        crrt_clearance::Scope use(clearance[i]);
        std::string why;
        if (straight_path(q_start, job.q_goal, rs, jmg, scene, why)) {
            found[i] = {q_start, job.q_goal};
            found_how[i] = "the straight path";
        } else {
            for (const auto& kept : kept_paths(q_start, job.q_goal)) {
                if (wraps(kept.path) || !path_valid(kept.path, rs, jmg, scene)) continue;
                found[i] = kept.path;
                found_how[i] = kept.reversed ? "a kept path, reversed" : "a kept path";
                kept_path[i] = true;
                break;
            }
        }
        const std::string kept = found[i].size() > 2 ? "; " + found_how[i] + " passes" : "";
        RCLCPP_INFO(log, "[IRRT*] %s: start->goal %.2f rad%s; straight path %s%s", job.label.c_str(),
            joint_dist(q_start, job.q_goal), floor_note.c_str(), why.c_str(), kept.c_str());
        if (!found[i].empty()) instant = i;
    }
    if (!result.any_goal_valid)
        return result;

    // Search the jobs before the one with a path already (all of them if none).
    struct Shared {
        std::mutex m;
        std::condition_variable cv;
        std::vector<std::vector<JointVec>> found;
        std::vector<std::string> found_how;
        std::vector<int> running;                    // threads still searching, per job
        std::optional<Clock::time_point> first;      // when any job first had a path
        IrrtStats stats;                             // summed over the threads, for the log
        int iters = 0;
    } sh;
    sh.found = std::move(found);
    sh.found_how = std::move(found_how);
    sh.running.assign(n, 0);
    if (instant) sh.first = Clock::now();

    std::vector<std::pair<size_t, size_t>> work;   // (job, thread number)
    for (size_t i = 0; i < (instant ? *instant : n); ++i)
        for (int k = 0; alive[i] && k < jobs[i].threads; ++k)
            work.push_back({i, work.size()});
    const auto& pool = solvers(jmg, work.size());
    if (pool.size() < work.size()) {
        RCLCPP_WARN(log, "[IRRT*] Only %zu IK solvers for %zu searches; running %zu.",
                    pool.size(), work.size(), std::max<size_t>(1, pool.size()));
        work.resize(std::max<size_t>(1, pool.size()));   // one search can use the group's own solver
    }

    std::deque<std::atomic<bool>> stop(n);
    for (auto& s : stop) s.store(false);
    std::vector<unsigned> seeds;
    std::random_device rd;
    for (size_t k = 0; k < work.size(); ++k) seeds.push_back(rd());
    for (const auto& [job, k] : work) ++sh.running[job];

    std::vector<std::thread> threads;
    if (!work.empty())
        RCLCPP_INFO(log, "[IRRT*] %zu searches at once (up to %.1f s, eta %.2f), first path wins.",
                    work.size(), time_sec, crrt_cfg::RRTSTAR_ETA);
    for (const auto& [job, k] : work) {
        threads.emplace_back([&, job = job, k = k]() {
            crrt_ik::solver = k < pool.size() ? pool[k].get() : nullptr;
            crrt_clearance::state() = clearance[job];
            irrt_stats = {};
            IrrtGrowth g;
            try {
                moveit::core::RobotState rs_k(robot_model);
                rs_k.setToDefaultValues();
                std::mt19937 rng(seeds[k]);
                g = irrtstar_grow(q_start, jobs[job].q_goal, time_sec, max_iter, rs_k, jmg, scene, limits,
                                  rng, false, &stop[job], nullptr, t0);
            } catch (const std::exception& e) {
                RCLCPP_WARN(log, "[IRRT*] Search %zu failed: %s", k, e.what());
            }
            crrt_clearance::state() = {};
            std::lock_guard<std::mutex> lock(sh.m);
            sh.stats.ik += irrt_stats.ik;
            sh.stats.not_closer += irrt_stats.not_closer;
            sh.stats.collision += irrt_stats.collision;
            sh.stats.edge_ee_down += irrt_stats.edge_ee_down;
            sh.stats.edge_collision += irrt_stats.edge_collision;
            sh.iters += g.iter;
            if (!g.path.empty() && sh.found[job].empty()) {
                sh.found[job] = std::move(g.path);
                char how[96];
                std::snprintf(how, sizeof how, "search %zu, iter %d, %.2f s", k, g.iter,
                              std::chrono::duration<double>(Clock::now() - t0).count());
                sh.found_how[job] = how;
                if (!sh.first) sh.first = Clock::now();
                for (size_t j = job; j < n; ++j) stop[j] = true;   // this job's other searches, and the worse jobs
            }
            --sh.running[job];
            sh.cv.notify_all();
        });
    }

    // Wait for the best job with a path: until no better job is still searching, or PREFER_WAIT_SEC after
    // the first path; with no path, until every search has ended.
    {
        std::unique_lock<std::mutex> lock(sh.m);
        while (true) {
            std::optional<size_t> best;
            for (size_t i = 0; i < n && !best; ++i)
                if (!sh.found[i].empty()) best = i;
            bool better_running = false, any_running = false;
            for (size_t i = 0; i < n; ++i) {
                if (sh.running[i] > 0) any_running = true;
                if (best && i < *best && sh.running[i] > 0) better_running = true;
            }
            if (best) {
                const auto give_up = *sh.first + std::chrono::duration_cast<Clock::duration>(
                                                     std::chrono::duration<double>(PREFER_WAIT_SEC));
                if (!better_running || Clock::now() >= give_up) break;
                sh.cv.wait_until(lock, give_up);
            } else {
                if (!any_running) break;
                sh.cv.wait(lock);
            }
        }
    }
    for (auto& s : stop) s = true;
    for (auto& t : threads) t.join();

    std::optional<size_t> best;
    for (size_t i = 0; i < n && !best; ++i)
        if (!sh.found[i].empty()) best = i;
    const double sec = std::chrono::duration<double>(Clock::now() - t0).count();
    if (!best) {
        RCLCPP_WARN(log,
            "[IRRT*] No solution in %.1f s: %zu searches, %d iters in all. Steer stopped by: IK %d, not closer %d, "
            "collision %d. Edges rejected: ee_down %d, collision %d.",
            sec, work.size(), sh.iters, sh.stats.ik, sh.stats.not_closer, sh.stats.collision,
            sh.stats.edge_ee_down, sh.stats.edge_collision);
        return result;
    }
    RCLCPP_INFO(log, "[IRRT*] Taking the path to %s (%s); searching took %.2f s.",
                jobs[*best].label.c_str(), sh.found_how[*best].c_str(), sec);

    crrt_clearance::Scope use(clearance[*best]);   // the shortcut keeps the same clearance
    result.plan = crrt_finish_path(std::move(sh.found[*best]), q_start, jobs[*best].q_goal, arm_group, node,
                                   robot_model, jmg, joint_names, scene, viz, t0, !kept_path[*best]);
    result.job = *best;
    return result;
}

} // namespace crrt_parallel
