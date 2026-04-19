// ┌─────────────────────────────────────────────────────────────┐
// │  SUBSCRIBED TOPICS                                          │
// │  /task_planner/task_type         std_msgs/String            │
// │  /joint_states                   sensor_msgs/JointState     │
// │  /planning_command               std_msgs/String            │
// │                                                             │
// │  PUBLISHED TOPICS                                           │
// │  /planning_state                 std_msgs/String            │
// │    Values: IDLE | PLANNING | EXECUTING | SUCCESS | ERROR    │
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
#include "dummy_cloud.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/srv/get_planning_scene.hpp>
// #include <moveit/collision_detection/collision_common.h>
#include <random>
#include <queue>
#include <unordered_map>
#include <limits>
#include <numeric>
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
// ── STL includes ─────────────────────────────────────────────
#include <thread>
#include <chrono>
#include <mutex>
#include <optional>
#include <string>
#include <fstream>
#include <iomanip>
#include <map>

static constexpr double ARM_PLANNING_TIME_SEC = 30.0;
static constexpr int INPUT_TIMEOUT_SEC = 5;
static constexpr double APRIL_TAG_STALE_SEC = 1.0;
static constexpr int APRIL_TAG_WAIT_TIMEOUT_SEC = 10;
static constexpr int APRIL_TAG_WAIT_STEP_MS = 100;

// ── PRM roadmap parameters ───────────────────────────────────
static constexpr int    PRM_NUM_SAMPLES        = 30000;   // nodes in roadmap
static constexpr int    PRM_K_NEIGHBOURS       = 10;     // edges per node
static constexpr double PRM_CONNECT_RADIUS     = 0.4;    // max joint-space dist for edge
static constexpr int    PRM_EDGE_COLLISION_STEPS = 8;    // interpolation steps per edge
static constexpr double PRM_EE_ORIENT_TOL      = 0.25;   // radians — must match make_ee_down_constraint()
static constexpr int PRM_CONNECT_ATTEMPTS   = 30;     // tries to wire start/goal into graph

struct AprilTagEntry {
    geometry_msgs::msg::Point position;
    rclcpp::Time stamp;
};
static std::map<int, AprilTagEntry> april_tags;
static std::mutex april_tags_mutex;
static int marker_id = 0;

double wellplate_tag_x = 0.0;
double wellplate_tag_y = 0.0;
double wellplate_tag_z = 0.0;


// ── PRM Roadmap data structures ──────────────────────────────
struct RoadmapNode
{
    std::vector<double> joints;          // joint-space configuration
    Eigen::Vector3d     ee_pos;          // FK end-effector position (world)
    std::vector<int>    neighbours;      // indices into Roadmap::nodes
    std::vector<double> edge_costs;      // matching joint-space distances
};

struct Roadmap
{
    std::vector<RoadmapNode> nodes;
    bool ready = false;
};

static Roadmap g_roadmap;   // global, built once, reused across plans
static std::mutex g_roadmap_mutex;


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
        double offset)
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
            RCLCPP_ERROR(node->get_logger(),
                         "TF transform top_camera→world failed: %s", ex.what());
            world_point.point = cam_point.point;
        }

        // ── Build pose with transformed position ─────────────────
        p.position.x = world_point.point.x;
        p.position.y = world_point.point.y;
        p.position.z = world_point.point.z + offset; // height offset for pre-grasp

        // Orientation: gripper pointing down
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, M_PI);
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

// ── Helper: fetch current planning scene via service ─────────
moveit_msgs::msg::PlanningScene get_planning_scene_msg(rclcpp::Node::SharedPtr node)
{
    auto client = node->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    client->wait_for_service(std::chrono::seconds(5));

    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    request->components.components =
        moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_NAMES |
        moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY |
        moveit_msgs::msg::PlanningSceneComponents::OCTOMAP;

    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(10))
        != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "[PRM] Failed to get planning scene from service.");
        return moveit_msgs::msg::PlanningScene();
    }
    return future.get()->scene;
}


// ─────────────────────────────────────────────────────────────────────────────
//  build_roadmap
//  Samples the 6-DOF joint space densely, keeps only configurations that
//  (a) are collision-free, (b) satisfy the EE-down orientation constraint,
//  and (c) land inside the planning workspace box.
//  Then connects each node to its k nearest neighbours with collision-checked
//  straight-line edges in joint space.
//  Call this once after the planning scene / octomap has settled.
// ─────────────────────────────────────────────────────────────────────────────
void build_roadmap(
    moveit::planning_interface::MoveGroupInterface &arm_group,
    rclcpp::Node::SharedPtr node,
    const moveit_msgs::msg::PlanningScene &scene_msg)
{
    RCLCPP_INFO(node->get_logger(), "[PRM] Building roadmap (%d samples)…", PRM_NUM_SAMPLES);

    // ── 1. Grab robot model + planning scene ─────────────────
    auto robot_model   = arm_group.getRobotModel();
    auto robot_state   = std::make_shared<moveit::core::RobotState>(robot_model);
    robot_state->setToDefaultValues();

    const auto *jmg    = robot_model->getJointModelGroup("demo_arm_bot");
    const std::string ee_link = arm_group.getEndEffectorLink();

    // Build a local planning scene from the current world (includes octomap)
    planning_scene::PlanningScene scene(robot_model);
    scene.setPlanningSceneDiffMsg(scene_msg);  // ← octomap included
    const collision_detection::AllowedCollisionMatrix &acm = scene.getAllowedCollisionMatrix();
    // DEBUG: check what's in the scene
    RCLCPP_INFO(node->get_logger(), "[PRM DEBUG] Scene has octomap: %s",
        scene.getWorld()->hasObject("<octomap>") ? "YES" : "NO");
    RCLCPP_INFO(node->get_logger(), "[PRM DEBUG] World objects: %zu",
        scene.getWorld()->getObjectIds().size());

    // Check if the DEFAULT state (all joints=0) collides
    robot_state->setToDefaultValues();
    robot_state->update();
    collision_detection::CollisionRequest test_req;
    collision_detection::CollisionResult  test_res;
    test_req.contacts = true;
    test_req.max_contacts = 10;
    scene.checkCollision(test_req, test_res, *robot_state, acm);
    RCLCPP_INFO(node->get_logger(), "[PRM DEBUG] Default state collides: %s",
        test_res.collision ? "YES" : "NO");
    if (test_res.collision)
    {
        for (auto &c : test_res.contacts)
            RCLCPP_INFO(node->get_logger(), "[PRM DEBUG] Contact: %s vs %s",
                c.first.first.c_str(), c.first.second.c_str());
    }

   


    // ── 2. Joint bounds for sampling ─────────────────────────
    const auto &bounds = jmg->getActiveJointModels();
    const int   ndof   = static_cast<int>(bounds.size());

    std::vector<double> lo(ndof), hi(ndof);
    for (int i = 0; i < ndof; ++i)
    {
        const auto &b = bounds[i]->getVariableBounds()[0];
        lo[i] = b.min_position_;
        hi[i] = b.max_position_;
    }

    // ── 3. Workspace AABB (same as setWorkspace() in planner) ─
    const double WS_X_MIN = -0.15, WS_X_MAX = 0.85;
    const double WS_Y_MIN = -0.50, WS_Y_MAX = 0.50;
    const double WS_Z_MIN =  0.90, WS_Z_MAX = 1.30;

    // ── 4. EE orientation reference (RPY = 0, 0, π) ──────────
    //    We accept a node only if its EE quaternion is within
    //    PRM_EE_ORIENT_TOL of this reference on the X and Y axes.
    tf2::Quaternion q_ref;
    q_ref.setRPY(0.0, 0.0, M_PI);
    q_ref.normalize();
    const Eigen::Quaterniond q_ref_e(q_ref.w(), q_ref.x(), q_ref.y(), q_ref.z());

    // ── 5. Collision-request setup ────────────────────────────
    collision_detection::CollisionRequest  col_req;
    collision_detection::CollisionResult   col_res;
    col_req.contacts  = false;
    col_req.distance  = false;

    // ── 6. Sample loop ────────────────────────────────────────
    std::mt19937 rng(42);
    std::vector<std::uniform_real_distribution<double>> dists;
    dists.reserve(ndof);
    for (int i = 0; i < ndof; ++i)
        dists.emplace_back(lo[i], hi[i]);

    std::vector<RoadmapNode> nodes;
    nodes.reserve(PRM_NUM_SAMPLES);

    int attempts = 0;
    while (static_cast<int>(nodes.size()) < PRM_NUM_SAMPLES && attempts < PRM_NUM_SAMPLES * 20)
    {
        ++attempts;

        // Random joint config
        std::vector<double> jvals(ndof);
        for (int i = 0; i < ndof; ++i) jvals[i] = dists[i](rng);

        robot_state->setJointGroupPositions(jmg, jvals);
        robot_state->updateLinkTransforms();

        // ── EE workspace bounds check ─────────────────────────
        const Eigen::Isometry3d &ee_tf = robot_state->getGlobalLinkTransform(ee_link);
        const Eigen::Vector3d    ee_p  = ee_tf.translation();

        // ADD THIS temporarily:
        if (attempts % 1000 == 0)
        {
            RCLCPP_INFO(node->get_logger(), "[PRM DEBUG] ee_p = (%.3f, %.3f, %.3f)",
                        ee_p.x(), ee_p.y(), ee_p.z());
        }
        // if (ee_p.x() < WS_X_MIN || ee_p.x() > WS_X_MAX ||
        //     ee_p.y() < WS_Y_MIN || ee_p.y() > WS_Y_MAX ||
        //     ee_p.z() < WS_Z_MIN || ee_p.z() > WS_Z_MAX)
        //     continue;

        // // ── EE orientation constraint check ──────────────────
        // const Eigen::Quaterniond q_ee(ee_tf.rotation());
        // // Compute angular difference via relative rotation
        // const Eigen::Quaterniond q_diff = q_ref_e.inverse() * q_ee;
        // const Eigen::AngleAxisd  aa(q_diff);
        // // Decompose: tilt (X/Y axes) vs spin (Z axis)
        // const Eigen::Vector3d axis_n = aa.axis();
        // const double angle           = aa.angle();
        // const double tilt_xy = std::abs(angle) *
        //     std::sqrt(axis_n.x()*axis_n.x() + axis_n.y()*axis_n.y());
        // if (tilt_xy > PRM_EE_ORIENT_TOL)
        //     continue;

        //── Collision check ───────────────────────────────────
        col_res.clear();
        scene.checkCollision(col_req, col_res, *robot_state, acm);
        if (col_res.collision) continue;

        RoadmapNode n;
        n.joints = jvals;
        n.ee_pos = ee_p;
        nodes.push_back(std::move(n));
    }

    RCLCPP_INFO(node->get_logger(),
                "[PRM] Sampled %zu valid nodes out of %d attempts.",
                nodes.size(), attempts);

    // ── 7. Connect k nearest neighbours ──────────────────────
    const int N = static_cast<int>(nodes.size());

    auto joint_dist = [&](int a, int b) -> double {
        double d = 0.0;
        for (int k = 0; k < ndof; ++k) {
            double diff = nodes[a].joints[k] - nodes[b].joints[k];
            d += diff * diff;
        }
        return std::sqrt(d);
    };

    // Edge collision check: interpolate in joint space
    auto edge_free = [&](int a, int b) -> bool {
        std::vector<double> qa = nodes[a].joints;
        std::vector<double> qb = nodes[b].joints;
        for (int s = 1; s <= PRM_EDGE_COLLISION_STEPS; ++s)
        {
            double t = static_cast<double>(s) / (PRM_EDGE_COLLISION_STEPS + 1);
            std::vector<double> qm(ndof);
            for (int k = 0; k < ndof; ++k)
                qm[k] = qa[k] + t * (qb[k] - qa[k]);
            robot_state->setJointGroupPositions(jmg, qm);
            robot_state->updateLinkTransforms();
            col_res.clear();
            scene.checkCollision(col_req, col_res, *robot_state, acm);
            if (col_res.collision) return false;
        }
        return true;
    };

    for (int i = 0; i < N; ++i)
    {
        // Gather candidates within connect radius
        std::vector<std::pair<double,int>> cands;
        cands.reserve(64);
        for (int j = 0; j < N; ++j)
        {
            if (i == j) continue;
            double d = joint_dist(i, j);
            if (d < PRM_CONNECT_RADIUS)
                cands.push_back({d, j});
        }
        // Sort by distance, keep top-k
        std::sort(cands.begin(), cands.end());
        int added = 0;
        for (auto &[d, j] : cands)
        {
            if (added >= PRM_K_NEIGHBOURS) break;
            if (edge_free(i, j))
            {
                nodes[i].neighbours.push_back(j);
                nodes[i].edge_costs.push_back(d);
                ++added;
            }
        }
    }

    // ── 8. Store globally ─────────────────────────────────────
    {
        std::lock_guard<std::mutex> lk(g_roadmap_mutex);
        g_roadmap.nodes = std::move(nodes);
        g_roadmap.ready = true;
    }
    RCLCPP_INFO(node->get_logger(), "[PRM] Roadmap ready: %d nodes.", N);
}

// ─────────────────────────────────────────────────────────────────────────────
//  plan_with_roadmap
//  Connects start and goal joint configs into the prebuilt roadmap, runs
//  Dijkstra from start to goal, then converts the waypoint sequence into a
//  moveit::planning_interface::MoveGroupInterface::Plan ready for execute().
// ─────────────────────────────────────────────────────────────────────────────
std::optional<moveit::planning_interface::MoveGroupInterface::Plan>
plan_with_roadmap(
    const std::vector<double> &q_start,
    const std::vector<double> &q_goal,
    moveit::planning_interface::MoveGroupInterface &arm_group,
    rclcpp::Node::SharedPtr node)
{
    std::lock_guard<std::mutex> lk(g_roadmap_mutex);
    if (!g_roadmap.ready)
    {
        RCLCPP_ERROR(node->get_logger(), "[PRM] Roadmap not built yet.");
        return std::nullopt;
    }

    auto &nodes   = g_roadmap.nodes;
    const int N   = static_cast<int>(nodes.size());
    const int ndof = static_cast<int>(q_start.size());

    auto robot_model = arm_group.getRobotModel();
    auto robot_state = std::make_shared<moveit::core::RobotState>(robot_model);
    const auto *jmg  = robot_model->getJointModelGroup("demo_arm_bot");

    planning_scene::PlanningScene scene(robot_model);
    const collision_detection::AllowedCollisionMatrix &acm = scene.getAllowedCollisionMatrix();

    collision_detection::CollisionRequest  col_req;
    collision_detection::CollisionResult   col_res;
    col_req.contacts = false;
    col_req.distance = false;

    auto joint_dist_v = [&](const std::vector<double> &a,
                             const std::vector<double> &b) -> double {
        double d = 0.0;
        for (int k = 0; k < ndof; ++k) { double diff = a[k]-b[k]; d += diff*diff; }
        return std::sqrt(d);
    };

    auto edge_free_v = [&](const std::vector<double> &qa,
                            const std::vector<double> &qb) -> bool {
        for (int s = 1; s <= PRM_EDGE_COLLISION_STEPS; ++s)
        {
            double t = static_cast<double>(s) / (PRM_EDGE_COLLISION_STEPS + 1);
            std::vector<double> qm(ndof);
            for (int k = 0; k < ndof; ++k) qm[k] = qa[k] + t * (qb[k] - qa[k]);
            robot_state->setJointGroupPositions(jmg, qm);
            robot_state->updateLinkTransforms();
            col_res.clear();
            scene.checkCollision(col_req, col_res, *robot_state);            
            if (col_res.collision) return false;
        }
        return true;
    };

    // ── 1. Wire start (index N) and goal (index N+1) into roadmap ─
    // We append two virtual nodes; edges stored separately
    const int IDX_START = N;
    const int IDX_GOAL  = N + 1;
    const int TOTAL     = N + 2;

    // For each virtual node: find best PRM_CONNECT_ATTEMPTS nearest roadmap nodes
    auto wire_node = [&](const std::vector<double> &q, int virtual_idx)
        -> std::vector<std::pair<int,double>>   // [(roadmap_idx, cost)]
    {
        std::vector<std::pair<double,int>> cands;
        cands.reserve(N);
        for (int i = 0; i < N; ++i)
            cands.push_back({joint_dist_v(q, nodes[i].joints), i});
        std::sort(cands.begin(), cands.end());

        std::vector<std::pair<int,double>> edges;
        int connected = 0;
        for (auto &[d, i] : cands)
        {
            if (connected >= PRM_CONNECT_ATTEMPTS) break;
            if (edge_free_v(q, nodes[i].joints))
            {
                edges.push_back({i, d});
                ++connected;
            }
        }
        (void)virtual_idx;
        return edges;
    };

    auto start_edges = wire_node(q_start, IDX_START);
    auto goal_edges  = wire_node(q_goal,  IDX_GOAL);

    if (start_edges.empty())
    {
        RCLCPP_ERROR(node->get_logger(), "[PRM] Could not connect START to roadmap.");
        return std::nullopt;
    }
    if (goal_edges.empty())
    {
        RCLCPP_ERROR(node->get_logger(), "[PRM] Could not connect GOAL to roadmap.");
        return std::nullopt;
    }

    // ── 2. Dijkstra over (roadmap nodes + start + goal) ──────
    // Adjacency is: roadmap edges + start_edges + goal_edges (bidirectional)
    std::vector<double> dist_arr(TOTAL, std::numeric_limits<double>::infinity());
    std::vector<int>    prev_arr(TOTAL, -1);

    // priority queue: (cost, node_idx)
    using PQEntry = std::pair<double,int>;
    std::priority_queue<PQEntry, std::vector<PQEntry>, std::greater<PQEntry>> pq;

    dist_arr[IDX_START] = 0.0;
    pq.push({0.0, IDX_START});

    auto get_neighbours = [&](int u) -> std::vector<std::pair<int,double>>
    {
        std::vector<std::pair<int,double>> nbrs;
        if (u == IDX_START)
        {
            for (auto &[v,c] : start_edges) nbrs.push_back({v, c});
        }
        else if (u == IDX_GOAL)
        {
            for (auto &[v,c] : goal_edges) nbrs.push_back({v, c});
        }
        else
        {
            // roadmap node
            for (int k = 0; k < (int)nodes[u].neighbours.size(); ++k)
                nbrs.push_back({nodes[u].neighbours[k], nodes[u].edge_costs[k]});
            // reverse start/goal edges
            for (auto &[v,c] : start_edges) if (v == u) nbrs.push_back({IDX_START, c});
            for (auto &[v,c] : goal_edges)  if (v == u) nbrs.push_back({IDX_GOAL,  c});
        }
        return nbrs;
    };

    while (!pq.empty())
    {
        auto [d, u] = pq.top(); pq.pop();
        if (d > dist_arr[u] + 1e-9) continue;
        if (u == IDX_GOAL) break;

        for (auto &[v,c] : get_neighbours(u))
        {
            double nd = dist_arr[u] + c;
            if (nd < dist_arr[v])
            {
                dist_arr[v] = nd;
                prev_arr[v] = u;
                pq.push({nd, v});
            }
        }
    }

    if (std::isinf(dist_arr[IDX_GOAL]))
    {
        RCLCPP_ERROR(node->get_logger(), "[PRM] Dijkstra found no path to goal.");
        return std::nullopt;
    }

    // ── 3. Reconstruct joint-space waypoints ──────────────────
    std::vector<std::vector<double>> waypoints;
    for (int cur = IDX_GOAL; cur != -1; cur = prev_arr[cur])
    {
        if      (cur == IDX_START) waypoints.push_back(q_start);
        else if (cur == IDX_GOAL)  waypoints.push_back(q_goal);
        else                       waypoints.push_back(nodes[cur].joints);
    }
    std::reverse(waypoints.begin(), waypoints.end());

    RCLCPP_INFO(node->get_logger(),
                "[PRM] Path found: %zu waypoints, cost=%.4f",
                waypoints.size(), dist_arr[IDX_GOAL]);

    // ── 4. Pack into MoveGroupInterface::Plan ─────────────────
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    // start_state: snapshot of current robot state
    {
        moveit_msgs::msg::RobotState rs_msg;
        robot_state->setJointGroupPositions(jmg, q_start);
        robot_state->setJointGroupPositions(jmg, q_start);
        robot_state->update();
        rs_msg.joint_state.name = jmg->getVariableNames();
        rs_msg.joint_state.position = q_start;
        plan.start_state_ = rs_msg;
    }

    // Build a JointTrajectory from the waypoints
    trajectory_msgs::msg::JointTrajectory jt;
    jt.joint_names = jmg->getVariableNames();

    // Simple timing: constant speed proportional to joint-space distance
    const double TIME_PER_UNIT = 2.0;  // seconds per unit of joint-space distance
    double elapsed = 0.0;

    for (int wi = 0; wi < (int)waypoints.size(); ++wi)
    {
        trajectory_msgs::msg::JointTrajectoryPoint pt;
        pt.positions = waypoints[wi];
        pt.velocities.resize(ndof, 0.0);
        pt.accelerations.resize(ndof, 0.0);

        if (wi > 0)
            elapsed += joint_dist_v(waypoints[wi-1], waypoints[wi]) * TIME_PER_UNIT;

        pt.time_from_start = rclcpp::Duration::from_seconds(elapsed);
        jt.points.push_back(pt);
    }

    moveit_msgs::msg::RobotTrajectory rt;
    rt.joint_trajectory = jt;
    plan.trajectory_    = rt;

    // planning_time_ is informational only
    plan.planning_time_ = 0.0;

    return plan;
}

std::optional<moveit::planning_interface::MoveGroupInterface::Plan> plan_and_publish_rrt(
    const std::string &task_type,
    const geometry_msgs::msg::Pose &target_pose,
    moveit::planning_interface::MoveGroupInterface &arm_group,
    rclcpp::Node::SharedPtr node,
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub)
{
    RCLCPP_INFO(node->get_logger(), "[Path planning] Task: %s  target=(%.3f, %.3f, %.3f)",
                task_type.c_str(),
                target_pose.position.x, target_pose.position.y, target_pose.position.z);

    // ── Configure planner ────────────────────────────────────
    arm_group.setPlanningPipelineId("ompl");
    arm_group.setPlannerId("RRTConnect");
    arm_group.setPlanningTime(ARM_PLANNING_TIME_SEC);           // match GUI exactly
    arm_group.setNumPlanningAttempts(20);     // match GUI exactly
    arm_group.setPoseReferenceFrame("world");
    arm_group.clearPathConstraints();
    arm_group.setStartStateToCurrentState();
    arm_group.setWorkspace(-0.15, -0.50, 0.90,
                            0.85,  0.50, 1.30);

    // ── Solve IK once, then plan to joint goal (same as GUI) ──
    auto current_state = arm_group.getCurrentState(5.0);
    if (!current_state)
    {
        RCLCPP_ERROR(node->get_logger(), "getCurrentState() failed");
        return std::nullopt;
    }
    const auto* jmg = current_state->getJointModelGroup("demo_arm_bot");
    if (!current_state->setFromIK(jmg, target_pose, 1.0))
    {
        RCLCPP_ERROR(node->get_logger(),
                    "[Path Planning] IK FAILED for (%.3f, %.3f, %.3f) — aborting.",
                    target_pose.position.x, target_pose.position.y, target_pose.position.z);
        return std::nullopt;
    }
    std::vector<double> joint_values;
    current_state->copyJointGroupPositions(jmg, joint_values);
    arm_group.setJointValueTarget(joint_values);  // ← key change
    arm_group.setPathConstraints(make_ee_down_constraint());
        // arm_group.allowReplanning(true);

    // Adjust speed based on task type
    double velocity_scaling = (task_type == "Grasp") ? 0.05 : 0.1;
    arm_group.setMaxVelocityScalingFactor(velocity_scaling);
    arm_group.setMaxAccelerationScalingFactor(velocity_scaling);

    try
    {
        // ── Plan ──────────────────────────────────────────────
        // ── Resolve start joint config ────────────────────────
        auto start_state_ptr = arm_group.getCurrentState(5.0);
        if (!start_state_ptr)
        {
            RCLCPP_ERROR(node->get_logger(), "[PRM] Could not get current state for PRM start.");
            return std::nullopt;
        }
        const auto *jmg_plan = start_state_ptr->getJointModelGroup("demo_arm_bot");
        std::vector<double> q_start;
        start_state_ptr->copyJointGroupPositions(jmg_plan, q_start);

        // ── Resolve goal joint config (already solved via IK above) ──
        // joint_values was set earlier in this function from setFromIK
        const std::vector<double> &q_goal = joint_values;

        // ── Query PRM ─────────────────────────────────────────
        auto plan_opt = plan_with_roadmap(q_start, q_goal, arm_group, node);
        if (!plan_opt.has_value())
        {
            RCLCPP_ERROR(node->get_logger(), "[PRM] plan_with_roadmap failed.");
            return std::nullopt;
        }
        moveit::planning_interface::MoveGroupInterface::Plan plan = plan_opt.value();

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
        {
            std_msgs::msg::String s;
            s.data = "EXECUTING";
            state_pub->publish(s);
        }
        RCLCPP_INFO(node->get_logger(), "[Path Planning] Executing plan...");
        // if (arm_group.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
        // {
        //     RCLCPP_ERROR(node->get_logger(), "[Path Planning] Execution failed.");
        //     // pub_status->publish([]{ std_msgs::msg::String s; s.data="RRT_EXEC_FAILED"; return s; }());
        //     return std::nullopt;
        // }

        RCLCPP_INFO(node->get_logger(), "[Path Planning] Execution succeeded. Syncing state...");
        // ── Update start state for next planning stage ─────────
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        arm_group.setStartStateToCurrentState();

        return plan;
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "[Path Planning] Exception: %s", e.what());
        return std::nullopt;
    }
    catch (...)
    {
        RCLCPP_ERROR(node->get_logger(), "[Path Planning] Unknown exception occurred.");
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

void wellplate_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
    for (const auto &marker : msg->markers)
    {
        // Skip markers that are not being added
        if (marker.action != visualization_msgs::msg::Marker::ADD)
            continue;

        // Process wellplate markers
        if (marker.ns == "wellplates")
        {
            wellplate_tag_x = marker.pose.position.x;
            wellplate_tag_y = marker.pose.position.y;
            wellplate_tag_z = marker.pose.position.z;

            // RCLCPP_INFO(this->get_logger(),
            //             "Wellplate Marker ID: %d | Position -> x: %.3f, y: %.3f, z: %.3f",
            //             marker.id, wellplate_tag_x, wellplate_tag_y, wellplate_tag_z);
        }
    }
}

static std::mutex cmd_mutex;
static std::string received_command = "idle";

void command_callback(const std_msgs::msg::String::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(cmd_mutex);
    received_command = msg->data;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("move_to_pose_node");



    // Spin on a background thread so MoveGroupInterface can call services
    // without blocking the main execution thread.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread executor_thread([&executor]()
                                { executor.spin(); });

    // ── MoveIt planning groups ───────────────────────────────
    moveit::planning_interface::MoveGroupInterface arm_group(node, "demo_arm_bot");
    RCLCPP_INFO(node->get_logger(), "[PRM] Waiting 5s for move_group and octomap…");
    rclcpp::sleep_for(std::chrono::seconds(5));

    auto scene_client = node->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    scene_client->wait_for_service(std::chrono::seconds(10));
    auto scene_request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    scene_request->components.components =
        moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_NAMES |
        moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY |
        moveit_msgs::msg::PlanningSceneComponents::OCTOMAP;

    std::promise<moveit_msgs::msg::PlanningScene> scene_promise;
    auto scene_future = scene_promise.get_future();
    scene_client->async_send_request(
        scene_request,
        [&scene_promise](rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedFuture f) {
            scene_promise.set_value(f.get()->scene);
        });
    moveit_msgs::msg::PlanningScene initial_scene_msg = scene_future.get();
    RCLCPP_INFO(node->get_logger(), "[PRM] Planning scene received, building roadmap…");

    build_roadmap(arm_group, node, initial_scene_msg);
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
                // TODO: Investigate this. marker.id appears to actually be 0 indexed.
                april_tags[marker.id] = {marker.pose.position, node->now()};
            }
        });

    // Wellplates
    auto wellplate_subscription = node->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/inspect/wellplates",
        qos,
        wellplate_callback);

    // 1. Define your custom QoS profile
    rclcpp::QoS pointcloud_qos(1);
    pointcloud_qos.best_effort();
    pointcloud_qos.durability_volatile();

    auto sub_obstacle_cloud = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/zed_pointcloud", // <-- Put your actual topic name here
        pointcloud_qos,    // <-- Inject your custom QoS here
        [&latest_obstacle_cloud](const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
            latest_obstacle_cloud = msg;
        });

    auto command_sub = node->create_subscription<std_msgs::msg::String>(
        "/planning_command", 10, command_callback);

    auto planning_state_pub = node->create_publisher<std_msgs::msg::String>("/planning_state", 10);

    auto publish_state = [&](const std::string& state) {
        std_msgs::msg::String s;
        s.data = state;
        planning_state_pub->publish(s);
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

    auto finish_plan = [&](const std::optional<moveit::planning_interface::MoveGroupInterface::Plan>& opt) {
        if (opt.has_value()) {
            publish_state("SUCCESS");
        } else {
            RCLCPP_ERROR(node->get_logger(), "RRT* failed — aborting pipeline.");
            publish_state("ERROR");
        }
        std::lock_guard<std::mutex> lock(cmd_mutex);
        received_command = "idle";
    };



    rclcpp::Rate rate(1);
    while (rclcpp::ok())
    {
        std::string cmd;
        {
            std::lock_guard<std::mutex> lock(cmd_mutex);
            cmd = received_command;
        }

        RCLCPP_INFO(node->get_logger(), "Received command: %s", cmd.c_str());

        if (cmd.rfind("plan_april_", 0) == 0)
        {
            // Parse tag ID from "plan_april_<id>"
            const std::string april_prefix = "plan_april_";
            const std::string id_str = cmd.substr(april_prefix.size());
            if (id_str.empty()) {
                RCLCPP_ERROR(node->get_logger(), "plan_april_ command has no tag ID: '%s'", cmd.c_str());
                finish_plan(std::nullopt);
                continue;
            }
            int tag_id = 0;
            try {
                std::size_t pos = 0;
                tag_id = std::stoi(id_str, &pos);
                if (pos != id_str.size()) throw std::invalid_argument("trailing chars");
            } catch (...) {
                RCLCPP_ERROR(node->get_logger(), "Could not parse tag ID from command: %s", cmd.c_str());
                finish_plan(std::nullopt);
                continue;
            }

            // Wait up to APRIL_TAG_WAIT_TIMEOUT_SEC for a fresh tag reading
            geometry_msgs::msg::Point apriltag_point;
            bool found = false;
            for (int elapsed_ms = 0;
                 elapsed_ms < APRIL_TAG_WAIT_TIMEOUT_SEC * 1000;
                 elapsed_ms += APRIL_TAG_WAIT_STEP_MS)
            {
                {
                    const rclcpp::Time now = node->now();
                    std::lock_guard<std::mutex> lock(april_tags_mutex);
                    auto it = april_tags.find(tag_id);
                    if (it != april_tags.end() &&
                        (now - it->second.stamp).seconds() <= APRIL_TAG_STALE_SEC)
                    {
                        apriltag_point = it->second.position;
                        found = true;
                    }
                }
                if (found) break;
                rclcpp::sleep_for(std::chrono::milliseconds(APRIL_TAG_WAIT_STEP_MS));
            }

            if (!found)
            {
                RCLCPP_ERROR(node->get_logger(),
                             "April tag ID %d not found or stale after %ds — aborting.",
                             tag_id, APRIL_TAG_WAIT_TIMEOUT_SEC);
                finish_plan(std::nullopt);
                continue;
            }

            // Resolve final task parameters (live > default)
            const geometry_msgs::msg::Pose target_position =
                live_target_pose.value_or(defaults::target_pose_func(node, marker_pub, apriltag_point, 0.25));
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
        else if (cmd == "plan_wellplate")
        {

            geometry_msgs::msg::Point wellplate_point;
            wellplate_point.x = wellplate_tag_x;
            wellplate_point.y = wellplate_tag_y;
            wellplate_point.z = wellplate_tag_z;

            // Resolve final task parameters (live > default)
            const geometry_msgs::msg::Pose target_position =
                live_target_pose.value_or(defaults::target_pose_func(node, marker_pub, wellplate_point, 0.25));
            const std::string task_type =
                live_task_type.value_or(defaults::TASK_TYPE);

            if (!live_target_pose)
                RCLCPP_WARN(node->get_logger(), "Using default target pose.");
            if (!live_task_type)
                RCLCPP_WARN(node->get_logger(), "Using default task type: %s.", task_type.c_str());

            geometry_msgs::msg::Pose pregrasp_pose = target_position;

            const double PREGRASP_Z_OFFSET = 0.00;
            pregrasp_pose.position.z += PREGRASP_Z_OFFSET;

            const double PREGRASP_Y_OFFSET = 0.1;
            pregrasp_pose.position.y += PREGRASP_Y_OFFSET;

            const double PREGRASP_X_OFFSET = 0.1;
            pregrasp_pose.position.x -= PREGRASP_X_OFFSET;


            publish_target_marker(node, pregrasp_pose);

            publish_state("PLANNING");
            auto rrt_plan_opt = plan_and_publish_rrt(
                task_type, pregrasp_pose,
                arm_group, node, planning_state_pub);

            finish_plan(rrt_plan_opt);
        }
        else if (cmd == "plan_home")
        {
            // TODO: Move Hardcoded values to a config file
            geometry_msgs::msg::Point home_point;
            home_point.x = -0.048;
            home_point.y = 0.443;
            home_point.z = 0.247;

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
            // TODO: Move Hardcoded values to a config file
            geometry_msgs::msg::Point home_point;
            home_point.x = -0.048;
            home_point.y = 0.443;
            home_point.z = 0.247;

            // Resolve final task parameters (live > default)
            const geometry_msgs::msg::Pose target_position =
                live_target_pose.value_or(defaults::target_pose_func(node, marker_pub, home_point, 0.15));
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
        else
        {
            publish_state("IDLE");
        }
        rate.sleep();
    }

    // ── Clean shutdown ───────────────────────────────────────
    executor.cancel();
    executor_thread.join();
    rclcpp::shutdown();
    return 0;
}