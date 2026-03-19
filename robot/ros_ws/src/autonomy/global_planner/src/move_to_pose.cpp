// ============================================================
//  move_to_pose.cpp
//
//  Single node for motion and trajectory planning.
//
//  Pipeline:
//    1. Wait INPUT_TIMEOUT_SEC for live perception topics
//    2. Fall back to hard-coded defaults if nothing arrives
//    3. Stage 1 — RRT* (OMPL): collision-free path to target
//    4. Stage 2 — CHOMP (optional, Move tasks only): smoother path
//    5. Grasp / Place / Attach logic depending on task type
//
// ┌─────────────────────────────────────────────────────────────┐
// │  SUBSCRIBED TOPICS                                           │
// │  /perception/object_pointcloud   sensor_msgs/PointCloud2     │
// │  /perception/obstacle_pointcloud sensor_msgs/PointCloud2     │
// │  /perception/target_pose         geometry_msgs/PoseStamped   │
// │  /perception/grasp_width         std_msgs/Float64            │
// │  /task_planner/task_type         std_msgs/String             │
// │  /joint_states                   sensor_msgs/JointState      │
// │                                                              │
// │  PUBLISHED TOPICS                                            │
// │  /planning/rrt_path              nav_msgs/Path               │
// │  /planning/chomp_trajectory      moveit_msgs/RobotTrajectory │
// │  /planning/status                std_msgs/String             │
// │  /planning/rrt_viz               visualization_msgs/Marker   │
// │  /planning/chomp_viz             visualization_msgs/Marker   │
// └─────────────────────────────────────────────────────────────┘
// ============================================================

// ── ROS / MoveIt includes ────────────────────────────────────
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
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

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// ── TF2 includes ─────────────────────────────────────────────
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// ── STL includes ─────────────────────────────────────────────
#include <thread>
#include <chrono>
#include <optional>
#include <string>
#include <fstream>
#include <iomanip>


// ╔═══════════════════════════════════════════════════════════╗
// ║  CONSTANTS                                                ║
// ╚═══════════════════════════════════════════════════════════╝

static constexpr double ARM_PLANNING_TIME_SEC     = 15.0;
static constexpr double GRIPPER_PLANNING_TIME_SEC = 2.0;
static constexpr int    INPUT_TIMEOUT_SEC         = 5;


// ╔═══════════════════════════════════════════════════════════╗
// ║  HARD-CODED DEFAULTS                                      ║
// ║  Used when no live perception data arrives in time        ║
// ╚═══════════════════════════════════════════════════════════╝

namespace defaults
{
    static const std::string TASK_TYPE = "Grasp";

    // Pre-grasp pose: EE above the wellplate centre, RPY=(0,0,π)
    // so the gripper faces down and the fingers straddle the plate.
    static geometry_msgs::msg::Pose target_pose()
    {
        geometry_msgs::msg::Pose p;
        p.position.x = 0.25;   // aligned with wellplate X
        p.position.y = -0.1;   // aligned with wellplate Y
        p.position.z = 0.88;   // high enough for open fingers to clear the plate

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, M_PI);   // gripper pointing down
        q.normalize();
        p.orientation = tf2::toMsg(q);
        return p;
    }

    // Place pose: inside the liquid handler
    static geometry_msgs::msg::Pose place_pose()
    {
        geometry_msgs::msg::Pose p;
        p.position.x = 0.50;
        p.position.y = 0.43;
        p.position.z = 0.94;


        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, M_PI / 2.0);
        q.normalize();
        p.orientation = tf2::toMsg(q);
        return p;
    }
}


// ╔═══════════════════════════════════════════════════════════╗
// ║  PLANNING SCENE HELPERS                                   ║
// ╚═══════════════════════════════════════════════════════════╝

// ── Flat table below the robot base ──────────────────────────
void add_collision_table(moveit::planning_interface::PlanningSceneInterface& psi)
{
    moveit_msgs::msg::CollisionObject table;
    table.header.frame_id = "world";
    table.id = "table";

    shape_msgs::msg::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions = {1.2, 1.2, 0.05};   // 1.2m × 1.2m × 5cm

    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = 0.75;
    pose.position.y = 0.0;
    pose.position.z = 0.715;   // table surface ≈ z=0.74

    table.primitives.push_back(box);
    table.primitive_poses.push_back(pose);
    table.operation = table.ADD;
    psi.applyCollisionObject(table);
}

// ── SBS-footprint wellplate sitting on the table ─────────────
void add_wellplate_object(moveit::planning_interface::PlanningSceneInterface& psi)
{
    moveit_msgs::msg::CollisionObject wellplate;
    wellplate.header.frame_id = "world";
    wellplate.id = "wellplate";

    shape_msgs::msg::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions = {0.127, 0.08545, 0.02};   // standard SBS footprint, 2cm tall

    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = 0.25;
    pose.position.y = -0.1;
    pose.position.z = 0.75;   // table surface (0.74) + half plate height (0.01)

    wellplate.primitives.push_back(box);
    wellplate.primitive_poses.push_back(pose);
    wellplate.operation = wellplate.ADD;
    psi.applyCollisionObject(wellplate);
}

// ── Hollow 5-sided box obstacle (fallback when no live cloud) ─
// Front face (facing the robot) is left open so the arm can enter.
void add_hollow_point_cloud_obstacle(moveit::planning_interface::PlanningSceneInterface& psi)
{
    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = "world";
    obj.id = "hollow_front_open_cube";

    // Each "point" is a small sphere
    shape_msgs::msg::SolidPrimitive sphere;
    sphere.type = sphere.SPHERE;
    sphere.dimensions = {0.02};   // 2 cm diameter

    const double cx = 0.7, cy = 0.35, cz = 1.02;   // cube centre
    const double s  = 0.25;                          // half-size (0.5m cube)
    const double res = 0.04;                         // point spacing

    auto add_pt = [&](double x, double y, double z)
    {
        geometry_msgs::msg::Pose p;
        p.position.x = x;  p.position.y = y;  p.position.z = z;
        p.orientation.w = 1.0;
        obj.primitives.push_back(sphere);
        obj.primitive_poses.push_back(p);
    };

    for (double i = -s; i <= s; i += res)
    {
        for (double j = -s; j <= s; j += res)
        {
            add_pt(cx + i, cy + j, cz - s);   // bottom face
            add_pt(cx + i, cy + j, cz + s);   // top face
            add_pt(cx + s, cy + i, cz + j);   // back face  (away from robot)
            add_pt(cx + i, cy - s, cz + j);   // left face
            add_pt(cx + i, cy + s, cz + j);   // right face
            // front face (cx - s) intentionally omitted
        }
    }

    obj.operation = obj.ADD;
    psi.applyCollisionObject(obj);
}


// ╔═══════════════════════════════════════════════════════════╗
// ║  ORIENTATION CONSTRAINT                                   ║
// ║  Keeps EE Z-axis pointing downward throughout motion.     ║
// ║  Z rotation is free (spinning the gripper is harmless).   ║
// ╚═══════════════════════════════════════════════════════════╝

moveit_msgs::msg::Constraints make_ee_down_constraint()
{
    moveit_msgs::msg::OrientationConstraint ocm;
    ocm.link_name       = "end_effector_p4_1";
    ocm.header.frame_id = "world";

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, M_PI);   // target orientation: RPY=(0,0,π)
    q.normalize();
    ocm.orientation = tf2::toMsg(q);

    ocm.absolute_x_axis_tolerance = 0.2;   // ≈11° tilt tolerance
    ocm.absolute_y_axis_tolerance = 0.2;   // ≈11° tilt tolerance
    ocm.absolute_z_axis_tolerance = M_PI;  // spin freely around Z
    ocm.weight = 1.0;

    moveit_msgs::msg::Constraints c;
    c.name = "ee_down";
    c.orientation_constraints.push_back(ocm);
    return c;
}


// ╔═══════════════════════════════════════════════════════════╗
// ║  RVIZ VISUALISATION HELPER                                ║
// ║  Converts a joint trajectory → LINE_STRIP marker via FK.  ║
// ╚═══════════════════════════════════════════════════════════╝

visualization_msgs::msg::Marker make_path_marker(
    const trajectory_msgs::msg::JointTrajectory& jt,
    moveit::core::RobotStatePtr                  robot_state,
    const moveit::core::JointModelGroup*         jmg,
    const std::string&                           ee_link,
    const std::string&                           ns,
    float r, float g, float b,
    rclcpp::Node::SharedPtr                      node)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id    = "world";
    marker.header.stamp       = node->now();
    marker.ns                 = ns;
    marker.id                 = 0;
    marker.type               = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action             = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x            = 0.01;   // line width in metres
    marker.color              = [&]{ std_msgs::msg::ColorRGBA c; c.r=r; c.g=g; c.b=b; c.a=1.0; return c; }();
    marker.lifetime           = rclcpp::Duration::from_seconds(3600);

    for (const auto& pt : jt.points)
    {
        robot_state->setJointGroupPositions(jmg, pt.positions);
        robot_state->updateLinkTransforms();
        const Eigen::Isometry3d& tf = robot_state->getGlobalLinkTransform(ee_link);

        geometry_msgs::msg::Point p;
        p.x = tf.translation().x();
        p.y = tf.translation().y();
        p.z = tf.translation().z();
        marker.points.push_back(p);
    }
    return marker;
}


// ╔═══════════════════════════════════════════════════════════╗
// ║  TRAJECTORY FILE EXPORT                                   ║
// ║  Writes time + joint positions to a plain-text file.      ║
// ╚═══════════════════════════════════════════════════════════╝

void save_trajectory_to_file(
    const moveit_msgs::msg::RobotTrajectory& traj,
    const std::string& filename)
{
    std::ofstream file(filename);
    file << std::fixed << std::setprecision(6);

    for (const auto& pt : traj.joint_trajectory.points)
    {
        double t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9;
        file << t << "  [";
        for (size_t i = 0; i < pt.positions.size(); ++i)
        {
            file << pt.positions[i];
            if (i + 1 < pt.positions.size()) file << " ";
        }
        file << "]\n";
    }
    file.close();
    std::cout << "Saved trajectory to " << filename << "\n";
}

// ╔═══════════════════════════════════════════════════════════╗
// ║  POSE ERROR LOGGER                                        ║
// ║  Compares actual EE pose vs goal pose after execution.    ║
// ║  Prints position error (mm) and orientation error (deg).  ║
// ╚═══════════════════════════════════════════════════════════╝

void log_pose_error(
    moveit::planning_interface::MoveGroupInterface& arm_group,
    const geometry_msgs::msg::Pose&                 goal_pose,
    const std::string&                              label,
    rclcpp::Node::SharedPtr                         node)
{
    auto actual = arm_group.getCurrentPose().pose;

    // ── Position error ───────────────────────────────────────
    double dx = actual.position.x - goal_pose.position.x;
    double dy = actual.position.y - goal_pose.position.y;
    double dz = actual.position.z - goal_pose.position.z;
    double pos_error_mm = std::sqrt(dx*dx + dy*dy + dz*dz) * 1000.0;

    // ── Orientation error ────────────────────────────────────
    tf2::Quaternion q_actual, q_goal;
    tf2::fromMsg(actual.orientation, q_actual);
    tf2::fromMsg(goal_pose.orientation, q_goal);

    // Angular difference between two quaternions
    tf2::Quaternion q_error = q_goal.inverse() * q_actual;
    double angle_error_deg  = std::abs(q_error.getAngle()) * 180.0 / M_PI;
    // Normalize to [0, 180]
    if (angle_error_deg > 180.0) angle_error_deg = 360.0 - angle_error_deg;

    RCLCPP_INFO(node->get_logger(),
        "[%s] Pose error — position: %.2f mm  orientation: %.2f deg",
        label.c_str(), pos_error_mm, angle_error_deg);
    RCLCPP_INFO(node->get_logger(),
        "[%s]   goal:   (%.4f, %.4f, %.4f)",
        label.c_str(),
        goal_pose.position.x, goal_pose.position.y, goal_pose.position.z);
    RCLCPP_INFO(node->get_logger(),
        "[%s]   actual: (%.4f, %.4f, %.4f)",
        label.c_str(),
        actual.position.x, actual.position.y, actual.position.z);
    RCLCPP_INFO(node->get_logger(),
        "[%s]   delta:  (%.4f, %.4f, %.4f) m",
        label.c_str(), dx, dy, dz);
}



// ╔═══════════════════════════════════════════════════════════╗
// ║  STAGE 1 — RRT* via OMPL                                  ║
// ║                                                           ║
// ║  • Runs IK sanity-check before planning                   ║
// ║  • Plans, executes, waits for state sync                  ║
// ║  • Publishes path + green RViz marker                     ║
// ║  • Returns the plan so Stage 2 can reuse the joint goal   ║
// ╚═══════════════════════════════════════════════════════════╝

std::optional<moveit::planning_interface::MoveGroupInterface::Plan> plan_and_publish_rrt(
    const std::string&                                                    task_type,
    const geometry_msgs::msg::Pose&                                       target_pose,
    const std::optional<double>&                                          live_grasp_width,
    moveit::planning_interface::MoveGroupInterface&                       arm_group,
    moveit::planning_interface::MoveGroupInterface&                       gripper_group,
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr&                    pub_rrt_path,
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr&                  pub_status,
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr&        pub_rrt_viz,
    rclcpp::Node::SharedPtr                                               node)
{
    RCLCPP_INFO(node->get_logger(), "[Path planning] Task: %s  target=(%.3f, %.3f, %.3f)",
        task_type.c_str(),
        target_pose.position.x, target_pose.position.y, target_pose.position.z);

    // ── Configure planner ────────────────────────────────────
    arm_group.setPlanningPipelineId("ompl");
    arm_group.setPlannerId("demo_arm_bot");
    arm_group.setPlanningTime(ARM_PLANNING_TIME_SEC);
    arm_group.setPoseReferenceFrame("world");
    arm_group.setPathConstraints(make_ee_down_constraint());
    arm_group.setStartStateToCurrentState();
    arm_group.setJointValueTarget(target_pose);
    
    // If task type is "Grasp" , then we'll constrain velocity and acceleration to encourage a slower, more deliberate approach. For "Move", we can be more aggressive. 
    if (task_type == "Grasp") {
        arm_group.setMaxVelocityScalingFactor(0.2);      // 0.0 to 1.0 — 1.0 = full speed
        arm_group.setMaxAccelerationScalingFactor(0.2);  // 0.0 to 1.0 — 1.0 = full a
    }
    else  {
    arm_group.setMaxVelocityScalingFactor(0.4);      // 0.0 to 1.0 — 1.0 = full speed
    arm_group.setMaxAccelerationScalingFactor(0.2);  // 0.0 to 1.0 — 1.0 = full a
    }
    // ── IK sanity-check (fast, no collision check) ──────────
    // If IK fails here the planner will fail too — bail early.
    moveit::core::RobotStatePtr test_state = arm_group.getCurrentState(5.0);
    const auto* ik_jmg = test_state->getJointModelGroup("demo_arm_bot");
    if (!test_state->setFromIK(ik_jmg, target_pose, 5.0))
    {
        RCLCPP_ERROR(node->get_logger(),
            "[Path Planning] IK FAILED for (%.3f, %.3f, %.3f) — aborting.",
            target_pose.position.x, target_pose.position.y, target_pose.position.z);
        pub_status->publish([]{ std_msgs::msg::String s; s.data="PATH_PLANNING_FAILED_IK"; return s; }());
        return std::nullopt;
    }
    RCLCPP_INFO(node->get_logger(), "[Path Planning] IK check passed.");

    // ── Plan ─────────────────────────────────────────────────
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (arm_group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "[Path Planning] Planning failed.");
        pub_status->publish([]{ std_msgs::msg::String s; s.data="PATH_PLANNING_FAILED"; return s; }());
        return std::nullopt;
    }

    // ── Publish green RViz path marker ───────────────────────
    {
        moveit::core::RobotStatePtr viz_state = arm_group.getCurrentState(5.0);
        const auto* jmg    = viz_state->getJointModelGroup(arm_group.getName());
        const auto  ee_lnk = arm_group.getEndEffectorLink();
        pub_rrt_viz->publish(make_path_marker(
            plan.trajectory_.joint_trajectory, viz_state, jmg, ee_lnk,
            "rrt_path", 0.0f, 1.0f, 0.0f, node));
        RCLCPP_INFO(node->get_logger(), "[Path Planning] RViz marker published (green).");
    }

    // ── Execute ──────────────────────────────────────────────
    RCLCPP_INFO(node->get_logger(), "[Path Planning] Executing plan...");
    if (arm_group.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "[Path Planning] Execution failed.");
        pub_status->publish([]{ std_msgs::msg::String s; s.data="RRT_EXEC_FAILED"; return s; }());
        return std::nullopt;
    }

    // Give the Planning Scene Monitor time to sync joint states from /joint_states
    RCLCPP_INFO(node->get_logger(), "[Path Planning] Execution succeeded. Syncing state...");
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    arm_group.setStartStateToCurrentState();
    gripper_group.setStartStateToCurrentState();
    
    log_pose_error(arm_group, target_pose, "Path Planning", node);

    // ── Publish nav_msgs/Path (Cartesian waypoints) ──────────
    nav_msgs::msg::Path rrt_path;
    rrt_path.header.stamp    = node->now();
    rrt_path.header.frame_id = "world";
    for (const auto& pt : plan.trajectory_.joint_trajectory.points)
    {
        geometry_msgs::msg::PoseStamped wp;
        wp.header = rrt_path.header;
        wp.pose   = target_pose;   // FK not resolved here; use target as proxy
        rrt_path.poses.push_back(wp);
    }
    pub_rrt_path->publish(rrt_path);
    RCLCPP_INFO(node->get_logger(), "[Path Planning] Path published (%zu waypoints).", rrt_path.poses.size());

    pub_status->publish([]{ std_msgs::msg::String s; s.data="PATH_PLANNING_SUCCESS"; return s; }());
    return plan;
}


// ╔═══════════════════════════════════════════════════════════╗
// ║  STAGE 2 — CHOMP (optional, Move tasks only)              ║
// ║                                                           ║
// ║  CHOMP only accepts joint-space goals.                    ║
// ║  We reuse the final joint state from the RRT plan         ║
// ║  so no separate IK call is needed.                        ║
// ║                                                           ║
// ║  • Plans from current state to RRT's goal joints          ║
// ║  • Executes + saves trajectory to file                    ║
// ║  • Publishes trajectory + blue RViz marker                ║
// ╚═══════════════════════════════════════════════════════════╝

void plan_and_publish_chomp(
    const std::string&                                                    task_type,
    const moveit::planning_interface::MoveGroupInterface::Plan&           rrt_plan,
    moveit::planning_interface::MoveGroupInterface&                       arm_group,
    moveit::planning_interface::MoveGroupInterface&                       gripper_group,
    rclcpp::Publisher<moveit_msgs::msg::RobotTrajectory>::SharedPtr&      pub_chomp_traj,
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr&                  pub_status,
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr&        pub_chomp_viz,
    rclcpp::Node::SharedPtr                                               node)
{
    RCLCPP_INFO(node->get_logger(), "[CHOMP] Task: %s", task_type.c_str());

    // ── Extract joint goal from last RRT waypoint ────────────
    const auto& rrt_points = rrt_plan.trajectory_.joint_trajectory.points;
    if (rrt_points.empty())
    {
        RCLCPP_ERROR(node->get_logger(), "[CHOMP] RRT plan is empty — cannot extract goal joints.");
        pub_status->publish([]{ std_msgs::msg::String s; s.data="CHOMP_FAILED_NO_RRT"; return s; }());
        return;
    }
    const std::vector<double> goal_joints = rrt_points.back().positions;
    RCLCPP_INFO(node->get_logger(), "[CHOMP] Goal: %zu joints from RRT end-state.", goal_joints.size());

    // ── Configure CHOMP ──────────────────────────────────────
    // NOTE: Reset pipeline to OMPL after this function returns (CHOMP state persists).
    arm_group.setPlanningPipelineId("chomp");
    arm_group.setPlannerId("chomp");
    arm_group.setPlanningTime(ARM_PLANNING_TIME_SEC);
    arm_group.clearPathConstraints();   // CHOMP ignores path constraints
    arm_group.setStartStateToCurrentState();
    arm_group.setJointValueTarget(goal_joints);

    // ── Plan ─────────────────────────────────────────────────
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (arm_group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "[CHOMP] Planning failed.");
        pub_status->publish([]{ std_msgs::msg::String s; s.data="CHOMP_FAILED"; return s; }());
        // Reset pipeline so callers aren't left with chomp as the active pipeline
        arm_group.setPlanningPipelineId("ompl");
        arm_group.setPlannerId("demo_arm_bot");
        return;
    }

    // ── Execute + save ───────────────────────────────────────
    RCLCPP_INFO(node->get_logger(), "[CHOMP] Executing plan...");
    save_trajectory_to_file(plan.trajectory_, "chomp_trajectory.txt");
    arm_group.execute(plan);

    // ── Publish trajectory message ───────────────────────────
    pub_chomp_traj->publish(plan.trajectory_);
    RCLCPP_INFO(node->get_logger(), "[CHOMP] Trajectory published (%zu points).",
        plan.trajectory_.joint_trajectory.points.size());

    // ── Publish blue RViz path marker ────────────────────────
    {
        moveit::core::RobotStatePtr viz_state = arm_group.getCurrentState(5.0);
        const auto* jmg    = viz_state->getJointModelGroup(arm_group.getName());
        const auto  ee_lnk = arm_group.getEndEffectorLink();
        pub_chomp_viz->publish(make_path_marker(
            plan.trajectory_.joint_trajectory, viz_state, jmg, ee_lnk,
            "chomp_path", 0.0f, 0.4f, 1.0f, node));
        RCLCPP_INFO(node->get_logger(), "[CHOMP] RViz marker published (blue).");
    }

    // Reset pipeline back to OMPL for any subsequent planning calls
    arm_group.setPlanningPipelineId("ompl");
    arm_group.setPlannerId("demo_arm_bot");

    pub_status->publish([]{ std_msgs::msg::String s; s.data="CHOMP_SUCCESS"; return s; }());
}


// ╔═══════════════════════════════════════════════════════════╗
// ║  GRASP HELPERS                                            ║
// ╚═══════════════════════════════════════════════════════════╝

// ── Open the gripper to full width before approaching ────────
void open_gripper(
    moveit::planning_interface::MoveGroupInterface& gripper_group,
    rclcpp::Node::SharedPtr node)
{
    RCLCPP_INFO(node->get_logger(), "[Gripper] Opening...");
    std::map<std::string, double> joints;
    joints["Slider_1"] = 0.05;
    joints["Slider_2"] = 0.05;
    gripper_group.setJointValueTarget(joints);
    gripper_group.move();
}

// ── Close the gripper to the specified width ─────────────────
// width is the total gap between fingers (metres).
void close_gripper(
    moveit::planning_interface::MoveGroupInterface& gripper_group,
    double width,
    rclcpp::Node::SharedPtr node)
{
    const double slider_val = (width-0.015) / 2.0;
    RCLCPP_INFO(node->get_logger(), 
        "[Gripper] Closing to width=%.4f m (slider=%.4f).", width, slider_val);
    
    gripper_group.setStartStateToCurrentState();
    std::map<std::string, double> joints;
    joints["Slider_1"] = slider_val;
    joints["Slider_2"] = slider_val;
    gripper_group.setJointValueTarget(joints);

    bool success = (gripper_group.move() == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (!success)
        RCLCPP_ERROR(node->get_logger(), "[Gripper] Close FAILED!");
    else
        RCLCPP_INFO(node->get_logger(), "[Gripper] Closed successfully.");
}

// ── Attach the wellplate to the EE in the planning scene ─────
// Must be called BEFORE closing the gripper so MoveIt doesn't
// flag the finger↔plate contact as a collision.
void attach_wellplate(
    moveit::planning_interface::PlanningSceneInterface& psi,
    rclcpp::Node::SharedPtr node)
{
    RCLCPP_INFO(node->get_logger(), "[Scene] Attaching wellplate to end_effector_p4_1.");

    moveit_msgs::msg::AttachedCollisionObject obj;
    obj.link_name              = "end_effector_p4_1";
    obj.object.id              = "wellplate";
    obj.object.header.frame_id = "end_effector_p4_1";
    // All links that may legally contact the plate during carry
    obj.touch_links = {
        "end_effector_p4_1",
        "Slider_1", "Slider_2",
        "Finger_left_2", "Finger_right_2"
    };
    obj.object.operation = moveit_msgs::msg::CollisionObject::ADD;
    psi.applyAttachedCollisionObject(obj);
}

// ── Detach the wellplate from the EE ─────────────────────────
void detach_wellplate(
    moveit::planning_interface::PlanningSceneInterface& psi,
    rclcpp::Node::SharedPtr node)
{
    RCLCPP_INFO(node->get_logger(), "[Scene] Detaching wellplate.");
    moveit_msgs::msg::AttachedCollisionObject obj;
    obj.object.id        = "wellplate";
    obj.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    psi.applyAttachedCollisionObject(obj);
}


// ╔═══════════════════════════════════════════════════════════╗
// ║  PLACE MOTION                                             ║
// ║  Plans + executes an OMPL move to the place pose,         ║
// ║  then opens the gripper and detaches the object.          ║
// ╚═══════════════════════════════════════════════════════════╝

void execute_place(
    const geometry_msgs::msg::Pose&                     place_pose,
    moveit::planning_interface::MoveGroupInterface&     arm_group,
    moveit::planning_interface::MoveGroupInterface&     gripper_group,
    moveit::planning_interface::PlanningSceneInterface& psi,
    rclcpp::Node::SharedPtr                             node)
{
    RCLCPP_INFO(node->get_logger(), "[Place] Planning to (%.3f, %.3f, %.3f)...",
        place_pose.position.x, place_pose.position.y, place_pose.position.z);

    // Always reset pipeline to OMPL before place (CHOMP may have left it dirty)
    arm_group.setPlanningPipelineId("ompl");
    arm_group.setPlannerId("demo_arm_bot");
    arm_group.setStartStateToCurrentState();
    arm_group.setPoseTarget(place_pose);
    arm_group.setPathConstraints(make_ee_down_constraint());
    arm_group.setPlanningTime(ARM_PLANNING_TIME_SEC);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (arm_group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "[Place] Planning failed!");
        return;
    }

    arm_group.execute(plan);
    RCLCPP_INFO(node->get_logger(), "[Place] Reached place position.");

    open_gripper(gripper_group, node);
    detach_wellplate(psi, node);
}


// ╔═══════════════════════════════════════════════════════════╗
// ║  LOGGING HELPER                                           ║
// ║  Prints joint state to the console at startup.            ║
// ╚═══════════════════════════════════════════════════════════╝

void log_joint_state(
    const sensor_msgs::msg::JointState::SharedPtr& js,
    rclcpp::Node::SharedPtr node)
{
    if (!js) {
        RCLCPP_WARN(node->get_logger(), "No joint state received yet on /joint_states.");
        return;
    }
    RCLCPP_INFO(node->get_logger(), "Current joint state (%zu joints):", js->name.size());
    for (size_t i = 0; i < js->name.size(); ++i)
        RCLCPP_INFO(node->get_logger(), "  %-30s pos=%.4f  vel=%.4f",
            js->name[i].c_str(),
            js->position.size() > i ? js->position[i] : 0.0,
            js->velocity.size() > i ? js->velocity[i] : 0.0);
}

void log_ee_pose(
    moveit::planning_interface::MoveGroupInterface& arm_group,
    rclcpp::Node::SharedPtr node)
{
    auto ee = arm_group.getCurrentPose();
    RCLCPP_INFO(node->get_logger(), "Current EE pose:");
    RCLCPP_INFO(node->get_logger(), "  position:    x=%.4f  y=%.4f  z=%.4f",
        ee.pose.position.x, ee.pose.position.y, ee.pose.position.z);
    RCLCPP_INFO(node->get_logger(), "  orientation: x=%.4f  y=%.4f  z=%.4f  w=%.4f",
        ee.pose.orientation.x, ee.pose.orientation.y,
        ee.pose.orientation.z, ee.pose.orientation.w);

    tf2::Quaternion q;
    tf2::fromMsg(ee.pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    RCLCPP_INFO(node->get_logger(), "  RPY (deg):   r=%.2f  p=%.2f  y=%.2f",
        roll * 180.0/M_PI, pitch * 180.0/M_PI, yaw * 180.0/M_PI);
}
// ╔═══════════════════════════════════════════════════════════╗
// ║  CARTESIAN MOTION HELPER                                  ║
// ║  Moves EE in a straight line to target_pose.              ║
// ║  Returns false if less than min_fraction achieved.        ║
// ╚═══════════════════════════════════════════════════════════╝

bool execute_cartesian_motion(
    moveit::planning_interface::MoveGroupInterface& arm_group,
    const geometry_msgs::msg::Pose&                 target_pose,
    double                                          eef_step,
    double                                          min_fraction,
    const std::string&                              label,
    rclcpp::Node::SharedPtr                         node)
{
    std::vector<geometry_msgs::msg::Pose> waypoints = {target_pose};

    moveit_msgs::msg::RobotTrajectory traj;
    arm_group.clearPathConstraints();
    double fraction = arm_group.computeCartesianPath(
        waypoints, eef_step, 0.0, traj);

    RCLCPP_INFO(node->get_logger(),
        "[%s] Cartesian path: %.1f%% achieved.", label.c_str(), fraction * 100.0);

    if (fraction < min_fraction) {
        RCLCPP_ERROR(node->get_logger(),
            "[%s] Cartesian path below minimum (%.0f%%) — aborting.",
            label.c_str(), min_fraction * 100.0);
        return false;
    }

    if (arm_group.execute(traj) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), 
            "[%s] Cartesian execution failed.", label.c_str());
        return false;
    }

    rclcpp::sleep_for(std::chrono::milliseconds(300));
    arm_group.setStartStateToCurrentState();
    log_pose_error(arm_group, target_pose, label, node);
    return true;
}



// ╔═══════════════════════════════════════════════════════════╗
// ║  MAIN                                                     ║
// ╚═══════════════════════════════════════════════════════════╝

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("move_to_pose_node");

    // Spin on a background thread so MoveGroupInterface can call services
    // without blocking the main execution thread.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread executor_thread([&executor]() { executor.spin(); });


    // ── MoveIt planning groups ───────────────────────────────
    moveit::planning_interface::MoveGroupInterface arm_group(node, "demo_arm_bot");
    moveit::planning_interface::MoveGroupInterface gripper_group(node, "demo_gripper_bot");
    moveit::planning_interface::PlanningSceneInterface psi;

    arm_group.setPoseReferenceFrame("world");
    arm_group.setPlanningTime(ARM_PLANNING_TIME_SEC);
    gripper_group.setPlanningTime(GRIPPER_PLANNING_TIME_SEC);

    // ── Populate planning scene ──────────────────────────────
    add_collision_table(psi);
    add_wellplate_object(psi);
    // Wait for the Planning Scene Monitor to register the objects
    // before any planning starts (avoids race condition).
    rclcpp::sleep_for(std::chrono::seconds(3));

    // Log startup EE pose for debugging
    log_ee_pose(arm_group, node);


    // ── Publishers ───────────────────────────────────────────
    auto pub_rrt_path   = node->create_publisher<nav_msgs::msg::Path>("/planning/rrt_path", 10);
    auto pub_chomp_traj = node->create_publisher<moveit_msgs::msg::RobotTrajectory>("/planning/chomp_trajectory", 10);
    auto pub_status     = node->create_publisher<std_msgs::msg::String>("/planning/status", 10);
    auto pub_rrt_viz    = node->create_publisher<visualization_msgs::msg::Marker>("/planning/rrt_viz", 10);
    auto pub_chomp_viz  = node->create_publisher<visualization_msgs::msg::Marker>("/planning/chomp_viz", 10);
    // Unused in current code; placeholder for future obstacle cloud publishing
    auto pub_dummy_cloud = node->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/dummy_obstacle_cloud", 10);


    // ── Live input storage (filled by subscribers below) ────
    std::optional<geometry_msgs::msg::Pose> live_target_pose;
    std::optional<std::string>              live_task_type;
    std::optional<double>                   live_grasp_width;
    sensor_msgs::msg::JointState::SharedPtr live_joint_state;
    sensor_msgs::msg::PointCloud2::SharedPtr latest_obstacle_cloud;


    // ── Subscribers ──────────────────────────────────────────

    // Object point cloud — forwarded to grasp-pose estimator (TODO)
    auto sub_object_cloud = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/perception/object_pointcloud",
        rclcpp::QoS(1).durability_volatile().best_effort(),
        [&node](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            RCLCPP_DEBUG(node->get_logger(), "Object cloud: %u×%u pts", msg->width, msg->height);
        });

    // Obstacle point cloud — stored for optional planning scene injection
    auto sub_obstacle_cloud = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/perception/obstacle_pointcloud",
        rclcpp::QoS(1).best_effort(),
        [&latest_obstacle_cloud](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            latest_obstacle_cloud = msg;
        });

    // Target pose from perception pipeline
    auto sub_target_pose = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/perception/target_pose",
        rclcpp::QoS(10).reliable(),
        [&live_target_pose, &node](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            live_target_pose = msg->pose;
            RCLCPP_INFO(node->get_logger(), "Target pose received: (%.3f, %.3f, %.3f)",
                msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        });

    // Grasp width from perception (total gap between fingers in metres)
    auto sub_grasp_width = node->create_subscription<std_msgs::msg::Float64>(
        "/perception/grasp_width",
        rclcpp::QoS(10).reliable(),
        [&live_grasp_width, &node](const std_msgs::msg::Float64::SharedPtr msg) {
            live_grasp_width = msg->data;
            RCLCPP_INFO(node->get_logger(), "Grasp width received: %.3f m", msg->data);
        });

    // Task type: "Move" | "Grasp" | "Place"
    auto sub_task_type = node->create_subscription<std_msgs::msg::String>(
        "/task_planner/task_type",
        rclcpp::QoS(10).reliable(),
        [&live_task_type, &node](const std_msgs::msg::String::SharedPtr msg) {
            live_task_type = msg->data;
            RCLCPP_INFO(node->get_logger(), "Task type received: %s", msg->data.c_str());
        });

    // Joint state — MoveIt reads /joint_states internally, but we also store
    // it here for logging and any custom validation logic.
    auto sub_joint_state = node->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states",
        rclcpp::QoS(10).reliable(),
        [&live_joint_state](const sensor_msgs::msg::JointState::SharedPtr msg) {
            live_joint_state = msg;
        });


    // ── Wait for perception input ────────────────────────────
    RCLCPP_INFO(node->get_logger(), "Waiting %d s for perception topics...", INPUT_TIMEOUT_SEC);
    std::this_thread::sleep_for(std::chrono::seconds(INPUT_TIMEOUT_SEC));

    // If no live obstacle cloud arrived, use the hollow-box fallback
    if (latest_obstacle_cloud) {
        RCLCPP_INFO(node->get_logger(), "Live obstacle cloud detected.");
    } else {
        RCLCPP_WARN(node->get_logger(), "No obstacle cloud — adding hollow fallback obstacle.");
        add_hollow_point_cloud_obstacle(psi);
    }

    // Resolve final task parameters (live > default)
    const geometry_msgs::msg::Pose target_pose =
        live_target_pose.value_or(defaults::target_pose());
    const std::string task_type =
        live_task_type.value_or(defaults::TASK_TYPE);

    if (!live_target_pose) RCLCPP_WARN(node->get_logger(), "Using default target pose.");
    if (!live_task_type)   RCLCPP_WARN(node->get_logger(), "Using default task type: %s.", task_type.c_str());

    log_joint_state(live_joint_state, node);


    // ╔═══════════════════════════════════════════════════════╗
    // ║  EXECUTION PIPELINE                                   ║
    // ╚═══════════════════════════════════════════════════════╝

    // ── Pre-grasp: open gripper ──────────────────────────────
    if (task_type == "Grasp")
        open_gripper(gripper_group, node);

    // ── Stage 1: RRT* → pre-grasp pose (offset above target) ─
    const double PREGRASP_Z_OFFSET = 0.07;   // tune this
    const double POSTGRASP_Z_OFFSET = 0.03;  // extra lift after grasping (tune if needed)

    geometry_msgs::msg::Pose pregrasp_pose = target_pose;
    pregrasp_pose.position.z += PREGRASP_Z_OFFSET;
    geometry_msgs::msg::Pose postgrasp_pose = target_pose;
    postgrasp_pose.position.z += POSTGRASP_Z_OFFSET;

    auto rrt_plan_opt = plan_and_publish_rrt(
        task_type, pregrasp_pose, live_grasp_width,   // ← offset pose
        arm_group, gripper_group,
        pub_rrt_path, pub_status, pub_rrt_viz, node);


    if (!rrt_plan_opt.has_value())
    {
        RCLCPP_ERROR(node->get_logger(), "RRT* failed — aborting pipeline.");
        executor.cancel();
        executor_thread.join();
        rclcpp::shutdown();
        return 1;
    }

    // ── Stage 2: CHOMP (Move tasks only) ────────────────────
    // CHOMP smooths the RRT path. Skipped for Grasp/Place because
    // the arm is already at the target after Stage 1.
    if (task_type == "Move" || task_type == "Place")
    {
        // plan_and_publish_chomp(
        //     task_type, rrt_plan_opt.value(),
        //     arm_group, gripper_group,
        //     pub_chomp_traj, pub_status, pub_chomp_viz, node);
    }


    if (task_type == "Grasp")
    {
        const double PLATE_WIDTH  = 0.08545;  // SBS footprint Y dimension
        const double grasp_width  = live_grasp_width.value_or(PLATE_WIDTH); 

    // ── Cartesian descent → actual target_pose ───────────
        if (!execute_cartesian_motion(
                arm_group, target_pose,        // ← original pose, no offset
                0.005, 0.95, "Descent", node)) {
            executor.cancel();
            executor_thread.join();
            rclcpp::shutdown();
            return 1;
        }

        close_gripper(gripper_group, grasp_width, node);
        // ADD THIS — abort if gripper failed to close
        // Check by reading back actual joint state
        auto current_state = gripper_group.getCurrentState(5.0);
        std::vector<double> gripper_positions;
        current_state->copyJointGroupPositions(
            current_state->getJointModelGroup("demo_gripper_bot"), 
            gripper_positions);

        const double expected_slider = grasp_width / 2.0;
        const double actual_slider   = gripper_positions[0];
        const double tolerance       = 0.005;   // 5mm

        // if (std::abs(actual_slider - expected_slider) > tolerance) {
        //     RCLCPP_ERROR(node->get_logger(), 
        //         "[Grasp] Gripper did not reach target (expected=%.4f actual=%.4f) — aborting.",
        //         expected_slider, actual_slider);
        //     executor.cancel();
        //     executor_thread.join();
        //     rclcpp::shutdown();
        //     return 1;
        // }

                // ── Step 5: Attach NOW ───────────────────────────────────
        // Plate is clear of table — no table↔plate collision in scene.
        // touch_links handles finger↔plate contact during carry.
        attach_wellplate(psi, node);
        rclcpp::sleep_for(std::chrono::milliseconds(500));   // let PSM register the attach

        // ── Cartesian lift → back to pre-grasp height ────────
        if (!execute_cartesian_motion(
                arm_group, postgrasp_pose,      // ← same offset pose
                0.005, 0.95, "Lift", node)) {
            executor.cancel();
            executor_thread.join();
            rclcpp::shutdown();
            return 1;
        }


    }

    // ── Place: carry object to destination ──────────────────────
    if (task_type == "Grasp" || task_type == "Place")
    {
        // Why separate place function? Use the same rrt function call
        rrt_plan_opt = plan_and_publish_rrt(
            task_type, defaults::place_pose(), live_grasp_width,
            arm_group, gripper_group,
            pub_rrt_path, pub_status, pub_rrt_viz, node);
        if (!rrt_plan_opt.has_value())
        {
            RCLCPP_ERROR(node->get_logger(), "RRT* failed — aborting pipeline.");
            executor.cancel();
            executor_thread.join();
            rclcpp::shutdown();
            return 1;
        }
        // execute_place(defaults::place_pose(), arm_group, gripper_group, psi, node);
    }



    // ── Clean shutdown ───────────────────────────────────────
    executor.cancel();
    executor_thread.join();
    rclcpp::shutdown();
    return 0;
}