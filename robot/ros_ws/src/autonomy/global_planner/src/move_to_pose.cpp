// ============================================================
//  move_to_pose.cpp
//
//  Single node for motion and trajectory planning.
//  Subscribes to perception / task-planner topics and produces
//  RRT* (OMPL) paths and CHOMP trajectories for the planning
//  subsystem. Falls back to hard-coded defaults if no live input
//  arrives within INPUT_TIMEOUT_SEC.
//
// ┌─────────────────────────────────────────────────────────────┐
// │  SUBSCRIBED TOPICS                                           │
// │  /perception/object_pointcloud   sensor_msgs/PointCloud2     │
// │  /perception/obstacle_pointcloud sensor_msgs/PointCloud2     │
// │  /perception/target_pose         geometry_msgs/PoseStamped   │
// │  /task_planner/task_type         std_msgs/String             │
// │                                                              │
// │  PUBLISHED TOPICS                                            │
// │  /planning/rrt_path          nav_msgs/Path               │
// │  /planning/chomp_trajectory  moveit_msgs/RobotTrajectory │
// │  /planning/status            std_msgs/String             │
// └─────────────────────────────────────────────────────────────┘
// ============================================================

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <thread>
#include <chrono>
#include <optional>
#include <string>

#include <fstream>
#include <iomanip>

void saveTrajectoryToFile(
    const moveit_msgs::msg::RobotTrajectory &traj,
    const std::string &filename)
{
    std::ofstream file(filename);

    file << std::fixed << std::setprecision(6);

    const auto &jt = traj.joint_trajectory;

    for (const auto &point : jt.points)
    {
        double t =
            point.time_from_start.sec +
            point.time_from_start.nanosec * 1e-9;

        file << t << "  [";

        for (size_t i = 0; i < point.positions.size(); ++i)
        {
            file << point.positions[i];

            if (i != point.positions.size() - 1)
                file << " ";
        }

        file << "]\n";
    }

    file.close();

    std::cout << "Saved trajectory to " << filename << std::endl;
}
// ──────────────────────────────────────────────────────────────
//  Constants
// ──────────────────────────────────────────────────────────────
static constexpr double ARM_PLANNING_TIME_SEC     = 15.0;
static constexpr double GRIPPER_PLANNING_TIME_SEC = 2.0;
static constexpr int    INPUT_TIMEOUT_SEC         = 5;

// ──────────────────────────────────────────────────────────────
//  Hard-coded defaults
//  Used when no live perception data arrives within the timeout
// ──────────────────────────────────────────────────────────────




namespace defaults
{
    static const std::string TASK_TYPE = "Move";

    static geometry_msgs::msg::Pose target_pose()
    {
        geometry_msgs::msg::Pose p;
        p.position.x = 0.4282039701938629;
        p.position.y = 0.0022386356722563505;
        p.position.z = 0.023787112906575203;

        // Matches interactive marker exactly: z=1, w≈0 → RPY=(0, 0, PI)
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, M_PI);
        q.normalize();
        p.orientation = tf2::toMsg(q);
        return p;
    }
}

// ──────────────────────────────────────────────────────────────
//  Orientation constraint
//  Keeps end-effector Z-axis pointing downward throughout motion
// ──────────────────────────────────────────────────────────────
moveit_msgs::msg::Constraints make_ee_down_constraint()
{
    moveit_msgs::msg::OrientationConstraint ocm;
    ocm.link_name       = "end_effector_p4_1";
    ocm.header.frame_id = "world";

    // Exactly matches target — RPY=(0, 0, PI)
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, M_PI);
    q.normalize();
    ocm.orientation = tf2::toMsg(q);

    ocm.absolute_x_axis_tolerance = 0.2;  // ~3° — LOCKED, no tilt
    ocm.absolute_y_axis_tolerance = 0.2;  // ~3° — LOCKED, no tilt
    ocm.absolute_z_axis_tolerance = M_PI;  // free — spinning won't spill
    ocm.weight = 1.0;

    moveit_msgs::msg::Constraints c;
    c.name = "make_ee_down_constraint";
    c.orientation_constraints.push_back(ocm);
    return c;
}


// ──────────────────────────────────────────────────────────────
//  Build a LINE_STRIP marker from a joint trajectory.
//  Uses MoveIt FK to convert each joint-space point → Cartesian.
//  color_r/g/b in [0,1].  ns = marker namespace string.
// ──────────────────────────────────────────────────────────────
visualization_msgs::msg::Marker make_path_marker(
    const trajectory_msgs::msg::JointTrajectory          &jt,
    moveit::core::RobotStatePtr                           robot_state,
    const moveit::core::JointModelGroup                  *jmg,
    const std::string                                    &ee_link,
    const std::string                                    &ns,
    float r, float g, float b,
    rclcpp::Node::SharedPtr                               node)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp    = node->now();
    marker.ns              = ns;
    marker.id              = 0;
    marker.type            = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action          = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.01;   // line width in metres
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;
    marker.lifetime = rclcpp::Duration::from_seconds(3600); // 1 hour;   // persist until overwritten

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

// ──────────────────────────────────────────────────────────────
//  Stage 1 — RRT* via OMPL
//  Publishes coarse collision-free path to /planning/rrt_path
//  Returns the plan so CHOMP can reuse the goal joint state.
// ──────────────────────────────────────────────────────────────
std::optional<moveit::planning_interface::MoveGroupInterface::Plan> plan_and_publish_rrt(
    const std::string                                              &task_type,
    const geometry_msgs::msg::Pose                                 &target_pose,
    moveit::planning_interface::MoveGroupInterface                 &arm_group,
    moveit::planning_interface::MoveGroupInterface                 &gripper_group,
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr              &pub_rrt_path,
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            &pub_status,
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr  &pub_rrt_viz,
    rclcpp::Node::SharedPtr                                         node)
{
    RCLCPP_INFO(node->get_logger(), "[RRT*] Planning for task: %s", task_type.c_str());

    // -- planner setup --
    // RRTConnect is always available in MoveIt Humble OMPL.
    // To use RRTstar, add it explicitly to ompl_planning.yaml first.
    arm_group.setPlanningPipelineId("ompl");
    arm_group.setPlannerId("demo_arm_bot");

    arm_group.setPathConstraints(make_ee_down_constraint());
    arm_group.setPlanningTime(ARM_PLANNING_TIME_SEC);
    arm_group.setPoseReferenceFrame("world");
    arm_group.setJointValueTarget(target_pose);
    
    // Path constraint removed: robot home pose has EE pointing UP which
    // violates a "Z-down" constraint at the start state, causing planning
    // to fail. Re-enable once the robot is already in a valid EE-down pose.
 
    // arm_group.clearPathConstraints();

    // -- always start from actual current state, never override with zeros --
    arm_group.setStartStateToCurrentState();

    // Test IK before planning
    moveit::core::RobotStatePtr test_state = arm_group.getCurrentState(5.0);
    // To this:
    const moveit::core::JointModelGroup* ik_jmg =     // rename to avoid clash
        test_state->getJointModelGroup("demo_arm_bot");

    bool ik_ok = test_state->setFromIK(ik_jmg, target_pose, 5.0);
    RCLCPP_INFO(node->get_logger(), "IK test result: %s", ik_ok ? "SUCCESS" : "FAILED");

    if (!ik_ok) {
        RCLCPP_ERROR(node->get_logger(), 
            "Target pose is not reachable! Check position (%.3f, %.3f, %.3f)",
            target_pose.position.x,
            target_pose.position.y,
            target_pose.position.z);
        return std::nullopt;
    }


    moveit::planning_interface::MoveGroupInterface::Plan plan;

    if (arm_group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "[RRT*] Planning failed.");
        std_msgs::msg::String status;  status.data = "RRT_FAILED";
        pub_status->publish(status);
        return std::nullopt;
    }
        // -- green LINE_STRIP visualisation in RViz --
    moveit::core::RobotStatePtr viz_state = arm_group.getCurrentState(5.0);
    const moveit::core::JointModelGroup *jmg =
        viz_state->getJointModelGroup(arm_group.getName());
    const std::string ee_link = arm_group.getEndEffectorLink();
    auto rrt_marker = make_path_marker(
        plan.trajectory_.joint_trajectory, viz_state, jmg, ee_link,
        "rrt_path", 0.0f, 1.0f, 0.0f, node);   // GREEN
    pub_rrt_viz->publish(rrt_marker);
    RCLCPP_INFO(node->get_logger(), "[RRT*] Visualisation marker published.");

    RCLCPP_INFO(node->get_logger(), "[RRT*] Plan succeeded, executing...");


    arm_group.execute(plan);

    // -- publish path --
    nav_msgs::msg::Path rrt_path;
    rrt_path.header.stamp    = node->now();
    rrt_path.header.frame_id = "world";
    for (const auto &point : plan.trajectory_.joint_trajectory.points)
    {
        geometry_msgs::msg::PoseStamped wp;
        wp.header = rrt_path.header;
        wp.pose   = target_pose;   // Cartesian FK not done here; downstream can solve if needed
        rrt_path.poses.push_back(wp);
    }
    pub_rrt_path->publish(rrt_path);
    RCLCPP_INFO(node->get_logger(), "[RRT*] Path published (%zu waypoints).",
                rrt_path.poses.size());


    // -- gripper --
    if (task_type == "Grasp")
    {
        gripper_group.setNamedTarget("gripper_open");
        moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
        if (gripper_group.plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(node->get_logger(), "[RRT*] Gripper executing...");
            gripper_group.execute(gripper_plan);
        }
        else
            RCLCPP_WARN(node->get_logger(), "[RRT*] Gripper plan failed, skipping.");
    }

    std_msgs::msg::String status;  status.data = "RRT_SUCCESS";
    pub_status->publish(status);
    return plan;
}

// ──────────────────────────────────────────────────────────────
//  Stage 2 — CHOMP
//  Publishes smooth trajectory to /planning/chomp_trajectory
//  NOTE: CHOMP only accepts joint-space goals, not Cartesian poses.
//  We solve IK first, then pass joint values as the goal.
// ──────────────────────────────────────────────────────────────
void plan_and_publish_chomp(
    const std::string                                                       &task_type,
    const moveit::planning_interface::MoveGroupInterface::Plan              &rrt_plan,
    moveit::planning_interface::MoveGroupInterface                          &arm_group,
    moveit::planning_interface::MoveGroupInterface                          &gripper_group,
    rclcpp::Publisher<moveit_msgs::msg::RobotTrajectory>::SharedPtr         &pub_chomp_traj,
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                     &pub_status,
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr           &pub_chomp_viz,
    rclcpp::Node::SharedPtr                                                  node)
{
    RCLCPP_INFO(node->get_logger(), "[CHOMP] Planning for task: %s", task_type.c_str());

    // ── Step 1: Extract goal joint values from the last point of the RRT plan ─
    // CHOMP only supports joint-space goals. Since there is no IK plugin
    // configured, we reuse the joint solution that RRT already found.
    const auto &rrt_points = rrt_plan.trajectory_.joint_trajectory.points;
    if (rrt_points.empty())
    {
        RCLCPP_ERROR(node->get_logger(), "[CHOMP] RRT plan is empty — cannot extract goal joints.");
        std_msgs::msg::String status;  status.data = "CHOMP_FAILED_NO_RRT";
        pub_status->publish(status);
        return;
    }
    const std::vector<double> goal_joint_values = rrt_points.back().positions;
    RCLCPP_INFO(node->get_logger(), "[CHOMP] Using RRT end-state as joint-space goal (%zu joints).",
                goal_joint_values.size());

    // ── Step 2: Configure CHOMP with joint-space goal ────────────────────
    arm_group.setPlanningPipelineId("chomp");
    arm_group.setPlannerId("chomp");
    arm_group.setPlanningTime(ARM_PLANNING_TIME_SEC);
    arm_group.setJointValueTarget(goal_joint_values);
    // arm_group.clearPathConstraints();   // CHOMP does not support path constraints

    // -- start from actual current robot state --
    arm_group.setStartStateToCurrentState();

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    if (arm_group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "[CHOMP] Planning failed.");
        std_msgs::msg::String status;  status.data = "CHOMP_FAILED";
        pub_status->publish(status);
        return;
    }

    RCLCPP_INFO(node->get_logger(), "[CHOMP] Plan succeeded, executing...");
    // Save RRT trajectory to file
    saveTrajectoryToFile(plan.trajectory_, "rrt_trajectory.txt");
    arm_group.execute(plan);

    // -- publish trajectory --
    moveit_msgs::msg::RobotTrajectory traj_msg = plan.trajectory_;
    pub_chomp_traj->publish(traj_msg);
    // Save CHOMP trajectory to file
    saveTrajectoryToFile(traj_msg, "chomp_trajectory.txt");
    RCLCPP_INFO(node->get_logger(), "[CHOMP] Trajectory published (%zu points).",
                traj_msg.joint_trajectory.points.size());

    // -- blue LINE_STRIP visualisation in RViz --
    moveit::core::RobotStatePtr viz_state = arm_group.getCurrentState(5.0);
    const moveit::core::JointModelGroup *jmg =
        viz_state->getJointModelGroup(arm_group.getName());
    const std::string ee_link = arm_group.getEndEffectorLink();
    auto chomp_marker = make_path_marker(
        plan.trajectory_.joint_trajectory, viz_state, jmg, ee_link,
        "chomp_path", 0.0f, 0.4f, 1.0f, node);   // BLUE
    pub_chomp_viz->publish(chomp_marker);
    RCLCPP_INFO(node->get_logger(), "[CHOMP] Visualisation marker published.");

    // -- gripper --
    if (task_type == "Grasp")
    {
        gripper_group.setNamedTarget("gripper_open");
        moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
        if (gripper_group.plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(node->get_logger(), "[CHOMP] Gripper executing...");
            gripper_group.execute(gripper_plan);
        }
        else
            RCLCPP_WARN(node->get_logger(), "[CHOMP] Gripper plan failed, skipping.");
    }

    std_msgs::msg::String status;  status.data = "CHOMP_SUCCESS";
    pub_status->publish(status);
}

// ──────────────────────────────────────────────────────────────
//  main
// ──────────────────────────────────────────────────────────────
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("move_to_pose_node");

    // -- spin on background thread so MoveGroupInterface can call services --
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread executor_thread([&executor]() { executor.spin(); });

    // ── MoveIt planning groups ─────────────────────────────────────────────
    moveit::planning_interface::MoveGroupInterface arm_group(node, "demo_arm_bot");
    moveit::planning_interface::MoveGroupInterface gripper_group(node, "demo_gripper_bot");

    arm_group.setPoseReferenceFrame("world");
    arm_group.setPlanningTime(ARM_PLANNING_TIME_SEC);


        // ── Print current EE pose ─────────────────────────────────────
    auto ee_pose = arm_group.getCurrentPose();
    RCLCPP_INFO(node->get_logger(), "Current EE pose:");
    RCLCPP_INFO(node->get_logger(), "  position:    x=%.4f  y=%.4f  z=%.4f",
                ee_pose.pose.position.x,
                ee_pose.pose.position.y,
                ee_pose.pose.position.z);
    RCLCPP_INFO(node->get_logger(), "  orientation: x=%.4f  y=%.4f  z=%.4f  w=%.4f",
                ee_pose.pose.orientation.x,
                ee_pose.pose.orientation.y,
                ee_pose.pose.orientation.z,
                ee_pose.pose.orientation.w);

    // Also print as RPY for easier interpretation
    tf2::Quaternion q;
    tf2::fromMsg(ee_pose.pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    RCLCPP_INFO(node->get_logger(), "  RPY (rad):   r=%.4f  p=%.4f  y=%.4f",
                roll, pitch, yaw);
    RCLCPP_INFO(node->get_logger(), "  RPY (deg):   r=%.2f  p=%.2f  y=%.2f",
                roll * 180.0/M_PI,
                pitch * 180.0/M_PI,
                yaw * 180.0/M_PI);

    gripper_group.setPlanningTime(GRIPPER_PLANNING_TIME_SEC);

    // ── Publishers ────────────────────────────────────────────────────────
    auto pub_rrt_path   = node->create_publisher<nav_msgs::msg::Path>(
                              "/planning/rrt_path",         10);
    auto pub_chomp_traj = node->create_publisher<moveit_msgs::msg::RobotTrajectory>(
                              "/planning/chomp_trajectory", 10);
    auto pub_status     = node->create_publisher<std_msgs::msg::String>(
                              "/planning/status",           10);
    // Visualisation: green = RRT path,  blue = CHOMP trajectory
    // Add /planning/rrt_viz and /planning/chomp_viz as
    // Marker displays in RViz to see the paths.
    auto pub_rrt_viz    = node->create_publisher<visualization_msgs::msg::Marker>(
                              "/planning/rrt_viz",          10);
    auto pub_chomp_viz  = node->create_publisher<visualization_msgs::msg::Marker>(
                              "/planning/chomp_viz",        10);

    // ── Live inputs ───────────────────────────────────────────────────────
    std::optional<geometry_msgs::msg::Pose>      live_target_pose;
    std::optional<std::string>                   live_task_type;
    sensor_msgs::msg::JointState::SharedPtr      live_joint_state;

    // ── Subscribers ───────────────────────────────────────────────────────

    // Object point cloud from perception
    auto sub_object_cloud = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/perception/object_pointcloud",
        rclcpp::QoS(1).durability_volatile().best_effort(),
        [&node](const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
            RCLCPP_DEBUG(node->get_logger(), "Object cloud received: %u x %u points.",
                         msg->width, msg->height);
            // TODO: pass to grasp-pose estimator / ICP pipeline
        });

    // Obstacle point cloud from perception  →  collision scene
    auto sub_obstacle_cloud = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/perception/obstacle_pointcloud",
        rclcpp::QoS(1).durability_volatile().best_effort(),
        [&node](const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
            RCLCPP_DEBUG(node->get_logger(), "Obstacle cloud received: %u x %u points.",
                         msg->width, msg->height);
            // TODO: forward to MoveIt PlanningSceneInterface as collision objects
        });

    // Target pose from perception
    auto sub_target_pose = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/perception/target_pose",
        rclcpp::QoS(10).reliable(),
        [&live_target_pose, &node](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            live_target_pose = msg->pose;
            RCLCPP_INFO(node->get_logger(), "Target pose received: (%.3f, %.3f, %.3f)",
                        msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        });

    // Task type from task planner  ("Move" | "Grasp" | "Place")
    auto sub_task_type = node->create_subscription<std_msgs::msg::String>(
        "/task_planner/task_type",
        rclcpp::QoS(10).reliable(),
        [&live_task_type, &node](const std_msgs::msg::String::SharedPtr msg)
        {
            live_task_type = msg->data;
            RCLCPP_INFO(node->get_logger(), "Task type received: %s", msg->data.c_str());
        });

    // Current robot joint state
    // MoveIt reads /joint_states internally, but we subscribe here too
    // so the node has direct access to the latest joint positions,
    // velocities, and efforts for logging, validation, or custom logic.
    auto sub_joint_state = node->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states",
        rclcpp::QoS(10).reliable(),
        [&live_joint_state, &node](const sensor_msgs::msg::JointState::SharedPtr msg)
        {
            live_joint_state = msg;
            RCLCPP_DEBUG(node->get_logger(),
                         "Joint state received: %zu joints, t=%.3f",
                         msg->name.size(),
                         rclcpp::Time(msg->header.stamp).seconds());
        });

    // ── Wait for perception input, then fall back to defaults if needed ───
    RCLCPP_INFO(node->get_logger(), "Waiting %d s for perception input...", INPUT_TIMEOUT_SEC);
    std::this_thread::sleep_for(std::chrono::seconds(INPUT_TIMEOUT_SEC));

    const geometry_msgs::msg::Pose target_pose =
        live_target_pose.has_value() ? live_target_pose.value() : defaults::target_pose();
    const std::string task_type =
        live_task_type.has_value() ? live_task_type.value() : defaults::TASK_TYPE;

    if (!live_target_pose.has_value())
        RCLCPP_WARN(node->get_logger(), "No target pose received — using hard-coded default.");
    if (!live_task_type.has_value())
        RCLCPP_WARN(node->get_logger(), "No task type received — defaulting to '%s'.", task_type.c_str());
    if (live_joint_state)
    {
        RCLCPP_INFO(node->get_logger(), "Current joint state (%zu joints):",
                    live_joint_state->name.size());
        for (size_t i = 0; i < live_joint_state->name.size(); ++i)
            RCLCPP_INFO(node->get_logger(), "  %-30s pos=%.4f  vel=%.4f",
                        live_joint_state->name[i].c_str(),
                        live_joint_state->position.size() > i ? live_joint_state->position[i] : 0.0,
                        live_joint_state->velocity.size() > i ? live_joint_state->velocity[i] : 0.0);
    }
    else
        RCLCPP_WARN(node->get_logger(), "No joint state received yet on /joint_states.");

    // ── Stage 1: RRT* (OMPL)  →  /planning/rrt_path ─────────────────
    auto rrt_plan_opt = plan_and_publish_rrt(
        task_type, target_pose,
        arm_group, gripper_group,
        pub_rrt_path, pub_status, pub_rrt_viz, node);

    // ── Stage 2: CHOMP  →  /planning/chomp_trajectory ────────────────
    // CHOMP reuses the RRT goal joint state (no IK plugin required)
    if (rrt_plan_opt.has_value())
    {
        plan_and_publish_chomp(
            task_type, rrt_plan_opt.value(),
            arm_group, gripper_group,
            pub_chomp_traj, pub_status, pub_chomp_viz, node);
    }
    else
    {
        RCLCPP_WARN(node->get_logger(), "Skipping CHOMP: RRT did not produce a valid plan.");
    }

    // ── Clean shutdown ────────────────────────────────────────────────────
    executor.cancel();
    executor_thread.join();
    rclcpp::shutdown();
    // rclcpp::spin(node);
    return 0;
}