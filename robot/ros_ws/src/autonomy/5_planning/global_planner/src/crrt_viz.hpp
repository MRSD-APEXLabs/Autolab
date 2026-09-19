// crrt_viz.hpp
// RViz visualization of a plan: the raw RRT-Connect path and the same path
// after shortcutting, each as one single-colour line, plus a text legend.
// Path waypoints are joint-space; what is drawn is the end-effector position
// (FK on EE_LINK) in the world frame. No-op when `crrt_viz.enabled` is false.
// Depends on: crrt_types.hpp

#pragma once
#include "crrt_types.hpp"

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace crrt_viz {

static const std::string TOPIC = "/crrt/viz";
static const std::string FRAME = "world";
static const std::string NS    = "crrt";

enum Id : int { ID_PATH_RAW = 0, ID_PATH_SHORT, ID_LEGEND_RAW, ID_LEGEND_SHORT };

using Marker      = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;

struct Rgb { float r, g, b; };
static const Rgb COLOR_RAW   = {1.0f, 0.55f, 0.0f};   // orange
static const Rgb COLOR_SHORT = {0.0f, 0.90f, 1.0f};   // cyan

struct Viz {
    rclcpp::Node::SharedPtr node;
    rclcpp::Publisher<MarkerArray>::SharedPtr pub;
    bool enabled = false;

    std::unique_ptr<moveit::core::RobotState> rs;   // planning thread only
    const moveit::core::JointModelGroup* jmg = nullptr;

    Marker base(int id, int type, Rgb c, double scale)
    {
        Marker m;
        m.header.frame_id = FRAME;
        m.header.stamp    = node->now();
        m.ns     = NS;
        m.id     = id;
        m.type   = type;
        m.action = Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.scale.x = m.scale.y = m.scale.z = scale;
        m.color.r = c.r; m.color.g = c.g; m.color.b = c.b; m.color.a = 1.0f;
        return m;
    }

    Marker strip(int id, const std::vector<JointVec>& path, Rgb c, double width)
    {
        Marker m = base(id, Marker::LINE_STRIP, c, width);
        m.points.reserve(path.size());
        for (const auto& q : path) {
            rs->setJointGroupPositions(jmg, q);
            rs->updateLinkTransforms();
            const auto& t = rs->getGlobalLinkTransform(crrt_cfg::EE_LINK).translation();
            geometry_msgs::msg::Point p;
            p.x = t.x(); p.y = t.y(); p.z = t.z();
            m.points.push_back(p);
        }
        return m;
    }

    // One legend row per path, above the workspace box, in the path's colour.
    Marker legend(int id, const std::string& text, Rgb c, int row)
    {
        Marker m = base(id, Marker::TEXT_VIEW_FACING, c, 0.04);
        m.pose.position.x = 0.5 * (crrt_cfg::WS_X_MIN + crrt_cfg::WS_X_MAX);
        m.pose.position.y = 0.5 * (crrt_cfg::WS_Y_MIN + crrt_cfg::WS_Y_MAX);
        m.pose.position.z = crrt_cfg::WS_Z_MAX + 0.15 - 0.05 * row;
        m.text = text;
        return m;
    }

    // Clear the previous plan's lines.
    void begin()
    {
        if (!enabled) return;
        MarkerArray arr;
        Marker del; del.header.frame_id = FRAME; del.ns = NS; del.action = Marker::DELETEALL;
        arr.markers.push_back(del);
        pub->publish(arr);
    }

    void path(const std::vector<JointVec>& p, bool shortcut)
    {
        if (!enabled) return;
        MarkerArray arr;
        if (shortcut) {
            arr.markers.push_back(strip (ID_PATH_SHORT, p, COLOR_SHORT, 0.008));
            arr.markers.push_back(legend(ID_LEGEND_SHORT, "after shortcutting", COLOR_SHORT, 1));
        } else {
            arr.markers.push_back(strip (ID_PATH_RAW, p, COLOR_RAW, 0.006));
            arr.markers.push_back(legend(ID_LEGEND_RAW, "raw planner path", COLOR_RAW, 0));
        }
        pub->publish(arr);
    }
};

// One publisher for the life of the process. `crrt_viz.enabled` is re-read on
// every call, so `ros2 param set` takes effect on the next plan.
static Viz& get(rclcpp::Node::SharedPtr node,
                const moveit::core::RobotModelConstPtr& robot_model,
                const moveit::core::JointModelGroup* jmg)
{
    static Viz viz;
    static std::once_flag flag;
    std::call_once(flag, [&]() {
        viz.node = node;
        viz.pub  = node->create_publisher<MarkerArray>(TOPIC, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
        viz.rs   = std::make_unique<moveit::core::RobotState>(robot_model);
        viz.rs->setToDefaultValues();
        viz.jmg  = jmg;
        if (!node->has_parameter("crrt_viz.enabled"))
            node->declare_parameter("crrt_viz.enabled", true);
        RCLCPP_INFO(node->get_logger(), "[CRRT] Viz publisher on %s", TOPIC.c_str());
    });
    viz.enabled = node->get_parameter("crrt_viz.enabled").as_bool();
    return viz;
}

} // namespace crrt_viz
