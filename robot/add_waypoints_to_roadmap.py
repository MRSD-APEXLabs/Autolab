#!/usr/bin/env python3
"""
add_waypoints_to_roadmap.py

Loads manual_waypoints.json, validates each waypoint against:
  1. ee_down constraint (via ROS2/MoveIt)
  2. Collision (via MoveIt planning scene)
  3. Duplicate check against existing roadmap

Then connects valid waypoints to K nearest neighbors with
collision-free edge checking, and saves the updated roadmap.

Run AFTER the ROS stack is up:
  ros2 run global_planner add_waypoints_to_roadmap.py
"""
import rclpy
from rclpy.node import Node
import struct
import json
import math
import os
import sys

from moveit.core.robot_model import RobotModel
from moveit.core.robot_state import RobotState
from moveit.core.planning_scene import PlanningScene
from moveit_msgs.msg import PlanningScene as PlanningSceneMsg
from moveit_ros_planning_interface import MoveGroupInterface

WAYPOINTS_FILE  = "/home/robot/AutoLab/robot/manual_waypoints.json"
ROADMAP_FILE    = "/home/robot/AutoLab/robot/prm_roadmap.bin"
GROUP_NAME      = "demo_arm_bot"
EE_LINK         = "end_effector_p4_1"
STEP_SIZE       = 0.02   # rad — must match crrt_cfg::STEP_SIZE
CONNECT_TOL     = 0.02   # rad — duplicate threshold
K_NEIGHBORS     = 15     # more neighbors for manually recorded waypoints
EE_ROLL_TOL     = 0.2    # rad — must match crrt_cfg
EE_PITCH_TOL    = 0.2    # rad

# ── Helpers ───────────────────────────────────────────────────

def joint_dist(a, b):
    return math.sqrt(sum((x - y)**2 for x, y in zip(a, b)))

def steer(frm, to, step):
    d = joint_dist(frm, to)
    if d <= step:
        return to
    r = step / d
    return [f + r * (t - f) for f, t in zip(frm, to)]

def load_roadmap(path):
    if not os.path.exists(path):
        print(f"[ERROR] Roadmap not found: {path}")
        return None, None, None
    with open(path, "rb") as f:
        n, dof = struct.unpack("QQ", f.read(16))
        nodes = []
        neighbors = []
        for _ in range(n):
            q = list(struct.unpack(f"{dof}d", f.read(dof * 8)))
            nn = struct.unpack("Q", f.read(8))[0]
            nb = list(struct.unpack(f"{nn}i", f.read(nn * 4))) if nn > 0 else []
            nodes.append(q)
            neighbors.append(nb)
    print(f"[PRM] Loaded {n} nodes, DOF={dof}")
    return nodes, neighbors, dof

def save_roadmap(path, nodes, neighbors, dof):
    with open(path, "wb") as f:
        n = len(nodes)
        f.write(struct.pack("QQ", n, dof))
        for q, nb in zip(nodes, neighbors):
            f.write(struct.pack(f"{dof}d", *q))
            f.write(struct.pack("Q", len(nb)))
            if nb:
                f.write(struct.pack(f"{len(nb)}i", *nb))
    print(f"[PRM] Saved {len(nodes)} nodes to {path}")

# ── ROS2 Node ─────────────────────────────────────────────────

class WaypointAdder(Node):
    def __init__(self):
        super().__init__("waypoint_adder")

    def run(self):
        from moveit.planning import MoveItPy
        from moveit.core.robot_state import RobotState

        # ── Init MoveIt ───────────────────────────────────────
        moveit = MoveItPy(node_name="waypoint_adder_moveit")
        robot_model = moveit.get_robot_model()
        jmg = robot_model.get_joint_model_group(GROUP_NAME)

        # ── Get planning scene ────────────────────────────────
        planning_scene_monitor = moveit.get_planning_scene_monitor()
        with planning_scene_monitor.read_only() as scene:
            scene_copy = scene  # we'll use this for collision checks

        # ── Load files ────────────────────────────────────────
        nodes, neighbors, dof = load_roadmap(ROADMAP_FILE)
        if nodes is None:
            return

        with open(WAYPOINTS_FILE) as f:
            waypoints = json.load(f)
        print(f"[WP] Loaded {len(waypoints)} manual waypoints")

        # ── Process each waypoint ─────────────────────────────
        added = 0
        skipped_duplicate = 0
        skipped_constraint = 0
        skipped_collision = 0
        edges_added = 0

        for wp in waypoints:
            q = wp["angles_rad"]
            note = wp.get("note", "")
            idx_label = wp["index"]

            if len(q) != dof:
                print(f"  [{idx_label}] SKIP: DOF mismatch ({len(q)} vs {dof})")
                continue

            # 1. Duplicate check
            is_dup = any(joint_dist(q, existing) < CONNECT_TOL for existing in nodes)
            if is_dup:
                print(f"  [{idx_label}] SKIP: duplicate of existing roadmap node")
                skipped_duplicate += 1
                continue

            # 2. Constraint check (ee_down)
            rs = RobotState(robot_model)
            rs.set_joint_group_positions(GROUP_NAME, q)
            rs.update()
            ee_tf = rs.get_global_link_transform(EE_LINK)
            R = ee_tf[:3, :3]
            roll  = math.atan2(R[2,1], R[2,2])
            pitch = math.atan2(-R[2,0], math.sqrt(R[2,1]**2 + R[2,2]**2))
            if abs(roll) >= EE_ROLL_TOL or abs(pitch) >= EE_PITCH_TOL:
                print(f"  [{idx_label}] SKIP: ee_down constraint violated "
                      f"(roll={math.degrees(roll):.1f}°, pitch={math.degrees(pitch):.1f}°)")
                skipped_constraint += 1
                continue

            # 3. Collision check
            with planning_scene_monitor.read_only() as scene:
                rs2 = RobotState(robot_model)
                rs2.set_joint_group_positions(GROUP_NAME, q)
                rs2.update()
                in_collision = scene.is_state_colliding(
                    robot_state=rs2,
                    joint_model_group_name=GROUP_NAME,
                    verbose=False
                )
            if in_collision:
                print(f"  [{idx_label}] SKIP: in collision")
                skipped_collision += 1
                continue

            # 4. Add to roadmap
            new_idx = len(nodes)
            nodes.append(q)
            neighbors.append([])

            # 5. Connect to K nearest with collision-free edge check
            dists = sorted(
                [(joint_dist(q, nodes[i]), i) for i in range(new_idx)],
                key=lambda x: x[0]
            )

            connected = 0
            for d, i in dists:
                if connected >= K_NEIGHBORS:
                    break

                # Edge collision check — step along edge
                edge_ok = True
                cur = list(q)
                target = nodes[i]
                while joint_dist(cur, target) > STEP_SIZE:
                    cur = steer(cur, target, STEP_SIZE)
                    rs_edge = RobotState(robot_model)
                    rs_edge.set_joint_group_positions(GROUP_NAME, cur)
                    rs_edge.update()
                    with planning_scene_monitor.read_only() as scene:
                        if scene.is_state_colliding(
                            robot_state=rs_edge,
                            joint_model_group_name=GROUP_NAME,
                            verbose=False
                        ):
                            edge_ok = False
                            break

                if edge_ok:
                    neighbors[new_idx].append(i)
                    neighbors[i].append(new_idx)
                    connected += 1
                    edges_added += 1

            print(f"  [{idx_label}] ADDED — {connected} edges  note='{note}'")
            added += 1

        # ── Save ──────────────────────────────────────────────
        if added > 0:
            save_roadmap(ROADMAP_FILE, nodes, neighbors, dof)
        else:
            print("[PRM] No new waypoints added — roadmap unchanged.")

        print(f"\n=== Summary ===")
        print(f"  Added:              {added}")
        print(f"  Edges added:        {edges_added}")
        print(f"  Skipped duplicate:  {skipped_duplicate}")
        print(f"  Skipped constraint: {skipped_constraint}")
        print(f"  Skipped collision:  {skipped_collision}")


def main():
    rclpy.init()
    node = WaypointAdder()
    node.run()
    rclpy.shutdown()

if __name__ == "__main__":
    main()