#!/usr/bin/env python3
"""
add_waypoints_to_roadmap.py

Loads manual_waypoints.json, validates each waypoint against:
  1. ee_down constraint (via ROS2/MoveIt)
  2. Collision (via MoveIt planning scene)
  3. Duplicate check against existing roadmap

Then connects valid waypoints to K nearest neighbors with
collision-free edge checking, and saves the updated roadmap.

NOW SUPPORTS:
  - starting from an empty/nonexistent roadmap .bin file
"""

import rclpy
from rclpy.node import Node

import struct
import json
import math
import os

WAYPOINTS_FILE = "/home/robot/AutoLab/robot/manual_waypoints_0.json"
ROADMAP_FILE   = "/home/robot/AutoLab/robot/prm_roadmap.bin"

GROUP_NAME     = "demo_arm_bot"
EE_LINK        = "end_effector_p4_1"

STEP_SIZE      = 0.02
CONNECT_TOL    = 0.02
K_NEIGHBORS    = 15

EE_ROLL_TOL    = 0.2
EE_PITCH_TOL   = 0.2


# ─────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────

def joint_dist(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def steer(frm, to, step):
    d = joint_dist(frm, to)

    if d <= step:
        return to

    r = step / d

    return [f + r * (t - f) for f, t in zip(frm, to)]


def load_roadmap(path):
    """
    Loads roadmap if it exists.

    Returns:
        nodes, neighbors, dof
    """

    if not os.path.exists(path):
        print(f"[PRM] No roadmap found — starting fresh")
        return [], [], None

    with open(path, "rb") as f:

        header = f.read(16)

        if len(header) != 16:
            raise RuntimeError("Roadmap file corrupted")

        n, dof = struct.unpack("QQ", header)

        nodes = []
        neighbors = []

        for _ in range(n):

            q = list(struct.unpack(f"{dof}d", f.read(dof * 8)))

            nn = struct.unpack("Q", f.read(8))[0]

            nb = (
                list(struct.unpack(f"{nn}i", f.read(nn * 4)))
                if nn > 0 else []
            )

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


# ─────────────────────────────────────────────────────────────
# ROS2 Node
# ─────────────────────────────────────────────────────────────

class WaypointAdder(Node):

    def __init__(self):
        super().__init__("waypoint_adder")

    def run(self):

        # -----------------------------------------------------
        # MoveIt imports
        # -----------------------------------------------------

        # from moveit.planning import MoveItPy
        # from moveit.core.robot_state import RobotState

        # # -----------------------------------------------------
        # # Init MoveIt
        # # -----------------------------------------------------

        # moveit = MoveItPy(node_name="waypoint_adder_moveit")

        # robot_model = moveit.get_robot_model()

        # jmg = robot_model.get_joint_model_group(GROUP_NAME)

        # planning_scene_monitor = moveit.get_planning_scene_monitor()

        # -----------------------------------------------------
        # Load roadmap
        # -----------------------------------------------------

        nodes, neighbors, dof = load_roadmap(ROADMAP_FILE)

        # -----------------------------------------------------
        # Load waypoints
        # -----------------------------------------------------

        with open(WAYPOINTS_FILE) as f:
            waypoints = json.load(f)

        print(f"[WP] Loaded {len(waypoints)} manual waypoints")

        if len(waypoints) == 0:
            print("[WP] No waypoints found")
            return

        # -----------------------------------------------------
        # Infer DOF if roadmap doesn't exist yet
        # -----------------------------------------------------

        if dof is None:

            dof = len(waypoints[0]["angles_rad"])

            print(f"[PRM] Inferred DOF={dof} from waypoint file")

        # -----------------------------------------------------
        # Stats
        # -----------------------------------------------------

        added = 0
        skipped_duplicate = 0
        skipped_constraint = 0
        skipped_collision = 0
        edges_added = 0

        # -----------------------------------------------------
        # Process waypoints
        # -----------------------------------------------------

        for wp in waypoints:

            q = wp["angles_rad"]

            note = wp.get("note", "")

            idx_label = wp.get("index", "?")

            # -------------------------------------------------
            # DOF check
            # -------------------------------------------------

            if len(q) != dof:

                print(
                    f"  [{idx_label}] SKIP: "
                    f"DOF mismatch ({len(q)} vs {dof})"
                )

                continue

            # -------------------------------------------------
            # Duplicate check
            # -------------------------------------------------

            is_dup = any(
                joint_dist(q, existing) < CONNECT_TOL
                for existing in nodes
            )

            if is_dup:

                print(
                    f"  [{idx_label}] "
                    f"SKIP: duplicate roadmap node"
                )

                skipped_duplicate += 1

                continue

            # -------------------------------------------------
            # Constraint check
            # -------------------------------------------------

            rs = RobotState(robot_model)

            rs.set_joint_group_positions(GROUP_NAME, q)

            rs.update()

            ee_tf = rs.get_global_link_transform(EE_LINK)

            R = ee_tf[:3, :3]

            roll = math.atan2(R[2, 1], R[2, 2])

            pitch = math.atan2(
                -R[2, 0],
                math.sqrt(R[2, 1] ** 2 + R[2, 2] ** 2)
            )

            if (
                abs(roll) >= EE_ROLL_TOL or
                abs(pitch) >= EE_PITCH_TOL
            ):

                print(
                    f"  [{idx_label}] SKIP: "
                    f"ee_down violated "
                    f"(roll={math.degrees(roll):.1f}°, "
                    f"pitch={math.degrees(pitch):.1f}°)"
                )

                skipped_constraint += 1

                continue

            # -------------------------------------------------
            # Collision check
            # -------------------------------------------------

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

            # -------------------------------------------------
            # Add node
            # -------------------------------------------------

            new_idx = len(nodes)

            nodes.append(q)

            neighbors.append([])

            # -------------------------------------------------
            # First node in empty roadmap
            # -------------------------------------------------

            if new_idx == 0:

                print(
                    f"  [{idx_label}] "
                    f"ADDED as seed node"
                )

                added += 1

                continue

            # -------------------------------------------------
            # Connect to nearest neighbors
            # -------------------------------------------------

            dists = sorted(
                [
                    (joint_dist(q, nodes[i]), i)
                    for i in range(new_idx)
                ],
                key=lambda x: x[0]
            )

            connected = 0

            for d, i in dists:

                if connected >= K_NEIGHBORS:
                    break

                edge_ok = True

                cur = list(q)

                target = nodes[i]

                while joint_dist(cur, target) > STEP_SIZE:

                    cur = steer(cur, target, STEP_SIZE)

                    rs_edge = RobotState(robot_model)

                    rs_edge.set_joint_group_positions(
                        GROUP_NAME,
                        cur
                    )

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

            print(
                f"  [{idx_label}] "
                f"ADDED — {connected} edges "
                f"note='{note}'"
            )

            added += 1

        # -----------------------------------------------------
        # Save roadmap
        # -----------------------------------------------------

        if added > 0:

            save_roadmap(
                ROADMAP_FILE,
                nodes,
                neighbors,
                dof
            )

        else:

            print("[PRM] No new waypoints added")

        # -----------------------------------------------------
        # Summary
        # -----------------------------------------------------

        print("\n=== Summary ===")

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