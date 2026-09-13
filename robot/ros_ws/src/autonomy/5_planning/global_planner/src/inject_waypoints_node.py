#!/usr/bin/env python3
import struct
import json
import math
from pathlib import Path

ROADMAP_FILE = "/home/robot/AutoLab/robot/prm_roadmap.bin"
WAYPOINTS_FILE = "/home/robot/AutoLab/robot/manual_waypoints_0.json"

STEP_SIZE = 0.05
K_NEIGHBORS = 15

# Default DOF to use if no roadmap exists yet
DEFAULT_DOF = 6


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
    Loads an existing roadmap if present.
    Returns:
        roadmap, dof
    """
    path = Path(path)

    if not path.exists():
        print(f"[INJECT] No roadmap found at {path}")
        return [], None

    nodes = []

    with open(path, "rb") as f:
        header = f.read(16)

        if len(header) != 16:
            raise RuntimeError("Roadmap file is corrupted or incomplete")

        n, dof = struct.unpack("QQ", header)

        for _ in range(n):
            q = list(struct.unpack(f"{dof}d", f.read(dof * 8)))

            nn, = struct.unpack("Q", f.read(8))

            neighbors = list(struct.unpack(f"{nn}i", f.read(nn * 4)))

            nodes.append({
                "q": q,
                "neighbors": neighbors
            })

    return nodes, dof


def save_roadmap(path, nodes, dof):
    with open(path, "wb") as f:
        f.write(struct.pack("QQ", len(nodes), dof))

        for node in nodes:
            f.write(struct.pack(f"{dof}d", *node["q"]))

            nn = len(node["neighbors"])

            f.write(struct.pack("Q", nn))

            if nn > 0:
                f.write(struct.pack(f"{nn}i", *node["neighbors"]))


def load_waypoints(path, dof=None):
    """
    Loads waypoints from JSON.

    If dof is None, infer it from the first waypoint.
    """
    with open(path) as f:
        data = json.load(f)

    if not data:
        raise RuntimeError("Waypoint file is empty")

    # Infer DOF if needed
    if dof is None:
        first = data[0]["angles_rad"]
        dof = len(first)

        if dof == 0:
            dof = DEFAULT_DOF

        print(f"[INJECT] Inferred DOF={dof} from waypoint file")

    waypoints = []

    for wp in data:
        q = wp["angles_rad"][:dof]

        if len(q) != dof:
            print("[WARN] Skipping malformed waypoint")
            continue

        waypoints.append(q)

    return waypoints, dof


def edge_valid(q_from, q_to):
    """
    Placeholder edge validation.

    Since waypoints are trusted, we skip collision checks.
    """
    cur = q_from[:]

    while joint_dist(cur, q_to) > STEP_SIZE:
        cur = steer(cur, q_to, STEP_SIZE)

    return True


def inject(roadmap, new_waypoints, dof):
    for q_new in new_waypoints:
        new_idx = len(roadmap)

        new_node = {
            "q": q_new,
            "neighbors": []
        }

        # First node in a brand new roadmap
        if len(roadmap) == 0:
            roadmap.append(new_node)
            print(f"[INJECT] Seed node {new_idx} added")
            continue

        # Find nearest neighbors
        dists = sorted(
            [
                (joint_dist(q_new, roadmap[i]["q"]), i)
                for i in range(len(roadmap))
            ],
            key=lambda x: x[0]
        )

        for d, i in dists:
            if len(new_node["neighbors"]) >= K_NEIGHBORS:
                break

            if edge_valid(q_new, roadmap[i]["q"]):
                new_node["neighbors"].append(i)
                roadmap[i]["neighbors"].append(new_idx)

        roadmap.append(new_node)

        print(
            f"[INJECT] Node {new_idx} added "
            f"with {len(new_node['neighbors'])} edges"
        )

    return roadmap


if __name__ == "__main__":

    print("[INJECT] Loading roadmap...")
    roadmap, dof = load_roadmap(ROADMAP_FILE)

    if roadmap:
        print(f"[INJECT] Loaded {len(roadmap)} nodes, DOF={dof}")
    else:
        print("[INJECT] Starting new roadmap")

    print("[INJECT] Loading waypoints...")
    new_waypoints, inferred_dof = load_waypoints(WAYPOINTS_FILE, dof)

    # If roadmap didn't exist, adopt inferred DOF
    if dof is None:
        dof = inferred_dof

    print(f"[INJECT] Found {len(new_waypoints)} waypoints to inject")

    roadmap = inject(roadmap, new_waypoints, dof)

    print("[INJECT] Saving roadmap...")
    save_roadmap(ROADMAP_FILE, roadmap, dof)

    print(f"[INJECT] Done. Roadmap now has {len(roadmap)} nodes.")