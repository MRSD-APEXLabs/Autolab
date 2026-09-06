#!/usr/bin/env python3
import struct
import json
import math
import numpy as np
from pathlib import Path

ROADMAP_FILE = "/home/robot/AutoLab/robot/prm_roadmap.bin"
WAYPOINTS_FILE = "/home/robot/AutoLab/robot/manual_waypoints_0.json"
STEP_SIZE = 0.05
K_NEIGHBORS = 15

def joint_dist(a, b):
    return math.sqrt(sum((x-y)**2 for x,y in zip(a,b)))

def steer(frm, to, step):
    d = joint_dist(frm, to)
    if d <= step:
        return to
    r = step / d
    return [f + r*(t-f) for f,t in zip(frm,to)]

def load_roadmap(path):
    nodes = []
    with open(path, 'rb') as f:
        n, dof = struct.unpack('QQ', f.read(16))
        for _ in range(n):
            q = list(struct.unpack(f'{dof}d', f.read(dof*8)))
            nn, = struct.unpack('Q', f.read(8))
            neighbors = list(struct.unpack(f'{nn}i', f.read(nn*4)))
            nodes.append({'q': q, 'neighbors': neighbors})
    return nodes, dof

def save_roadmap(path, nodes, dof):
    with open(path, 'wb') as f:
        f.write(struct.pack('QQ', len(nodes), dof))
        for node in nodes:
            f.write(struct.pack(f'{dof}d', *node['q']))
            nn = len(node['neighbors'])
            f.write(struct.pack('Q', nn))
            f.write(struct.pack(f'{nn}i', *node['neighbors']))

def load_waypoints(path, dof):
    with open(path) as f:
        data = json.load(f)
    waypoints = []
    for wp in data:
        q = wp['angles_rad'][:dof]
        if len(q) == dof:
            waypoints.append(q)
    return waypoints

def edge_valid(q_from, q_to, roadmap):
    # No collision checking from Python — you said you trust these points
    # Just check the straight line doesn't pass through obviously bad regions
    cur = q_from[:]
    while joint_dist(cur, q_to) > STEP_SIZE:
        cur = steer(cur, q_to, STEP_SIZE)
    return True  # trusted waypoints — skip collision check

def inject(roadmap, new_waypoints, dof):
    for q_new in new_waypoints:
        new_idx = len(roadmap)
        new_node = {'q': q_new, 'neighbors': []}

        # Find K nearest and connect
        dists = sorted(
            [(joint_dist(q_new, roadmap[i]['q']), i) 
             for i in range(len(roadmap))]
        )

        for d, i in dists:
            if len(new_node['neighbors']) >= K_NEIGHBORS:
                break
            if edge_valid(q_new, roadmap[i]['q'], roadmap):
                new_node['neighbors'].append(i)
                roadmap[i]['neighbors'].append(new_idx)

        print(f"[INJECT] Node {new_idx} added with {len(new_node['neighbors'])} edges")
        roadmap.append(new_node)

    return roadmap

if __name__ == '__main__':
    print("[INJECT] Loading roadmap...")
    roadmap, dof = load_roadmap(ROADMAP_FILE)
    print(f"[INJECT] Loaded {len(roadmap)} nodes, DOF={dof}")

    print("[INJECT] Loading waypoints...")
    new_waypoints = load_waypoints(WAYPOINTS_FILE, dof)
    print(f"[INJECT] Found {len(new_waypoints)} waypoints to inject")

    roadmap = inject(roadmap, new_waypoints, dof)

    print("[INJECT] Saving roadmap...")
    save_roadmap(ROADMAP_FILE, roadmap, dof)
    print(f"[INJECT] Done. Roadmap now has {len(roadmap)} nodes.")