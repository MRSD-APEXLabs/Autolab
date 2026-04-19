#!/usr/bin/env python3
import struct
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

ROADMAP_FILE = "/home/robot/AutoLab/robot/prm_roadmap.bin"

# ── Load ──────────────────────────────────────────────────────
with open(ROADMAP_FILE, "rb") as f:
    n, dof = struct.unpack("QQ", f.read(16))
    print(f"Nodes: {n}, DOF: {dof}")

    nodes = []
    edges = []
    for i in range(n):
        q = struct.unpack(f"{dof}d", f.read(dof * 8))
        nn = struct.unpack("Q", f.read(8))[0]
        neighbors = list(struct.unpack(f"{nn}i", f.read(nn * 4)))
        nodes.append(q)
        for j in neighbors:
            if j > i:  # avoid duplicates
                edges.append((i, j))

nodes = np.array(nodes)
print(f"Edges: {len(edges)}")

# ── Plot — use first 3 joints as XYZ ─────────────────────────
fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection="3d")

# Draw edges (subsample if too many)
max_edges = 5000
step = max(1, len(edges) // max_edges)
for i, j in edges[::step]:
    ax.plot(
        [nodes[i, 0], nodes[j, 0]],
        [nodes[i, 1], nodes[j, 1]],
        [nodes[i, 2], nodes[j, 2]],
        "b-", alpha=0.1, linewidth=0.3
    )

# Draw nodes
ax.scatter(nodes[:, 0], nodes[:, 1], nodes[:, 2],
           c="red", s=1, alpha=0.3)

ax.set_xlabel("Joint 0 (rad)")
ax.set_ylabel("Joint 1 (rad)")
ax.set_zlabel("Joint 2 (rad)")
ax.set_title(f"PRM Roadmap — {n} nodes, {len(edges)} edges")
plt.tight_layout()
plt.savefig("/tmp/prm_roadmap.png", dpi=150)
print("Saved to /tmp/prm_roadmap.png")
plt.show()