#!/usr/bin/env python3
"""Eye-to-hand calibration for the fixed base ZED camera (`top_camera`).

The camera is bolted to the chassis and watches the arm, so an AprilTag taped to
the gripper gives us two independent readings of the same physical point. For
each arm pose i:

    X . A_i  =  B_i . Y

    X    Chassis_1 -> top_camera           the six numbers we are solving for
    A_i  top_camera -> tag                 measured by the camera (solvePnP)
    B_i  Chassis_1 -> end_effector_p4_1    read from TF (forward kinematics)
    Y    end_effector_p4_1 -> tag          where the tag sits on the gripper:
                                           unknown, but identical at every pose

Both sides describe Chassis_1 -> tag. X and Y are both unknown (12 numbers) and
each pose contributes 6 equations, so 3 poses is the theoretical minimum and
about 12 well-spread poses is what actually gives a usable fit.

    ./calibrate_top_camera.py capture     # jog the arm, ENTER to record a pose
    ./calibrate_top_camera.py solve       # fit, print the launch-file line

Run with /usr/bin/python3 from the repo root, with the planning stack up
(TF must be live) and the camera in inspect mode.
"""

import argparse
import asyncio
import json
import os
import sys
import threading
import time

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

import websockets

WS_URL = "ws://192.168.1.101:8766"
SAMPLES_PATH = "./top_camera_calib.json"
BASE_FRAME = "Chassis_1"
EE_FRAME = "end_effector_p4_1"

# The calibration currently hardcoded in
# xarm_moveit_config/launch/_robot_moveit_realmove.launch.py  (x y z yaw pitch roll)
CURRENT_XYZ = [0.012652, 0.265518, -0.584171]
CURRENT_YPR = [-1.5708, 0.0, 1.0472]


# --------------------------------------------------------------------------
# pose helpers.  A "pose" here is always a 4x4 homogeneous matrix.
# --------------------------------------------------------------------------

def mat(t, R):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def from_quat(t, q):
    """q is [x, y, z, w] -- the order the camera edge sends."""
    return mat(np.asarray(t, float), Rotation.from_quat(q).as_matrix())


def from_ypr(xyz, ypr):
    """static_transform_publisher convention: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    return mat(np.asarray(xyz, float), Rotation.from_euler("ZYX", ypr).as_matrix())


def to_ypr(T):
    yaw, pitch, roll = Rotation.from_matrix(T[:3, :3]).as_euler("ZYX")
    return T[:3, 3].tolist(), [yaw, pitch, roll]


def params_to_mat(p):
    """6 params -> 4x4.  Rotation carried as a rotation vector (no gimbal lock)."""
    return mat(p[:3], Rotation.from_rotvec(p[3:6]).as_matrix())


def mat_to_params(T):
    return np.concatenate([T[:3, 3], Rotation.from_matrix(T[:3, :3]).as_rotvec()])


def avg_quat(qs):
    """Markley average: principal eigenvector of sum(q q^T)."""
    qs = np.asarray(qs, float)
    qs = qs * np.sign(qs @ qs[0])[:, None]   # resolve q/-q double cover
    w, v = np.linalg.eigh(qs.T @ qs)
    return v[:, np.argmax(w)]


# --------------------------------------------------------------------------
# capture
# --------------------------------------------------------------------------

class TagFeed:
    """Background websocket reader holding the most recent tag detections."""

    def __init__(self, url):
        self.url = url
        self.lock = threading.Lock()
        self.frames = []          # rolling (monotonic_time, {id: pose_matrix})
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        asyncio.run(self._loop())

    async def _loop(self):
        while True:
            try:
                async with websockets.connect(self.url, max_size=None) as ws:
                    while True:
                        m = json.loads(await ws.recv())
                        if m.get("type") != "inspect_frame":
                            continue
                        tags = {}
                        for t in m.get("apriltags") or []:
                            p = t.get("pose")
                            if p:
                                # list, not a single pose: the same id can appear
                                # twice (the gripper tag plus a fixture carrying
                                # the same id) and overwriting would silently mix
                                # the two together
                                tags.setdefault(int(t["id"]), []).append(
                                    from_quat(p["position"], p["orientation"]))
                        with self.lock:
                            self.frames.append((time.monotonic(), tags))
                            self.frames = self.frames[-120:]
            except Exception as e:
                print(f"  [websocket: {e}; retrying]", file=sys.stderr)
                await asyncio.sleep(2.0)

    def visible(self):
        """{tag id: how many copies of it are in frame}."""
        with self.lock:
            if not self.frames:
                return {}
            return {k: len(v) for k, v in self.frames[-1][1].items()}

    def collect(self, tag_id, seconds):
        """Observations of tag_id in the last `seconds`.

        Returns (observations, n_ambiguous) where n_ambiguous counts frames that
        held more than one tag with this id -- those are unusable, because there
        is no way to tell the gripper's tag from the other one.
        """
        cut = time.monotonic() - seconds
        obs, ambiguous = [], 0
        with self.lock:
            for ts, f in self.frames:
                if ts < cut or tag_id not in f:
                    continue
                d = f[tag_id]
                if len(d) > 1:
                    ambiguous += 1
                else:
                    obs.append(d[0])
        return obs, ambiguous


def capture(args):
    rclpy.init()
    node = Node("top_camera_calib")
    buf = Buffer()
    TransformListener(buf, node)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    feed = TagFeed(args.url)
    print(f"connecting to {args.url} ...")
    time.sleep(2.5)

    samples = []
    if os.path.exists(args.out) and not args.overwrite:
        samples = json.load(open(args.out))["samples"]
        print(f"appending to {len(samples)} existing samples in {args.out}")

    print(f"""
Tag {args.tag_id} must be rigidly taped to the gripper and stay put for the
whole session -- if it shifts, every sample before the shift is wrong.

For each sample: jog the arm somewhere the tag is clearly visible, let it come
to a full stop, then press ENTER. Vary ORIENTATION aggressively, not just
position -- poses that differ only by translation cannot pin down the rotation
and the fit will look good while being wrong. Aim for {args.min_samples}+.

  ENTER   record        u  undo last        q  finish and exit
""")

    while True:
        try:
            cmd = input(f"[{len(samples)} samples] > ").strip().lower()
        except EOFError:
            break
        if cmd == "q":
            break
        if cmd == "u":
            if samples:
                samples.pop()
                print("  removed last sample")
            continue

        vis = feed.visible()
        obs, ambiguous = feed.collect(args.tag_id, args.window)
        if ambiguous:
            print(f"  TWO tags with id {args.tag_id} in frame ({ambiguous} frames) "
                  f"-- cover or remove the one that is not on the gripper. "
                  f"Not recorded: there is no way to tell them apart.")
            continue
        if len(obs) < args.min_frames:
            print(f"  tag {args.tag_id} seen in only {len(obs)} frames "
                  f"(need {args.min_frames}). Visible tags: "
                  f"{dict(vis) if vis else 'none'}")
            continue

        A = mat(np.mean([o[:3, 3] for o in obs], axis=0),
                Rotation.from_quat(avg_quat(
                    [Rotation.from_matrix(o[:3, :3]).as_quat() for o in obs]
                )).as_matrix())

        spread = np.std([o[:3, 3] for o in obs], axis=0)
        if spread.max() > args.max_jitter:
            print(f"  tag pose unstable (sigma {1000*spread.max():.1f} mm) "
                  f"-- is the arm still moving? not recorded")
            continue

        try:
            tf = buf.lookup_transform(BASE_FRAME, EE_FRAME, rclpy.time.Time())
        except Exception as e:
            print(f"  TF lookup failed: {e}")
            continue
        tr, ro = tf.transform.translation, tf.transform.rotation
        B = from_quat([tr.x, tr.y, tr.z], [ro.x, ro.y, ro.z, ro.w])

        samples.append({"A": A.tolist(), "B": B.tolist(), "frames": len(obs)})
        json.dump({"tag_id": args.tag_id, "samples": samples},
                  open(args.out, "w"), indent=1)
        print(f"  recorded: tag at ({A[0,3]:+.3f},{A[1,3]:+.3f},{A[2,3]:+.3f}) m "
              f"from {len(obs)} frames, sigma {1000*spread.max():.1f} mm -> {args.out}")

    rclpy.shutdown()
    print(f"\n{len(samples)} samples in {args.out}")
    if len(samples) >= 3:
        print("now run:  ./calibrate_top_camera.py solve")
    else:
        print("need at least 3 samples to solve")


# --------------------------------------------------------------------------
# solve
# --------------------------------------------------------------------------

def residuals(p, A, B, rot_w):
    """Full 6-DoF: uses both the tag's position and its orientation."""
    X, Y = params_to_mat(p[:6]), params_to_mat(p[6:])
    out = []
    for a, b in zip(A, B):
        E = np.linalg.inv(X @ a) @ (b @ Y)          # identity for a perfect fit
        out.append(np.concatenate([
            E[:3, 3],
            rot_w * Rotation.from_matrix(E[:3, :3]).as_rotvec(),
        ]))
    return np.concatenate(out)


def residuals_t(p, A, B, rot_w=None):
    """Position-only.  p = [X (6 params), Y translation (3 params)].

    solvePnP recovers a planar tag's POSITION well but its NORMAL poorly -- the
    classic planar pose ambiguity. On a small tag that leaves a systematic error
    of several degrees in the reported orientation, which no rigid transform can
    absorb, so it leaks into X and corrupts the answer while the translation
    residual still looks reasonable.

    Dropping the orientation costs nothing: X stays fully observable because the
    tag's position still sweeps through the camera's view as the arm moves, and
    Y's rotation becomes irrelevant. 9 unknowns, 3 equations per pose.
    """
    X, Yt = params_to_mat(p[:6]), p[6:9]
    return np.concatenate([(X @ a)[:3, 3] - (b[:3, :3] @ Yt + b[:3, 3])
                           for a, b in zip(A, B)])


def _fit(A, B, full, rot_w, restarts):
    """Y is entirely unknown, so try several starts and keep the best."""
    X0 = from_ypr(CURRENT_XYZ, CURRENT_YPR)
    f, ny = (residuals, 6) if full else (residuals_t, 3)
    best, rng = None, np.random.default_rng(0)
    for k in range(restarts):
        if k == 0:
            y0 = np.zeros(ny)
        else:
            y0 = np.concatenate([rng.normal(0, .09, 3), rng.normal(0, 1., 3)])[:ny]
        try:
            res = least_squares(f, np.concatenate([mat_to_params(X0), y0]),
                                args=(A, B, rot_w), method="lm", max_nfev=40000)
        except Exception:
            continue
        if best is None or res.cost < best.cost:
            best = res
    return best


def _per_sample(p, A, B, full):
    if full:
        r = residuals(p, A, B, 1.0).reshape(-1, 6)
        return (1000 * np.linalg.norm(r[:, :3], axis=1),
                np.degrees(np.linalg.norm(r[:, 3:], axis=1)))
    return 1000 * np.linalg.norm(residuals_t(p, A, B).reshape(-1, 3), axis=1), None


def solve(args):
    data = json.load(open(args.samples))
    A = [np.array(s["A"]) for s in data["samples"]]
    B = [np.array(s["B"]) for s in data["samples"]]
    n = len(A)
    full = args.use_rotation
    print(f"{n} samples from {args.samples} (tag id {data.get('tag_id')}), "
          f"{'6-DoF' if full else 'position-only'} fit\n")
    if n < 4:
        sys.exit("need at least 4 samples")

    dup = [(i, j) for i in range(n) for j in range(i + 1, n)
           if np.linalg.norm(B[i][:3, 3] - B[j][:3, 3]) < 1e-4
           and Rotation.from_matrix(B[i][:3, :3] @ B[j][:3, :3].T).magnitude() < 1e-3]
    for i, j in dup:
        print(f"  note: samples {i} and {j} are the same arm pose -- {j} adds nothing")

    best = _fit(A, B, full, args.rot_weight, args.restarts)
    t_mm, r_deg = _per_sample(best.x, A, B, full)

    print("\nper-sample residual:")
    for k in range(n):
        line = f"  {k:2d}  {t_mm[k]:6.1f} mm"
        if r_deg is not None:
            line += f"  {r_deg[k]:7.2f} deg"
        print(line + ("   <-- outlier" if t_mm[k] > 3 * np.median(t_mm) else ""))

    X0 = from_ypr(CURRENT_XYZ, CURRENT_YPR)
    y0 = np.zeros(6 if full else 3)
    t0, _ = _per_sample(np.concatenate([mat_to_params(X0), y0]), A, B, full)
    print(f"\n  before: {t0.mean():6.1f} mm mean / {t0.max():6.1f} max")
    print(f"  after:  {t_mm.mean():6.1f} mm mean / {t_mm.max():6.1f} max")
    if r_deg is not None:
        print(f"  rotation residual {r_deg.mean():.2f} deg mean / {r_deg.max():.2f} max")
        if r_deg.mean() > 2.0:
            print("  ^ too large to trust. The tag orientation is unreliable; "
                  "re-run WITHOUT --use-rotation.")

    # Leave-one-out: the honest measure of whether this is a measurement or a
    # coincidence. A low residual on a badly-conditioned capture proves nothing.
    Xs = []
    for lo in range(n):
        idx = [i for i in range(n) if i != lo]
        r = _fit([A[i] for i in idx], [B[i] for i in idx], full,
                 args.rot_weight, args.restarts)
        if r is not None:
            Xs.append(r.x[:6])
    Xs = np.array(Xs)
    t_sig = 1000 * Xs[:, :3].std(0)
    r_sig = np.degrees(Xs[:, 3:6].std(0)).max()
    print(f"\nleave-one-out stability over {len(Xs)} refits:")
    print(f"  translation  {t_sig[0]:.1f}, {t_sig[1]:.1f}, {t_sig[2]:.1f} mm")
    print(f"  rotation     {r_sig:.2f} deg")
    if t_sig.max() > 10 or r_sig > 2:
        print("  ^ unstable: the answer depends on which samples you happened to "
              "take. Capture more poses with more orientation variety.")

    X = params_to_mat(best.x[:6])
    Yt = best.x[6:9] if not full else params_to_mat(best.x[6:])[:3, 3]
    xyz, ypr = to_ypr(X)
    print(f"\n  tag sits {1000*np.linalg.norm(Yt):.0f} mm from {EE_FRAME} "
          f"-- compare against where you actually mounted it. If this is "
          f"nonsense the fit is nonsense, whatever the residual says.")

    print(f"""
Replace the arguments in
robot/ros_ws/src/autonomy/5_planning/xarm_ros2/xarm_moveit_config/launch/_robot_moveit_realmove.launch.py

        arguments=['{xyz[0]:.6f}', '{xyz[1]:.6f}', '{xyz[2]:.6f}',
                   '{ypr[0]:.6f}', '{ypr[1]:.6f}', '{ypr[2]:.6f}',   # yaw pitch roll
                   'Chassis_1', 'top_camera']

was ['{CURRENT_XYZ[0]:.6f}', '{CURRENT_XYZ[1]:.6f}', '{CURRENT_XYZ[2]:.6f}', \
'{CURRENT_YPR[0]:.6f}', '{CURRENT_YPR[1]:.6f}', '{CURRENT_YPR[2]:.6f}']
moved {1000*np.linalg.norm(np.array(xyz)-np.array(CURRENT_XYZ)):.1f} mm""")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    c = sub.add_parser("capture", help="record tag/arm pose pairs")
    c.add_argument("--tag-id", type=int, default=1)
    c.add_argument("--url", default=WS_URL)
    c.add_argument("--out", default=SAMPLES_PATH)
    c.add_argument("--overwrite", action="store_true")
    c.add_argument("--window", type=float, default=1.5,
                   help="seconds of detections averaged per sample")
    c.add_argument("--min-frames", type=int, default=5)
    c.add_argument("--min-samples", type=int, default=12)
    c.add_argument("--max-jitter", type=float, default=0.004,
                   help="reject a sample if tag position sigma exceeds this (m)")
    c.set_defaults(func=capture)

    s = sub.add_parser("solve", help="fit X and print the launch-file line")
    s.add_argument("--samples", default=SAMPLES_PATH)
    s.add_argument("--rot-weight", type=float, default=0.1,
                   help="metres per radian when balancing the two error terms")
    s.add_argument("--restarts", type=int, default=12)
    s.add_argument("--use-rotation", action="store_true",
                   help="also fit the tag's orientation (6-DoF). Off by default: "
                        "a small planar tag's normal is unreliable and corrupts X")
    s.set_defaults(func=solve)

    args = ap.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
