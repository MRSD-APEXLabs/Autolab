"""Check a recorded episode (or list a whole dataset).

    ./inspect.sh data/wellplate_place/episodes/episode_0001.hdf5 --montage m.png --video v.mp4 --plot p.png
    ./inspect.sh data/wellplate_place          # index of all kept episodes + totals
"""

import argparse
import json
import sys
from pathlib import Path

import cv2
import h5py
import numpy as np

from .episode_io import decode_depth, decode_image, episode_files, index_rows


def depth_vis(d: np.ndarray, lo: float = 100.0, hi: float = 1500.0) -> np.ndarray:
    v = cv2.applyColorMap((255 * (1 - np.clip((d.astype(np.float32) - lo) / (hi - lo), 0, 1))).astype(np.uint8), cv2.COLORMAP_TURBO)
    v[d == 0] = 0
    return v


def panel(f: h5py.File, i: int) -> np.ndarray:
    return np.hstack([decode_image(f["frames/cam_left"][i]), decode_image(f["frames/cam_right"][i]), depth_vis(decode_depth(f["frames/depth"][i]))])


def plot(f: h5py.File, out: Path):
    t0 = f["frames/t"][0]
    W, H, pad = 1400, 900, 50
    img = np.full((H, W, 3), 255, np.uint8)
    blocks = [("joints q (deg)", f["robot/t"][:], f["robot/q_deg"][:]),
              ("TCP x y z (mm)", f["robot/t"][:], f["robot/tcp_pose"][:, :3]),
              ("commanded vx vy vz (mm/s) wz (deg/s)", f["control/t"][:], f["control/cmd_vel"][:])]
    colors = [(200, 0, 0), (0, 150, 0), (0, 0, 220), (0, 140, 255), (160, 0, 160), (90, 90, 90)]
    bh = (H - pad) // len(blocks)
    tmax = max(b[1][-1] for b in blocks) - t0
    for bi, (title, t, y) in enumerate(blocks):
        y0 = pad // 2 + bi * bh
        cv2.putText(img, title, (10, y0 + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        top, bottom = y0 + 24, y0 + bh - 12
        lo, hi = float(y.min()), float(y.max())
        span = (hi - lo) or 1.0
        cv2.rectangle(img, (pad, top), (W - 20, bottom), (200, 200, 200), 1)
        cv2.putText(img, f"{hi:.1f}", (2, top + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)
        cv2.putText(img, f"{lo:.1f}", (2, bottom), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)
        for c in range(y.shape[1]):
            xs = pad + (t - t0) / tmax * (W - 20 - pad)
            ys = bottom - (y[:, c] - lo) / span * (bottom - top)
            cv2.polylines(img, [np.stack([xs, ys], 1).astype(np.int32)], False, colors[c % 6], 1, cv2.LINE_AA)
    cv2.putText(img, f"time 0 .. {tmax:.1f} s", (pad, H - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
    cv2.imwrite(str(out), img)


def inspect_file(path: Path, args):
    with h5py.File(path) as f:
        a = f.attrs
        t, rt, ct = f["frames/t"][:], f["robot/t"][:], f["control/t"][:]
        n = len(t)
        print(f"{path.name}: dataset '{a['dataset']}', task '{a['task']}', created {a['created']}, git {a['git_hash']}")
        rec = int(a.get("stat_n_recorded", n))
        print(f"  duration {a['stat_saved_duration_s'] if 'stat_saved_duration_s' in a else a['stat_duration_s']:.2f} s saved"
              + (f" (the recording lasted {a['stat_duration_s']:.1f} s)" if 'stat_saved_duration_s' in a else "") + f" | camera recorded {rec} frames @ {a['stat_cam_hz']:.2f} Hz (max gap "
              f"{a['stat_max_frame_gap_ms']:.0f} ms, {a['stat_frame_id_gaps']} lost) -> {n} kept, {rec - n} duplicates removed"
              f" | image {[int(x) for x in a['image_size_wh']]} scale {a['image_scale']:.3f}, depth {a['depth_codec']}")
        print(f"  robot {len(rt)} rows @ {(len(rt)-1)/(rt[-1]-rt[0]):.1f} Hz (max gap {np.diff(rt).max()*1e3:.0f} ms) | "
              f"control {len(ct)} rows | gripper {len(f['gripper/t'])} reads | faults {a['stat_faults']}")
        print(f"  clock offset Xavier-labx: {a['clock_offset_pre_s']*1e3:+.2f} ms -> {a['clock_offset_post_s']*1e3:+.2f} ms "
              f"(rtt {a['clock_rtt_pre_ms']:.1f}/{a['clock_rtt_post_ms']:.1f} ms), timestamps: {a['timestamp_source']}")
        span_ok = t[0] >= rt[0] and t[-1] <= rt[-1]
        print(f"  camera frames inside robot log span: {span_ok}; kept-frame spacing: median {np.median(np.diff(t))*1e3:.0f} ms, max {np.diff(t).max()*1e3:.0f} ms")
        q = f["robot/q_deg"][:]
        print("  joint range (deg): " + "  ".join(f"J{j+1} {q[:,j].min():.1f}..{q[:,j].max():.1f}" for j in range(6)))
        tcp = f["robot/tcp_pose"][:]
        print(f"  TCP start {np.round(tcp[0,:3],1)} -> end {np.round(tcp[-1,:3],1)} mm; travelled x {np.ptp(tcp[:,0]):.1f} y {np.ptp(tcp[:,1]):.1f} z {np.ptp(tcp[:,2]):.1f} mm")
        cmd = f["control/cmd_vel"][:]
        print(f"  commanded peaks: vx {np.abs(cmd[:,0]).max():.1f} vy {np.abs(cmd[:,1]).max():.1f} vz {np.abs(cmd[:,2]).max():.1f} mm/s, wz {np.abs(cmd[:,3]).max():.1f} deg/s")
        idx = np.clip(np.searchsorted(rt, t), 0, len(rt) - 1)
        print(f"  aligned qpos vs nearest raw robot row: max diff {np.abs(f['aligned/qpos'][:] - q[idx]).max():.3f} deg")
        ev = json.loads(a["events"])
        if ev:
            print("  events: " + "; ".join(f"+{e[0]-t[0]:.1f}s {e[1]}" for e in ev))
        gr = f["gripper/fsr"][:]
        if len(gr):
            print(f"  gripper setpoint {f['gripper/setpoint'][0]}, FSR A/B mean {gr[:,0].mean():.0f}/{gr[:,1].mean():.0f}")
        d0 = decode_depth(f["frames/depth"][n // 2])
        print(f"  depth (middle frame): valid {100*(d0>0).mean():.1f}%, range {d0[d0>0].min() if (d0>0).any() else 0}..{d0.max()} mm")
        if args.montage:
            picks = [int(x) for x in np.linspace(0, n - 1, 5)]
            m = np.vstack([cv2.resize(panel(f, i), None, fx=0.5, fy=0.5) for i in picks])
            cv2.imwrite(args.montage, m)
            print(f"  montage of frames {picks} (left | right | depth) -> {args.montage}")
        if args.plot:
            plot(f, Path(args.plot))
            print(f"  plot -> {args.plot}")
        if args.video:
            hz = 30.0  # kept frames are not evenly spaced: the video shows them one after another at a fixed rate
            first = panel(f, 0)
            vw = cv2.VideoWriter(args.video, cv2.VideoWriter_fourcc(*"mp4v"), hz, (first.shape[1], first.shape[0]))
            if not vw.isOpened():
                print("  video: this OpenCV build cannot write mp4v; use --montage instead", file=sys.stderr)
            else:
                for i in range(n):
                    vw.write(panel(f, i))
                vw.release()
                print(f"  video ({n} frames @ {hz:.1f} Hz) -> {args.video}")


def list_dataset(root: Path):
    rows = index_rows(root)  # read from the episode files themselves, so it is always current
    files = episode_files(root)
    print(f"{root}: {len(files)} episode files")
    tot = 0.0
    for r in rows:
        tot += float(r["duration_s"])
        print(f"  {int(r['episode_id']):4d}  {r['duration_s']:>6} s  {r['n_frames']:>5}/{r['n_recorded']:<5} fr kept  cam {r['cam_hz']:>5} Hz  gap {r['max_frame_gap_ms']:>5} ms  lost {r['frame_id_gaps']:>3}  "
              f"robot {r['robot_hz']:>5} Hz  faults {r['faults']}  {r['size_mb']:>6} MB  {r['notes']}")
    print(f"  total {tot/60:.1f} min of demonstrations, {sum(p.stat().st_size for p in files)/1e9:.2f} GB")


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("path", help="episode .hdf5 or a dataset directory")
    ap.add_argument("--montage")
    ap.add_argument("--video")
    ap.add_argument("--plot")
    args = ap.parse_args(argv)
    p = Path(args.path)
    if p.is_dir():
        list_dataset(p)
    else:
        inspect_file(p, args)


if __name__ == "__main__":
    main()
