# Calibrate the base camera (`top_camera`)

The base ZED's pose is six hardcoded numbers in
[`_robot_moveit_realmove.launch.py`](../robot/ros_ws/src/autonomy/5_planning/xarm_ros2/xarm_moveit_config/launch/_robot_moveit_realmove.launch.py)
(~line 227). Nothing in the repo produced them and nothing checks them. When the
point cloud in RViz sits at an angle to the real world, or plans aim beside the
target rather than at it, these are the numbers that are wrong.

`calibrate_top_camera.py` measures them. Tape an AprilTag to the gripper, move
the arm to a dozen poses, and for each pose two independent readings of the same
point exist: where the camera sees the tag, and where forward kinematics says the
gripper is. The transform that reconciles all of them is the calibration.

    X . A_i = B_i . Y

    X    Chassis_1 -> top_camera           what we want
    A_i  top_camera -> tag                 camera, via solvePnP
    B_i  Chassis_1 -> end_effector_p4_1    TF
    Y    end_effector_p4_1 -> tag          where the tag sits on the gripper

`Y` is solved for, not measured — **you do not need to place the tag anywhere in
particular or know its offset.** 12 unknowns, 6 equations per pose: 3 poses is the
theoretical minimum, ~12 is what actually works.

Hand-tuning these numbers instead is a trap: `world -> Chassis_1` carries a 180°
pitch flip ([`robot.urdf`](../robot/ros_ws/src/autonomy/5_planning/base_urdf/robot.urdf) ~line 298),
so `Chassis_1`'s +Z points **down** and every sign is inverted from what you expect.

## 1. Print the tag

`tag36h11`, **id 1**, black border exactly **62.5 mm**, with white quiet zone at
least one tag-cell wide left around it.

`inspect_cam.py` only reports ids 0, 1 and 2 (`if r.tag_id not in [0, 1, 2]`), so
the gripper tag has to be one of those three. **Exactly one tag with the chosen
id may be in frame** -- two tags sharing an id are indistinguishable, and
`capture` refuses the sample rather than guessing which is which.

The tag physically stuck on the OT-2 deck is **id 2** (confirmed from the
detector overlay, whatever `launch-planning.md` says about a shaker), so id 1 is
clear as long as no other id-1 tag is in view.

`plan_april_1` targets wherever tag 1 is currently seen, so while the tag is on
the gripper that command follows it and stops being meaningful.

**Measure the print with calipers before mounting it.** `apriltag_size_m = 0.0625`
is hardcoded in `inspect_cam.py` on the Xavier. "Fit to page" silently scales the
print, and a 5% scale error puts a 5% error into every translation the camera
reports. This is the most common way this calibration fails, and it fails
*quietly* — detection still works and the residuals still look small.

## 2. Mount it

Glue the tag to a **rigid flat card**, then attach the card to the gripper:

- **offset to one side, facing up and outward.** The camera is ~1.48 m above
  looking down, and the gripper's Z axis points down at the work, so anything on
  the underside is never seen. joint6 rolls about that same Z, so a tag centred
  on the axis barely moves when you roll — mounting it off-axis makes joint6
  sweep it through a wide arc, which is what pins down the rotation.
- **rigid.** `Y` is assumed identical for every pose. A tag that shifts mid-session
  makes the fit average the before and after instead of rejecting either — a
  confidently wrong answer with small-looking residuals.
- **flat.** `solvePnP` models the tag as a perfect plane; taped over a curved
  housing bakes in a bias.
- the card sticks out past the gripper — check it clears the workspace before
  jogging.

## 3. Bring the stack up

Planning stack running ([launch-planning.md](launch-planning.md) step 5; TF must
be live) and the camera in inspect mode (step 3). Verify from the Jetson:

```bash
ros2 run tf2_ros tf2_echo Chassis_1 end_effector_p4_1    # must update as the arm moves
```

## 4. Capture

```bash
cd ~/coding/Autolab && /usr/bin/python3 calibrate_top_camera.py capture
```

Jog the arm, let it come to a **full stop**, press ENTER. `u` undoes the last
sample, `q` finishes. Samples append to `top_camera_calib.json`, so you can stop
and resume.

Each sample averages 1.5 s of detections and is rejected if the tag position
sigma exceeds 4 mm — that rejection means the arm was still moving.

**Vary orientation aggressively, not just position.** Poses that differ only by
translation cannot observe the rotation at all: the fit will report a small
residual while being wrong. Roll and pitch the wrist differently on every pose.
Aim for 12+, spread across the workspace and across the camera's field of view.

## 5. Solve

```bash
/usr/bin/python3 calibrate_top_camera.py solve
```

**By default this fits the tag's position only and ignores its orientation.**
`solvePnP` recovers a planar tag's position well but its surface normal poorly,
and on a small tag that leaves a systematic several-degree error which no rigid
transform can absorb — it leaks into the answer while the translation residual
still looks fine. `--use-rotation` fits the full 6-DoF version; if its rotation
residual is more than ~2 deg mean, don't use the result.

Three things to read, in order of how much they matter:

- **leave-one-out stability** — refits with each sample removed in turn. This is
  the honest measure. Under ~3 mm and ~0.5 deg means the answer does not depend on
  which poses you happened to take. Large values mean the capture is
  under-constrained no matter how small the residual looks.
- **residual, before vs after** — a few mm is normal. Tens of mm means a bad tag
  print (step 1), a tag that moved (step 2), or too little orientation spread
  (step 4). Per-sample values are listed so you can delete an outlier and re-solve.
- **tag distance from the end effector** — compare against where you physically
  mounted it. If it's nonsense the fit is nonsense, whatever the residual says.

Duplicate arm poses are detected and called out; they add nothing to the fit.

Rotation error is amplified by distance: 0.5 deg is ~13 mm of lateral error out at
1.5 m. If you capture with the arm close to the camera, that is the accuracy you
are extrapolating to the working volume.

## 6. Apply

Paste the six printed numbers over the `arguments=[...]` list in
`_robot_moveit_realmove.launch.py` (~line 227; order is `x y z yaw pitch roll`),
then relaunch planning. It is a `static_transform_publisher`, so with
`--symlink-install` no rebuild is needed; if the old numbers persist:

```bash
docker exec autolab-robot-l4t-1 bash -lc 'cd ~/AutoLab/robot/ros_ws && colcon build --symlink-install --packages-select xarm_moveit_config'
```

Check it: re-run `pc3.py --fresh` and confirm in RViz that the table surface is
level and at the right height, and that the OT-2 deck lands where it physically is.

Commit the new numbers with the date and the residual the solve reported — it is
the only record that the calibration was ever done.
