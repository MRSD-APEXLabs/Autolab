#!/usr/bin/env python3
"""Configure Isaac Sim to publish the moving robot base transform.

Run with Isaac Sim's ``python.sh`` while the GUI process is stopped.  The
script is idempotent: it replaces only ``/World/TFActionGraph`` and authors
ROS frame-name overrides on the selected prims.
"""

import argparse
import traceback

ROBOT_ROOT_PREFIX = "/World/Autolab/Autolab"
ZED_X_CAMERA = (
    "/World/Autolab/Autolab/Group_2/Chassis_01/"
    "ZED_X/base_link/ZED_X/CameraLeft"
)
ZED_NANO_CAMERA = (
    "/World/Autolab/Autolab/Group_1/ZED_X_Nano/"
    "base_link/ZED_X_Nano/CameraLeft"
)
BASE_TF_PROXY = "/World/Autolab/Autolab/Group_2/TFBaseFootprint"

# USD import appended a zero to a number of CAD link names.  These are the
# corresponding link names in robot_isaac.urdf and therefore the frame names
# RViz's RobotModel expects.
FRAME_NAME_OVERRIDES = {
    "Group_2": "Chassis_1",
    "Group_1": "end_effector_p4_1",
    "Chassis_01": "Chassis_1",
    "arm_shoulder241": "arm_shoulder241_1",
    "arm_shoulder241_01": "arm_shoulder241_1",
    "arm_elbow_4": "arm_elbow_4_1",
    "arm_elbow_4_01": "arm_elbow_4_1",
    "elbow2_p4": "elbow2_p4_1",
    "elbow2_p4_01": "elbow2_p4_1",
    "wrist_1_p4": "wrist_1_p4_1",
    "wrist_1_p4_01": "wrist_1_p4_1",
    "wrist_2_p4": "wrist_2_p4_1",
    "wrist_2_p4_01": "wrist_2_p4_1",
    "end_effector_p4_01": "end_effector_p4_1",
    "gripper_v43_01": "gripper_v43_1",
    "Finger_left": "Finger_left_2",
    "Finger_right": "Finger_right_2",
    "Az_Front_Left": "Az_Front_Left_1",
    "Az_Front_right": "Az_Front_right_1",
    "Az_Back_Right": "Az_Back_Right_1",
    "Az_Back_left": "Az_Back_left_1",
    "Wheel_Front_Left": "Wheel_Front_Left_1",
    "Wheel_Front_Right": "Wheel_Front_Right_1",
    "Wheel_Back_Right": "Wheel_Back_Right_1",
    "Wheel_Back_Left": "Wheel_Back_Left_1",
}


def update_frame_names(stage_path: str) -> None:
    """Update frame metadata and remove obsolete TF publishers using USD only."""
    from pxr import Gf, Sdf, Usd, UsdGeom

    stage = Usd.Stage.Open(stage_path)
    if stage is None:
        raise RuntimeError(f"Could not open USD stage: {stage_path}")

    for obsolete_path in ("/World/TFBaseFootprint",):
        obsolete = stage.GetPrimAtPath(obsolete_path)
        if obsolete.IsValid():
            stage.RemovePrim(obsolete.GetPath())

    for prim in stage.Traverse():
        if not str(prim.GetPath()).startswith(ROBOT_ROOT_PREFIX):
            continue
        override = FRAME_NAME_OVERRIDES.get(prim.GetName())
        if override:
            prim.CreateAttribute("isaac:nameOverride", Sdf.ValueTypeNames.String).Set(override)

    # An identity child of the moving chassis gives Isaac a dedicated ROS
    # frame to publish without also taking ownership of the URDF link tree.
    base_proxy = UsdGeom.Xform.Define(stage, BASE_TF_PROXY).GetPrim()
    base_proxy.CreateAttribute("isaac:nameOverride", Sdf.ValueTypeNames.String).Set(
        "base_footprint"
    )
    xformable = UsdGeom.Xformable(base_proxy)
    xformable.ClearXformOpOrder()
    xformable.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
    xformable.AddOrientOp().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    xformable.AddScaleOp().Set(Gf.Vec3d(1.0, 1.0, 1.0))

    for camera_path, frame_name in (
        (ZED_X_CAMERA, "zed_x_left_camera"),
        (ZED_NANO_CAMERA, "zed_x_nano_left_camera"),
    ):
        camera = stage.GetPrimAtPath(camera_path)
        if not camera.IsValid():
            raise RuntimeError(f"Invalid camera prim: {camera_path}")
        camera.CreateAttribute("isaac:nameOverride", Sdf.ValueTypeNames.String).Set(frame_name)

    # The display URDF's local link origins differ from the imported PhysX
    # rigid-body origins.  Publish cameras from Isaac, but let
    # robot_state_publisher own the connected mechanical robot tree.
    for node_name in (
        "ComputeRobotTF",
        "PublishRobotTF",
        "ComputeZedXLeftTF",
        "PublishZedXLeftTF",
        "ComputeZedXNanoLeftTF",
        "PublishZedXNanoLeftTF",
    ):
        node_path = f"/World/TFActionGraph/{node_name}"
        if stage.GetPrimAtPath(node_path).IsValid():
            stage.RemovePrim(node_path)

    stage.GetRootLayer().Save()
    print(f"Updated Isaac TF frame names in {stage_path}")


def configure(stage_path: str, inspect_only: bool = False) -> None:
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True})
    try:
        import omni.graph.core as og
        import omni.kit.app
        import omni.usd
        from pxr import Sdf, UsdPhysics

        # Ensure both node families are registered before authoring the graph.
        manager = omni.kit.app.get_app().get_extension_manager()
        manager.set_extension_enabled_immediate("isaacsim.core.nodes", True)
        manager.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)

        context = omni.usd.get_context()
        if not context.open_stage(stage_path):
            raise RuntimeError(f"Could not open USD stage: {stage_path}")
        stage = context.get_stage()

        articulation_roots = []
        rigid_prims = []
        for prim in stage.Traverse():
            path = str(prim.GetPath())
            if not path.startswith(ROBOT_ROOT_PREFIX):
                continue
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                articulation_roots.append(path)
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                rigid_prims.append(path)

        print("Articulation roots:")
        for path in articulation_roots:
            print(f"  {path}")
        print("Robot rigid-body prims:")
        for path in rigid_prims:
            print(f"  {path}")

        if inspect_only:
            return
        if not articulation_roots:
            raise RuntimeError(f"No articulation root found under {ROBOT_ROOT_PREFIX}")

        for prim in stage.Traverse():
            if not str(prim.GetPath()).startswith(ROBOT_ROOT_PREFIX):
                continue
            override = FRAME_NAME_OVERRIDES.get(prim.GetName())
            if override:
                prim.CreateAttribute("isaac:nameOverride", Sdf.ValueTypeNames.String).Set(override)

        from pxr import Gf, UsdGeom

        # This prim follows the physical chassis but has the ROS root-frame
        # name expected by robot_state_publisher.
        base_proxy = UsdGeom.Xform.Define(stage, BASE_TF_PROXY).GetPrim()
        base_proxy.CreateAttribute("isaac:nameOverride", Sdf.ValueTypeNames.String).Set(
            "base_footprint"
        )
        xformable = UsdGeom.Xformable(base_proxy)
        xformable.ClearXformOpOrder()
        xformable.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
        xformable.AddOrientOp().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        xformable.AddScaleOp().Set(Gf.Vec3d(1.0, 1.0, 1.0))

        graph_path = "/World/TFActionGraph"
        if stage.GetPrimAtPath(graph_path).IsValid():
            stage.RemovePrim(graph_path)

        keys = og.Controller.Keys
        create_nodes = [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("RosContext", "isaacsim.ros2.bridge.ROS2Context"),
        ]
        set_values = [
            ("ReadSimTime.inputs:resetOnStop", True),
        ]
        create_nodes.extend(
            [
                ("ComputeBaseTF", "isaacsim.core.nodes.IsaacComputeTransformTree"),
                ("PublishBaseTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ]
        )
        set_values.extend(
            [
                ("ComputeBaseTF.inputs:targetPrims", [Sdf.Path(BASE_TF_PROXY)]),
                ("PublishBaseTF.inputs:topicName", "/tf"),
            ]
        )
        connections = [
            ("OnPlaybackTick.outputs:tick", "ComputeBaseTF.inputs:execIn"),
            ("ComputeBaseTF.outputs:execOut", "PublishBaseTF.inputs:execIn"),
            ("ComputeBaseTF.outputs:parentFrames", "PublishBaseTF.inputs:parentFrames"),
            ("ComputeBaseTF.outputs:childFrames", "PublishBaseTF.inputs:childFrames"),
            ("ComputeBaseTF.outputs:translations", "PublishBaseTF.inputs:translations"),
            ("ComputeBaseTF.outputs:orientations", "PublishBaseTF.inputs:orientations"),
            ("ReadSimTime.outputs:simulationTime", "PublishBaseTF.inputs:timeStamp"),
            ("RosContext.outputs:context", "PublishBaseTF.inputs:context"),
        ]

        og.Controller.edit(
            {
                "graph_path": graph_path,
                "evaluator_name": "execution",
                "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
            },
            {
                keys.CREATE_NODES: create_nodes,
                keys.SET_VALUES: set_values,
                keys.CONNECT: connections,
            },
        )

        context.save_stage()
        print(f"Configured Isaac TF publishing in {stage_path}")
    except Exception:
        # Isaac 6 can crash while tearing down OmniGraph after an authoring
        # error, which otherwise hides the useful Python exception.
        traceback.print_exc()
        raise
    finally:
        app.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("stage", help="USD stage to inspect or update")
    parser.add_argument("--inspect-only", action="store_true")
    parser.add_argument(
        "--update-frame-names-only",
        action="store_true",
        help="Patch an existing TF graph without starting the full SimulationApp",
    )
    args = parser.parse_args()
    if args.update_frame_names_only:
        update_frame_names(args.stage)
    else:
        configure(args.stage, args.inspect_only)
