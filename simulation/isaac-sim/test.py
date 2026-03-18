from pxr import UsdPhysics, Usd
import omni.usd

stage = omni.usd.get_context().get_stage()

print("=== Articulation Roots ===")
for prim in stage.Traverse():
    if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        print(f"  ArticulationRoot at: {prim.GetPath()}")

print("\n=== Fixed Joints ===")
for prim in stage.Traverse():
    if prim.IsA(UsdPhysics.FixedJoint):
        b0 = prim.GetRelationship("physics:body0").GetTargets()
        b1 = prim.GetRelationship("physics:body1").GetTargets()
        print(f"  FixedJoint: {prim.GetPath()} | body0={b0} | body1={b1}")

print("\n=== Revolute Joints + Drive Targets ===")
for prim in stage.Traverse():
    if prim.IsA(UsdPhysics.RevoluteJoint):
        drive = UsdPhysics.DriveAPI.Get(prim, "angular")
        stiffness = drive.GetStiffnessAttr().Get() if drive else "NO DRIVE"
        target = drive.GetTargetPositionAttr().Get() if drive else "NO DRIVE"
        print(f"  {prim.GetPath()} | stiffness={stiffness} | target={target}")


from pxr import UsdPhysics
import omni.usd

stage = omni.usd.get_context().get_stage()

for prim in stage.Traverse():
    if prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.PrismaticJoint):
        drive_type = "angular" if prim.IsA(UsdPhysics.RevoluteJoint) else "linear"
        drive = UsdPhysics.DriveAPI.Get(prim, drive_type)
        if drive:
            drive.GetStiffnessAttr().Set(1000000.0)
            drive.GetDampingAttr().Set(100.0)
            drive.GetTargetPositionAttr().Set(0.0)
            print(f"Reset: {prim.GetPath()}")

print("Done!")