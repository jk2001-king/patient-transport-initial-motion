"""Prepare a temporary Isaac Sim stage for active-wheel zero-offset checks.

Run with the Timeline stopped.  All edits are authored into the session layer
and must not be saved to the workspace USD.
"""

import omni.timeline
import omni.usd
from pxr import PhysxSchema, UsdPhysics


STEERING_JOINTS = (
    "/World/patient_transport/joints/front_steering_joint",
    "/World/patient_transport/joints/rear_steering_joint",
)
TIRE_MATERIAL = "/World/PatientTransportCalibrationMaterials/Tire"
GROUND_MATERIAL = "/World/PatientTransportCalibrationMaterials/Ground"
GROUND_COLLIDER = "/FlatGrid/GroundPlane/CollisionPlane"


def prepare_ackermann_zero_check(test_friction: float = 0.5, physics_hz: int = 120):
    timeline = omni.timeline.get_timeline_interface()
    if timeline.is_playing():
        raise RuntimeError("Press Stop before preparing the ACKERMANN zero check")
    if not 0.0 < float(test_friction) <= 1.5:
        raise ValueError("test_friction must be in (0, 1.5]")

    stage = omni.usd.get_context().get_stage()
    previous_target = stage.GetEditTarget()
    stage.SetEditTarget(stage.GetSessionLayer())
    try:
        scenes = [prim for prim in stage.Traverse() if prim.IsA(UsdPhysics.Scene)]
        if not scenes:
            raise RuntimeError("No PhysicsScene found")
        PhysxSchema.PhysxSceneAPI.Apply(scenes[0]).CreateTimeStepsPerSecondAttr(
            int(physics_hz)
        ).Set(int(physics_hz))

        # The persistent model is limited to raw +/-45 deg.  That range is
        # temporarily widened so the CAD-exported +/-90 deg zero candidates
        # can be tested.  This does not define the final physical limit.
        for path in STEERING_JOINTS:
            prim = stage.GetPrimAtPath(path)
            if not prim.IsValid() or not prim.IsA(UsdPhysics.RevoluteJoint):
                raise RuntimeError(f"Missing steering revolute joint: {path}")
            joint = UsdPhysics.RevoluteJoint(prim)
            joint.CreateLowerLimitAttr(-180.0).Set(-180.0)
            joint.CreateUpperLimitAttr(+180.0).Set(+180.0)

        for path in (TIRE_MATERIAL, GROUND_MATERIAL):
            prim = stage.GetPrimAtPath(path)
            if not prim.IsValid():
                raise RuntimeError(f"Missing physics material: {path}")
            material = UsdPhysics.MaterialAPI.Apply(prim)
            material.CreateStaticFrictionAttr(float(test_friction)).Set(
                float(test_friction)
            )
            material.CreateDynamicFrictionAttr(float(test_friction)).Set(
                float(test_friction)
            )

        ground = stage.GetPrimAtPath(GROUND_COLLIDER)
        if not ground.IsValid():
            raise RuntimeError(f"Missing ground collider: {GROUND_COLLIDER}")
        UsdPhysics.CollisionAPI.Apply(ground).CreateCollisionEnabledAttr(True).Set(True)
    finally:
        stage.SetEditTarget(previous_target)

    print("Temporary ACKERMANN-zero check stage prepared (session layer only).")
    print(f"raw steering search limits: -180 .. +180 deg, physics={physics_hz} Hz")
    print(f"direction-test friction={float(test_friction):.3f} (not calibrated)")
    print("Press Play, load isaac_ackermann_zero_check.py, and run the check.")
    print("Do not save the USD file.")


print("ACKERMANN-zero stage helper loaded. Timeline must be stopped.")
