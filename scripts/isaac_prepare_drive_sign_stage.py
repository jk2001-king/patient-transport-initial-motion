"""Prepare a temporary ground-contact stage for the low-speed drive sign test."""

import omni.timeline
import omni.usd
from pxr import PhysxSchema, UsdPhysics


TIRE_MATERIAL = "/World/PatientTransportCalibrationMaterials/Tire"
GROUND_MATERIAL = "/World/PatientTransportCalibrationMaterials/Ground"
GROUND_COLLIDER = "/FlatGrid/GroundPlane/CollisionPlane"
ROBOT_ROOT = "/World/patient_transport"
STEERING_JOINTS = (
    "/World/patient_transport/joints/front_steering_joint",
    "/World/patient_transport/joints/rear_steering_joint",
)
STEERING_RAW_ZERO_DEG = 90.0


def prepare_drive_sign_stage(
    test_friction: float = 0.5,
    physics_hz: int = 120,
    initial_steering_limit_deg: float = 45.0,
):
    timeline = omni.timeline.get_timeline_interface()
    if timeline.is_playing():
        raise RuntimeError("Press Stop before preparing the drive sign test")
    if not 0.0 < float(test_friction) <= 1.5:
        raise ValueError("test_friction must be in (0, 1.5]")
    if not 0.0 < float(initial_steering_limit_deg) <= 90.0:
        raise ValueError("initial_steering_limit_deg must be in (0, 90]")

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

        raw_lower = STEERING_RAW_ZERO_DEG - float(initial_steering_limit_deg)
        raw_upper = STEERING_RAW_ZERO_DEG + float(initial_steering_limit_deg)
        for path in STEERING_JOINTS:
            prim = stage.GetPrimAtPath(path)
            if not prim.IsValid() or not prim.IsA(UsdPhysics.RevoluteJoint):
                raise RuntimeError(f"Missing steering revolute joint: {path}")
            joint = UsdPhysics.RevoluteJoint(prim)
            joint.CreateLowerLimitAttr(raw_lower).Set(raw_lower)
            joint.CreateUpperLimitAttr(raw_upper).Set(raw_upper)

        for path in (TIRE_MATERIAL, GROUND_MATERIAL):
            prim = stage.GetPrimAtPath(path)
            if not prim.IsValid():
                raise RuntimeError(f"Missing persistent calibration material: {path}")
            api = UsdPhysics.MaterialAPI.Apply(prim)
            api.CreateStaticFrictionAttr(float(test_friction)).Set(float(test_friction))
            api.CreateDynamicFrictionAttr(float(test_friction)).Set(float(test_friction))

        ground = stage.GetPrimAtPath(GROUND_COLLIDER)
        if not ground.IsValid():
            raise RuntimeError(f"Missing ground collider: {GROUND_COLLIDER}")
        UsdPhysics.CollisionAPI.Apply(ground).CreateCollisionEnabledAttr(True).Set(True)

        rigid_body_count = 0
        for prim in stage.Traverse():
            path = str(prim.GetPath())
            if not (path == ROBOT_ROOT or path.startswith(ROBOT_ROOT + "/")):
                continue
            if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                continue
            PhysxSchema.PhysxRigidBodyAPI.Apply(
                prim
            ).CreateDisableGravityAttr(False).Set(False)
            rigid_body_count += 1
        if rigid_body_count == 0:
            raise RuntimeError(f"No rigid bodies found below {ROBOT_ROOT}")
    finally:
        stage.SetEditTarget(previous_target)

    print("Temporary drive-sign stage prepared (session layer only).")
    print(f"test friction={float(test_friction)}, physics={int(physics_hz)} Hz")
    print(
        f"temporary initial steering range=+/-{float(initial_steering_limit_deg):.1f} deg "
        f"(raw limits {raw_lower:.1f} .. {raw_upper:.1f} deg)"
    )
    print("This friction is for direction/geometry testing, not a calibrated tire value.")
    print("Press Play, load isaac_drive_sign_test.py, then start one run.")
    print("Do not save the USD file.")


print("Drive-sign stage helper loaded. Timeline must be stopped.")
