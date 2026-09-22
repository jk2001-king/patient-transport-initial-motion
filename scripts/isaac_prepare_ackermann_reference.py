"""Temporarily apply the verified ACKERMANN steering reference.

Run with the Timeline stopped.  Values are written only to the session layer
so Play-start behaviour can be checked before the workspace USD is modified.
"""

import omni.timeline
import omni.usd
from pxr import PhysxSchema, UsdPhysics


STEERING_JOINTS = (
    "/World/patient_transport/joints/front_steering_joint",
    "/World/patient_transport/joints/rear_steering_joint",
)

RAW_ACKERMANN_ZERO_DEG = 90.0
PHYSICAL_LIMIT_DEG = 45.0
RAW_LOWER_LIMIT_DEG = RAW_ACKERMANN_ZERO_DEG - PHYSICAL_LIMIT_DEG
RAW_UPPER_LIMIT_DEG = RAW_ACKERMANN_ZERO_DEG + PHYSICAL_LIMIT_DEG
STEERING_KP = 10000.0
STEERING_KD = 1000.0


def prepare_ackermann_reference():
    timeline = omni.timeline.get_timeline_interface()
    if timeline.is_playing():
        raise RuntimeError("Press Stop before preparing the ACKERMANN reference")

    stage = omni.usd.get_context().get_stage()
    previous_target = stage.GetEditTarget()
    stage.SetEditTarget(stage.GetSessionLayer())
    try:
        for path in STEERING_JOINTS:
            prim = stage.GetPrimAtPath(path)
            if not prim.IsValid() or not prim.IsA(UsdPhysics.RevoluteJoint):
                raise RuntimeError(f"Missing steering revolute joint: {path}")

            joint = UsdPhysics.RevoluteJoint(prim)
            joint.CreateLowerLimitAttr(RAW_LOWER_LIMIT_DEG).Set(
                RAW_LOWER_LIMIT_DEG
            )
            joint.CreateUpperLimitAttr(RAW_UPPER_LIMIT_DEG).Set(
                RAW_UPPER_LIMIT_DEG
            )

            drive = UsdPhysics.DriveAPI.Get(prim, "angular")
            if not drive:
                drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
            drive.CreateTypeAttr("acceleration").Set("acceleration")
            drive.CreateStiffnessAttr(STEERING_KP).Set(STEERING_KP)
            drive.CreateDampingAttr(STEERING_KD).Set(STEERING_KD)
            drive.CreateTargetPositionAttr(RAW_ACKERMANN_ZERO_DEG).Set(
                RAW_ACKERMANN_ZERO_DEG
            )
            drive.CreateTargetVelocityAttr(0.0).Set(0.0)

            state = PhysxSchema.JointStateAPI.Get(prim, "angular")
            if not state:
                state = PhysxSchema.JointStateAPI.Apply(prim, "angular")
            if not state:
                raise RuntimeError(f"Could not apply angular JointStateAPI: {path}")
            state.CreatePositionAttr(RAW_ACKERMANN_ZERO_DEG).Set(
                RAW_ACKERMANN_ZERO_DEG
            )
            state.CreateVelocityAttr(0.0).Set(0.0)
    finally:
        stage.SetEditTarget(previous_target)

    print("Temporary ACKERMANN reference applied (session layer only).")
    print(
        f"front/rear raw initial={RAW_ACKERMANN_ZERO_DEG:.1f} deg, "
        f"target={RAW_ACKERMANN_ZERO_DEG:.1f} deg"
    )
    print(
        f"raw limits={RAW_LOWER_LIMIT_DEG:.1f} .. "
        f"{RAW_UPPER_LIMIT_DEG:.1f} deg"
    )
    print(
        f"physical mapping: delta_deg=raw_deg-{RAW_ACKERMANN_ZERO_DEG:.1f}; "
        f"physical limits=+/-{PHYSICAL_LIMIT_DEG:.1f} deg"
    )
    print("Press Play, load isaac_verify_ackermann_reference.py, and run verification.")
    print("Do not save the USD file yet.")


print("ACKERMANN-reference preparation helper loaded. Timeline must be stopped.")
