"""Persist only the verified ACKERMANN initial JointState values.

The steering limits and drive settings must already have been authored by the
user.  This helper refuses to write if any of those prerequisite values differ
from the verified ACKERMANN reference.
"""

from pathlib import Path

import omni.timeline
import omni.usd
from pxr import PhysxSchema, UsdPhysics


EXPECTED_USD = Path(
    "/home/jk/Research/patient_initial_ws/isaac_model/patient_transport.usd"
).resolve()
STEERING_JOINTS = (
    "/World/patient_transport/joints/front_steering_joint",
    "/World/patient_transport/joints/rear_steering_joint",
)

EXPECTED = {
    "lower": 45.0,
    "upper": 135.0,
    "target_position": 90.0,
    "target_velocity": 0.0,
    "stiffness": 10000.0,
    "damping": 1000.0,
    "type": "acceleration",
}
INITIAL_POSITION_DEG = 90.0
INITIAL_VELOCITY_DEG_S = 0.0


def _close(actual, expected, tolerance=1e-5):
    return actual is not None and abs(float(actual) - float(expected)) <= tolerance


def persist_ackermann_joint_state():
    timeline = omni.timeline.get_timeline_interface()
    if timeline.is_playing():
        raise RuntimeError("Press Stop before writing the ACKERMANN Joint State")

    stage = omni.usd.get_context().get_stage()
    root_layer = stage.GetRootLayer()
    actual_path = Path(root_layer.realPath).resolve() if root_layer.realPath else None
    if actual_path != EXPECTED_USD:
        raise RuntimeError(
            f"Wrong stage root: {actual_path}. Open {EXPECTED_USD} and try again."
        )

    # Validate first so a partial or mistyped manual edit is never saved over.
    inspected = []
    for path in STEERING_JOINTS:
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid() or not prim.IsA(UsdPhysics.RevoluteJoint):
            raise RuntimeError(f"Missing steering revolute joint: {path}")
        joint = UsdPhysics.RevoluteJoint(prim)
        drive = UsdPhysics.DriveAPI.Get(prim, "angular")
        if not drive:
            raise RuntimeError(f"Missing angular drive: {path}")
        values = {
            "lower": joint.GetLowerLimitAttr().Get(),
            "upper": joint.GetUpperLimitAttr().Get(),
            "target_position": drive.GetTargetPositionAttr().Get(),
            "target_velocity": drive.GetTargetVelocityAttr().Get(),
            "stiffness": drive.GetStiffnessAttr().Get(),
            "damping": drive.GetDampingAttr().Get(),
            "type": str(drive.GetTypeAttr().Get()),
        }
        numeric_keys = (
            "lower",
            "upper",
            "target_position",
            "target_velocity",
            "stiffness",
            "damping",
        )
        mismatches = [
            f"{key}: actual={values[key]}, expected={EXPECTED[key]}"
            for key in numeric_keys
            if not _close(values[key], EXPECTED[key])
        ]
        if values["type"] != EXPECTED["type"]:
            mismatches.append(
                f"type: actual={values['type']}, expected={EXPECTED['type']}"
            )
        if mismatches:
            raise RuntimeError(
                f"Prerequisite ACKERMANN settings are incorrect at {path}: "
                + "; ".join(mismatches)
            )
        inspected.append((path, prim, values))

    previous_target = stage.GetEditTarget()
    stage.SetEditTarget(root_layer)
    try:
        for path, prim, _ in inspected:
            state = PhysxSchema.JointStateAPI.Get(prim, "angular")
            if not state:
                state = PhysxSchema.JointStateAPI.Apply(prim, "angular")
            if not state:
                raise RuntimeError(f"Could not apply angular JointStateAPI: {path}")
            state.CreatePositionAttr(INITIAL_POSITION_DEG).Set(INITIAL_POSITION_DEG)
            state.CreateVelocityAttr(INITIAL_VELOCITY_DEG_S).Set(
                INITIAL_VELOCITY_DEG_S
            )
    finally:
        stage.SetEditTarget(previous_target)

    layer_data = dict(root_layer.customLayerData)
    layer_data["patientTransportAckermannReference"] = {
        "frontRawZeroDeg": INITIAL_POSITION_DEG,
        "rearRawZeroDeg": INITIAL_POSITION_DEG,
        "rawLowerLimitDeg": EXPECTED["lower"],
        "rawUpperLimitDeg": EXPECTED["upper"],
        "physicalAngleDefinition": "physical_deg = raw_deg - 90",
        "source": "isaac_ackermann_zero_check run01",
    }
    root_layer.customLayerData = layer_data
    root_layer.Save()

    print("ACKERMANN initial Joint State saved to the workspace USD.")
    print(f"file: {EXPECTED_USD}")
    for path in STEERING_JOINTS:
        print(
            f"{path}: position={INITIAL_POSITION_DEG:.1f} deg, "
            f"velocity={INITIAL_VELOCITY_DEG_S:.1f} deg/s"
        )
    print("Close and reopen the USD, then verify values before pressing Play.")


print("ACKERMANN Joint-State writer loaded. Timeline must be stopped.")
