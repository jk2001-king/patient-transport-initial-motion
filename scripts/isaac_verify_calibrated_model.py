"""Verify persistent calibration in the open workspace patient-transport USD."""

import math

import omni.timeline
import omni.usd
from pxr import UsdPhysics


EXPECTED_ROOT = "/home/jk/Research/patient_initial_ws/isaac_model/patient_transport.usd"
EXPECTED = {
    "physics_hz": 120.0,
    "lower_deg": -45.0,
    "upper_deg": 45.0,
    "kp": 10000.0,
    "kd": 1000.0,
    "friction": 0.0025,
}
JOINT_PATHS = (
    "/World/patient_transport/joints/front_steering_joint",
    "/World/patient_transport/joints/rear_steering_joint",
)
TIRE_MATERIAL = "/World/PatientTransportCalibrationMaterials/Tire"
GROUND_MATERIAL = "/World/PatientTransportCalibrationMaterials/Ground"
WHEEL_BODY_PATHS = (
    "/World/patient_transport/front_wheel_link_1",
    "/World/patient_transport/rear_wheel_link_1",
)
GROUND_COLLIDER = "/FlatGrid/GroundPlane/CollisionPlane"


def _value(prim, name):
    attribute = prim.GetAttribute(name)
    return attribute.Get() if attribute.IsValid() else None


def _close(actual, expected, tolerance=1e-6):
    return actual is not None and math.isclose(
        float(actual), float(expected), abs_tol=tolerance, rel_tol=0.0
    )


def verify_calibrated_model():
    timeline = omni.timeline.get_timeline_interface()
    if timeline.is_playing():
        raise RuntimeError("Press Stop before verifying the calibrated stage")
    stage = omni.usd.get_context().get_stage()
    errors = []
    root_path = stage.GetRootLayer().realPath
    if root_path != EXPECTED_ROOT:
        errors.append(f"root layer={root_path}, expected={EXPECTED_ROOT}")

    physics_scenes = [
        prim for prim in stage.Traverse() if prim.IsA(UsdPhysics.Scene)
    ]
    physics_hz = (
        _value(physics_scenes[0], "physxScene:timeStepsPerSecond")
        if physics_scenes
        else None
    )
    if not _close(physics_hz, EXPECTED["physics_hz"]):
        errors.append(f"physics_hz={physics_hz}")

    print("=== Persistent workspace steering calibration ===")
    print(f"root layer: {root_path}")
    print(f"physics rate: {physics_hz} Hz")
    for path in JOINT_PATHS:
        prim = stage.GetPrimAtPath(path)
        values = {
            "lower": _value(prim, "physics:lowerLimit") if prim.IsValid() else None,
            "upper": _value(prim, "physics:upperLimit") if prim.IsValid() else None,
            "type": _value(prim, "drive:angular:physics:type") if prim.IsValid() else None,
            "kp": _value(prim, "drive:angular:physics:stiffness") if prim.IsValid() else None,
            "kd": _value(prim, "drive:angular:physics:damping") if prim.IsValid() else None,
        }
        print(f"{path}: {values}")
        if not _close(values["lower"], EXPECTED["lower_deg"]):
            errors.append(f"{path} lower={values['lower']}")
        if not _close(values["upper"], EXPECTED["upper_deg"]):
            errors.append(f"{path} upper={values['upper']}")
        if str(values["type"]) != "acceleration":
            errors.append(f"{path} drive type={values['type']}")
        if not _close(values["kp"], EXPECTED["kp"]):
            errors.append(f"{path} kp={values['kp']}")
        if not _close(values["kd"], EXPECTED["kd"]):
            errors.append(f"{path} kd={values['kd']}")

    for path in (TIRE_MATERIAL, GROUND_MATERIAL):
        prim = stage.GetPrimAtPath(path)
        static = _value(prim, "physics:staticFriction") if prim.IsValid() else None
        dynamic = _value(prim, "physics:dynamicFriction") if prim.IsValid() else None
        print(f"{path}: static={static}, dynamic={dynamic}")
        if not _close(static, EXPECTED["friction"]):
            errors.append(f"{path} staticFriction={static}")
        if not _close(dynamic, EXPECTED["friction"]):
            errors.append(f"{path} dynamicFriction={dynamic}")

    expected_bindings = {
        **{path: TIRE_MATERIAL for path in WHEEL_BODY_PATHS},
        GROUND_COLLIDER: GROUND_MATERIAL,
    }
    for path, expected_target in expected_bindings.items():
        prim = stage.GetPrimAtPath(path)
        relationship = (
            prim.GetRelationship("material:binding:physics")
            if prim.IsValid()
            else None
        )
        targets = [str(target) for target in relationship.GetTargets()] if relationship else []
        print(f"{path} physics material: {targets}")
        if targets != [expected_target]:
            errors.append(f"{path} material targets={targets}")

    ground = stage.GetPrimAtPath(GROUND_COLLIDER)
    collision_enabled = _value(ground, "physics:collisionEnabled")
    print(f"ground collision enabled: {collision_enabled}")
    if collision_enabled is not True:
        errors.append(f"ground collisionEnabled={collision_enabled}")

    if errors:
        print("WORKSPACE MODEL VERIFICATION: FAILED")
        for error in errors:
            print(f"  - {error}")
        raise RuntimeError(f"Calibrated model has {len(errors)} verification error(s)")

    print("WORKSPACE MODEL VERIFICATION: PASSED")


print("Workspace-model verifier loaded. Timeline must be stopped.")
