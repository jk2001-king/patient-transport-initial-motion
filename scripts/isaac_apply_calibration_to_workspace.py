"""Persist validated calibration into the workspace USD currently open in Isaac."""

from pathlib import Path

import omni.timeline
import omni.usd
from pxr import PhysxSchema, UsdGeom, UsdPhysics, UsdShade


EXPECTED_USD = Path(
    "/home/jk/Research/patient_initial_ws/isaac_model/patient_transport.usd"
).resolve()
STEERING_JOINT_PATHS = (
    "/World/patient_transport/joints/front_steering_joint",
    "/World/patient_transport/joints/rear_steering_joint",
)
ACTIVE_WHEEL_BODY_PATHS = (
    "/World/patient_transport/front_wheel_link_1",
    "/World/patient_transport/rear_wheel_link_1",
)
ROBOT_ROOT_PATH = "/World/patient_transport"
GROUND_COLLIDER_PATH = "/FlatGrid/GroundPlane/CollisionPlane"
MATERIAL_SCOPE_PATH = "/World/PatientTransportCalibrationMaterials"

PHYSICS_HZ = 120
STEERING_KP = 10000.0
STEERING_KD = 1000.0
EQUIVALENT_FRICTION = 0.0025


def apply_calibration_to_workspace_usd():
    """Author calibration to the root layer and save the workspace USD in place."""
    timeline = omni.timeline.get_timeline_interface()
    if timeline.is_playing():
        raise RuntimeError("Press Stop before applying persistent calibration")

    stage = omni.usd.get_context().get_stage()
    root_layer = stage.GetRootLayer()
    actual_path = Path(root_layer.realPath).resolve() if root_layer.realPath else None
    if actual_path != EXPECTED_USD:
        raise RuntimeError(
            f"Wrong stage root: {actual_path}. Open {EXPECTED_USD} and try again."
        )

    previous_target = stage.GetEditTarget()
    stage.SetEditTarget(root_layer)
    try:
        for path in STEERING_JOINT_PATHS:
            prim = stage.GetPrimAtPath(path)
            if not prim.IsValid():
                raise RuntimeError(f"Missing steering joint: {path}")
            joint = UsdPhysics.RevoluteJoint(prim)
            joint.CreateLowerLimitAttr(-45.0).Set(-45.0)
            joint.CreateUpperLimitAttr(45.0).Set(45.0)
            drive = UsdPhysics.DriveAPI.Get(prim, "angular")
            if not drive:
                drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
            drive.CreateTypeAttr("acceleration").Set("acceleration")
            drive.CreateStiffnessAttr(STEERING_KP).Set(STEERING_KP)
            drive.CreateDampingAttr(STEERING_KD).Set(STEERING_KD)
            drive.CreateTargetPositionAttr(0.0).Set(0.0)
            drive.CreateTargetVelocityAttr(0.0).Set(0.0)

        physics_scene = next(
            (prim for prim in stage.Traverse() if prim.IsA(UsdPhysics.Scene)), None
        )
        if physics_scene is None:
            raise RuntimeError("No PhysicsScene found")
        PhysxSchema.PhysxSceneAPI.Apply(
            physics_scene
        ).CreateTimeStepsPerSecondAttr(PHYSICS_HZ).Set(PHYSICS_HZ)

        ground = stage.GetPrimAtPath(GROUND_COLLIDER_PATH)
        if not ground.IsValid():
            raise RuntimeError(f"Missing ground collider: {GROUND_COLLIDER_PATH}")
        UsdPhysics.CollisionAPI.Apply(ground).CreateCollisionEnabledAttr(True).Set(True)

        rigid_body_count = 0
        for prim in stage.Traverse():
            path = str(prim.GetPath())
            if not (
                path == ROBOT_ROOT_PATH or path.startswith(ROBOT_ROOT_PATH + "/")
            ):
                continue
            if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                continue
            PhysxSchema.PhysxRigidBodyAPI.Apply(
                prim
            ).CreateDisableGravityAttr(False).Set(False)
            rigid_body_count += 1
        if rigid_body_count == 0:
            raise RuntimeError(f"No rigid bodies found below {ROBOT_ROOT_PATH}")

        UsdGeom.Scope.Define(stage, MATERIAL_SCOPE_PATH)
        tire = UsdShade.Material.Define(stage, f"{MATERIAL_SCOPE_PATH}/Tire")
        ground_material = UsdShade.Material.Define(
            stage, f"{MATERIAL_SCOPE_PATH}/Ground"
        )
        for material in (tire, ground_material):
            api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
            api.CreateStaticFrictionAttr(EQUIVALENT_FRICTION).Set(
                EQUIVALENT_FRICTION
            )
            api.CreateDynamicFrictionAttr(EQUIVALENT_FRICTION).Set(
                EQUIVALENT_FRICTION
            )
            api.CreateRestitutionAttr(0.0).Set(0.0)

        for path in ACTIVE_WHEEL_BODY_PATHS:
            body = stage.GetPrimAtPath(path)
            if not body.IsValid():
                raise RuntimeError(f"Missing active wheel body: {path}")
            body.CreateRelationship("material:binding:physics", False).SetTargets(
                [tire.GetPath()]
            )
        ground.CreateRelationship("material:binding:physics", False).SetTargets(
            [ground_material.GetPath()]
        )
    finally:
        stage.SetEditTarget(previous_target)

    layer_data = dict(root_layer.customLayerData)
    layer_data["patientTransportSteeringCalibration"] = {
        "equivalentFriction": EQUIVALENT_FRICTION,
        "physicsHz": PHYSICS_HZ,
        "steeringKd": STEERING_KD,
        "steeringKp": STEERING_KP,
        "steeringLimitDeg": 45.0,
        "source": "real ground steering data; Isaac mu=0.0025 runs 01-03",
    }
    root_layer.customLayerData = layer_data
    root_layer.Save()

    print("Workspace USD calibration saved in place.")
    print(f"file: {EXPECTED_USD}")
    print(
        f"physics={PHYSICS_HZ} Hz, steering=+/-45 deg, "
        f"kp={STEERING_KP}, kd={STEERING_KD}, mu_eq={EQUIVALENT_FRICTION}"
    )
    print("Reopen this USD without saving the session layer, then run verification.")


print("Workspace calibration writer loaded. Timeline must be stopped.")
