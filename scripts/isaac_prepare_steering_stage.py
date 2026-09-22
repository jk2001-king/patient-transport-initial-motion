"""Prepare temporary Isaac steering-test settings while Timeline is stopped.

This script authors only to the USD session layer. Do not save the USD after
running it.
"""

from __future__ import annotations

import omni.timeline
import omni.usd
from pxr import PhysxSchema, UsdPhysics, UsdGeom, UsdShade


PREP_STEERING_JOINT_PATHS = (
    "/World/patient_transport/joints/front_steering_joint",
    "/World/patient_transport/joints/rear_steering_joint",
)
PREP_ACTIVE_WHEEL_BODY_PATHS = (
    "/World/patient_transport/front_wheel_link_1",
    "/World/patient_transport/rear_wheel_link_1",
)
PREP_GROUND_COLLIDER_PATH = "/FlatGrid/GroundPlane/CollisionPlane"
PREP_MATERIAL_SCOPE_PATH = "/World/SteeringCalibrationMaterials"
PREP_ROBOT_ROOT_PATH = "/World/patient_transport"


def prepare_steering_stage(
    condition: str = "air",
    friction: float | None = None,
    physics_hz: int = 120,
):
    """Set physics rate, +/-45 deg limits, and optional ground materials."""
    timeline = omni.timeline.get_timeline_interface()
    if timeline.is_playing():
        raise RuntimeError("Press Stop before preparing the steering-test stage")
    condition = condition.lower()
    if condition not in ("air", "ground"):
        raise ValueError("condition must be 'air' or 'ground'")
    if condition == "ground" and friction is None:
        raise ValueError("ground preparation requires friction")
    if friction is not None and float(friction) < 0.0:
        raise ValueError("friction must be nonnegative")
    if int(physics_hz) <= 0:
        raise ValueError("physics_hz must be positive")

    stage = omni.usd.get_context().get_stage()
    previous_target = stage.GetEditTarget()
    stage.SetEditTarget(stage.GetSessionLayer())
    try:
        physics_scene_prim = next(
            (prim for prim in stage.Traverse() if prim.IsA(UsdPhysics.Scene)), None
        )
        if physics_scene_prim is None:
            raise RuntimeError("No UsdPhysics.Scene prim was found")
        physx_scene = PhysxSchema.PhysxSceneAPI.Apply(physics_scene_prim)
        physx_scene.GetTimeStepsPerSecondAttr().Set(int(physics_hz))

        # Prefix these globals because Script Editor exec() calls share one
        # namespace.  The runtime runner has a different STEERING_JOINTS map.
        for path in PREP_STEERING_JOINT_PATHS:
            prim = stage.GetPrimAtPath(path)
            if not prim.IsValid():
                raise RuntimeError(f"Missing steering joint: {path}")
            joint = UsdPhysics.RevoluteJoint(prim)
            joint.CreateLowerLimitAttr(-45.0)
            joint.CreateUpperLimitAttr(45.0)

        ground = stage.GetPrimAtPath(PREP_GROUND_COLLIDER_PATH)
        if not ground.IsValid():
            raise RuntimeError(
                f"Missing ground collider: {PREP_GROUND_COLLIDER_PATH}"
            )

        # A 5 m lift alone is not contact-free: under gravity the robot falls
        # back to the floor in about one second.  Air runs therefore disable
        # both robot gravity and the floor collider in the temporary session
        # layer. Ground preparation explicitly restores both settings.
        air_condition = condition == "air"
        ground_collision = UsdPhysics.CollisionAPI.Apply(ground)
        ground_collision.CreateCollisionEnabledAttr(not air_condition)
        rigid_body_count = 0
        for prim in stage.Traverse():
            prim_path = str(prim.GetPath())
            if not (
                prim_path == PREP_ROBOT_ROOT_PATH
                or prim_path.startswith(PREP_ROBOT_ROOT_PATH + "/")
            ):
                continue
            if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                continue
            PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateDisableGravityAttr(
                air_condition
            )
            rigid_body_count += 1
        if rigid_body_count == 0:
            raise RuntimeError(
                f"No rigid bodies found below {PREP_ROBOT_ROOT_PATH}"
            )

        if condition == "ground":
            wheel_bodies = [
                stage.GetPrimAtPath(path) for path in PREP_ACTIVE_WHEEL_BODY_PATHS
            ]
            missing = [
                path
                for path, prim in zip(PREP_ACTIVE_WHEEL_BODY_PATHS, wheel_bodies)
                if not prim.IsValid()
            ]
            if missing:
                raise RuntimeError(f"Missing active wheel body prim(s): {missing}")
            UsdGeom.Scope.Define(stage, PREP_MATERIAL_SCOPE_PATH)
            tire_material = UsdShade.Material.Define(
                stage, f"{PREP_MATERIAL_SCOPE_PATH}/Tire"
            )
            ground_material = UsdShade.Material.Define(
                stage, f"{PREP_MATERIAL_SCOPE_PATH}/Ground"
            )
            for material in (tire_material, ground_material):
                material_api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
                material_api.CreateStaticFrictionAttr(float(friction))
                material_api.CreateDynamicFrictionAttr(float(friction))
                material_api.CreateRestitutionAttr(0.0)
            for wheel_body in wheel_bodies:
                wheel_body.CreateRelationship("material:binding:physics", False).SetTargets(
                    [tire_material.GetPath()]
                )
            ground.CreateRelationship("material:binding:physics", False).SetTargets(
                [ground_material.GetPath()]
            )
    finally:
        stage.SetEditTarget(previous_target)

    print("Temporary steering-test stage prepared (session layer only).")
    print(f"condition={condition}, friction={friction}, physics_hz={int(physics_hz)}")
    print(
        f"air isolation: robot_gravity_disabled={condition == 'air'}, "
        f"ground_collision_enabled={condition != 'air'}"
    )
    print("steering limits: -45 .. +45 deg")
    print("Press Play, load isaac_dual_steering_test.py, and start one run.")
    print("Do not save the USD file.")


print("Steering stage-preparation helper loaded. Timeline must be stopped.")
