"""Interactive steering calibration helpers for Isaac Sim Script Editor.

Press Play, execute this file once, and then call these helpers separately:

    set_physical_steering_deg(front_deg=5.0)
    read_steering()
    set_physical_steering_deg(front_deg=0.0)

The CAD-exported raw ACKERMANN zero is +90 deg for both steering joints.
Raw targets therefore use 45..135 deg, while physical targets use -45..+45 deg.
"""

import asyncio
import time

import numpy as np
import omni.kit.app
import omni.physx
import omni.timeline
import omni.usd

from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction


ARTICULATION_ROOT = "/World/patient_transport/base_link"
STEERING_JOINTS = {
    "front": "front_steering_joint",
    "rear": "rear_steering_joint",
}
RAW_ACKERMANN_ZERO_DEG = {"front": 90.0, "rear": 90.0}
# Provisional signs.  Change an entry to -1 only if the visual +/-5 deg sign
# check shows that increasing raw angle is negative physical steering.
RAW_TO_PHYSICAL_SIGN = {"front": 1.0, "rear": 1.0}
PHYSICAL_TARGET_LIMIT_DEG = 45.0
RAW_TARGET_LOWER_DEG = 45.0
RAW_TARGET_UPPER_DEG = 135.0
FRONT_WHEEL_BODY_PATH = "/World/patient_transport/front_wheel_link_1"
GROUND_COLLIDER_PATH = "/FlatGrid/GroundPlane/CollisionPlane"
DIAGNOSTIC_MATERIAL_SCOPE = "/World/SteeringContactDiagnostics"


robot = SingleArticulation(ARTICULATION_ROOT)
robot.initialize()
name_to_index = {name: index for index, name in enumerate(robot.dof_names)}
timeline = omni.timeline.get_timeline_interface()

# Timeline time can continue even when PhysX has stopped after an error.  Count
# actual physics-step callbacks separately so the two cases can be distinguished.
physics_step_count = 0
physics_elapsed_s = 0.0


def _on_physics_step(step_dt):
    global physics_step_count, physics_elapsed_s
    physics_step_count += 1
    physics_elapsed_s += float(step_dt)


physics_step_subscription = (
    omni.physx.get_physx_interface().subscribe_physics_step_events(_on_physics_step)
)

_initial_kps, _initial_kds = robot.get_articulation_controller().get_gains()
_initial_kps = np.asarray(_initial_kps, dtype=np.float32).copy()
_initial_kds = np.asarray(_initial_kds, dtype=np.float32).copy()


def _require_running_physics():
    if not timeline.is_playing():
        raise RuntimeError(
            "Physics Timeline is not playing. Press Play, reload this script, "
            "and then send the command again."
        )


def _raw_to_physical_deg(location, raw_deg):
    return RAW_TO_PHYSICAL_SIGN[location] * (
        float(raw_deg) - RAW_ACKERMANN_ZERO_DEG[location]
    )


def _physical_to_raw_deg(location, physical_deg):
    return RAW_ACKERMANN_ZERO_DEG[location] + (
        float(physical_deg) / RAW_TO_PHYSICAL_SIGN[location]
    )


def read_steering():
    """Print current runtime steering positions in radians and degrees."""
    positions = robot.get_joint_positions()
    velocities = robot.get_joint_velocities()
    print("\n=== Current raw steering state ===")
    for location, joint_name in STEERING_JOINTS.items():
        joint_index = name_to_index[joint_name]
        position_rad = float(positions[joint_index])
        velocity_rad_s = float(velocities[joint_index])
        raw_deg = float(np.rad2deg(position_rad))
        physical_deg = _raw_to_physical_deg(location, raw_deg)
        print(
            f"{location}: index={joint_index}, "
            f"position={position_rad:.6f} rad "
            f"(raw={raw_deg:.3f} deg, physical={physical_deg:.3f} deg), "
            f"velocity={velocity_rad_s:.6f} rad/s"
        )


def diagnose_steering():
    """Print actual state, retained controller target, gains, and measured effort."""
    positions = robot.get_joint_positions()
    velocities = robot.get_joint_velocities()
    applied_action = robot.get_applied_action()
    gains = robot.get_articulation_controller().get_gains()
    measured_efforts = robot.get_measured_joint_efforts()

    target_positions = applied_action.joint_positions
    stiffnesses, dampings = gains

    print("\n=== Steering drive diagnostic ===")
    print("timeline playing:", timeline.is_playing())
    for location, joint_name in STEERING_JOINTS.items():
        joint_index = name_to_index[joint_name]
        target_rad = None if target_positions is None else float(target_positions[joint_index])
        target_text = "None" if target_rad is None else (
            f"{target_rad:.6f} rad "
            f"(raw={np.rad2deg(target_rad):.3f} deg, "
            f"physical={_raw_to_physical_deg(location, np.rad2deg(target_rad)):.3f} deg)"
        )
        actual_raw_deg = float(np.rad2deg(float(positions[joint_index])))
        print(
            f"{location}: actual={float(positions[joint_index]):.6f} rad "
            f"(raw={actual_raw_deg:.3f} deg, "
            f"physical={_raw_to_physical_deg(location, actual_raw_deg):.3f} deg), "
            f"velocity={float(velocities[joint_index]):.6f} rad/s, "
            f"target={target_text}, "
            f"kp={float(stiffnesses[joint_index]):.6f}, "
            f"kd={float(dampings[joint_index]):.6f}, "
            f"measured_effort={float(measured_efforts[joint_index]):.6f}"
        )


def set_runtime_steering_gains(kp=100.0, kd=20.0):
    """Temporarily set front/rear steering gains without saving them to USD."""
    controller = robot.get_articulation_controller()
    stiffnesses, dampings = controller.get_gains()
    stiffnesses = np.asarray(stiffnesses, dtype=np.float32).copy()
    dampings = np.asarray(dampings, dtype=np.float32).copy()

    steering_indices = [
        name_to_index[STEERING_JOINTS["front"]],
        name_to_index[STEERING_JOINTS["rear"]],
    ]
    stiffnesses[steering_indices] = float(kp)
    dampings[steering_indices] = float(kd)
    controller.set_gains(stiffnesses, dampings, save_to_usd=False)
    print(
        f"Temporary runtime steering gains applied: kp={float(kp):.3f}, "
        f"kd={float(kd):.3f} (USD not modified)"
    )


def restore_loaded_steering_gains():
    """Restore the gains captured when this helper file was loaded."""
    robot.get_articulation_controller().set_gains(
        _initial_kps.copy(), _initial_kds.copy(), save_to_usd=False
    )
    front_index = name_to_index[STEERING_JOINTS["front"]]
    print(
        "Loaded steering gains restored: "
        f"kp={float(_initial_kps[front_index]):.6f}, "
        f"kd={float(_initial_kds[front_index]):.6f} (USD not modified)"
    )


def inspect_steering_usd(location="front"):
    """Print the authored joint, connected bodies, and nearby constraints."""
    if location not in STEERING_JOINTS:
        raise ValueError(f"location must be one of {tuple(STEERING_JOINTS)}")

    stage = omni.usd.get_context().get_stage()
    joint_name = STEERING_JOINTS[location]
    matches = [prim for prim in stage.Traverse() if prim.GetName() == joint_name]

    print(f"\n=== USD steering inspection: {location} ===")
    if not matches:
        print(f"No prim named {joint_name!r} was found")
        return

    for joint_prim in matches:
        print(f"joint path: {joint_prim.GetPath()}")
        print(f"joint type: {joint_prim.GetTypeName()}")
        connected_body_paths = set()

        for relationship in joint_prim.GetRelationships():
            targets = list(relationship.GetTargets())
            if targets:
                print(f"relationship {relationship.GetName()}: {targets}")
                if relationship.GetName() in ("physics:body0", "physics:body1"):
                    connected_body_paths.update(targets)

        interesting_tokens = (
            "axis",
            "limit",
            "enabled",
            "exclude",
            "break",
            "localpos",
            "localrot",
            "drive:angular",
        )
        for attribute in joint_prim.GetAttributes():
            attribute_name = attribute.GetName()
            if any(token in attribute_name.lower() for token in interesting_tokens):
                print(f"attribute {attribute_name}: {attribute.Get()}")

        print("joints sharing either connected body:")
        shared_joint_count = 0
        for candidate in stage.Traverse():
            if candidate == joint_prim or "Joint" not in candidate.GetTypeName():
                continue
            candidate_targets = set()
            for relationship_name in ("physics:body0", "physics:body1"):
                candidate_targets.update(
                    candidate.GetRelationship(relationship_name).GetTargets()
                )
            shared = connected_body_paths.intersection(candidate_targets)
            if shared:
                shared_joint_count += 1
                print(
                    f"  {candidate.GetPath()} ({candidate.GetTypeName()}), "
                    f"shared={sorted(str(path) for path in shared)}"
                )
        if shared_joint_count == 0:
            print("  none")

        print("collision prims below connected bodies:")
        collision_count = 0
        for body_path in sorted(connected_body_paths, key=str):
            body_prim = stage.GetPrimAtPath(body_path)
            if not body_prim.IsValid():
                print(f"  invalid body target: {body_path}")
                continue
            for descendant in Usd.PrimRange(body_prim):
                if descendant.HasAPI(UsdPhysics.CollisionAPI):
                    collision_count += 1
                    collision_enabled = descendant.GetAttribute(
                        "physics:collisionEnabled"
                    ).Get()
                    print(
                        f"  {descendant.GetPath()} "
                        f"({descendant.GetTypeName()}), enabled={collision_enabled}"
                    )
        if collision_count == 0:
            print("  none")


def inspect_front_wheel_contact_usd():
    """Print front-wheel and ground collision/material properties."""
    stage = omni.usd.get_context().get_stage()
    print("\n=== Front wheel contact inspection ===")

    wheel_joint_matches = [
        prim for prim in stage.Traverse() if prim.GetName() == "front_wheel_joint"
    ]
    wheel_body_paths = set()
    if not wheel_joint_matches:
        print("No prim named 'front_wheel_joint' was found")
    for joint_prim in wheel_joint_matches:
        print(f"wheel joint: {joint_prim.GetPath()} ({joint_prim.GetTypeName()})")
        for relationship_name in ("physics:body0", "physics:body1"):
            targets = joint_prim.GetRelationship(relationship_name).GetTargets()
            print(f"  {relationship_name}: {list(targets)}")
            wheel_body_paths.update(targets)
        for attribute_name in (
            "physics:axis",
            "physics:lowerLimit",
            "physics:upperLimit",
            "drive:angular:physics:type",
            "drive:angular:physics:maxForce",
            "drive:angular:physics:damping",
        ):
            attribute = joint_prim.GetAttribute(attribute_name)
            if attribute.IsValid():
                print(f"  {attribute_name}: {attribute.Get()}")

    for body_path in sorted(wheel_body_paths, key=str):
        body_prim = stage.GetPrimAtPath(body_path)
        if body_prim.IsValid():
            prototype = body_prim.GetPrototype()
            prototype_path = prototype.GetPath() if prototype.IsValid() else None
            print(
                f"body: {body_path}, type={body_prim.GetTypeName()}, "
                f"instance={body_prim.IsInstance()}, "
                f"instanceable={body_prim.IsInstanceable()}, "
                f"prototype={prototype_path}"
            )

    collision_prims = []
    traversal = Usd.PrimRange.Stage(
        stage, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)
    )
    for prim in traversal:
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        path_lower = str(prim.GetPath()).lower()
        below_wheel_body = any(
            prim.GetPath().HasPrefix(body_path) for body_path in wheel_body_paths
        )
        if below_wheel_body or "ground" in path_lower or "flatgrid" in path_lower:
            collision_prims.append(prim)

    print(f"matching collision prims: {len(collision_prims)}")
    if not collision_prims:
        print("  none")
        return

    interesting_tokens = (
        "collisionenabled",
        "approximation",
        "contactoffset",
        "restoffset",
        "material:binding",
        "scale",
        "radius",
        "height",
        "axis",
    )
    for prim in collision_prims:
        print(
            f"collision: {prim.GetPath()} ({prim.GetTypeName()}), "
            f"instance_proxy={prim.IsInstanceProxy()}"
        )
        for attribute in prim.GetAttributes():
            attribute_name = attribute.GetName()
            if any(token in attribute_name.lower() for token in interesting_tokens):
                print(f"  attribute {attribute_name}: {attribute.Get()}")

        if prim.GetTypeName() == "Cylinder":
            cylinder_axis = prim.GetAttribute("axis").Get()
            local_axis_by_token = {
                "X": Gf.Vec3d(1.0, 0.0, 0.0),
                "Y": Gf.Vec3d(0.0, 1.0, 0.0),
                "Z": Gf.Vec3d(0.0, 0.0, 1.0),
            }
            local_axis = local_axis_by_token.get(str(cylinder_axis))
            if local_axis is not None:
                local_to_world = UsdGeom.Xformable(
                    prim
                ).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                world_axis = local_to_world.TransformDir(local_axis)
                if world_axis.GetLength() > 0.0:
                    world_axis.Normalize()
                print(
                    "  computed cylinder symmetry axis in world: "
                    f"({world_axis[0]:.6f}, {world_axis[1]:.6f}, "
                    f"{world_axis[2]:.6f})"
                )

                print("  authored Xform ops from collider up to wheel body:")
                ancestor = prim
                while ancestor.IsValid():
                    xformable = UsdGeom.Xformable(ancestor)
                    if xformable:
                        ordered_ops = xformable.GetOrderedXformOps()
                        for xform_op in ordered_ops:
                            print(
                                f"    {ancestor.GetPath()} / "
                                f"{xform_op.GetOpName()}: {xform_op.Get()}"
                            )
                    if ancestor.GetPath() in wheel_body_paths:
                        break
                    ancestor = ancestor.GetParent()
        for relationship in prim.GetRelationships():
            relationship_name = relationship.GetName()
            if "material:binding" in relationship_name.lower():
                print(
                    f"  relationship {relationship_name}: "
                    f"{list(relationship.GetTargets())}"
                )

        material_targets = []
        ancestor = prim
        while ancestor.IsValid() and not ancestor.IsPseudoRoot():
            binding = ancestor.GetRelationship("material:binding:physics")
            targets = list(binding.GetTargets()) if binding.IsValid() else []
            if targets:
                print(
                    f"  inherited physics material at {ancestor.GetPath()}: "
                    f"{targets}"
                )
                material_targets.extend(targets)
                break
            ancestor = ancestor.GetParent()

        if not material_targets:
            print("  effective physics material: no authored binding found")
        for material_path in material_targets:
            material_prim = stage.GetPrimAtPath(material_path)
            print(f"  physics material prim: {material_path}")
            for attribute_name in (
                "physics:staticFriction",
                "physics:dynamicFriction",
                "physics:restitution",
                "physxMaterial:frictionCombineMode",
                "physxMaterial:restitutionCombineMode",
            ):
                attribute = material_prim.GetAttribute(attribute_name)
                if attribute.IsValid():
                    print(f"    {attribute_name}: {attribute.Get()}")

    print("authored physics materials on stage:")
    material_count = 0
    for prim in stage.Traverse():
        if not prim.HasAPI(UsdPhysics.MaterialAPI):
            continue
        material_count += 1
        print(f"  {prim.GetPath()}")
        for attribute_name in (
            "physics:staticFriction",
            "physics:dynamicFriction",
            "physics:restitution",
        ):
            attribute = prim.GetAttribute(attribute_name)
            print(f"    {attribute_name}: {attribute.Get()}")
    if material_count == 0:
        print("  none")


def apply_session_contact_materials(
    tire_static=0.0,
    tire_dynamic=0.0,
    ground_static=0.0,
    ground_dynamic=0.0,
):
    """Bind temporary front-tire/ground materials in the USD session layer."""
    stage = omni.usd.get_context().get_stage()
    wheel_body = stage.GetPrimAtPath(FRONT_WHEEL_BODY_PATH)
    ground_collider = stage.GetPrimAtPath(GROUND_COLLIDER_PATH)
    if not wheel_body.IsValid():
        raise RuntimeError(f"Missing wheel body: {FRONT_WHEEL_BODY_PATH}")
    if not ground_collider.IsValid():
        raise RuntimeError(f"Missing ground collider: {GROUND_COLLIDER_PATH}")

    values = {
        "tire_static": float(tire_static),
        "tire_dynamic": float(tire_dynamic),
        "ground_static": float(ground_static),
        "ground_dynamic": float(ground_dynamic),
    }
    if any(value < 0.0 for value in values.values()):
        raise ValueError(f"Friction values must be nonnegative: {values}")

    previous_edit_target = stage.GetEditTarget()
    stage.SetEditTarget(stage.GetSessionLayer())
    try:
        UsdGeom.Scope.Define(stage, DIAGNOSTIC_MATERIAL_SCOPE)
        tire_material = UsdShade.Material.Define(
            stage, f"{DIAGNOSTIC_MATERIAL_SCOPE}/FrontTireMaterial"
        )
        ground_material = UsdShade.Material.Define(
            stage, f"{DIAGNOSTIC_MATERIAL_SCOPE}/GroundMaterial"
        )

        tire_physics = UsdPhysics.MaterialAPI.Apply(tire_material.GetPrim())
        tire_physics.CreateStaticFrictionAttr(values["tire_static"])
        tire_physics.CreateDynamicFrictionAttr(values["tire_dynamic"])
        tire_physics.CreateRestitutionAttr(0.0)

        ground_physics = UsdPhysics.MaterialAPI.Apply(ground_material.GetPrim())
        ground_physics.CreateStaticFrictionAttr(values["ground_static"])
        ground_physics.CreateDynamicFrictionAttr(values["ground_dynamic"])
        ground_physics.CreateRestitutionAttr(0.0)

        wheel_body.CreateRelationship("material:binding:physics", False).SetTargets(
            [tire_material.GetPath()]
        )
        ground_collider.CreateRelationship(
            "material:binding:physics", False
        ).SetTargets([ground_material.GetPath()])
    finally:
        stage.SetEditTarget(previous_edit_target)

    print("Temporary session-layer contact materials applied:", values)
    print("They are not saved to the USD file.")
    print("Press Stop, press Play, reload this script, and rerun the ground test.")


def set_raw_steering_deg(*, front_deg=None, rear_deg=None):
    """Command selected raw steering targets without touching other DOFs."""
    _require_running_physics()
    requested = {"front": front_deg, "rear": rear_deg}
    joint_indices = []
    target_positions_rad = []

    for location, target_deg in requested.items():
        if target_deg is None:
            continue
        target_deg = float(target_deg)
        if not RAW_TARGET_LOWER_DEG <= target_deg <= RAW_TARGET_UPPER_DEG:
            raise ValueError(
                f"{location} raw target {target_deg} deg is outside "
                f"[{RAW_TARGET_LOWER_DEG}, {RAW_TARGET_UPPER_DEG}] deg "
                f"(physical limit +/-{PHYSICAL_TARGET_LIMIT_DEG} deg about raw 90 deg)"
            )
        joint_indices.append(name_to_index[STEERING_JOINTS[location]])
        target_positions_rad.append(np.deg2rad(target_deg))

    if not joint_indices:
        raise ValueError("Specify front_deg, rear_deg, or both")

    robot.apply_action(
        ArticulationAction(
            joint_positions=np.asarray(target_positions_rad, dtype=np.float32),
            joint_indices=np.asarray(joint_indices, dtype=np.int32),
        )
    )
    print(
        "Applied raw steering target(s):",
        {key: value for key, value in requested.items() if value is not None},
    )
    print("Wait about one second, then run: read_steering()")


def set_physical_steering_deg(*, front_deg=None, rear_deg=None):
    """Command physical ACKERMANN angles in the range -45..+45 deg."""
    requested = {"front": front_deg, "rear": rear_deg}
    raw_targets = {}
    for location, target_deg in requested.items():
        if target_deg is None:
            continue
        target_deg = float(target_deg)
        if abs(target_deg) > PHYSICAL_TARGET_LIMIT_DEG:
            raise ValueError(
                f"{location} physical target {target_deg} deg exceeds "
                f"+/-{PHYSICAL_TARGET_LIMIT_DEG} deg"
            )
        raw_targets[location] = _physical_to_raw_deg(location, target_deg)

    if not raw_targets:
        raise ValueError("Specify front_deg, rear_deg, or both")
    set_raw_steering_deg(
        front_deg=raw_targets.get("front"),
        rear_deg=raw_targets.get("rear"),
    )
    print(
        "Applied physical steering target(s):",
        {key: value for key, value in requested.items() if value is not None},
    )


async def _run_front_step_test(
    target_deg=5.0,
    zero_settle_s=2.0,
    duration_s=5.0,
    sample_period_s=0.25,
    lift_m=0.0,
):
    """Run and print a non-blocking physical front-steering step response."""
    global last_front_step_samples

    app = omni.kit.app.get_app()
    timeline = omni.timeline.get_timeline_interface()
    front_index = name_to_index[STEERING_JOINTS["front"]]
    last_front_step_samples = []

    print("\n=== Front steering step test: zero settling ===")
    set_physical_steering_deg(front_deg=0.0)
    zero_start = time.perf_counter()
    while time.perf_counter() - zero_start < float(zero_settle_s):
        await app.next_update_async()

    if float(lift_m) > 0.0:
        root_position, root_orientation = robot.get_world_pose()
        lifted_position = np.asarray(root_position, dtype=np.float32).copy()
        lifted_position[2] += float(lift_m)
        robot.set_world_pose(position=lifted_position, orientation=root_orientation)
        robot.set_linear_velocity(np.zeros(3, dtype=np.float32))
        robot.set_angular_velocity(np.zeros(3, dtype=np.float32))
        await app.next_update_async()
        print(
            f"Temporarily lifted articulation by {float(lift_m):.3f} m "
            "for a contact-free test. Press Stop after this test."
        )

    print(f"\n=== Front steering step test: 0 -> {float(target_deg):.3f} deg ===")
    print(
        "wall_t_s, timeline_t_s, physics_t_s, physics_steps, playing, "
        "actual_deg, velocity_deg_s, target_deg, error_deg, measured_effort"
    )
    set_physical_steering_deg(front_deg=float(target_deg))

    start = time.perf_counter()
    sim_start = float(timeline.get_current_time())
    physics_step_start = physics_step_count
    physics_time_start = physics_elapsed_s
    next_sample_time = 0.0
    while True:
        await app.next_update_async()
        elapsed = time.perf_counter() - start
        if elapsed + 1e-9 < next_sample_time:
            continue

        positions = robot.get_joint_positions()
        velocities = robot.get_joint_velocities()
        efforts = robot.get_measured_joint_efforts()
        applied_action = robot.get_applied_action()

        actual_raw_deg = float(np.rad2deg(positions[front_index]))
        actual_deg = _raw_to_physical_deg("front", actual_raw_deg)
        velocity_deg_s = float(np.rad2deg(velocities[front_index]))
        retained_target_raw_deg = float(
            np.rad2deg(applied_action.joint_positions[front_index])
        )
        retained_target_deg = _raw_to_physical_deg(
            "front", retained_target_raw_deg
        )
        error_deg = retained_target_deg - actual_deg
        measured_effort = float(efforts[front_index])
        sim_elapsed = float(timeline.get_current_time()) - sim_start
        physics_steps = physics_step_count - physics_step_start
        physics_elapsed = physics_elapsed_s - physics_time_start
        is_playing = bool(timeline.is_playing())
        sample = (
            elapsed,
            sim_elapsed,
            physics_elapsed,
            physics_steps,
            is_playing,
            actual_deg,
            velocity_deg_s,
            retained_target_deg,
            error_deg,
            measured_effort,
        )
        last_front_step_samples.append(sample)
        print(
            f"{elapsed:.3f}, {sim_elapsed:.3f}, {physics_elapsed:.3f}, "
            f"{physics_steps}, {is_playing}, "
            f"{actual_deg:.3f}, {velocity_deg_s:.3f}, "
            f"{retained_target_deg:.3f}, {error_deg:.3f}, {measured_effort:.6f}"
        )

        if elapsed >= float(duration_s):
            break
        next_sample_time += float(sample_period_s)

    actual_values = [sample[5] for sample in last_front_step_samples]
    final_sample = last_front_step_samples[-1]
    print("\n=== Front steering step summary ===")
    print(f"samples: {len(last_front_step_samples)}")
    print(f"actual range [deg]: {min(actual_values):.3f} .. {max(actual_values):.3f}")
    print(f"simulation time advanced [s]: {final_sample[1]:.3f}")
    print(f"physics time advanced [s]: {final_sample[2]:.3f}")
    print(f"physics steps observed: {final_sample[3]}")
    print(f"Timeline playing at final sample: {final_sample[4]}")
    print(f"final actual [deg]: {final_sample[5]:.3f}")
    print(f"final velocity [deg/s]: {final_sample[6]:.3f}")
    print(f"final error [deg]: {final_sample[8]:.3f}")
    if float(lift_m) > 0.0:
        print("Air test complete: press Stop now to restore the model pose.")
    print("Copy the CSV rows and summary for diagnosis.")


def start_front_step_test(
    target_deg=5.0,
    zero_settle_s=2.0,
    duration_s=5.0,
    sample_period_s=0.25,
    lift_m=0.0,
):
    """Schedule a steering step test without blocking Isaac Sim physics."""
    global steering_step_task
    if "steering_step_task" in globals() and not steering_step_task.done():
        raise RuntimeError("A steering step test is already running")
    steering_step_task = asyncio.ensure_future(
        _run_front_step_test(
            target_deg=target_deg,
            zero_settle_s=zero_settle_s,
            duration_s=duration_s,
            sample_period_s=sample_period_s,
            lift_m=lift_m,
        )
    )
    expected_wall_s = float(zero_settle_s) + float(duration_s)
    print(
        "Front steering step test scheduled; keep Timeline playing for about "
        f"{expected_wall_s:.1f} seconds."
    )


print("Steering calibration helpers loaded.")
print("ACKERMANN raw zero: front=90 deg, rear=90 deg.")
print("Use set_physical_steering_deg(...) for physical -45..+45 deg commands.")
print("Timeline playing:", timeline.is_playing())
read_steering()
