"""Dual-steering calibration runner for the Isaac Sim 5.0 Script Editor.

Usage (Timeline playing):

    exec(open("/home/jk/Research/patient_initial_ws/scripts/isaac_dual_steering_test.py").read())
    start_dual_steering_test(condition="air", run_id="run01")

Before Play, use isaac_prepare_steering_stage.py to set the requested physics
rate, +/-45 degree limits, and the optional ground contact material in the
session layer.

The commanded position follows the real 2ELD2 Profile Position limits rather
than jumping directly from zero to +/-45 degrees.
"""

from __future__ import annotations

import asyncio
import csv
import json
import math
from pathlib import Path
import time

import numpy as np
import omni.kit.app
import omni.physx
import omni.timeline
import omni.usd
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction
from pxr import UsdPhysics


# Script Editor reloads share a Python namespace. Cancel a still-pending test
# from the previous load so it cannot wake up during the next run.
_previous_dual_steering_task = globals().get("dual_steering_task")
if (
    _previous_dual_steering_task is not None
    and not _previous_dual_steering_task.done()
):
    _previous_dual_steering_task.cancel()


WORKSPACE = Path("/home/jk/Research/patient_initial_ws")
OUTPUT_ROOT = WORKSPACE / "results" / "isaac_steering"
ARTICULATION_ROOT = "/World/patient_transport/base_link"
STEERING_JOINTS = {
    "front": "front_steering_joint",
    "rear": "rear_steering_joint",
}
STEERING_JOINT_PATHS = {
    "front": "/World/patient_transport/joints/front_steering_joint",
    "rear": "/World/patient_transport/joints/rear_steering_joint",
}
CALIBRATION_TIRE_MATERIAL = "/World/SteeringCalibrationMaterials/Tire"
CALIBRATION_GROUND_MATERIAL = "/World/SteeringCalibrationMaterials/Ground"
GROUND_COLLIDER_PATH = "/FlatGrid/GroundPlane/CollisionPlane"
ROBOT_ROOT_PATH = "/World/patient_transport"
# Values used by the real motor_node/2ELD2 Profile Position configuration.
ENCODER_COUNTS_PER_REV = 131072.0
STEERING_REDUCTION = 10.0
PROFILE_VELOCITY_COUNTS_S = 50000.0
PROFILE_ACCELERATION_COUNTS_S2 = 50000.0
PROFILE_DECELERATION_COUNTS_S2 = 80000.0
MAX_VELOCITY_DEG_S = (
    PROFILE_VELOCITY_COUNTS_S * 360.0 / (ENCODER_COUNTS_PER_REV * STEERING_REDUCTION)
)
MAX_ACCELERATION_DEG_S2 = (
    PROFILE_ACCELERATION_COUNTS_S2
    * 360.0
    / (ENCODER_COUNTS_PER_REV * STEERING_REDUCTION)
)
MAX_DECELERATION_DEG_S2 = (
    PROFILE_DECELERATION_COUNTS_S2
    * 360.0
    / (ENCODER_COUNTS_PER_REV * STEERING_REDUCTION)
)


robot = SingleArticulation(ARTICULATION_ROOT)
robot.initialize()
name_to_index = {name: index for index, name in enumerate(robot.dof_names)}
front_index = name_to_index[STEERING_JOINTS["front"]]
rear_index = name_to_index[STEERING_JOINTS["rear"]]
steering_indices = np.asarray([front_index, rear_index], dtype=np.int32)
timeline = omni.timeline.get_timeline_interface()

physics_step_count = 0
physics_elapsed_s = 0.0


def _on_physics_step(step_dt):
    global physics_step_count, physics_elapsed_s
    physics_step_count += 1
    physics_elapsed_s += float(step_dt)


physics_step_subscription = (
    omni.physx.get_physx_interface().subscribe_physics_step_events(_on_physics_step)
)


def _require_playing():
    if not timeline.is_playing():
        raise RuntimeError("Press Play, reload this file, and run the command again.")


def _validate_prepared_stage(
    condition: str, friction: float | None, expected_physics_hz: int
):
    stage = omni.usd.get_context().get_stage()
    physics_scenes = [prim for prim in stage.Traverse() if prim.GetTypeName() == "PhysicsScene"]
    if not physics_scenes:
        raise RuntimeError("No PhysicsScene was found")
    physics_hz = physics_scenes[0].GetAttribute("physxScene:timeStepsPerSecond").Get()
    if physics_hz is None or abs(float(physics_hz) - float(expected_physics_hz)) > 1e-6:
        raise RuntimeError(
            f"Physics rate is {physics_hz}, expected {expected_physics_hz} Hz. "
            "Press Stop and run "
            "isaac_prepare_steering_stage.py first."
        )
    for location, path in STEERING_JOINT_PATHS.items():
        prim = stage.GetPrimAtPath(path)
        lower = prim.GetAttribute("physics:lowerLimit").Get() if prim.IsValid() else None
        upper = prim.GetAttribute("physics:upperLimit").Get() if prim.IsValid() else None
        if lower is None or upper is None or abs(float(lower) + 45.0) > 1e-6 or abs(float(upper) - 45.0) > 1e-6:
            raise RuntimeError(
                f"{location} limit is [{lower}, {upper}], expected [-45, 45] deg. "
                "Prepare the stage before Play."
            )
    if condition == "ground":
        for path in (CALIBRATION_TIRE_MATERIAL, CALIBRATION_GROUND_MATERIAL):
            prim = stage.GetPrimAtPath(path)
            value = (
                prim.GetAttribute("physics:staticFriction").Get()
                if prim.IsValid()
                else None
            )
            if value is None or abs(float(value) - float(friction)) > 1e-6:
                raise RuntimeError(
                    f"Prepared material {path} has friction={value}, expected {friction}. "
                    "Press Stop and prepare the ground stage again."
                )
    ground = stage.GetPrimAtPath(GROUND_COLLIDER_PATH)
    collision_enabled = (
        ground.GetAttribute("physics:collisionEnabled").Get()
        if ground.IsValid()
        else None
    )
    expected_ground_collision = condition == "ground"
    if collision_enabled is None or bool(collision_enabled) != expected_ground_collision:
        raise RuntimeError(
            f"Ground collisionEnabled={collision_enabled}, expected "
            f"{expected_ground_collision}. Press Stop and prepare the stage again."
        )
    rigid_bodies = [
        prim
        for prim in stage.Traverse()
        if (
            str(prim.GetPath()) == ROBOT_ROOT_PATH
            or str(prim.GetPath()).startswith(ROBOT_ROOT_PATH + "/")
        )
        and prim.HasAPI(UsdPhysics.RigidBodyAPI)
    ]
    if not rigid_bodies:
        raise RuntimeError(f"No rigid bodies found below {ROBOT_ROOT_PATH}")
    expected_disable_gravity = condition == "air"
    bad_gravity = []
    for prim in rigid_bodies:
        value = prim.GetAttribute("physxRigidBody:disableGravity").Get()
        if value is None or bool(value) != expected_disable_gravity:
            bad_gravity.append((str(prim.GetPath()), value))
    if bad_gravity:
        raise RuntimeError(
            f"Unexpected disableGravity setting on {bad_gravity[:3]}. "
            "Press Stop and prepare the stage again."
        )
    print(
        f"Prepared stage verified: physics={expected_physics_hz} Hz, "
        f"limits=+/-45 deg, condition={condition}, "
        f"gravity_disabled={expected_disable_gravity}, "
        f"ground_collision={expected_ground_collision}"
    )


async def _wait_for_physics(duration_s: float, label: str):
    """Wait in simulation time, but fail instead of hanging if PhysX stops."""
    app = omni.kit.app.get_app()
    physics_start = physics_elapsed_s
    wall_start = time.perf_counter()
    wall_timeout = max(10.0, float(duration_s) * 8.0)
    while physics_elapsed_s - physics_start < float(duration_s):
        await app.next_update_async()
        if not timeline.is_playing():
            raise RuntimeError(f"Timeline stopped during {label}")
        if time.perf_counter() - wall_start > wall_timeout:
            raise RuntimeError(
                f"Physics did not advance enough during {label}; "
                "check the Console for a PhysX error"
            )


def _set_targets(
    front_deg: float,
    rear_deg: float,
    front_velocity_deg_s: float = 0.0,
    rear_velocity_deg_s: float = 0.0,
):
    robot.apply_action(
        ArticulationAction(
            joint_positions=np.deg2rad(
                np.asarray([front_deg, rear_deg], dtype=np.float32)
            ),
            joint_velocities=np.deg2rad(
                np.asarray(
                    [front_velocity_deg_s, rear_velocity_deg_s], dtype=np.float32
                )
            ),
            joint_indices=steering_indices,
        )
    )


def set_runtime_steering_gains(kp: float = 10000.0, kd: float = 1000.0):
    """Apply temporary gains to both steering joints without saving the USD."""
    stiffnesses, dampings = robot.get_articulation_controller().get_gains()
    stiffnesses = np.asarray(stiffnesses, dtype=np.float32).copy()
    dampings = np.asarray(dampings, dtype=np.float32).copy()
    stiffnesses[steering_indices] = float(kp)
    dampings[steering_indices] = float(kd)
    robot.get_articulation_controller().set_gains(
        stiffnesses, dampings, save_to_usd=False
    )
    print(
        f"Runtime steering gains: kp={float(kp):.3f}, kd={float(kd):.3f} "
        "(USD not modified)"
    )


def _profile_parameters(distance_deg: float):
    distance = abs(float(distance_deg))
    accel_time = MAX_VELOCITY_DEG_S / MAX_ACCELERATION_DEG_S2
    decel_time = MAX_VELOCITY_DEG_S / MAX_DECELERATION_DEG_S2
    accel_distance = 0.5 * MAX_ACCELERATION_DEG_S2 * accel_time**2
    decel_distance = 0.5 * MAX_DECELERATION_DEG_S2 * decel_time**2
    if accel_distance + decel_distance <= distance:
        peak_velocity = MAX_VELOCITY_DEG_S
        cruise_time = (distance - accel_distance - decel_distance) / peak_velocity
    else:
        peak_velocity = math.sqrt(
            2.0
            * distance
            / (1.0 / MAX_ACCELERATION_DEG_S2 + 1.0 / MAX_DECELERATION_DEG_S2)
        )
        accel_time = peak_velocity / MAX_ACCELERATION_DEG_S2
        decel_time = peak_velocity / MAX_DECELERATION_DEG_S2
        accel_distance = 0.5 * MAX_ACCELERATION_DEG_S2 * accel_time**2
        decel_distance = 0.5 * MAX_DECELERATION_DEG_S2 * decel_time**2
        cruise_time = 0.0
    return {
        "distance": distance,
        "peak_velocity": peak_velocity,
        "accel_time": accel_time,
        "cruise_time": cruise_time,
        "decel_time": decel_time,
        "accel_distance": accel_distance,
        "decel_distance": decel_distance,
        "total_time": accel_time + cruise_time + decel_time,
    }


def _profile_position(elapsed_s: float, parameters: dict[str, float]) -> float:
    t = max(0.0, float(elapsed_s))
    ta = parameters["accel_time"]
    tc = parameters["cruise_time"]
    td = parameters["decel_time"]
    if t < ta:
        return 0.5 * MAX_ACCELERATION_DEG_S2 * t**2
    if t < ta + tc:
        return parameters["accel_distance"] + parameters["peak_velocity"] * (t - ta)
    if t < ta + tc + td:
        remaining = ta + tc + td - t
        return parameters["distance"] - 0.5 * MAX_DECELERATION_DEG_S2 * remaining**2
    return parameters["distance"]


def _profile_velocity(elapsed_s: float, parameters: dict[str, float]) -> float:
    """Return the Profile Position trajectory velocity in deg/s."""
    t = max(0.0, float(elapsed_s))
    ta = parameters["accel_time"]
    tc = parameters["cruise_time"]
    td = parameters["decel_time"]
    if t < ta:
        return MAX_ACCELERATION_DEG_S2 * t
    if t < ta + tc:
        return parameters["peak_velocity"]
    if t < ta + tc + td:
        remaining = ta + tc + td - t
        return MAX_DECELERATION_DEG_S2 * remaining
    return 0.0


def _safe_label(value: str) -> str:
    safe = "".join(character if character.isalnum() or character in "-_" else "_" for character in value)
    if not safe:
        raise ValueError("run_id must contain at least one letter or digit")
    return safe


async def _run_dual_steering_test(
    condition: str,
    friction: float | None,
    run_id: str,
    kp: float,
    kd: float,
    zero_settle_s: float,
    hold_s: float,
    sample_period_s: float,
    lift_m: float,
    physics_hz: int,
):
    _require_playing()
    app = omni.kit.app.get_app()
    condition = condition.lower()
    if condition not in ("air", "ground"):
        raise ValueError("condition must be 'air' or 'ground'")
    if condition == "ground" and friction is None:
        raise ValueError("ground runs require the configured friction value")
    _validate_prepared_stage(condition, friction, physics_hz)

    set_runtime_steering_gains(kp=kp, kd=kd)
    _set_targets(0.0, 0.0)
    await _wait_for_physics(float(zero_settle_s), "zero settling")

    original_position = None
    original_orientation = None
    if condition == "air":
        original_position, original_orientation = robot.get_world_pose()
        lifted = np.asarray(original_position, dtype=np.float32).copy()
        lifted[2] += float(lift_m)
        robot.set_world_pose(position=lifted, orientation=original_orientation)
        robot.set_linear_velocity(np.zeros(3, dtype=np.float32))
        robot.set_angular_velocity(np.zeros(3, dtype=np.float32))
        await _wait_for_physics(0.25, "air-pose settling")
        print(f"Contact-free model lift: {float(lift_m):.3f} m")

    profile = _profile_parameters(45.0)
    rows = []
    physics_start = physics_elapsed_s
    physics_step_start = physics_step_count
    wall_start = time.perf_counter()
    next_sample_s = 0.0
    total_duration = profile["total_time"] + float(hold_s)
    last_physics_progress_wall = wall_start
    last_physics_step = physics_step_count

    print("\n=== Isaac dual steering 0 -> +/-45 deg ===")
    print(f"condition={condition}, friction={friction}, run_id={run_id}")
    print(
        f"profile: vmax={MAX_VELOCITY_DEG_S:.6f} deg/s, "
        f"accel={MAX_ACCELERATION_DEG_S2:.6f} deg/s^2, "
        f"decel={MAX_DECELERATION_DEG_S2:.6f} deg/s^2, "
        f"motion_time={profile['total_time']:.6f} s"
    )

    while True:
        elapsed = physics_elapsed_s - physics_start
        command_magnitude = _profile_position(elapsed, profile)
        command_velocity = _profile_velocity(elapsed, profile)
        front_target = -command_magnitude
        rear_target = command_magnitude
        front_velocity_target = -command_velocity
        rear_velocity_target = command_velocity
        _set_targets(
            front_target,
            rear_target,
            front_velocity_target,
            rear_velocity_target,
        )
        await app.next_update_async()
        elapsed = physics_elapsed_s - physics_start
        if not timeline.is_playing():
            raise RuntimeError("Timeline stopped during steering test")
        if physics_step_count != last_physics_step:
            last_physics_step = physics_step_count
            last_physics_progress_wall = time.perf_counter()
        elif time.perf_counter() - last_physics_progress_wall > 10.0:
            raise RuntimeError(
                "Physics stopped advancing during steering test; "
                "check the Console for a PhysX error"
            )
        if elapsed + 1e-9 < next_sample_s:
            if elapsed < total_duration:
                continue
        positions = robot.get_joint_positions()
        velocities = robot.get_joint_velocities()
        efforts = robot.get_measured_joint_efforts()
        rows.append(
            {
                "time_s": elapsed,
                "physics_steps": physics_step_count - physics_step_start,
                "front_actual_deg": float(np.rad2deg(positions[front_index])),
                "rear_actual_deg": float(np.rad2deg(positions[rear_index])),
                "front_velocity_deg_s": float(np.rad2deg(velocities[front_index])),
                "rear_velocity_deg_s": float(np.rad2deg(velocities[rear_index])),
                "front_target_deg": front_target,
                "rear_target_deg": rear_target,
                "front_target_velocity_deg_s": front_velocity_target,
                "rear_target_velocity_deg_s": rear_velocity_target,
                "front_error_deg": front_target - float(np.rad2deg(positions[front_index])),
                "rear_error_deg": rear_target - float(np.rad2deg(positions[rear_index])),
                "front_measured_effort": float(efforts[front_index]),
                "rear_measured_effort": float(efforts[rear_index]),
            }
        )
        next_sample_s += float(sample_period_s)
        if elapsed >= total_duration:
            break

    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    friction_label = "none" if friction is None else f"{float(friction):.4f}".replace(".", "p")
    stem = f"{condition}_mu_{friction_label}_{_safe_label(run_id)}"
    csv_path = OUTPUT_ROOT / f"{stem}.csv"
    metadata_path = OUTPUT_ROOT / f"{stem}.json"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    metadata = {
        "condition": condition,
        "friction": friction,
        "run_id": run_id,
        "articulation_root": ARTICULATION_ROOT,
        "steering_joint_indices": {"front": front_index, "rear": rear_index},
        "kp": float(kp),
        "kd": float(kd),
        "physics_hz": int(physics_hz),
        "zero_settle_s": float(zero_settle_s),
        "hold_s": float(hold_s),
        "requested_sample_period_s": float(sample_period_s),
        "observed_physics_steps": physics_step_count - physics_step_start,
        "observed_physics_time_s": physics_elapsed_s - physics_start,
        "wall_time_s": time.perf_counter() - wall_start,
        "profile": profile,
        "profile_limits": {
            "velocity_deg_s": MAX_VELOCITY_DEG_S,
            "acceleration_deg_s2": MAX_ACCELERATION_DEG_S2,
            "deceleration_deg_s2": MAX_DECELERATION_DEG_S2,
        },
    }
    metadata_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    _set_targets(0.0, 0.0)

    print(f"Saved CSV: {csv_path}")
    print(f"Saved metadata: {metadata_path}")
    if condition == "air":
        print("Air-model run complete. Press Stop now to restore the authored pose.")
    else:
        print("Ground run complete. The steering target has been returned to zero.")


def start_dual_steering_test(
    condition: str = "air",
    friction: float | None = None,
    run_id: str = "run01",
    kp: float = 10000.0,
    kd: float = 1000.0,
    zero_settle_s: float = 2.0,
    hold_s: float = 2.0,
    sample_period_s: float = 0.02,
    lift_m: float = 5.0,
    physics_hz: int = 120,
):
    """Schedule a non-blocking dual-axis calibration run."""
    global dual_steering_task
    if "dual_steering_task" in globals() and not dual_steering_task.done():
        raise RuntimeError("A dual steering test is already running")
    dual_steering_task = asyncio.ensure_future(
        _run_dual_steering_test(
            condition=condition,
            friction=friction,
            run_id=run_id,
            kp=kp,
            kd=kd,
            zero_settle_s=zero_settle_s,
            hold_s=hold_s,
            sample_period_s=sample_period_s,
            lift_m=lift_m,
            physics_hz=physics_hz,
        )
    )
    dual_steering_task.add_done_callback(_report_task_result)
    profile = _profile_parameters(45.0)
    print(
        "Dual steering test scheduled. Keep Timeline playing for about "
        f"{float(zero_settle_s) + profile['total_time'] + float(hold_s) + 0.5:.1f} s."
    )


def _report_task_result(task):
    """Consume async exceptions and print one concise Script Editor message."""
    try:
        task.result()
    except asyncio.CancelledError:
        print("Previous dual steering task cancelled during script reload.")
    except Exception as error:
        print(f"Dual steering test aborted: {type(error).__name__}: {error}")


print("Isaac dual-steering calibration runner loaded.")
print(f"Timeline playing: {timeline.is_playing()}")
print(f"DOF indices: front={front_index}, rear={rear_index}")
print(
    f"Real profile limits: velocity={MAX_VELOCITY_DEG_S:.6f} deg/s, "
    f"acceleration={MAX_ACCELERATION_DEG_S2:.6f} deg/s^2, "
    f"deceleration={MAX_DECELERATION_DEG_S2:.6f} deg/s^2"
)
