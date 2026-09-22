"""Run the minimal 3x3 straight initial-transient pilot inside Isaac Sim.

Load this file from Isaac Sim's Script Editor after preparing the temporary
drive stage and pressing Play. Results are screening data, not paper results.
"""

from __future__ import annotations

import asyncio
import csv
import itertools
import json
import math
from pathlib import Path
import time

import numpy as np
import omni.kit.app
import omni.physx
import omni.timeline
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction


WORKSPACE = Path("/home/jk/Research/patient_initial_ws")
OUTPUT_ROOT = WORKSPACE / "results" / "isaac_initial_transient_pilot"
ARTICULATION_ROOT = "/World/patient_transport/base_link"

PHYSICS_HZ = 120
WHEEL_RADIUS_M = 0.075
TARGET_SPEED_M_S = 0.075
TARGET_WHEEL_RAD_S = TARGET_SPEED_M_S / WHEEL_RADIUS_M
STEERING_RAW_ZERO_DEG = 90.0
DRIVE_DURATION_S = 4.0
INITIAL_WINDOW_S = 2.0
SETTLE_S = 1.0
SAMPLE_S = 1.0 / 60.0

# Raw swivel angles in FL, FR, RL, RR order.  A forward straight check with
# active steering at raw +90 deg identified the restoring/trailing reference
# as [+90, -90, +90, -90].  Raw zero on all four casters is 90 deg transverse
# to that reference, and the 180-deg-opposed configuration is leading/reversed.
CASTER_ALIGNED_DEG = np.asarray([90.0, -90.0, 90.0, -90.0], dtype=float)
INITIAL_CONDITIONS_DEG = {
    "aligned_0deg": [90.0, -90.0, 90.0, -90.0],
    "transverse_90deg": [0.0, 0.0, 0.0, 0.0],
    "reversed_180deg": [-90.0, 90.0, -90.0, 90.0],
}
METHODS = ("normal", "fixed_ramp", "proposed")
FIXED_RAMP_S = 1.0
PROPOSED_MIN_RAMP_S = 0.25
PROPOSED_MAX_RAMP_S = 2.0

DOF_NAMES = {
    "caster_fl": "caster_fl_swivel_joint",
    "caster_fr": "caster_fr_swivel_joint",
    "caster_rl": "caster_rl_swivel_joint",
    "caster_rr": "caster_rr_swivel_joint",
    "front_steering": "front_steering_joint",
    "rear_steering": "rear_steering_joint",
    "front_wheel": "front_wheel_joint",
    "rear_wheel": "rear_wheel_joint",
}

_old_task = globals().get("initial_transient_pilot_task")
if _old_task is not None and not _old_task.done():
    _old_task.cancel()

robot = SingleArticulation(ARTICULATION_ROOT)
robot.initialize()
index = {name: i for i, name in enumerate(robot.dof_names)}
missing = [name for name in DOF_NAMES.values() if name not in index]
if missing:
    raise RuntimeError(f"Missing required DOFs: {missing}")

caster_indices = np.asarray(
    [index[DOF_NAMES[key]] for key in ("caster_fl", "caster_fr", "caster_rl", "caster_rr")],
    dtype=np.int32,
)
steering_indices = np.asarray(
    [index[DOF_NAMES["front_steering"]], index[DOF_NAMES["rear_steering"]]],
    dtype=np.int32,
)
wheel_indices = np.asarray(
    [index[DOF_NAMES["front_wheel"]], index[DOF_NAMES["rear_wheel"]]],
    dtype=np.int32,
)
timeline = omni.timeline.get_timeline_interface()

physics_steps = 0
physics_time_s = 0.0


def _physics_step(dt):
    global physics_steps, physics_time_s
    physics_steps += 1
    physics_time_s += float(dt)


physics_subscription = omni.physx.get_physx_interface().subscribe_physics_step_events(
    _physics_step
)


def _yaw_from_wxyz(q):
    w, x, y, z = [float(value) for value in q]
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _smoothstep(progress):
    p = min(1.0, max(0.0, float(progress)))
    return p * p * (3.0 - 2.0 * p)


def _proposed_ramp_duration(initial_caster_degs):
    """One-shot initial-state schedule; no online caster observer is used."""
    deltas = np.deg2rad(
        np.asarray(initial_caster_degs, dtype=float) - CASTER_ALIGNED_DEG
    )
    deltas = np.arctan2(np.sin(deltas), np.cos(deltas))
    directional_severity = float(np.mean(0.5 * (1.0 - np.cos(deltas))))
    return PROPOSED_MIN_RAMP_S + (
        PROPOSED_MAX_RAMP_S - PROPOSED_MIN_RAMP_S
    ) * directional_severity


def _command_scale(method, elapsed_s, initial_caster_degs):
    if method == "normal":
        return 1.0, 0.0
    if method == "fixed_ramp":
        return _smoothstep(elapsed_s / FIXED_RAMP_S), FIXED_RAMP_S
    if method == "proposed":
        ramp_s = _proposed_ramp_duration(initial_caster_degs)
        return _smoothstep(elapsed_s / ramp_s), ramp_s
    raise ValueError(f"Unknown method: {method}")


def _set_drive(scale):
    # Verified robot-forward signs at ACKERMANN raw +90/+90: front +, rear -.
    robot.apply_action(
        ArticulationAction(
            joint_velocities=np.asarray(
                [TARGET_WHEEL_RAD_S * scale, -TARGET_WHEEL_RAD_S * scale],
                dtype=np.float32,
            ),
            joint_indices=wheel_indices,
        )
    )


async def _wait_physics(duration_s, label):
    app = omni.kit.app.get_app()
    start = physics_time_s
    wall_start = time.perf_counter()
    while physics_time_s - start < float(duration_s):
        await app.next_update_async()
        if not timeline.is_playing():
            raise RuntimeError(f"Timeline stopped during {label}")
        if time.perf_counter() - wall_start > max(10.0, float(duration_s) * 10.0):
            raise RuntimeError(f"Physics timeout during {label}")


async def _reset_case(root_position, root_orientation, initial_caster_degs):
    _set_drive(0.0)
    # First settle the chassis in a neutral configuration. Passive casters are
    # assigned only after settling; otherwise contact forces change the desired
    # initial angle before the case starts.
    joint_positions = np.zeros(len(robot.dof_names), dtype=np.float32)
    joint_positions[steering_indices] = np.deg2rad(STEERING_RAW_ZERO_DEG)
    robot.set_world_pose(position=root_position, orientation=root_orientation)
    robot.set_joint_positions(joint_positions)
    robot.set_joint_velocities(np.zeros(len(robot.dof_names), dtype=np.float32))
    robot.apply_action(
        ArticulationAction(
            joint_positions=np.deg2rad(
                np.asarray(
                    [STEERING_RAW_ZERO_DEG, STEERING_RAW_ZERO_DEG],
                    dtype=np.float32,
                )
            ),
            joint_velocities=np.zeros(2, dtype=np.float32),
            joint_indices=steering_indices,
        )
    )
    await _wait_physics(SETTLE_S, "case reset settling")

    targets = np.deg2rad(np.asarray(initial_caster_degs, dtype=np.float32))
    joint_positions = robot.get_joint_positions().copy()
    joint_positions[caster_indices] = targets
    joint_positions[steering_indices] = np.deg2rad(STEERING_RAW_ZERO_DEG)
    robot.set_joint_positions(joint_positions)
    robot.set_joint_velocities(np.zeros(len(robot.dof_names), dtype=np.float32))
    actual = robot.get_joint_positions()[caster_indices]
    wrapped_error = np.arctan2(np.sin(actual - targets), np.cos(actual - targets))
    max_error_deg = float(np.max(np.abs(np.rad2deg(wrapped_error))))
    if max_error_deg > 2.0:
        raise RuntimeError(
            f"Caster reset did not hold: max error={max_error_deg:.3f} deg"
        )
    actual_steering_deg = np.rad2deg(
        robot.get_joint_positions()[steering_indices]
    )
    steering_error_deg = float(
        np.max(np.abs(actual_steering_deg - STEERING_RAW_ZERO_DEG))
    )
    if steering_error_deg > 0.25:
        raise RuntimeError(
            "ACKERMANN steering zero did not hold: "
            f"actual raw={actual_steering_deg.tolist()}"
        )


def _case_metrics(rows):
    initial_rows = [row for row in rows if row["time_s"] <= INITIAL_WINDOW_S + 1e-9]
    times = np.asarray([row["time_s"] for row in initial_rows], dtype=float)
    lateral = np.asarray([row["lateral_m"] for row in initial_rows], dtype=float)
    yaw_error = np.asarray([row["yaw_error_rad"] for row in initial_rows], dtype=float)
    speed_scale = np.asarray([row["command_scale"] for row in rows], dtype=float)
    all_times = np.asarray([row["time_s"] for row in rows], dtype=float)
    reached = np.flatnonzero(speed_scale >= 0.99)
    reach_time = float(all_times[reached[0]]) if len(reached) else None
    return {
        "initial_max_abs_lateral_m": float(np.max(np.abs(lateral))),
        "initial_integral_abs_lateral_m_s": float(np.trapz(np.abs(lateral), times)),
        "initial_max_abs_yaw_deg": float(np.rad2deg(np.max(np.abs(yaw_error)))),
        "target_speed_99pct_time_s": reach_time,
        "final_longitudinal_m": float(rows[-1]["longitudinal_m"]),
        "final_lateral_m": float(rows[-1]["lateral_m"]),
        "final_yaw_error_deg": float(math.degrees(rows[-1]["yaw_error_rad"])),
        "initial_peak_abs_drive_effort_sim": float(
            max(
                abs(row["front_wheel_effort_sim"])
                + abs(row["rear_wheel_effort_sim"])
                for row in initial_rows
            )
        ),
        "initial_max_caster_misalignment_index": float(
            max(row["caster_misalignment_index"] for row in initial_rows)
        ),
        "final_caster_misalignment_index": float(
            rows[-1]["caster_misalignment_index"]
        ),
    }


async def _run_case(root_position, root_orientation, condition, caster_degs, method, batch_id):
    await _reset_case(root_position, root_orientation, caster_degs)
    start_position, start_orientation = robot.get_world_pose()
    start_yaw = _yaw_from_wxyz(start_orientation)
    # ACKERMANN raw +90/+90 and front+/rear- drive were verified to move along
    # chassis local +X.  The corresponding planar left axis is local +Y.
    forward_xy = np.asarray([math.cos(start_yaw), math.sin(start_yaw)])
    left_xy = np.asarray([-math.sin(start_yaw), math.cos(start_yaw)])
    start_time = physics_time_s
    next_sample = 0.0
    rows = []
    app = omni.kit.app.get_app()

    while physics_time_s - start_time < DRIVE_DURATION_S:
        elapsed = physics_time_s - start_time
        scale, ramp_s = _command_scale(method, elapsed, caster_degs)
        _set_drive(scale)
        await app.next_update_async()
        if not timeline.is_playing():
            raise RuntimeError("Timeline stopped during pilot motion")
        elapsed = physics_time_s - start_time
        if elapsed + 1e-9 < next_sample:
            continue

        position, orientation = robot.get_world_pose()
        q = robot.get_joint_positions()
        qd = robot.get_joint_velocities()
        efforts = robot.get_measured_joint_efforts()
        caster_delta = q[caster_indices] - np.deg2rad(CASTER_ALIGNED_DEG)
        caster_delta = np.arctan2(np.sin(caster_delta), np.cos(caster_delta))
        caster_misalignment_index = float(
            np.mean(0.5 * (1.0 - np.cos(caster_delta)))
        )
        delta_xy = np.asarray(position[:2], dtype=float) - np.asarray(
            start_position[:2], dtype=float
        )
        rows.append(
            {
                "time_s": float(elapsed),
                "batch_id": batch_id,
                "condition": condition,
                "initial_caster_fl_deg": float(caster_degs[0]),
                "initial_caster_fr_deg": float(caster_degs[1]),
                "initial_caster_rl_deg": float(caster_degs[2]),
                "initial_caster_rr_deg": float(caster_degs[3]),
                "method": method,
                "ramp_duration_s": float(ramp_s),
                "command_scale": float(scale),
                "v_command_m_s": float(TARGET_SPEED_M_S * scale),
                "x_world_m": float(position[0]),
                "y_world_m": float(position[1]),
                "longitudinal_m": float(np.dot(delta_xy, forward_xy)),
                "lateral_m": float(np.dot(delta_xy, left_xy)),
                "yaw_error_rad": float(_wrap(_yaw_from_wxyz(orientation) - start_yaw)),
                "front_steering_raw_rad": float(q[steering_indices[0]]),
                "rear_steering_raw_rad": float(q[steering_indices[1]]),
                "front_steering_physical_deg": float(
                    np.rad2deg(q[steering_indices[0]]) - STEERING_RAW_ZERO_DEG
                ),
                "rear_steering_physical_deg": float(
                    np.rad2deg(q[steering_indices[1]]) - STEERING_RAW_ZERO_DEG
                ),
                "caster_fl_rad": float(q[caster_indices[0]]),
                "caster_fr_rad": float(q[caster_indices[1]]),
                "caster_rl_rad": float(q[caster_indices[2]]),
                "caster_rr_rad": float(q[caster_indices[3]]),
                "caster_misalignment_index": caster_misalignment_index,
                "front_wheel_velocity_rad_s": float(qd[wheel_indices[0]]),
                "rear_wheel_velocity_rad_s": float(qd[wheel_indices[1]]),
                "front_wheel_effort_sim": float(efforts[wheel_indices[0]]),
                "rear_wheel_effort_sim": float(efforts[wheel_indices[1]]),
            }
        )
        next_sample += SAMPLE_S

    _set_drive(0.0)
    await _wait_physics(0.5, "case stopping")
    metrics = _case_metrics(rows)
    metrics.update(
        {
            "batch_id": batch_id,
            "condition": condition,
            "initial_caster_degs": [float(value) for value in caster_degs],
            "method": method,
            "samples": len(rows),
        }
    )

    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    stem = f"{batch_id}_{condition}_{method}"
    with (OUTPUT_ROOT / f"{stem}.csv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    (OUTPUT_ROOT / f"{stem}.json").write_text(
        json.dumps(metrics, indent=2), encoding="utf-8"
    )
    print(
        f"{condition:>18} / {method:<10} | "
        f"max|lat|={metrics['initial_max_abs_lateral_m']:.5f} m, "
        f"max|yaw|={metrics['initial_max_abs_yaw_deg']:.3f} deg, "
        f"peak effort={metrics['initial_peak_abs_drive_effort_sim']:.4f}, "
        f"final M={metrics['final_caster_misalignment_index']:.3f}, "
        f"reach99={metrics['target_speed_99pct_time_s']} s"
    )
    return metrics


async def _run_batch(batch_id):
    if not timeline.is_playing():
        raise RuntimeError("Press Play before starting the pilot batch")
    root_position, root_orientation = robot.get_world_pose()
    all_metrics = []
    print("=== Initial transient 3x3 pilot ===")
    print("screening only: traction friction is not physically calibrated")
    print("ACKERMANN raw zero=+90/+90 deg; forward=local +X")
    print(f"caster trailing reference={CASTER_ALIGNED_DEG.tolist()} deg")
    for (condition, caster_degs), method in itertools.product(
        INITIAL_CONDITIONS_DEG.items(), METHODS
    ):
        all_metrics.append(
            await _run_case(
                root_position,
                root_orientation,
                condition,
                caster_degs,
                method,
                batch_id,
            )
        )

    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    summary_path = OUTPUT_ROOT / f"{batch_id}_summary.csv"
    with summary_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(all_metrics[0]))
        writer.writeheader()
        writer.writerows(all_metrics)
    print("=== Pilot batch complete ===")
    print(f"Saved summary: {summary_path}")
    print("Press Stop now; do not save the session layer.")


def _task_result(task):
    try:
        task.result()
    except asyncio.CancelledError:
        print("Previous initial-transient pilot task cancelled.")
    except Exception as error:
        print(f"Initial-transient pilot aborted: {type(error).__name__}: {error}")


def start_initial_transient_pilot(batch_id="pilot01"):
    global initial_transient_pilot_task
    if "initial_transient_pilot_task" in globals() and not initial_transient_pilot_task.done():
        raise RuntimeError("An initial-transient pilot is already running")
    initial_transient_pilot_task = asyncio.ensure_future(_run_batch(str(batch_id)))
    initial_transient_pilot_task.add_done_callback(_task_result)
    print("3x3 pilot scheduled; keep Timeline playing for about 55 seconds.")


print("Isaac initial-transient pilot runner loaded.")
print(f"Timeline playing: {timeline.is_playing()}")
print(f"DOF indices: caster={caster_indices.tolist()}, steering={steering_indices.tolist()}, wheel={wheel_indices.tolist()}")
print("Run start_initial_transient_pilot() when ready.")
