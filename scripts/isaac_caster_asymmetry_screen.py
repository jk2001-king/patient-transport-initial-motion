"""Screen for measurable caster-induced body disturbance in ACKERMANN travel.

This is a causality/signal gate, not a controller comparison and not a
calibrated paper experiment.  It uses mirrored left/right asymmetric caster
states.  A real caster effect should produce comparable disturbance magnitude
with opposite lateral/yaw sign for the mirrored cases.
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
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction


WORKSPACE = Path("/home/jk/Research/patient_initial_ws")
OUTPUT_ROOT = WORKSPACE / "results" / "isaac_caster_asymmetry_screen"
ARTICULATION_ROOT = "/World/patient_transport/base_link"

WHEEL_RADIUS_M = 0.075
TARGET_SPEED_M_S = 0.20
TARGET_WHEEL_RAD_S = TARGET_SPEED_M_S / WHEEL_RADIUS_M
STEERING_RAW_ZERO_DEG = 90.0
DRIVE_DURATION_S = 3.0
INITIAL_WINDOW_S = 2.0
COMMON_RAMP_S = 0.25
SETTLE_S = 1.0
SAMPLE_S = 1.0 / 60.0

CASTER_ALIGNED_DEG = np.asarray([90.0, -90.0, 90.0, -90.0], dtype=float)
CONDITIONS_DEG = {
    "aligned": [90.0, -90.0, 90.0, -90.0],
    "left_transverse": [0.0, -90.0, 0.0, -90.0],
    "right_transverse": [90.0, 0.0, 90.0, 0.0],
    # Exact 180 deg is an ideal unstable equilibrium.  A 10 deg perturbation
    # makes the leading fork flip direction deterministically.
    "left_leading_perturbed": [-80.0, -90.0, -80.0, -90.0],
    "right_leading_perturbed": [90.0, 80.0, 90.0, 80.0],
}

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

old_task = globals().get("caster_asymmetry_screen_task")
if old_task is not None and not old_task.done():
    old_task.cancel()

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

physics_time_s = 0.0


def _physics_step(dt):
    global physics_time_s
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


def _set_drive(scale):
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
        if time.perf_counter() - wall_start > max(10.0, duration_s * 10.0):
            raise RuntimeError(f"Physics timeout during {label}")


async def _reset_case(root_position, root_orientation, caster_degs):
    _set_drive(0.0)
    q = np.zeros(len(robot.dof_names), dtype=np.float32)
    q[steering_indices] = np.deg2rad(STEERING_RAW_ZERO_DEG)
    robot.set_world_pose(position=root_position, orientation=root_orientation)
    robot.set_joint_positions(q)
    robot.set_joint_velocities(np.zeros(len(robot.dof_names), dtype=np.float32))
    robot.apply_action(
        ArticulationAction(
            joint_positions=np.deg2rad(
                np.asarray([STEERING_RAW_ZERO_DEG, STEERING_RAW_ZERO_DEG], dtype=np.float32)
            ),
            joint_indices=steering_indices,
        )
    )
    await _wait_physics(SETTLE_S, "case reset settling")

    targets = np.deg2rad(np.asarray(caster_degs, dtype=np.float32))
    q = robot.get_joint_positions().copy()
    q[caster_indices] = targets
    q[steering_indices] = np.deg2rad(STEERING_RAW_ZERO_DEG)
    robot.set_joint_positions(q)
    robot.set_joint_velocities(np.zeros(len(robot.dof_names), dtype=np.float32))

    actual = robot.get_joint_positions()
    caster_error = np.arctan2(
        np.sin(actual[caster_indices] - targets),
        np.cos(actual[caster_indices] - targets),
    )
    if float(np.max(np.abs(np.rad2deg(caster_error)))) > 2.0:
        raise RuntimeError("Caster reset did not hold")
    steering_deg = np.rad2deg(actual[steering_indices])
    if float(np.max(np.abs(steering_deg - STEERING_RAW_ZERO_DEG))) > 0.25:
        raise RuntimeError(f"ACKERMANN zero did not hold: raw={steering_deg.tolist()}")


def _metrics(rows):
    initial = [row for row in rows if row["time_s"] <= INITIAL_WINDOW_S + 1e-9]
    lat_peak = max(initial, key=lambda row: abs(row["lateral_m"]))
    yaw_peak = max(initial, key=lambda row: abs(row["yaw_error_deg"]))
    return {
        "initial_peak_lateral_m": float(lat_peak["lateral_m"]),
        "initial_max_abs_lateral_m": float(abs(lat_peak["lateral_m"])),
        "initial_peak_lateral_time_s": float(lat_peak["time_s"]),
        "initial_peak_yaw_deg": float(yaw_peak["yaw_error_deg"]),
        "initial_max_abs_yaw_deg": float(abs(yaw_peak["yaw_error_deg"])),
        "initial_peak_yaw_time_s": float(yaw_peak["time_s"]),
        "initial_peak_abs_drive_effort_sim": float(
            max(
                abs(row["front_wheel_effort_sim"])
                + abs(row["rear_wheel_effort_sim"])
                for row in initial
            )
        ),
        "final_longitudinal_m": float(rows[-1]["longitudinal_m"]),
        "final_lateral_m": float(rows[-1]["lateral_m"]),
        "final_yaw_error_deg": float(rows[-1]["yaw_error_deg"]),
        "final_caster_misalignment_index": float(
            rows[-1]["caster_misalignment_index"]
        ),
    }


async def _run_case(
    root_position,
    root_orientation,
    condition,
    caster_degs,
    batch_id,
    pre_drive_hold_s=0.0,
):
    await _reset_case(root_position, root_orientation, caster_degs)
    if float(pre_drive_hold_s) > 0.0:
        await _wait_physics(float(pre_drive_hold_s), "recording pre-drive hold")
    start_position, start_orientation = robot.get_world_pose()
    start_position = np.asarray(start_position, dtype=float)
    start_yaw = _yaw_from_wxyz(start_orientation)
    forward_xy = np.asarray([math.cos(start_yaw), math.sin(start_yaw)])
    left_xy = np.asarray([-math.sin(start_yaw), math.cos(start_yaw)])
    start_time = physics_time_s
    next_sample = 0.0
    rows = []
    app = omni.kit.app.get_app()

    while physics_time_s - start_time < DRIVE_DURATION_S:
        elapsed = physics_time_s - start_time
        scale = _smoothstep(elapsed / COMMON_RAMP_S)
        _set_drive(scale)
        await app.next_update_async()
        if not timeline.is_playing():
            raise RuntimeError("Timeline stopped during asymmetry screen")
        elapsed = physics_time_s - start_time
        if elapsed + 1e-9 < next_sample:
            continue

        position, orientation = robot.get_world_pose()
        q = robot.get_joint_positions()
        efforts = robot.get_measured_joint_efforts()
        delta_xy = np.asarray(position[:2], dtype=float) - start_position[:2]
        caster_delta = q[caster_indices] - np.deg2rad(CASTER_ALIGNED_DEG)
        caster_delta = np.arctan2(np.sin(caster_delta), np.cos(caster_delta))
        rows.append(
            {
                "time_s": float(elapsed),
                "batch_id": batch_id,
                "condition": condition,
                "command_scale": float(scale),
                "v_command_m_s": float(TARGET_SPEED_M_S * scale),
                "longitudinal_m": float(np.dot(delta_xy, forward_xy)),
                "lateral_m": float(np.dot(delta_xy, left_xy)),
                "yaw_error_deg": float(
                    math.degrees(_wrap(_yaw_from_wxyz(orientation) - start_yaw))
                ),
                "caster_fl_deg": float(np.rad2deg(q[caster_indices[0]])),
                "caster_fr_deg": float(np.rad2deg(q[caster_indices[1]])),
                "caster_rl_deg": float(np.rad2deg(q[caster_indices[2]])),
                "caster_rr_deg": float(np.rad2deg(q[caster_indices[3]])),
                "caster_misalignment_index": float(
                    np.mean(0.5 * (1.0 - np.cos(caster_delta)))
                ),
                "front_wheel_effort_sim": float(efforts[wheel_indices[0]]),
                "rear_wheel_effort_sim": float(efforts[wheel_indices[1]]),
            }
        )
        next_sample += SAMPLE_S

    _set_drive(0.0)
    await _wait_physics(0.5, "case stopping")
    summary = _metrics(rows)
    summary.update(
        {
            "batch_id": batch_id,
            "condition": condition,
            "initial_caster_degs": [float(value) for value in caster_degs],
            "target_speed_m_s": TARGET_SPEED_M_S,
            "common_ramp_s": COMMON_RAMP_S,
            "drive_duration_s": DRIVE_DURATION_S,
            "samples": len(rows),
        }
    )

    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    stem = f"{batch_id}_{condition}"
    with (OUTPUT_ROOT / f"{stem}.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    (OUTPUT_ROOT / f"{stem}.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8"
    )
    print(
        f"{condition:>25} | "
        f"peak lat={summary['initial_peak_lateral_m']:+.6f} m, "
        f"peak yaw={summary['initial_peak_yaw_deg']:+.4f} deg, "
        f"effort={summary['initial_peak_abs_drive_effort_sim']:.4f}, "
        f"long={summary['final_longitudinal_m']:.4f} m"
    )
    return summary


async def _run_batch(batch_id):
    if not timeline.is_playing():
        raise RuntimeError("Press Play before starting the asymmetry screen")
    root_position, root_orientation = robot.get_world_pose()
    summaries = []
    print("=== ACKERMANN caster asymmetry signal screen ===")
    print(
        f"speed={TARGET_SPEED_M_S:.3f} m/s, common ramp={COMMON_RAMP_S:.2f} s, "
        f"duration={DRIVE_DURATION_S:.1f} s"
    )
    print("screening only: friction and caster damping are not calibrated")
    for condition, caster_degs in CONDITIONS_DEG.items():
        summaries.append(
            await _run_case(
                root_position,
                root_orientation,
                condition,
                caster_degs,
                str(batch_id),
            )
        )

    summary_path = OUTPUT_ROOT / f"{batch_id}_summary.csv"
    with summary_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(summaries[0]))
        writer.writeheader()
        writer.writerows(summaries)
    print("=== Asymmetry screen complete ===")
    print(f"Saved summary: {summary_path}")
    print("Press Stop now; do not save the session layer.")


async def _run_single(condition, batch_id):
    if not timeline.is_playing():
        raise RuntimeError("Press Play before starting the asymmetry screen")
    if condition not in CONDITIONS_DEG:
        raise ValueError(
            f"Unknown condition {condition!r}; choose one of {list(CONDITIONS_DEG)}"
        )
    root_position, root_orientation = robot.get_world_pose()
    print("=== ACKERMANN caster asymmetry single-case recording ===")
    print(
        f"condition={condition}, speed={TARGET_SPEED_M_S:.3f} m/s, "
        f"common ramp={COMMON_RAMP_S:.2f} s, duration={DRIVE_DURATION_S:.1f} s"
    )
    print("screening only: friction and caster damping are not calibrated")
    await _run_case(
        root_position,
        root_orientation,
        condition,
        CONDITIONS_DEG[condition],
        str(batch_id),
        pre_drive_hold_s=3.0,
    )
    print("=== Single-case recording complete ===")
    print("Press Stop now; do not save the session layer.")


def _task_result(task):
    try:
        task.result()
    except asyncio.CancelledError:
        print("Previous caster asymmetry screen cancelled.")
    except Exception as error:
        print(f"Caster asymmetry screen aborted: {type(error).__name__}: {error}")


def start_caster_asymmetry_screen(batch_id="stress01"):
    global caster_asymmetry_screen_task
    if (
        "caster_asymmetry_screen_task" in globals()
        and not caster_asymmetry_screen_task.done()
    ):
        raise RuntimeError("A caster asymmetry screen is already running")
    caster_asymmetry_screen_task = asyncio.ensure_future(_run_batch(str(batch_id)))
    caster_asymmetry_screen_task.add_done_callback(_task_result)
    print("Caster asymmetry screen scheduled; keep Timeline playing for about 25 s.")


def start_caster_asymmetry_video(
    condition="left_leading_perturbed", batch_id="seminar_video01"
):
    """Run one visually strong condition for an uncluttered recording."""
    global caster_asymmetry_screen_task
    if (
        "caster_asymmetry_screen_task" in globals()
        and not caster_asymmetry_screen_task.done()
    ):
        raise RuntimeError("A caster asymmetry screen is already running")
    caster_asymmetry_screen_task = asyncio.ensure_future(
        _run_single(str(condition), str(batch_id))
    )
    caster_asymmetry_screen_task.add_done_callback(_task_result)
    print(
        f"Caster recording case scheduled: {condition}; "
        "keep Timeline playing for about 7 s."
    )


print("Isaac caster-asymmetry screen loaded.")
print(f"Timeline playing: {timeline.is_playing()}")
print(
    f"DOF indices: caster={caster_indices.tolist()}, "
    f"steering={steering_indices.tolist()}, wheel={wheel_indices.tolist()}"
)
print("Run start_caster_asymmetry_screen() when ready.")
print(
    "For one seminar recording, run "
    "start_caster_asymmetry_video('left_leading_perturbed', 'seminar_video01')."
)
