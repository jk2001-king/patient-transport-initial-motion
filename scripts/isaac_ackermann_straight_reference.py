"""Verify ACKERMANN +X straight travel and identify the stable caster reference.

Run after ``prepare_drive_sign_stage(test_friction=0.5, physics_hz=120)``.
The temporary friction is only for a direction/geometry check.  This runner
tests the two 180-degree-opposed caster configurations whose wheel planes are
parallel to chassis local X; the stable trailing configuration should require
little swivel motion during forward travel.
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
OUTPUT_ROOT = WORKSPACE / "results" / "isaac_ackermann_straight_reference"
ARTICULATION_ROOT = "/World/patient_transport/base_link"
STEERING_RAW_ZERO_DEG = 90.0
TARGET_SPEED_M_S = 0.075
WHEEL_RADIUS_M = 0.075
TARGET_WHEEL_RAD_S = TARGET_SPEED_M_S / WHEEL_RADIUS_M
SETTLE_S = 1.0
DRIVE_S = 5.0
SAMPLE_S = 1.0 / 60.0

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

# Both configurations have caster wheel planes parallel to chassis local X.
# They differ by 180 degrees in fork/trail direction.  The forward-stable one
# is selected from measured swivel change rather than assumed from CAD frames.
CASTER_REFERENCE_DEG = {
    "candidate_posneg": [90.0, -90.0, 90.0, -90.0],
    "candidate_negpos": [-90.0, 90.0, -90.0, 90.0],
}

old_task = globals().get("ackermann_straight_reference_task")
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


def _wrap_array(values):
    values = np.asarray(values, dtype=float)
    return np.arctan2(np.sin(values), np.cos(values))


def _set_drive(scale):
    # Verified common forward signs.  At raw steering +90/+90 this produces
    # chassis local +X motion.
    robot.apply_action(
        ArticulationAction(
            joint_velocities=np.asarray(
                [TARGET_WHEEL_RAD_S * scale, -TARGET_WHEEL_RAD_S * scale],
                dtype=np.float32,
            ),
            joint_indices=wheel_indices,
        )
    )


def _set_steering_zero():
    robot.apply_action(
        ArticulationAction(
            joint_positions=np.deg2rad(
                np.asarray(
                    [STEERING_RAW_ZERO_DEG, STEERING_RAW_ZERO_DEG],
                    dtype=np.float32,
                )
            ),
            joint_indices=steering_indices,
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
    q[caster_indices] = np.deg2rad(np.asarray(caster_degs, dtype=np.float32))
    robot.set_world_pose(position=root_position, orientation=root_orientation)
    robot.set_joint_positions(q)
    robot.set_joint_velocities(np.zeros(len(robot.dof_names), dtype=np.float32))
    _set_steering_zero()
    await _wait_physics(SETTLE_S, "case settling")

    actual_steering_deg = np.rad2deg(robot.get_joint_positions()[steering_indices])
    if float(np.max(np.abs(actual_steering_deg - STEERING_RAW_ZERO_DEG))) > 0.25:
        raise RuntimeError(
            "ACKERMANN zero did not hold: "
            f"actual raw={actual_steering_deg.tolist()}"
        )

    # Reapply passive caster positions after chassis settling so every case
    # starts from the requested orientation at drive onset.
    q = robot.get_joint_positions().copy()
    targets = np.deg2rad(np.asarray(caster_degs, dtype=np.float32))
    q[caster_indices] = targets
    robot.set_joint_positions(q)
    robot.set_joint_velocities(np.zeros(len(robot.dof_names), dtype=np.float32))
    actual = robot.get_joint_positions()[caster_indices]
    error_deg = np.rad2deg(_wrap_array(actual - targets))
    if float(np.max(np.abs(error_deg))) > 2.0:
        raise RuntimeError(
            f"Caster reset did not hold: max error={np.max(np.abs(error_deg)):.3f} deg"
        )


async def _run_candidate(
    root_position,
    root_orientation,
    name,
    caster_degs,
    aligned_degs,
    run_id,
):
    await _reset_case(root_position, root_orientation, caster_degs)
    start_position, start_orientation = robot.get_world_pose()
    start_position = np.asarray(start_position, dtype=float)
    start_yaw = _yaw_from_wxyz(start_orientation)
    long_axis = np.asarray([math.cos(start_yaw), math.sin(start_yaw)])
    left_axis = np.asarray([-math.sin(start_yaw), math.cos(start_yaw)])
    initial_caster = robot.get_joint_positions()[caster_indices].copy()
    aligned_caster = np.deg2rad(np.asarray(aligned_degs, dtype=np.float32))
    start_time = physics_time_s
    next_sample = 0.0
    rows = []
    app = omni.kit.app.get_app()
    _set_drive(1.0)

    while physics_time_s - start_time < DRIVE_S:
        await app.next_update_async()
        if not timeline.is_playing():
            raise RuntimeError("Timeline stopped during ACKERMANN straight check")
        elapsed = physics_time_s - start_time
        if elapsed + 1e-9 < next_sample:
            continue
        position, orientation = robot.get_world_pose()
        q = robot.get_joint_positions()
        qd = robot.get_joint_velocities()
        delta_xy = np.asarray(position[:2], dtype=float) - start_position[:2]
        caster_change = _wrap_array(q[caster_indices] - initial_caster)
        alignment_error = _wrap_array(q[caster_indices] - aligned_caster)
        rows.append(
            {
                "time_s": float(elapsed),
                "run_id": run_id,
                "candidate": name,
                "longitudinal_m": float(np.dot(delta_xy, long_axis)),
                "lateral_m": float(np.dot(delta_xy, left_axis)),
                "yaw_error_deg": math.degrees(
                    math.atan2(
                        math.sin(_yaw_from_wxyz(orientation) - start_yaw),
                        math.cos(_yaw_from_wxyz(orientation) - start_yaw),
                    )
                ),
                "front_physical_steering_deg": float(
                    np.rad2deg(q[steering_indices[0]]) - STEERING_RAW_ZERO_DEG
                ),
                "rear_physical_steering_deg": float(
                    np.rad2deg(q[steering_indices[1]]) - STEERING_RAW_ZERO_DEG
                ),
                "caster_fl_raw_deg": float(np.rad2deg(q[caster_indices[0]])),
                "caster_fr_raw_deg": float(np.rad2deg(q[caster_indices[1]])),
                "caster_rl_raw_deg": float(np.rad2deg(q[caster_indices[2]])),
                "caster_rr_raw_deg": float(np.rad2deg(q[caster_indices[3]])),
                "max_caster_change_deg": float(
                    np.max(np.abs(np.rad2deg(caster_change)))
                ),
                "max_alignment_error_deg": float(
                    np.max(np.abs(np.rad2deg(alignment_error)))
                ),
                "front_wheel_rad_s": float(qd[wheel_indices[0]]),
                "rear_wheel_rad_s": float(qd[wheel_indices[1]]),
            }
        )
        next_sample += SAMPLE_S

    _set_drive(0.0)
    await _wait_physics(0.5, "candidate stopping")
    final = rows[-1]
    max_lateral = max(abs(row["lateral_m"]) for row in rows)
    max_yaw = max(abs(row["yaw_error_deg"]) for row in rows)
    max_caster_change = max(row["max_caster_change_deg"] for row in rows)
    initial_alignment_error = float(
        np.max(
            np.abs(
                np.rad2deg(
                    _wrap_array(initial_caster - aligned_caster)
                )
            )
        )
    )
    final_alignment_error = float(rows[-1]["max_alignment_error_deg"])
    summary = {
        "run_id": run_id,
        "candidate": name,
        "initial_caster_raw_deg": [float(value) for value in caster_degs],
        "aligned_caster_raw_deg": [float(value) for value in aligned_degs],
        "target_speed_m_s": TARGET_SPEED_M_S,
        "drive_duration_s": DRIVE_S,
        "final_longitudinal_m": float(final["longitudinal_m"]),
        "final_lateral_m": float(final["lateral_m"]),
        "max_abs_lateral_m": float(max_lateral),
        "final_yaw_error_deg": float(final["yaw_error_deg"]),
        "max_abs_yaw_error_deg": float(max_yaw),
        "max_caster_change_deg": float(max_caster_change),
        "initial_alignment_error_deg": initial_alignment_error,
        "final_alignment_error_deg": final_alignment_error,
        "final_over_initial_alignment_error": (
            final_alignment_error / initial_alignment_error
            if initial_alignment_error > 1e-6
            else None
        ),
    }
    return rows, summary


async def _run(run_id, perturbation_deg):
    if not timeline.is_playing():
        raise RuntimeError("Press Play before starting the straight-reference check")
    root_position, root_orientation = robot.get_world_pose()
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    summaries = []
    print("=== ACKERMANN +X straight / caster reference check ===")
    print(
        f"speed={TARGET_SPEED_M_S:.3f} m/s, duration={DRIVE_S:.1f} s, "
        f"caster perturbation={perturbation_deg:.1f} deg, "
        "temporary friction only; not a calibrated caster result"
    )

    candidates = {
        "candidate_posneg": [
            90.0 + perturbation_deg,
            -90.0 - perturbation_deg,
            90.0 + perturbation_deg,
            -90.0 - perturbation_deg,
        ],
        "candidate_negpos": [
            -90.0 + perturbation_deg,
            90.0 - perturbation_deg,
            -90.0 + perturbation_deg,
            90.0 - perturbation_deg,
        ],
    }
    for name, caster_degs in candidates.items():
        rows, summary = await _run_candidate(
            root_position,
            root_orientation,
            name,
            caster_degs,
            CASTER_REFERENCE_DEG[name],
            str(run_id),
        )
        summaries.append(summary)
        csv_path = OUTPUT_ROOT / f"{run_id}_{name}.csv"
        with csv_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        print(
            f"{name}: long={summary['final_longitudinal_m']:+.4f} m, "
            f"max|lat|={summary['max_abs_lateral_m']:.6f} m, "
            f"max|yaw|={summary['max_abs_yaw_error_deg']:.4f} deg, "
            f"caster align error "
            f"{summary['initial_alignment_error_deg']:.3f} -> "
            f"{summary['final_alignment_error_deg']:.3f} deg"
        )

    # A stable forward-trailing reference reduces a small initial alignment
    # perturbation.  A leading reference increases it.  Lateral/yaw are only
    # tie breakers because this is a direction/geometry check.
    for item in summaries:
        item["reference_score"] = -(
            item["final_alignment_error_deg"]
            + 1000.0 * item["max_abs_lateral_m"]
            + 0.1 * item["max_abs_yaw_error_deg"]
        )
    ranked = sorted(summaries, key=lambda item: item["reference_score"], reverse=True)
    summary_path = OUTPUT_ROOT / f"{run_id}_summary.json"
    summary_path.write_text(json.dumps(ranked, indent=2), encoding="utf-8")
    print("=== Caster reference ranking ===")
    for rank, item in enumerate(ranked, start=1):
        print(
            f"{rank}: {item['candidate']}, "
            f"initial={item['initial_caster_raw_deg']}, "
            f"score={item['reference_score']:+.4f}"
        )
    print(f"Saved summary: {summary_path}")
    print("Press Stop now; do not save the session layer.")


def _task_result(task):
    try:
        task.result()
    except asyncio.CancelledError:
        print("Previous ACKERMANN straight-reference task cancelled.")
    except Exception as error:
        print(
            "ACKERMANN straight-reference check aborted: "
            f"{type(error).__name__}: {error}"
        )


def start_ackermann_straight_reference(run_id="run01", perturbation_deg=10.0):
    global ackermann_straight_reference_task
    if (
        "ackermann_straight_reference_task" in globals()
        and not ackermann_straight_reference_task.done()
    ):
        raise RuntimeError("An ACKERMANN straight-reference check is already running")
    perturbation_deg = float(perturbation_deg)
    if not 0.0 <= perturbation_deg <= 30.0:
        raise ValueError("perturbation_deg must be in [0, 30]")
    ackermann_straight_reference_task = asyncio.ensure_future(
        _run(str(run_id), perturbation_deg)
    )
    ackermann_straight_reference_task.add_done_callback(_task_result)
    print("ACKERMANN straight-reference check scheduled; keep Timeline playing for 13 s.")


print("Isaac ACKERMANN straight-reference runner loaded.")
print(f"Timeline playing: {timeline.is_playing()}")
print(
    f"DOF indices: caster={caster_indices.tolist()}, "
    f"steering={steering_indices.tolist()}, wheel={wheel_indices.tolist()}"
)
print("Run start_ackermann_straight_reference() when ready.")
