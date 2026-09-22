"""Find the raw front/rear angles corresponding to physical ACKERMANN zero.

The CAD-exported model starts with its active wheel planes transverse to the
chassis long axis.  This runner tests all +/-90 deg front/rear combinations at
low speed.  It reports motion in the initial chassis frame where local +X is
the long-axis direction and local +Y is the transverse direction.

Before Play, run isaac_prepare_ackermann_zero_check.py with the Timeline
stopped.  All stage changes are temporary and the USD must not be saved.
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
OUTPUT_ROOT = WORKSPACE / "results" / "isaac_ackermann_zero_check"
ARTICULATION_ROOT = "/World/patient_transport/base_link"
WHEEL_RADIUS_M = 0.075
TEST_SPEED_M_S = 0.075
TEST_WHEEL_RAD_S = TEST_SPEED_M_S / WHEEL_RADIUS_M
SETTLE_S = 1.0
DRIVE_S = 2.0
SAMPLE_S = 1.0 / 60.0

# Existing drive-sign test established the signs needed for common travel:
# front positive, rear negative.
WHEEL_SIGNS = np.asarray([+1.0, -1.0], dtype=np.float32)
CANDIDATES_DEG = (
    (+90.0, +90.0),
    (+90.0, -90.0),
    (-90.0, +90.0),
    (-90.0, -90.0),
)


old_task = globals().get("ackermann_zero_check_task")
if old_task is not None and not old_task.done():
    old_task.cancel()

robot = SingleArticulation(ARTICULATION_ROOT)
robot.initialize()
index = {name: i for i, name in enumerate(robot.dof_names)}
required = (
    "front_steering_joint",
    "rear_steering_joint",
    "front_wheel_joint",
    "rear_wheel_joint",
)
missing = [name for name in required if name not in index]
if missing:
    raise RuntimeError(f"Missing required DOFs: {missing}")

steering_indices = np.asarray(
    [index["front_steering_joint"], index["rear_steering_joint"]],
    dtype=np.int32,
)
wheel_indices = np.asarray(
    [index["front_wheel_joint"], index["rear_wheel_joint"]],
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


def _set_drive(scale):
    robot.apply_action(
        ArticulationAction(
            joint_velocities=(WHEEL_SIGNS * TEST_WHEEL_RAD_S * float(scale)),
            joint_indices=wheel_indices,
        )
    )


def _set_steering_targets(front_deg, rear_deg):
    robot.apply_action(
        ArticulationAction(
            joint_positions=np.deg2rad(
                np.asarray([front_deg, rear_deg], dtype=np.float32)
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


async def _reset_case(root_position, root_orientation, candidate_deg):
    _set_drive(0.0)
    q = np.zeros(len(robot.dof_names), dtype=np.float32)
    q[steering_indices] = np.deg2rad(np.asarray(candidate_deg, dtype=np.float32))
    robot.set_world_pose(position=root_position, orientation=root_orientation)
    robot.set_joint_positions(q)
    robot.set_joint_velocities(np.zeros(len(robot.dof_names), dtype=np.float32))
    _set_steering_targets(*candidate_deg)
    await _wait_physics(SETTLE_S, "candidate settling")

    actual_deg = np.rad2deg(robot.get_joint_positions()[steering_indices])
    error = np.asarray(actual_deg, dtype=float) - np.asarray(candidate_deg, dtype=float)
    if float(np.max(np.abs(error))) > 2.0:
        raise RuntimeError(
            "Steering candidate did not hold: "
            f"target={candidate_deg}, actual={actual_deg.tolist()}"
        )


async def _run_candidate(root_position, root_orientation, candidate_deg):
    await _reset_case(root_position, root_orientation, candidate_deg)
    start_position, start_orientation = robot.get_world_pose()
    start_position = np.asarray(start_position, dtype=float)
    start_yaw = _yaw_from_wxyz(start_orientation)

    # The imported chassis long axis is local X: steering pivots are located
    # at X=+/-0.645 m.  Project world displacement into the initial body frame.
    long_axis = np.asarray([math.cos(start_yaw), math.sin(start_yaw)])
    left_axis = np.asarray([-math.sin(start_yaw), math.cos(start_yaw)])
    start_time = physics_time_s
    next_sample = 0.0
    rows = []
    app = omni.kit.app.get_app()
    _set_drive(1.0)

    while physics_time_s - start_time < DRIVE_S:
        await app.next_update_async()
        if not timeline.is_playing():
            raise RuntimeError("Timeline stopped during zero-offset check")
        elapsed = physics_time_s - start_time
        if elapsed + 1e-9 < next_sample:
            continue
        position, orientation = robot.get_world_pose()
        delta_xy = np.asarray(position[:2], dtype=float) - start_position[:2]
        q = robot.get_joint_positions()
        rows.append(
            {
                "time_s": float(elapsed),
                "front_target_raw_deg": float(candidate_deg[0]),
                "rear_target_raw_deg": float(candidate_deg[1]),
                "longitudinal_m": float(np.dot(delta_xy, long_axis)),
                "lateral_m": float(np.dot(delta_xy, left_axis)),
                "yaw_error_deg": math.degrees(
                    _wrap(_yaw_from_wxyz(orientation) - start_yaw)
                ),
                "front_actual_raw_deg": float(np.rad2deg(q[steering_indices[0]])),
                "rear_actual_raw_deg": float(np.rad2deg(q[steering_indices[1]])),
            }
        )
        next_sample += SAMPLE_S

    _set_drive(0.0)
    await _wait_physics(0.5, "candidate stopping")
    final = rows[-1]
    return rows, {
        "front_raw_zero_candidate_deg": float(candidate_deg[0]),
        "rear_raw_zero_candidate_deg": float(candidate_deg[1]),
        "longitudinal_m": float(final["longitudinal_m"]),
        "lateral_m": float(final["lateral_m"]),
        "abs_lateral_m": abs(float(final["lateral_m"])),
        "yaw_error_deg": float(final["yaw_error_deg"]),
        "abs_yaw_error_deg": abs(float(final["yaw_error_deg"])),
    }


async def _run_check(run_id):
    if not timeline.is_playing():
        raise RuntimeError("Press Play before starting the ACKERMANN zero check")
    root_position, root_orientation = robot.get_world_pose()
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    summaries = []
    print("=== ACKERMANN raw-zero candidate check ===")
    print("body long axis=local X; desired result: large |longitudinal|, small lateral/yaw")

    for front_deg, rear_deg in CANDIDATES_DEG:
        rows, summary = await _run_candidate(
            root_position, root_orientation, (front_deg, rear_deg)
        )
        summaries.append(summary)
        stem = f"{run_id}_front_{front_deg:+.0f}_rear_{rear_deg:+.0f}"
        with (OUTPUT_ROOT / f"{stem}.csv").open(
            "w", newline="", encoding="utf-8"
        ) as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        print(
            f"front={front_deg:+.0f}, rear={rear_deg:+.0f} deg | "
            f"long={summary['longitudinal_m']:+.4f} m, "
            f"lat={summary['lateral_m']:+.4f} m, "
            f"yaw={summary['yaw_error_deg']:+.3f} deg"
        )

    # This score is only a ranking aid.  Visual wheel-plane confirmation is
    # still required before adopting the winning offsets.
    for item in summaries:
        item["ranking_score"] = (
            abs(item["longitudinal_m"])
            - 3.0 * item["abs_lateral_m"]
            - 0.01 * item["abs_yaw_error_deg"]
        )
    ranked = sorted(summaries, key=lambda item: item["ranking_score"], reverse=True)
    summary_path = OUTPUT_ROOT / f"{run_id}_summary.json"
    summary_path.write_text(json.dumps(ranked, indent=2), encoding="utf-8")
    print("=== Candidate ranking ===")
    for rank, item in enumerate(ranked, start=1):
        print(
            f"{rank}: front={item['front_raw_zero_candidate_deg']:+.0f}, "
            f"rear={item['rear_raw_zero_candidate_deg']:+.0f}, "
            f"score={item['ranking_score']:+.4f}"
        )
    print(f"Saved summary: {summary_path}")
    print("Press Stop now. Visually inspect the top candidate; do not save the USD.")


def _task_result(task):
    try:
        task.result()
    except asyncio.CancelledError:
        print("Previous ACKERMANN zero check cancelled.")
    except Exception as error:
        print(f"ACKERMANN zero check aborted: {type(error).__name__}: {error}")


def start_ackermann_zero_check(run_id="run01"):
    global ackermann_zero_check_task
    if (
        "ackermann_zero_check_task" in globals()
        and not ackermann_zero_check_task.done()
    ):
        raise RuntimeError("An ACKERMANN zero check is already running")
    ackermann_zero_check_task = asyncio.ensure_future(_run_check(str(run_id)))
    ackermann_zero_check_task.add_done_callback(_task_result)
    print("ACKERMANN zero check scheduled; keep Timeline playing for about 15 seconds.")


print("Isaac ACKERMANN-zero runner loaded.")
print(f"Timeline playing: {timeline.is_playing()}")
print(f"steering indices={steering_indices.tolist()}, wheel indices={wheel_indices.tolist()}")
print("Run start_ackermann_zero_check() when ready.")
