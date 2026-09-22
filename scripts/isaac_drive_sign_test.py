"""Low-speed front/rear drive sign and rolling-direction test for Isaac Sim."""

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
OUTPUT_ROOT = WORKSPACE / "results" / "isaac_drive_sign"
ARTICULATION_ROOT = "/World/patient_transport/base_link"
WHEEL_RADIUS_M = 0.075

_old_task = globals().get("drive_sign_task")
if _old_task is not None and not _old_task.done():
    _old_task.cancel()

robot = SingleArticulation(ARTICULATION_ROOT)
robot.initialize()
index = {name: i for i, name in enumerate(robot.dof_names)}
front_wheel_i = index["front_wheel_joint"]
rear_wheel_i = index["rear_wheel_joint"]
front_steer_i = index["front_steering_joint"]
rear_steer_i = index["rear_steering_joint"]
wheel_indices = np.asarray([front_wheel_i, rear_wheel_i], dtype=np.int32)
steering_indices = np.asarray([front_steer_i, rear_steer_i], dtype=np.int32)
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


def _set_wheel_velocity(front_rad_s: float, rear_rad_s: float):
    robot.apply_action(
        ArticulationAction(
            joint_velocities=np.asarray([front_rad_s, rear_rad_s], dtype=np.float32),
            joint_indices=wheel_indices,
        )
    )


async def _wait_physics(duration_s: float, label: str):
    app = omni.kit.app.get_app()
    start = physics_time_s
    wall_start = time.perf_counter()
    while physics_time_s - start < duration_s:
        await app.next_update_async()
        if not timeline.is_playing():
            raise RuntimeError(f"Timeline stopped during {label}")
        if time.perf_counter() - wall_start > max(10.0, duration_s * 8.0):
            raise RuntimeError(f"Physics timeout during {label}")


async def _run(front_rad_s, rear_rad_s, run_id, settle_s, drive_s, sample_s):
    if not timeline.is_playing():
        raise RuntimeError("Press Play before starting the drive sign test")
    if max(abs(front_rad_s), abs(rear_rad_s)) > 1.0:
        raise ValueError("Initial sign test is limited to +/-1.0 rad/s")

    robot.apply_action(
        ArticulationAction(
            joint_positions=np.zeros(2, dtype=np.float32),
            joint_velocities=np.zeros(2, dtype=np.float32),
            joint_indices=steering_indices,
        )
    )
    _set_wheel_velocity(0.0, 0.0)
    await _wait_physics(float(settle_s), "settling")

    initial_position, initial_orientation = robot.get_world_pose()
    initial_yaw = _yaw_from_wxyz(initial_orientation)
    start_time = physics_time_s
    start_steps = physics_steps
    next_sample = 0.0
    rows = []
    _set_wheel_velocity(float(front_rad_s), float(rear_rad_s))
    app = omni.kit.app.get_app()

    while physics_time_s - start_time < float(drive_s):
        await app.next_update_async()
        if not timeline.is_playing():
            raise RuntimeError("Timeline stopped during drive motion")
        elapsed = physics_time_s - start_time
        if elapsed + 1e-9 < next_sample:
            continue
        position, orientation = robot.get_world_pose()
        q = robot.get_joint_positions()
        qd = robot.get_joint_velocities()
        rows.append(
            {
                "time_s": elapsed,
                "x_m": float(position[0]),
                "y_m": float(position[1]),
                "z_m": float(position[2]),
                "yaw_rad": _yaw_from_wxyz(orientation),
                "front_steering_deg": float(np.rad2deg(q[front_steer_i])),
                "rear_steering_deg": float(np.rad2deg(q[rear_steer_i])),
                "front_wheel_velocity_rad_s": float(qd[front_wheel_i]),
                "rear_wheel_velocity_rad_s": float(qd[rear_wheel_i]),
                "front_target_rad_s": float(front_rad_s),
                "rear_target_rad_s": float(rear_rad_s),
            }
        )
        next_sample += float(sample_s)

    _set_wheel_velocity(0.0, 0.0)
    await _wait_physics(0.5, "wheel stopping")
    final_position, final_orientation = robot.get_world_pose()
    dx = float(final_position[0] - initial_position[0])
    dy = float(final_position[1] - initial_position[1])
    dyaw = math.atan2(
        math.sin(_yaw_from_wxyz(final_orientation) - initial_yaw),
        math.cos(_yaw_from_wxyz(final_orientation) - initial_yaw),
    )

    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    stem = f"drive_sign_{run_id}"
    csv_path = OUTPUT_ROOT / f"{stem}.csv"
    json_path = OUTPUT_ROOT / f"{stem}.json"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    json_path.write_text(
        json.dumps(
            {
                "run_id": run_id,
                "front_target_rad_s": float(front_rad_s),
                "rear_target_rad_s": float(rear_rad_s),
                "nominal_wheel_surface_speed_m_s": WHEEL_RADIUS_M
                * max(abs(float(front_rad_s)), abs(float(rear_rad_s))),
                "drive_duration_s": float(drive_s),
                "physics_steps": physics_steps - start_steps,
                "dx_m": dx,
                "dy_m": dy,
                "distance_m": math.hypot(dx, dy),
                "delta_yaw_rad": dyaw,
            },
            indent=2,
        ),
        encoding="utf-8",
    )
    print("=== Drive sign result ===")
    print(f"targets [rad/s]: front={front_rad_s:+.3f}, rear={rear_rad_s:+.3f}")
    print(f"displacement [m]: dx={dx:+.6f}, dy={dy:+.6f}")
    print(f"distance [m]: {math.hypot(dx, dy):.6f}")
    print(f"delta yaw [deg]: {math.degrees(dyaw):+.6f}")
    print(f"Saved CSV: {csv_path}")
    print(f"Saved metadata: {json_path}")
    print("Press Stop now; do not save the session layer.")


def _task_result(task):
    try:
        task.result()
    except asyncio.CancelledError:
        print("Previous drive sign task cancelled.")
    except Exception as error:
        print(f"Drive sign test aborted: {type(error).__name__}: {error}")


def start_drive_sign_test(
    front_rad_s: float = 1.0,
    rear_rad_s: float = 1.0,
    run_id: str = "plus_plus",
    settle_s: float = 2.0,
    drive_s: float = 2.0,
    sample_s: float = 0.02,
):
    global drive_sign_task
    if "drive_sign_task" in globals() and not drive_sign_task.done():
        raise RuntimeError("A drive sign test is already running")
    drive_sign_task = asyncio.ensure_future(
        _run(front_rad_s, rear_rad_s, run_id, settle_s, drive_s, sample_s)
    )
    drive_sign_task.add_done_callback(_task_result)
    print("Drive sign test scheduled; keep Timeline playing for about 5 seconds.")


print("Isaac drive-sign runner loaded.")
print(f"Timeline playing: {timeline.is_playing()}")
print(f"DOF indices: front_wheel={front_wheel_i}, rear_wheel={rear_wheel_i}")
