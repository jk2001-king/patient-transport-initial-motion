"""Verify Play-start stability of the temporary ACKERMANN reference."""

from __future__ import annotations

import asyncio
import math
import time

import numpy as np
import omni.kit.app
import omni.physx
import omni.timeline
from isaacsim.core.prims import SingleArticulation


ARTICULATION_ROOT = "/World/patient_transport/base_link"
RAW_ZERO_DEG = 90.0
CHECK_DURATION_S = 2.0

old_task = globals().get("ackermann_reference_verify_task")
if old_task is not None and not old_task.done():
    old_task.cancel()

timeline = omni.timeline.get_timeline_interface()
robot = SingleArticulation(ARTICULATION_ROOT)
robot.initialize()
index = {name: i for i, name in enumerate(robot.dof_names)}
steering_indices = np.asarray(
    [index["front_steering_joint"], index["rear_steering_joint"]],
    dtype=np.int32,
)
wheel_indices = np.asarray(
    [index["front_wheel_joint"], index["rear_wheel_joint"]],
    dtype=np.int32,
)

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


async def _verify():
    if not timeline.is_playing():
        raise RuntimeError("Press Play before starting verification")

    start_position, start_orientation = robot.get_world_pose()
    start_position = np.asarray(start_position, dtype=float)
    start_yaw = _yaw_from_wxyz(start_orientation)
    start_q_deg = np.rad2deg(robot.get_joint_positions()[steering_indices])
    start_wheel_speed = robot.get_joint_velocities()[wheel_indices]
    start_time = physics_time_s
    wall_start = time.perf_counter()
    app = omni.kit.app.get_app()

    while physics_time_s - start_time < CHECK_DURATION_S:
        await app.next_update_async()
        if not timeline.is_playing():
            raise RuntimeError("Timeline stopped during ACKERMANN-reference verification")
        if time.perf_counter() - wall_start > 20.0:
            raise RuntimeError("Physics timeout during ACKERMANN-reference verification")

    final_position, final_orientation = robot.get_world_pose()
    final_position = np.asarray(final_position, dtype=float)
    final_yaw = _yaw_from_wxyz(final_orientation)
    final_q_deg = np.rad2deg(robot.get_joint_positions()[steering_indices])
    final_qd_deg_s = np.rad2deg(robot.get_joint_velocities()[steering_indices])
    final_wheel_speed = robot.get_joint_velocities()[wheel_indices]

    displacement_m = float(np.linalg.norm(final_position[:2] - start_position[:2]))
    yaw_change_deg = abs(math.degrees(_wrap(final_yaw - start_yaw)))
    initial_error_deg = float(np.max(np.abs(start_q_deg - RAW_ZERO_DEG)))
    final_error_deg = float(np.max(np.abs(final_q_deg - RAW_ZERO_DEG)))
    steering_speed_deg_s = float(np.max(np.abs(final_qd_deg_s)))
    wheel_speed_rad_s = float(
        max(np.max(np.abs(start_wheel_speed)), np.max(np.abs(final_wheel_speed)))
    )

    passed = (
        initial_error_deg <= 0.25
        and final_error_deg <= 0.25
        and steering_speed_deg_s <= 0.10
        and wheel_speed_rad_s <= 0.01
        and displacement_m <= 0.001
        and yaw_change_deg <= 0.05
    )

    print("=== ACKERMANN reference Play-start verification ===")
    print(f"initial steering raw [deg]: {start_q_deg.tolist()}")
    print(f"final steering raw [deg]:   {final_q_deg.tolist()}")
    print(f"final steering speed [deg/s]: {final_qd_deg_s.tolist()}")
    print(f"max wheel speed [rad/s]: {wheel_speed_rad_s:.8f}")
    print(f"chassis horizontal displacement [m]: {displacement_m:.8f}")
    print(f"chassis yaw change [deg]: {yaw_change_deg:.8f}")
    print(f"ACKERMANN REFERENCE VERIFICATION: {'PASSED' if passed else 'FAILED'}")
    print("Press Stop now. Do not save the USD session layer.")


def _task_result(task):
    try:
        task.result()
    except asyncio.CancelledError:
        print("Previous ACKERMANN-reference verification cancelled.")
    except Exception as error:
        print(
            "ACKERMANN-reference verification aborted: "
            f"{type(error).__name__}: {error}"
        )


def start_ackermann_reference_verification():
    global ackermann_reference_verify_task
    if (
        "ackermann_reference_verify_task" in globals()
        and not ackermann_reference_verify_task.done()
    ):
        raise RuntimeError("ACKERMANN-reference verification is already running")
    ackermann_reference_verify_task = asyncio.ensure_future(_verify())
    ackermann_reference_verify_task.add_done_callback(_task_result)
    print("ACKERMANN-reference verification scheduled; keep Timeline playing for 2 s.")


print("ACKERMANN-reference verifier loaded.")
print(f"Timeline playing: {timeline.is_playing()}")
print("Run start_ackermann_reference_verification() when ready.")
