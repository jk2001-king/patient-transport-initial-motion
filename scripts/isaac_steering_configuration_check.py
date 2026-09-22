"""Validate front-only, rear-only, and coordinated steering motion models.

This is a prerequisite motion-model check, not the initial-control experiment.
Each case starts with its steering angles already at the theoretical target,
then drives at wheel speeds consistent with the same commanded body-frame
longitudinal speed and yaw curvature.
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
OUTPUT_ROOT = WORKSPACE / "results" / "isaac_steering_configuration_check"
ARTICULATION_ROOT = "/World/patient_transport/base_link"

L_F_M = 0.645
L_R_M = 0.645
WHEELBASE_M = L_F_M + L_R_M
WHEEL_RADIUS_M = 0.075
BODY_LONGITUDINAL_SPEED_M_S = 0.10
TARGET_YAW_CURVATURE_1_M = 0.50
DRIVE_DURATION_S = 5.0
SETTLE_S = 1.0
SAMPLE_S = 1.0 / 60.0
STEERING_RAW_ZERO_DEG = 90.0
CASTER_FORWARD_ALIGNED_DEG = [90.0, -90.0, 90.0, -90.0]


def _configuration_targets():
    single_deg = math.degrees(math.atan(WHEELBASE_M * TARGET_YAW_CURVATURE_1_M))
    coordinated_deg = math.degrees(math.atan(L_F_M * TARGET_YAW_CURVATURE_1_M))
    beta_deg = coordinated_deg
    return {
        "front_only": {
            "front_physical_deg": single_deg,
            "rear_physical_deg": 0.0,
            "expected_beta_deg": beta_deg,
        },
        "rear_only": {
            "front_physical_deg": 0.0,
            "rear_physical_deg": -single_deg,
            "expected_beta_deg": -beta_deg,
        },
        "coordinated_counterphase": {
            "front_physical_deg": coordinated_deg,
            "rear_physical_deg": -coordinated_deg,
            "expected_beta_deg": 0.0,
        },
    }


CONFIGURATIONS = _configuration_targets()

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

old_task = globals().get("steering_configuration_check_task")
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


def _raw_steering_rad(front_physical_deg, rear_physical_deg):
    return np.deg2rad(
        np.asarray(
            [
                STEERING_RAW_ZERO_DEG + float(front_physical_deg),
                STEERING_RAW_ZERO_DEG + float(rear_physical_deg),
            ],
            dtype=np.float32,
        )
    )


def _rolling_speeds(config):
    front = math.radians(config["front_physical_deg"])
    rear = math.radians(config["rear_physical_deg"])
    front_speed = BODY_LONGITUDINAL_SPEED_M_S / math.cos(front)
    rear_speed = BODY_LONGITUDINAL_SPEED_M_S / math.cos(rear)
    return front_speed, rear_speed


def _set_drive(front_speed_m_s, rear_speed_m_s):
    robot.apply_action(
        ArticulationAction(
            joint_velocities=np.asarray(
                [
                    float(front_speed_m_s) / WHEEL_RADIUS_M,
                    -float(rear_speed_m_s) / WHEEL_RADIUS_M,
                ],
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


async def _reset_case(root_position, root_orientation, config):
    _set_drive(0.0, 0.0)
    steering_targets = _raw_steering_rad(
        config["front_physical_deg"], config["rear_physical_deg"]
    )
    q = np.zeros(len(robot.dof_names), dtype=np.float32)
    q[steering_indices] = steering_targets
    q[caster_indices] = np.deg2rad(
        np.asarray(CASTER_FORWARD_ALIGNED_DEG, dtype=np.float32)
    )
    robot.set_world_pose(position=root_position, orientation=root_orientation)
    robot.set_joint_positions(q)
    robot.set_joint_velocities(np.zeros(len(robot.dof_names), dtype=np.float32))
    robot.apply_action(
        ArticulationAction(
            joint_positions=steering_targets,
            joint_indices=steering_indices,
        )
    )
    await _wait_physics(SETTLE_S, "configuration settling")

    # Reapply caster reference immediately before motion so all cases begin
    # from the same passive-wheel state.
    q = robot.get_joint_positions().copy()
    q[steering_indices] = steering_targets
    q[caster_indices] = np.deg2rad(
        np.asarray(CASTER_FORWARD_ALIGNED_DEG, dtype=np.float32)
    )
    robot.set_joint_positions(q)
    robot.set_joint_velocities(np.zeros(len(robot.dof_names), dtype=np.float32))

    actual_physical = np.rad2deg(robot.get_joint_positions()[steering_indices]) - STEERING_RAW_ZERO_DEG
    expected_physical = np.asarray(
        [config["front_physical_deg"], config["rear_physical_deg"]], dtype=float
    )
    max_error = float(np.max(np.abs(actual_physical - expected_physical)))
    if max_error > 0.25:
        raise RuntimeError(
            f"Steering reset did not hold: actual={actual_physical.tolist()}, "
            f"expected={expected_physical.tolist()}"
        )


async def _run_case(root_position, root_orientation, name, config, run_id):
    await _reset_case(root_position, root_orientation, config)
    start_position, start_orientation = robot.get_world_pose()
    start_position = np.asarray(start_position, dtype=float)
    start_yaw = _yaw_from_wxyz(start_orientation)
    forward_xy = np.asarray([math.cos(start_yaw), math.sin(start_yaw)])
    left_xy = np.asarray([-math.sin(start_yaw), math.cos(start_yaw)])
    front_speed, rear_speed = _rolling_speeds(config)
    start_time = physics_time_s
    next_sample = 0.0
    previous_xy = start_position[:2].copy()
    accumulated_path = 0.0
    rows = []
    app = omni.kit.app.get_app()
    _set_drive(front_speed, rear_speed)

    while physics_time_s - start_time < DRIVE_DURATION_S:
        await app.next_update_async()
        if not timeline.is_playing():
            raise RuntimeError("Timeline stopped during configuration check")
        elapsed = physics_time_s - start_time
        if elapsed + 1e-9 < next_sample:
            continue

        position, orientation = robot.get_world_pose()
        position = np.asarray(position, dtype=float)
        q = robot.get_joint_positions()
        delta_step = position[:2] - previous_xy
        accumulated_path += float(np.linalg.norm(delta_step))
        previous_xy = position[:2].copy()
        delta_xy = position[:2] - start_position[:2]
        yaw_error = _wrap(_yaw_from_wxyz(orientation) - start_yaw)
        rows.append(
            {
                "time_s": float(elapsed),
                "run_id": run_id,
                "configuration": name,
                "front_target_physical_deg": float(config["front_physical_deg"]),
                "rear_target_physical_deg": float(config["rear_physical_deg"]),
                "expected_beta_deg": float(config["expected_beta_deg"]),
                "front_wheel_target_m_s": float(front_speed),
                "rear_wheel_target_m_s": float(rear_speed),
                "longitudinal_m": float(np.dot(delta_xy, forward_xy)),
                "lateral_m": float(np.dot(delta_xy, left_xy)),
                "path_length_m": float(accumulated_path),
                "yaw_error_rad": float(yaw_error),
                "yaw_error_deg": float(math.degrees(yaw_error)),
                "front_actual_physical_deg": float(
                    np.rad2deg(q[steering_indices[0]]) - STEERING_RAW_ZERO_DEG
                ),
                "rear_actual_physical_deg": float(
                    np.rad2deg(q[steering_indices[1]]) - STEERING_RAW_ZERO_DEG
                ),
                "caster_fl_deg": float(np.rad2deg(q[caster_indices[0]])),
                "caster_fr_deg": float(np.rad2deg(q[caster_indices[1]])),
                "caster_rl_deg": float(np.rad2deg(q[caster_indices[2]])),
                "caster_rr_deg": float(np.rad2deg(q[caster_indices[3]])),
            }
        )
        next_sample += SAMPLE_S

    _set_drive(0.0, 0.0)
    await _wait_physics(0.5, "configuration stopping")
    final = rows[-1]
    yaw_rad = float(final["yaw_error_rad"])
    longitudinal = float(final["longitudinal_m"])
    path_length = float(final["path_length_m"])
    measured_yaw_curvature = yaw_rad / longitudinal if abs(longitudinal) > 1e-6 else None
    measured_geometric_curvature = yaw_rad / path_length if path_length > 1e-6 else None
    target_angles = np.asarray(
        [config["front_physical_deg"], config["rear_physical_deg"]], dtype=float
    )
    steering_errors = [
        max(
            abs(row[key] - target)
            for row in rows
        )
        for key, target in zip(
            ("front_actual_physical_deg", "rear_actual_physical_deg"), target_angles
        )
    ]
    summary = {
        "run_id": run_id,
        "configuration": name,
        "target_yaw_curvature_1_m": TARGET_YAW_CURVATURE_1_M,
        "front_target_physical_deg": float(config["front_physical_deg"]),
        "rear_target_physical_deg": float(config["rear_physical_deg"]),
        "expected_beta_deg": float(config["expected_beta_deg"]),
        "front_wheel_target_m_s": float(front_speed),
        "rear_wheel_target_m_s": float(rear_speed),
        "final_longitudinal_m": longitudinal,
        "final_lateral_m": float(final["lateral_m"]),
        "path_length_m": path_length,
        "final_yaw_deg": float(final["yaw_error_deg"]),
        "measured_yaw_curvature_1_m": measured_yaw_curvature,
        "measured_geometric_curvature_1_m": measured_geometric_curvature,
        "yaw_curvature_error_pct": (
            100.0 * (measured_yaw_curvature - TARGET_YAW_CURVATURE_1_M)
            / TARGET_YAW_CURVATURE_1_M
            if measured_yaw_curvature is not None
            else None
        ),
        "max_front_steering_error_deg": float(steering_errors[0]),
        "max_rear_steering_error_deg": float(steering_errors[1]),
        "samples": len(rows),
    }

    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    stem = f"{run_id}_{name}"
    with (OUTPUT_ROOT / f"{stem}.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    (OUTPUT_ROOT / f"{stem}.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8"
    )
    print(
        f"{name:>28} | targets=({config['front_physical_deg']:+.2f}, "
        f"{config['rear_physical_deg']:+.2f}) deg, "
        f"long={longitudinal:+.4f} m, lat={final['lateral_m']:+.4f} m, "
        f"yaw={final['yaw_error_deg']:+.3f} deg, "
        f"k_yaw={measured_yaw_curvature:+.4f} 1/m"
    )
    return summary


async def _run_batch(run_id):
    if not timeline.is_playing():
        raise RuntimeError("Press Play before starting the configuration check")
    root_position, root_orientation = robot.get_world_pose()
    summaries = []
    print("=== Steering-configuration motion-model check ===")
    print(
        f"vx={BODY_LONGITUDINAL_SPEED_M_S:.3f} m/s, "
        f"target yaw curvature={TARGET_YAW_CURVATURE_1_M:.3f} 1/m, "
        f"duration={DRIVE_DURATION_S:.1f} s"
    )
    print("Steering is pre-positioned; this is not an initial-control result.")
    for name, config in CONFIGURATIONS.items():
        summaries.append(
            await _run_case(
                root_position, root_orientation, name, config, str(run_id)
            )
        )

    summary_path = OUTPUT_ROOT / f"{run_id}_summary.csv"
    with summary_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(summaries[0]))
        writer.writeheader()
        writer.writerows(summaries)
    print("=== Configuration check complete ===")
    print(f"Saved summary: {summary_path}")
    print("Press Stop now; do not save the session layer.")


def _task_result(task):
    try:
        task.result()
    except asyncio.CancelledError:
        print("Previous steering-configuration check cancelled.")
    except Exception as error:
        print(f"Configuration check aborted: {type(error).__name__}: {error}")


def start_steering_configuration_check(run_id="config01"):
    global steering_configuration_check_task
    if (
        "steering_configuration_check_task" in globals()
        and not steering_configuration_check_task.done()
    ):
        raise RuntimeError("A steering-configuration check is already running")
    steering_configuration_check_task = asyncio.ensure_future(_run_batch(str(run_id)))
    steering_configuration_check_task.add_done_callback(_task_result)
    print("Configuration check scheduled; keep Timeline playing for about 21 s.")


print("Isaac steering-configuration checker loaded.")
print(f"Timeline playing: {timeline.is_playing()}")
print(
    f"DOF indices: caster={caster_indices.tolist()}, "
    f"steering={steering_indices.tolist()}, wheel={wheel_indices.tolist()}"
)
print("Run start_steering_configuration_check() when ready.")
