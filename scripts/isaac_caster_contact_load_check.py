"""Measure static ground-contact loads carried by active and caster wheels.

Load and call ``prepare_contact_load_check()`` while the Timeline is stopped.
Then press Play, wait about one second, and call
``start_contact_load_check()`` from the same Script Editor namespace.
All contact-report APIs are authored only in the session layer.
"""

from __future__ import annotations

import asyncio
import csv
import math
from pathlib import Path
import time

import numpy as np
import omni.kit.app
import omni.physx
import omni.timeline
import omni.usd
from isaacsim.core.api.sensors import RigidContactView
from isaacsim.core.prims import SingleArticulation
from omni.physx.bindings._physx import ContactEventType
from pxr import PhysicsSchemaTools, PhysxSchema, UsdPhysics, UsdUtils


WORKSPACE = Path("/home/jk/Research/patient_initial_ws")
OUTPUT_PATH = WORKSPACE / "results" / "isaac_contact_load" / "static_load.csv"
ROBOT_ROOT = "/World/patient_transport"
GROUND_COLLIDER = "/FlatGrid/GroundPlane/CollisionPlane"
PHYSICS_DT_S = 1.0 / 120.0
SAMPLE_DURATION_S = 2.0

ROLE_TOKENS = {
    "caster_fl": "caster_fl_wheel_link",
    "caster_fr": "caster_fr_wheel_link",
    "caster_rl": "caster_rl_wheel_link",
    "caster_rr": "caster_rr_wheel_link",
    "active_front": "front_wheel_link",
    "active_rear": "rear_wheel_link",
    "base": "base_link",
}

timeline = omni.timeline.get_timeline_interface()
physics_time_s = 0.0


def _physics_step(dt):
    global physics_time_s
    physics_time_s += float(dt)


physics_subscription = omni.physx.get_physx_interface().subscribe_physics_step_events(
    _physics_step
)
contact_event_capture_active = False
contact_event_counts = {}
contact_event_data_counts = {}
contact_event_nonzero_counts = {}
contact_event_impulse_vectors = {}
contact_event_abs_vertical_impulses = {}
contact_event_impulse_norms = {}
contact_event_max_impulses = {}
contact_event_other_ground_pairs = {}


def _role_for_path(path):
    text = str(path)
    for role, token in ROLE_TOKENS.items():
        if token in text:
            if role.startswith("active_") and "caster_" in text:
                continue
            return role
    return None


def _is_ground_path(path):
    text = str(path)
    return text == GROUND_COLLIDER or "/FlatGrid/GroundPlane" in text


def _robot_body_key(actor_path, collider_path):
    """Return a named role, or an explicit key for another robot body."""
    role = _role_for_path(actor_path) or _role_for_path(collider_path)
    if role is not None:
        return role
    for path in (actor_path, collider_path):
        text = str(path)
        if text == ROBOT_ROOT or text.startswith(ROBOT_ROOT + "/"):
            return "other_robot:" + text
    return None


def _on_contact_report_event(contact_headers, contact_data):
    if not contact_event_capture_active:
        return
    for header in contact_headers:
        if header.type not in (
            ContactEventType.CONTACT_FOUND,
            ContactEventType.CONTACT_PERSIST,
        ):
            continue
        actor0 = PhysicsSchemaTools.intToSdfPath(header.actor0)
        actor1 = PhysicsSchemaTools.intToSdfPath(header.actor1)
        collider0 = PhysicsSchemaTools.intToSdfPath(header.collider0)
        collider1 = PhysicsSchemaTools.intToSdfPath(header.collider1)
        role0 = _robot_body_key(actor0, collider0)
        role1 = _robot_body_key(actor1, collider1)
        role = None
        if role0 is not None and (_is_ground_path(actor1) or _is_ground_path(collider1)):
            role = role0
        elif role1 is not None and (_is_ground_path(actor0) or _is_ground_path(collider0)):
            role = role1
        if role is None:
            continue
        contact_event_counts[role] = contact_event_counts.get(role, 0) + 1
        begin = header.contact_data_offset
        end = begin + header.num_contact_data
        for item in contact_data[begin:end]:
            impulse = np.asarray(item.impulse, dtype=float)
            if impulse.shape != (3,):
                impulse = impulse.reshape(3)
            impulse_norm = float(np.linalg.norm(impulse))
            contact_event_data_counts[role] = (
                contact_event_data_counts.get(role, 0) + 1
            )
            if impulse_norm > 1.0e-9:
                contact_event_nonzero_counts[role] = (
                    contact_event_nonzero_counts.get(role, 0) + 1
                )
            contact_event_impulse_vectors[role] = (
                contact_event_impulse_vectors.get(role, np.zeros(3, dtype=float))
                + impulse
            )
            contact_event_abs_vertical_impulses[role] = (
                contact_event_abs_vertical_impulses.get(role, 0.0)
                + abs(float(impulse[2]))
            )
            contact_event_impulse_norms[role] = (
                contact_event_impulse_norms.get(role, 0.0)
                + impulse_norm
            )
            contact_event_max_impulses[role] = max(
                contact_event_max_impulses.get(role, 0.0), impulse_norm
            )


contact_report_subscription = (
    omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(
        _on_contact_report_event
    )
)


def _find_wheel_bodies(stage):
    robot_rigid_paths = []
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if not (path == ROBOT_ROOT or path.startswith(ROBOT_ROOT + "/")):
            continue
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            robot_rigid_paths.append(path)

    found = {}
    for role, token in ROLE_TOKENS.items():
        matches = [path for path in robot_rigid_paths if token in path]
        if role.startswith("active_"):
            matches = [path for path in matches if "caster_" not in path]
        if role == "base":
            matches = [path for path in matches if path.endswith("/base_link")]
        if len(matches) != 1:
            raise RuntimeError(
                f"Expected one rigid body for {role} token={token!r}, got {matches}. "
                f"Robot rigid bodies={robot_rigid_paths}"
            )
        found[role] = matches[0]
    return found


def prepare_contact_load_check():
    global contact_roles, contact_paths, contact_views
    global articulation_root_path, robot_rigid_paths
    global contact_event_capture_active, contact_event_counts
    global contact_event_data_counts, contact_event_nonzero_counts
    global contact_event_impulse_vectors, contact_event_abs_vertical_impulses
    global contact_event_impulse_norms, contact_event_max_impulses
    global contact_event_other_ground_pairs
    if timeline.is_playing():
        raise RuntimeError("Press Stop before preparing contact-load sensors")

    stage = omni.usd.get_context().get_stage()
    found = _find_wheel_bodies(stage)
    contact_roles = list(found)
    contact_paths = [found[role] for role in contact_roles]

    previous_target = stage.GetEditTarget()
    stage.SetEditTarget(stage.GetSessionLayer())
    try:
        # Apply the report API to every robot rigid body, not only the six
        # wheels.  This exposes accidental support by a frame/link collider.
        robot_rigid_paths = []
        articulation_root_paths = []
        for prim in stage.Traverse():
            path = str(prim.GetPath())
            if not (path == ROBOT_ROOT or path.startswith(ROBOT_ROOT + "/")):
                continue
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                robot_rigid_paths.append(path)
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                articulation_root_paths.append(path)
        if len(articulation_root_paths) != 1:
            raise RuntimeError(
                "Expected exactly one articulation root below "
                f"{ROBOT_ROOT}, got {articulation_root_paths}"
            )
        articulation_root_path = articulation_root_paths[0]
        articulation_root = stage.GetPrimAtPath(articulation_root_path)
        articulation_api = PhysxSchema.PhysxArticulationAPI.Apply(articulation_root)
        articulation_api.CreateSleepThresholdAttr().Set(0.0)
        articulation_api.CreateStabilizationThresholdAttr().Set(0.0)
        for path in robot_rigid_paths:
            prim = stage.GetPrimAtPath(path)
            PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr().Set(0.0)
            PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateSleepThresholdAttr().Set(0.0)
        ground = stage.GetPrimAtPath(GROUND_COLLIDER)
        if not ground.IsValid():
            raise RuntimeError(f"Missing ground collider: {GROUND_COLLIDER}")
        PhysxSchema.PhysxContactReportAPI.Apply(ground).CreateThresholdAttr().Set(0.0)
    finally:
        stage.SetEditTarget(previous_target)

    # Use one unfiltered view per body and read its net contact force.  The
    # previous ground-filtered matrix returned zeros with this stage because
    # the plane collision prim is not exposed as the tensor filter body that
    # the high-level API expects.  At static equilibrium the unfiltered net
    # force is the ground reaction for wheel bodies.
    contact_views = [
        RigidContactView(
            prim_paths_expr=path,
            filter_paths_expr=[],
            name=f"patient_transport_contact_{role}",
            prepare_contact_sensors=False,
            disable_stablization=False,
        )
        for role, path in zip(contact_roles, contact_paths)
    ]
    # Arm the callback before Play.  The counters are reset again immediately
    # before the timed sampling window, so force proxies have the correct
    # denominator. CONTACT_PERSIST continues to provide contact data.
    contact_event_counts = {role: 0 for role in contact_roles}
    contact_event_data_counts = {role: 0 for role in contact_roles}
    contact_event_nonzero_counts = {role: 0 for role in contact_roles}
    contact_event_impulse_vectors = {
        role: np.zeros(3, dtype=float) for role in contact_roles
    }
    contact_event_abs_vertical_impulses = {role: 0.0 for role in contact_roles}
    contact_event_impulse_norms = {role: 0.0 for role in contact_roles}
    contact_event_max_impulses = {role: 0.0 for role in contact_roles}
    contact_event_other_ground_pairs = {}
    contact_event_capture_active = True
    print("Temporary wheel contact reporters prepared (session layer only).")
    print(f"  articulation root: {articulation_root_path}")
    print("  articulation sleep/stabilization thresholds: 0 (temporary)")
    for role, path in zip(contact_roles, contact_paths):
        print(f"  {role}: {path}")
    print("Press Play, wait about one second, then run start_contact_load_check().")
    print("Raw contact-event capture is armed before Play.")
    print("Do not save the USD file.")


async def _sample_contact_loads():
    global contact_event_capture_active, contact_event_counts
    global contact_event_data_counts, contact_event_nonzero_counts
    global contact_event_impulse_vectors, contact_event_abs_vertical_impulses
    global contact_event_impulse_norms, contact_event_max_impulses
    if not timeline.is_playing():
        raise RuntimeError("Press Play before sampling contact loads")
    if "contact_views" not in globals():
        raise RuntimeError("Run prepare_contact_load_check() while stopped first")

    for view in contact_views:
        view.initialize()
    app = omni.kit.app.get_app()
    await app.next_update_async()

    # Discard the settling interval between Play and this command.  All event
    # force estimates below therefore use exactly the same timed interval as
    # the tensor samples.
    keys = list(contact_roles)
    contact_event_counts = {key: 0 for key in keys}
    contact_event_data_counts = {key: 0 for key in keys}
    contact_event_nonzero_counts = {key: 0 for key in keys}
    contact_event_impulse_vectors = {
        key: np.zeros(3, dtype=float) for key in keys
    }
    contact_event_abs_vertical_impulses = {key: 0.0 for key in keys}
    contact_event_impulse_norms = {key: 0.0 for key in keys}
    contact_event_max_impulses = {key: 0.0 for key in keys}
    contact_event_capture_active = True

    # A stationary articulation normally goes to sleep before this command is
    # issued.  Waking only the individual links is insufficient because sleep
    # is managed at articulation level.  Wake the articulation after clearing
    # the settling events so CONTACT_PERSIST impulses belong to this window.
    stage = omni.usd.get_context().get_stage()
    stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
    simulation_interface = omni.physx.get_physx_simulation_interface()
    articulation_encoded = PhysicsSchemaTools.sdfPathToInt(articulation_root_path)
    simulation_interface.wake_up(stage_id, articulation_encoded)
    await app.next_update_async()
    articulation_sleeping = simulation_interface.is_sleeping(
        stage_id, articulation_encoded
    )
    print(
        "Articulation wake check: "
        + ("SLEEPING (unexpected)" if articulation_sleeping else "AWAKE")
    )

    start = physics_time_s
    wall_start = time.perf_counter()
    samples = []
    while physics_time_s - start < SAMPLE_DURATION_S:
        await app.next_update_async()
        if not timeline.is_playing():
            raise RuntimeError("Timeline stopped during contact-load sampling")
        forces = np.stack(
            [
                np.asarray(view.get_net_contact_forces(dt=PHYSICS_DT_S), dtype=float)[0]
                for view in contact_views
            ],
            axis=0,
        )
        if forces.shape != (len(contact_roles), 3):
            raise RuntimeError(f"Unexpected contact-force shape: {forces.shape}")
        samples.append(forces.copy())
        if time.perf_counter() - wall_start > 20.0:
            raise RuntimeError("Physics timeout during contact-load sampling")

    sample_duration_s = max(physics_time_s - start, 1.0e-9)
    contact_event_capture_active = False
    data = np.asarray(samples, dtype=float)
    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    rows = []
    print("=== Static wheel contact loads ===")
    print(
        "role, mean_Fx_N, mean_Fy_N, mean_Fz_N, mean_abs_Fz_N, "
        "mean_horizontal_N, max_abs_Fz_N"
    )
    for i, role in enumerate(contact_roles):
        mean_force = np.mean(data[:, i, :], axis=0)
        horizontal = np.linalg.norm(data[:, i, :2], axis=1)
        row = {
            "role": role,
            "prim_path": contact_paths[i],
            "mean_Fx_N": float(mean_force[0]),
            "mean_Fy_N": float(mean_force[1]),
            "mean_Fz_N": float(mean_force[2]),
            "mean_abs_Fz_N": float(np.mean(np.abs(data[:, i, 2]))),
            "mean_horizontal_N": float(np.mean(horizontal)),
            "max_abs_Fz_N": float(np.max(np.abs(data[:, i, 2]))),
            "ground_contact_event_count": int(contact_event_counts[role]),
            "event_contact_data_count": int(contact_event_data_counts[role]),
            "event_nonzero_impulse_count": int(
                contact_event_nonzero_counts[role]
            ),
            "event_mean_abs_Fz_N": float(
                contact_event_abs_vertical_impulses[role] / sample_duration_s
            ),
            "event_mean_impulse_norm_force_N": float(
                contact_event_impulse_norms[role] / sample_duration_s
            ),
            "event_max_single_impulse_Ns": float(
                contact_event_max_impulses[role]
            ),
            "sample_duration_s": float(sample_duration_s),
            "samples": int(len(samples)),
        }
        rows.append(row)
        print(
            f"{role}, {row['mean_Fx_N']:+.3f}, {row['mean_Fy_N']:+.3f}, "
            f"{row['mean_Fz_N']:+.3f}, {row['mean_abs_Fz_N']:.3f}, "
            f"{row['mean_horizontal_N']:.3f}, "
            f"{row['max_abs_Fz_N']:.3f}, "
            f"events={row['ground_contact_event_count']}, "
            f"contact_points={row['event_contact_data_count']}, "
            f"nonzero={row['event_nonzero_impulse_count']}, "
            f"event_|Fz|={row['event_mean_abs_Fz_N']:.3f} N"
        )

    with OUTPUT_PATH.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    total_load = sum(row["mean_abs_Fz_N"] for row in rows)
    caster_load = sum(
        row["mean_abs_Fz_N"] for row in rows if row["role"].startswith("caster_")
    )
    active_load = sum(
        row["mean_abs_Fz_N"] for row in rows if row["role"].startswith("active_")
    )
    base_load = sum(row["mean_abs_Fz_N"] for row in rows if row["role"] == "base")
    print(f"caster mean |Fz| sum: {caster_load:.3f} N")
    print(f"active mean |Fz| sum: {active_load:.3f} N")
    print(f"base mean |Fz|: {base_load:.3f} N")
    print(f"tracked-body mean |Fz| sum: {total_load:.3f} N")
    robot = SingleArticulation(f"{ROBOT_ROOT}/base_link")
    robot.initialize()
    body_masses = np.asarray(
        robot._articulation_view.get_body_masses(), dtype=float
    ).reshape(-1)
    total_mass = float(np.sum(body_masses))
    expected_weight = total_mass * 9.81
    print(f"articulation mass sum: {total_mass:.3f} kg")
    print(f"expected static weight: {expected_weight:.3f} N")
    print(
        "tracked contact / expected weight: "
        f"{total_load / expected_weight:.3f}"
        if expected_weight > 0.0
        else "tracked contact / expected weight: unavailable"
    )
    tensor_load_ratio = total_load / expected_weight if expected_weight > 0.0 else 0.0
    caster_share_of_weight = caster_load / expected_weight if expected_weight > 0.0 else 0.0
    base_share_of_weight = base_load / expected_weight if expected_weight > 0.0 else math.inf
    all_event_loads = {
        role: impulse / sample_duration_s
        for role, impulse in contact_event_abs_vertical_impulses.items()
    }
    event_caster_load = sum(
        load for role, load in all_event_loads.items() if role.startswith("caster_")
    )
    event_active_load = sum(
        load for role, load in all_event_loads.items() if role.startswith("active_")
    )
    event_base_load = all_event_loads.get("base", 0.0)
    event_tracked_load = sum(all_event_loads.values())
    event_load_ratio = (
        event_tracked_load / expected_weight if expected_weight > 0.0 else 0.0
    )
    event_caster_share = (
        event_caster_load / expected_weight if expected_weight > 0.0 else 0.0
    )
    print(f"event caster mean |Fz| sum: {event_caster_load:.3f} N")
    print(f"event active mean |Fz| sum: {event_active_load:.3f} N")
    print(f"event base mean |Fz|: {event_base_load:.3f} N")
    print(f"event tracked mean |Fz| sum: {event_tracked_load:.3f} N")
    print(f"event contact / expected weight: {event_load_ratio:.3f}")

    other_event_loads = {
        key: value
        for key, value in all_event_loads.items()
        if key.startswith("other_robot:") and value > 0.0
    }
    if other_event_loads:
        print("Other robot bodies carrying ground load:")
        for key, value in sorted(
            other_event_loads.items(), key=lambda item: item[1], reverse=True
        ):
            print(f"  {key.removeprefix('other_robot:')}: {value:.3f} N")
    caster_event_roles = [
        role
        for role in contact_roles
        if role.startswith("caster_") and contact_event_counts[role] > 0
    ]
    tensor_passed = (
        0.7 <= tensor_load_ratio <= 1.3
        and caster_share_of_weight >= 0.01
        and base_share_of_weight <= 0.10
    )
    event_balanced = 0.7 <= event_load_ratio <= 1.3
    event_load_verified = event_balanced and event_caster_share >= 0.01
    caster_touch_only = (
        len(caster_event_roles) == 4
        and event_balanced
        and event_caster_share < 0.005
    )
    if tensor_passed or event_load_verified:
        verdict = "LOAD VERIFIED"
    elif caster_touch_only:
        verdict = "CASTERS TOUCHING BUT NOT LOAD-BEARING"
    else:
        verdict = "INCONCLUSIVE"
    print(
        "CONTACT LOAD VALIDATION: "
        + verdict
        + f" (tensor_passed={tensor_passed}, "
        + f"event_balanced={event_balanced}, "
        + f"caster_event_roles={caster_event_roles})"
    )
    print(f"Saved: {OUTPUT_PATH}")
    print("Press Stop now; do not save the session layer.")


def _task_result(task):
    try:
        task.result()
    except asyncio.CancelledError:
        print("Contact-load sampling cancelled.")
    except Exception as error:
        print(f"Contact-load check aborted: {type(error).__name__}: {error}")


def start_contact_load_check():
    global contact_load_task
    if "contact_load_task" in globals() and not contact_load_task.done():
        raise RuntimeError("Contact-load sampling is already running")
    contact_load_task = asyncio.ensure_future(_sample_contact_loads())
    contact_load_task.add_done_callback(_task_result)
    print("Contact-load sampling scheduled; keep Timeline playing for about 2 seconds.")


print("Wheel contact-load diagnostic v8 loaded. Timeline must be stopped for preparation.")
