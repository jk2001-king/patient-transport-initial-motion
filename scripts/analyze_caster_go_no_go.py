#!/usr/bin/env python3
"""Analyse the real-platform caster Go/No-Go ROS 2 bag batch.

The script deliberately keeps wheel odometry and IMU results separate.  Wheel
odometry is useful for command/run quality control, but it is not an external
measurement of lateral tyre scrub.  Gyroscope signals are bias-corrected from
the one-second interval immediately before the first non-zero command.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
from collections import defaultdict, deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any

os.environ.setdefault("MPLCONFIGDIR", "/tmp/patient-caster-matplotlib")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


IMU_TOPICS = (
    "/livox/imu_192_168_1_135",
    "/livox/imu_192_168_2_102",
    "/livox/imu_192_168_3_144",
)
READ_TOPICS = {
    "/cmd_vel",
    "/odom",
    "/steer_angle_deg",
    "/tf_static",
    *IMU_TOPICS,
}


@dataclass
class Series:
    t: np.ndarray
    values: np.ndarray


@dataclass
class Run:
    name: str
    initial_state: str
    method: str
    repeat: int
    expected_duration_s: float
    command: Series
    odom: Series
    steering: Series
    imu: dict[str, Series]
    rotations: dict[tuple[str, str], np.ndarray]


def quaternion_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    q = np.asarray([w, x, y, z], dtype=float)
    norm = np.linalg.norm(q)
    if norm == 0:
        return np.eye(3)
    w, x, y, z = q / norm
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def parse_name(name: str) -> tuple[str, str, int, float]:
    stem, repeat_text = name.rsplit("_run", 1)
    repeat = int(repeat_text)
    if stem.endswith("_fixed"):
        initial_state, method, expected = stem[:-6], "fixed", 4.0
    elif stem.endswith("_long"):
        initial_state, method, expected = stem[:-5], "long", 10.0
    else:
        initial_state, method, expected = stem, "normal", 4.0
    return initial_state, method, repeat, expected


def rows_to_series(rows: list[tuple[float, ...]], width: int) -> Series:
    if not rows:
        return Series(np.empty(0), np.empty((0, width)))
    array = np.asarray(rows, dtype=float)
    order = np.argsort(array[:, 0])
    array = array[order]
    return Series(array[:, 0], array[:, 1:])


def read_run(path: Path) -> Run:
    initial_state, method, repeat, expected = parse_name(path.name)
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    message_types = {topic: get_message(types[topic]) for topic in READ_TOPICS if topic in types}
    command: list[tuple[float, ...]] = []
    odom: list[tuple[float, ...]] = []
    steering: list[tuple[float, ...]] = []
    imu: dict[str, list[tuple[float, ...]]] = defaultdict(list)
    rotations: dict[tuple[str, str], np.ndarray] = {}

    while reader.has_next():
        topic, raw, timestamp_ns = reader.read_next()
        if topic not in message_types:
            continue
        stamp = timestamp_ns * 1e-9
        message = deserialize_message(raw, message_types[topic])
        if topic == "/cmd_vel":
            command.append((stamp, message.linear.x, message.linear.y, message.angular.z))
        elif topic == "/odom":
            p, q, twist = message.pose.pose.position, message.pose.pose.orientation, message.twist.twist
            odom.append(
                (
                    stamp,
                    p.x,
                    p.y,
                    yaw_from_quaternion(q.x, q.y, q.z, q.w),
                    twist.linear.x,
                    twist.linear.y,
                    twist.angular.z,
                )
            )
        elif topic == "/steer_angle_deg" and len(message.data) >= 4:
            steering.append((stamp, *message.data[:4]))
        elif topic in IMU_TOPICS:
            g = message.angular_velocity
            a = message.linear_acceleration
            imu[topic].append((stamp, g.x, g.y, g.z, a.x, a.y, a.z))
        elif topic == "/tf_static":
            for transform in message.transforms:
                parent = transform.header.frame_id.lstrip("/")
                child = transform.child_frame_id.lstrip("/")
                q = transform.transform.rotation
                matrix = quaternion_matrix(q.x, q.y, q.z, q.w)
                rotations[(child, parent)] = matrix
                rotations[(parent, child)] = matrix.T

    return Run(
        name=path.name,
        initial_state=initial_state,
        method=method,
        repeat=repeat,
        expected_duration_s=expected,
        command=rows_to_series(command, 3),
        odom=rows_to_series(odom, 6),
        steering=rows_to_series(steering, 4),
        imu={topic: rows_to_series(rows, 6) for topic, rows in imu.items()},
        rotations=rotations,
    )


def longest_active_segment(command: Series) -> tuple[int, int]:
    if not len(command.t):
        raise RuntimeError("No /cmd_vel messages")
    active = np.abs(command.values[:, 0]) > 1e-5
    indices = np.flatnonzero(active)
    if not len(indices):
        raise RuntimeError("No non-zero /cmd_vel messages")
    groups: list[list[int]] = [[int(indices[0])]]
    for index in indices[1:]:
        previous = groups[-1][-1]
        if command.t[index] - command.t[previous] <= 0.35:
            groups[-1].append(int(index))
        else:
            groups.append([int(index)])
    group = max(groups, key=lambda item: command.t[item[-1]] - command.t[item[0]])
    return group[0], group[-1]


def rotation_between(
    rotations: dict[tuple[str, str], np.ndarray], source: str, target: str
) -> np.ndarray | None:
    source, target = source.lstrip("/"), target.lstrip("/")
    if source == target:
        return np.eye(3)
    graph: dict[str, list[tuple[str, np.ndarray]]] = defaultdict(list)
    for (a, b), matrix in rotations.items():
        graph[a].append((b, matrix))
    queue: deque[tuple[str, np.ndarray]] = deque([(source, np.eye(3))])
    seen = {source}
    while queue:
        node, accumulated = queue.popleft()
        for neighbor, matrix in graph[node]:
            if neighbor in seen:
                continue
            result = matrix @ accumulated
            if neighbor == target:
                return result
            seen.add(neighbor)
            queue.append((neighbor, result))
    return None


def interpolate_columns(series: Series, query_t: np.ndarray) -> np.ndarray:
    if not len(series.t):
        return np.full((len(query_t), series.values.shape[1]), np.nan)
    return np.column_stack(
        [np.interp(query_t, series.t, series.values[:, column]) for column in range(series.values.shape[1])]
    )


def integrate(t: np.ndarray, value: np.ndarray) -> np.ndarray:
    if len(t) == 0:
        return np.empty(0)
    result = np.zeros(len(t))
    if len(t) > 1:
        result[1:] = np.cumsum(np.diff(t) * (value[1:] + value[:-1]) * 0.5)
    return result


def time_to_fraction(t: np.ndarray, value: np.ndarray, target: float, fraction: float) -> float:
    threshold = fraction * target
    indices = np.flatnonzero(value >= threshold)
    return float(t[indices[0]]) if len(indices) else math.nan


def analyse_run(run: Run, samples_dir: Path) -> tuple[dict[str, Any], dict[str, Series]]:
    start_index, end_index = longest_active_segment(run.command)
    command_start = float(run.command.t[start_index])
    command_end = float(run.command.t[end_index])
    active_mask = (run.command.t >= command_start) & (run.command.t <= command_end)
    cmd_t = run.command.t[active_mask] - command_start
    cmd_v = run.command.values[active_mask, 0]
    peak_cmd = float(np.max(cmd_v))
    active_duration = float(cmd_t[-1]) if len(cmd_t) else math.nan
    command_area = float(np.trapz(cmd_v, cmd_t)) if len(cmd_t) > 1 else 0.0
    t90_command = time_to_fraction(cmd_t, cmd_v, peak_cmd, 0.9)
    command_rate = (len(cmd_t) - 1) / active_duration if active_duration > 0 else math.nan

    # Compare all signals over the command plus two seconds of settling.
    grid_end = max(run.expected_duration_s + 2.0, active_duration + 2.0)
    grid_t = np.arange(0.0, grid_end + 1e-9, 0.01)
    absolute_grid = command_start + grid_t
    odom = interpolate_columns(run.odom, absolute_grid)
    steering = interpolate_columns(run.steering, absolute_grid)

    x0, y0, yaw0 = odom[0, :3]
    dx, dy = odom[:, 0] - x0, odom[:, 1] - y0
    forward = np.cos(yaw0) * dx + np.sin(yaw0) * dy
    lateral = -np.sin(yaw0) * dx + np.cos(yaw0) * dy
    yaw = np.unwrap(odom[:, 2]) - yaw0

    imu_series: dict[str, Series] = {}
    imu_yaw_rates: list[np.ndarray] = []
    imu_pre_yaw_rates: list[np.ndarray] = []
    imu_accelerations: list[np.ndarray] = []
    pre_grid = np.arange(command_start - 2.0, command_start + 1e-9, 0.01)
    for topic, series in sorted(run.imu.items()):
        # Frame names are fixed by this platform's three Livox publishers.
        frame = {
            IMU_TOPICS[0]: "livox_top",
            IMU_TOPICS[1]: "livox_front",
            IMU_TOPICS[2]: "livox_rear",
        }[topic]
        rotation = rotation_between(run.rotations, frame, "base_link")
        if rotation is None:
            # front/rear z axes are parallel to base z in the recorded setup;
            # leave an explicit fallback for old bags lacking transient TF.
            rotation = np.eye(3)
        transformed = (rotation @ series.values[:, :3].T).T
        transformed_acceleration = (rotation @ series.values[:, 3:6].T).T
        baseline_mask = (series.t >= command_start - 1.0) & (series.t < command_start - 0.05)
        if np.count_nonzero(baseline_mask) < 5:
            baseline_mask = series.t < command_start
        bias = np.median(transformed[baseline_mask], axis=0) if np.any(baseline_mask) else transformed[0]
        acceleration_bias = (
            np.median(transformed_acceleration[baseline_mask], axis=0)
            if np.any(baseline_mask) else transformed_acceleration[0]
        )
        # The recorded Livox streams use approximately 1.0 at rest despite the
        # ROS SI-unit convention. Convert g-like streams to m/s^2 explicitly.
        acceleration_scale = 9.80665 if 0.5 <= np.linalg.norm(acceleration_bias) <= 1.5 else 1.0
        yaw_rate = np.interp(absolute_grid, series.t, transformed[:, 2] - bias[2])
        pre_yaw_rate = np.interp(pre_grid, series.t, transformed[:, 2] - bias[2])
        acceleration = np.column_stack(
            [
                np.interp(
                    absolute_grid,
                    series.t,
                    acceleration_scale * (transformed_acceleration[:, axis] - acceleration_bias[axis]),
                )
                for axis in range(3)
            ]
        )
        imu_yaw_rates.append(yaw_rate)
        imu_pre_yaw_rates.append(pre_yaw_rate)
        imu_accelerations.append(acceleration)
        imu_series[topic] = Series(grid_t, yaw_rate[:, None])
    combined_yaw_rate = np.median(np.vstack(imu_yaw_rates), axis=0) if imu_yaw_rates else np.full(len(grid_t), np.nan)
    combined_pre_yaw_rate = (
        np.median(np.vstack(imu_pre_yaw_rates), axis=0)
        if imu_pre_yaw_rates else np.full(len(pre_grid), np.nan)
    )
    combined_acceleration = (
        np.median(np.stack(imu_accelerations), axis=0)
        if imu_accelerations else np.full((len(grid_t), 3), np.nan)
    )
    imu_yaw = integrate(grid_t, combined_yaw_rate)
    imu_pre_yaw = integrate(pre_grid - pre_grid[0], combined_pre_yaw_rate)

    initial_mask = grid_t <= 2.0
    motion_mask = grid_t <= run.expected_duration_s
    final_index = int(np.argmin(np.abs(grid_t - run.expected_duration_s)))
    expected_samples = run.expected_duration_s * 20.0
    duration_error = abs(active_duration - run.expected_duration_s)
    profile_ok = (
        0.070 <= peak_cmd <= 0.080
        and duration_error <= 0.35
        and len(cmd_t) >= 0.55 * expected_samples
    )
    if run.method == "fixed":
        profile_ok = profile_ok and 0.65 <= t90_command <= 1.15
    elif run.method in ("normal", "long"):
        profile_ok = profile_ok and t90_command <= 0.15

    odom_speed = odom[:, 3]
    motion_indices = np.flatnonzero((grid_t <= run.expected_duration_s + 1.0) & (odom_speed >= 0.01))
    reach90_indices = np.flatnonzero((grid_t <= run.expected_duration_s + 1.0) & (odom_speed >= 0.9 * peak_cmd))
    odom_motion_onset = float(grid_t[motion_indices[0]]) if len(motion_indices) else math.nan
    odom_reach90 = float(grid_t[reach90_indices[0]]) if len(reach90_indices) else math.nan
    horizontal_acceleration = np.linalg.norm(combined_acceleration[:, :2], axis=1)

    row: dict[str, Any] = {
        "run": run.name,
        "initial_state": run.initial_state,
        "method": run.method,
        "repeat": run.repeat,
        "qc_pass": profile_ok,
        "command_samples": len(cmd_t),
        "command_rate_hz": command_rate,
        "command_peak_m_s": peak_cmd,
        "command_active_duration_s": active_duration,
        "command_t90_s": t90_command,
        "command_integral_m": command_area,
        "odom_forward_2s_m": float(forward[np.argmin(np.abs(grid_t - 2.0))]),
        "odom_motion_onset_s": odom_motion_onset,
        "odom_reach90_speed_s": odom_reach90,
        "odom_lateral_at_2s_m": float(lateral[np.argmin(np.abs(grid_t - 2.0))]),
        "odom_max_abs_lateral_2s_m": float(np.nanmax(np.abs(lateral[initial_mask]))),
        "odom_yaw_at_2s_deg": math.degrees(float(yaw[np.argmin(np.abs(grid_t - 2.0))])),
        "odom_max_abs_yaw_2s_deg": math.degrees(float(np.nanmax(np.abs(yaw[initial_mask])))),
        "odom_forward_end_m": float(forward[final_index]),
        "odom_lateral_end_m": float(lateral[final_index]),
        "odom_yaw_end_deg": math.degrees(float(yaw[final_index])),
        "steer_front_max_abs_deg": float(np.nanmax(np.abs(steering[motion_mask, 0]))),
        "steer_rear_max_abs_deg": float(np.nanmax(np.abs(steering[motion_mask, 1]))),
        "imu_count": len(imu_yaw_rates),
        "imu_peak_abs_yaw_rate_2s_deg_s": math.degrees(float(np.nanmax(np.abs(combined_yaw_rate[initial_mask])))),
        "imu_rms_yaw_rate_2s_deg_s": math.degrees(float(np.sqrt(np.nanmean(combined_yaw_rate[initial_mask] ** 2)))),
        "imu_integrated_yaw_2s_deg": math.degrees(float(imu_yaw[np.argmin(np.abs(grid_t - 2.0))])),
        "imu_max_abs_integrated_yaw_2s_deg": math.degrees(float(np.nanmax(np.abs(imu_yaw[initial_mask])))),
        "imu_pre_rms_yaw_rate_2s_deg_s": math.degrees(float(np.sqrt(np.nanmean(combined_pre_yaw_rate ** 2)))),
        "imu_pre_max_abs_integrated_yaw_2s_deg": math.degrees(float(np.nanmax(np.abs(imu_pre_yaw)))),
        "imu_peak_abs_lateral_accel_2s_m_s2": float(np.nanquantile(np.abs(combined_acceleration[initial_mask, 1]), 0.99)),
        "imu_rms_lateral_accel_2s_m_s2": float(np.sqrt(np.nanmean(combined_acceleration[initial_mask, 1] ** 2))),
        "imu_peak_horizontal_accel_2s_m_s2": float(np.nanquantile(horizontal_acceleration[initial_mask], 0.99)),
    }

    samples = {
        "command": Series(cmd_t, cmd_v[:, None]),
        "odom_forward": Series(grid_t, forward[:, None]),
        "odom_lateral": Series(grid_t, lateral[:, None]),
        "odom_yaw": Series(grid_t, np.degrees(yaw)[:, None]),
        "steering": Series(grid_t, steering[:, :2]),
        "imu_yaw_rate": Series(grid_t, np.degrees(combined_yaw_rate)[:, None]),
        "imu_yaw": Series(grid_t, np.degrees(imu_yaw)[:, None]),
        "imu_acceleration": Series(grid_t, combined_acceleration),
    }
    sample_path = samples_dir / f"{run.name}_timeseries.csv"
    with sample_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "time_s", "command_m_s", "odom_forward_m", "odom_lateral_m",
                "odom_yaw_deg", "steer_front_deg", "steer_rear_deg",
                "imu_yaw_rate_deg_s", "imu_integrated_yaw_deg",
                "imu_ax_m_s2", "imu_ay_m_s2", "imu_az_m_s2",
            ]
        )
        cmd_interp = np.interp(grid_t, cmd_t, cmd_v, left=0.0, right=0.0)
        for values in zip(
            grid_t, cmd_interp, forward, lateral, np.degrees(yaw), steering[:, 0],
            steering[:, 1], np.degrees(combined_yaw_rate), np.degrees(imu_yaw),
            combined_acceleration[:, 0], combined_acceleration[:, 1], combined_acceleration[:, 2],
        ):
            writer.writerow([f"{value:.9f}" for value in values])
    return row, samples


def write_dict_rows(path: Path, rows: list[dict[str, Any]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def group_summary(rows: list[dict[str, Any]], path: Path) -> None:
    metrics = [
        "odom_max_abs_lateral_2s_m",
        "odom_max_abs_yaw_2s_deg",
        "imu_peak_abs_yaw_rate_2s_deg_s",
        "imu_rms_yaw_rate_2s_deg_s",
        "imu_max_abs_integrated_yaw_2s_deg",
        "imu_pre_rms_yaw_rate_2s_deg_s",
        "imu_pre_max_abs_integrated_yaw_2s_deg",
        "imu_peak_abs_lateral_accel_2s_m_s2",
        "imu_rms_lateral_accel_2s_m_s2",
        "imu_peak_horizontal_accel_2s_m_s2",
        "odom_forward_end_m",
    ]
    output: list[dict[str, Any]] = []
    groups: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        groups[(row["initial_state"], row["method"])].append(row)
    for (state, method), selected in sorted(groups.items()):
        for metric in metrics:
            values = np.asarray([float(row[metric]) for row in selected if row["qc_pass"]], dtype=float)
            output.append(
                {
                    "initial_state": state,
                    "method": method,
                    "metric": metric,
                    "mean": float(np.mean(values)) if len(values) else math.nan,
                    "sample_std": float(np.std(values, ddof=1)) if len(values) > 1 else math.nan,
                    "n_qc_pass": len(values),
                    "n_total": len(selected),
                }
            )
    write_dict_rows(path, output)


def make_plots(rows: list[dict[str, Any]], samples: dict[str, dict[str, Series]], output: Path) -> None:
    colors = {"A": "tab:green", "BL": "tab:blue", "BR": "tab:orange"}
    styles = {"normal": "-", "fixed": "--", "long": ":"}
    fig, axes = plt.subplots(4, 1, figsize=(11, 12), sharex=True)
    for row in rows:
        run = row["run"]
        data = samples[run]
        label = run + ("" if row["qc_pass"] else " [QC FAIL]")
        color = colors[row["initial_state"]]
        style = styles[row["method"]]
        alpha = 0.78 if row["qc_pass"] else 0.25
        axes[0].plot(data["command"].t, data["command"].values[:, 0], style, color=color, alpha=alpha, label=label)
        axes[1].plot(data["odom_lateral"].t, 1000 * data["odom_lateral"].values[:, 0], style, color=color, alpha=alpha)
        axes[2].plot(data["odom_yaw"].t, data["odom_yaw"].values[:, 0], style, color=color, alpha=alpha)
        axes[3].plot(data["imu_yaw"].t, data["imu_yaw"].values[:, 0], style, color=color, alpha=alpha)
    axes[0].set_ylabel("cmd v [m/s]")
    axes[1].set_ylabel("wheel-odom lateral [mm]")
    axes[2].set_ylabel("wheel-odom yaw [deg]")
    axes[3].set_ylabel("gyro integrated yaw [deg]")
    axes[3].set_xlabel("time from first non-zero command [s]")
    for axis in axes:
        axis.axvline(2.0, color="black", linewidth=0.8, alpha=0.4)
        axis.grid(True, alpha=0.25)
        axis.set_xlim(0, 6)
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="center right", fontsize=7, ncol=1)
    fig.tight_layout(rect=(0, 0, 0.79, 1))
    fig.savefig(output / "all_runs_initial_6s.png", dpi=180)
    plt.close(fig)

    valid = [row for row in rows if row["qc_pass"] and row["method"] != "long"]
    metrics = [
        ("odom_max_abs_lateral_2s_m", 1000.0, "Max |lateral| in first 2 s [mm]"),
        ("odom_max_abs_yaw_2s_deg", 1.0, "Max |wheel-odom yaw| in first 2 s [deg]"),
        ("imu_max_abs_integrated_yaw_2s_deg", 1.0, "Max |gyro integrated yaw| in first 2 s [deg]"),
    ]
    fig, axes = plt.subplots(1, 3, figsize=(13, 4.2))
    xlabels = [(state, method) for state in ("A", "BL", "BR") for method in ("normal", "fixed")]
    for axis, (metric, scale, ylabel) in zip(axes, metrics):
        for x, (state, method) in enumerate(xlabels):
            values = [float(row[metric]) * scale for row in valid if row["initial_state"] == state and row["method"] == method]
            if not values:
                continue
            axis.scatter(np.full(len(values), x), values, color=colors[state], marker="o" if method == "normal" else "s", zorder=3)
            axis.errorbar(x, np.mean(values), yerr=np.std(values, ddof=1) if len(values) > 1 else 0, color="black", capsize=3)
        axis.set_xticks(range(len(xlabels)), [f"{state}\n{method}" for state, method in xlabels])
        axis.set_ylabel(ylabel)
        axis.grid(True, axis="y", alpha=0.25)
    fig.tight_layout()
    fig.savefig(output / "go_no_go_group_metrics.png", dpi=180)
    plt.close(fig)

    short = [row for row in rows if row["method"] != "long"]
    imu_metrics = [
        ("imu_max_abs_integrated_yaw_2s_deg", "Max |integrated yaw|, first 2 s [deg]"),
        ("imu_peak_abs_lateral_accel_2s_m_s2", "Peak |lateral accel|, first 2 s [m/s²]"),
        ("imu_peak_horizontal_accel_2s_m_s2", "Peak horizontal accel, first 2 s [m/s²]"),
    ]
    fig, axes = plt.subplots(1, 3, figsize=(13.5, 4.4))
    for axis, (metric, ylabel) in zip(axes, imu_metrics):
        for x, (state, method) in enumerate(xlabels):
            selected = [row for row in short if row["initial_state"] == state and row["method"] == method]
            values = np.asarray([float(row[metric]) for row in selected])
            for row, value in zip(selected, values):
                stable_pre = float(row["imu_pre_max_abs_integrated_yaw_2s_deg"]) <= 0.06
                axis.scatter(
                    x, value, color=colors[state], marker="o" if stable_pre else "x",
                    s=45, zorder=3,
                )
            axis.errorbar(
                x, np.mean(values), yerr=np.std(values, ddof=1),
                color="black", marker="_", markersize=12, capsize=3, zorder=2,
            )
        axis.set_xticks(range(len(xlabels)), [f"{state}\n{method}" for state, method in xlabels])
        axis.set_ylabel(ylabel)
        axis.grid(True, axis="y", alpha=0.25)
    fig.suptitle("Real-platform initial response (circle: stable pre-run gyro; x: flagged pre-run gyro)")
    fig.tight_layout()
    fig.savefig(output / "imu_initial_group_metrics.png", dpi=180)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bags", type=Path, default=Path("motor_bag/caster_go_no_go"))
    parser.add_argument("--output", type=Path, default=Path("results/real_caster_go_no_go/bag_analysis"))
    args = parser.parse_args()
    bag_paths = sorted(path for path in args.bags.iterdir() if path.is_dir() and (path / "metadata.yaml").exists())
    if not bag_paths:
        raise SystemExit(f"No ROS 2 bags found below {args.bags}")
    args.output.mkdir(parents=True, exist_ok=True)
    samples_dir = args.output / "timeseries"
    samples_dir.mkdir(exist_ok=True)

    rows: list[dict[str, Any]] = []
    samples: dict[str, dict[str, Series]] = {}
    for path in bag_paths:
        run = read_run(path)
        row, run_samples = analyse_run(run, samples_dir)
        rows.append(row)
        samples[run.name] = run_samples
        print(
            f"{run.name:20s} qc={str(row['qc_pass']):5s} "
            f"cmd={row['command_active_duration_s']:.3f}s "
            f"lat2={1000*row['odom_max_abs_lateral_2s_m']:.2f}mm "
            f"imu_yaw2={row['imu_max_abs_integrated_yaw_2s_deg']:.3f}deg"
        )
    write_dict_rows(args.output / "metrics_per_run.csv", rows)
    group_summary(rows, args.output / "metrics_group_summary.csv")
    make_plots(rows, samples, args.output)
    print(f"Analysed {len(rows)} runs -> {args.output}")


if __name__ == "__main__":
    main()
