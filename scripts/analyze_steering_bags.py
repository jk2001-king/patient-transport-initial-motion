#!/usr/bin/env python3
"""Extract and analyse 0 -> +/-45 degree steering steps from ROS 2 bags."""

from __future__ import annotations

import argparse
import csv
import math
import os
from dataclasses import dataclass
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/patient-steering-matplotlib")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.signal import savgol_filter


TOPIC = "/steer_angle_deg"
BAND_DEG = 1.5
SETTLE_HOLD_S = 0.5
FINAL_WINDOW_S = 1.0


@dataclass
class StepData:
    run: str
    time_s: np.ndarray
    front_actual_deg: np.ndarray
    rear_actual_deg: np.ndarray
    front_target_deg: np.ndarray
    rear_target_deg: np.ndarray


def read_step(bag: Path) -> StepData:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    if TOPIC not in topic_types:
        raise RuntimeError(f"{bag}: missing {TOPIC}")
    msg_type = get_message(topic_types[TOPIC])
    rows: list[tuple[float, float, float, float, float]] = []
    while reader.has_next():
        topic, raw, timestamp_ns = reader.read_next()
        if topic != TOPIC:
            continue
        data = deserialize_message(raw, msg_type).data
        if len(data) < 4:
            continue
        rows.append((timestamp_ns / 1e9, data[0], data[1], data[2], data[3]))
    if not rows:
        raise RuntimeError(f"{bag}: no valid {TOPIC} messages")

    values = np.asarray(rows, dtype=float)
    target_active = (np.abs(values[:, 3]) >= 40.0) | (np.abs(values[:, 4]) >= 40.0)
    active_indices = np.flatnonzero(target_active)
    if not len(active_indices):
        raise RuntimeError(f"{bag}: no +/-45 degree target step found")
    start = int(active_indices[0])
    stop_candidates = np.flatnonzero((np.arange(len(values)) > start) & ~target_active)
    stop = int(stop_candidates[0]) if len(stop_candidates) else len(values)
    step = values[start:stop]
    step[:, 0] -= step[0, 0]
    return StepData(
        run=bag.name,
        time_s=step[:, 0],
        front_actual_deg=step[:, 1],
        rear_actual_deg=step[:, 2],
        front_target_deg=step[:, 3],
        rear_target_deg=step[:, 4],
    )


def crossing_time(t: np.ndarray, value: np.ndarray, threshold: float) -> float:
    indices = np.flatnonzero(value >= threshold)
    if not len(indices):
        return math.nan
    i = int(indices[0])
    if i == 0 or value[i] == value[i - 1]:
        return float(t[i])
    ratio = (threshold - value[i - 1]) / (value[i] - value[i - 1])
    return float(t[i - 1] + ratio * (t[i] - t[i - 1]))


def settling_time(t: np.ndarray, error: np.ndarray) -> float:
    inside = error <= BAND_DEG
    for i in np.flatnonzero(inside):
        end = int(np.searchsorted(t, t[i] + SETTLE_HOLD_S, side="left"))
        if end < len(t) and np.all(inside[i : end + 1]):
            if i > 0 and error[i - 1] > BAND_DEG and error[i] != error[i - 1]:
                ratio = (BAND_DEG - error[i - 1]) / (error[i] - error[i - 1])
                return float(t[i - 1] + ratio * (t[i] - t[i - 1]))
            return float(t[i])
    return math.nan


def robust_peak_velocity(t: np.ndarray, actual: np.ndarray) -> float:
    if len(actual) < 7:
        return math.nan
    window = min(21, len(actual) if len(actual) % 2 else len(actual) - 1)
    window = max(window, 7)
    smooth = savgol_filter(actual, window_length=window, polyorder=2)
    velocity = np.gradient(smooth, t)
    return float(np.quantile(np.abs(velocity), 0.99))


def axis_metrics(run: StepData, axis: str) -> dict[str, float | str]:
    actual = getattr(run, f"{axis}_actual_deg")
    target_values = getattr(run, f"{axis}_target_deg")
    target = float(np.median(target_values))
    direction = 1.0 if target >= 0 else -1.0
    normalized = direction * actual
    target_magnitude = abs(target)
    error = np.abs(actual - target)
    entry = settling_time(run.time_s, error)
    final_mask = run.time_s >= max(0.0, run.time_s[-1] - FINAL_WINDOW_S)
    final_actual = float(np.median(actual[final_mask]))
    overshoot = max(0.0, float(np.max(direction * (actual - target))))
    return {
        "run": run.run,
        "axis": axis,
        "sample_period_s": float(np.median(np.diff(run.time_s))),
        "target_deg": target,
        "t90_s": crossing_time(run.time_s, normalized, 0.9 * target_magnitude),
        "band_entry_s": crossing_time(run.time_s, normalized, target_magnitude - BAND_DEG),
        "settling_time_s": entry,
        "max_velocity_deg_s": robust_peak_velocity(run.time_s, actual),
        "final_actual_deg": final_actual,
        "final_error_deg": abs(final_actual - target),
        "overshoot_deg": overshoot,
    }


def write_samples(out_dir: Path, run: StepData) -> None:
    path = out_dir / f"{run.run}_steering.csv"
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "time_s",
                "front_actual_deg",
                "rear_actual_deg",
                "front_target_deg",
                "rear_target_deg",
                "front_normalized_deg",
                "rear_normalized_deg",
            ]
        )
        for values in zip(
            run.time_s,
            run.front_actual_deg,
            run.rear_actual_deg,
            run.front_target_deg,
            run.rear_target_deg,
            -run.front_actual_deg,
            run.rear_actual_deg,
        ):
            writer.writerow([f"{value:.9f}" for value in values])


def write_metrics(out_dir: Path, metrics: list[dict[str, float | str]]) -> None:
    fields = list(metrics[0].keys())
    with (out_dir / "metrics_per_run.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(metrics)

    numeric = [
        "t90_s",
        "band_entry_s",
        "settling_time_s",
        "max_velocity_deg_s",
        "final_actual_deg",
        "final_error_deg",
        "overshoot_deg",
    ]
    with (out_dir / "metrics_summary.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["axis", "metric", "mean", "sample_std", "n"])
        for axis in ("front", "rear"):
            selected = [row for row in metrics if row["axis"] == axis]
            for name in numeric:
                values = np.asarray([float(row[name]) for row in selected], dtype=float)
                writer.writerow(
                    [axis, name, f"{np.mean(values):.9f}", f"{np.std(values, ddof=1):.9f}", len(values)]
                )


def make_plots(out_dir: Path, runs: list[StepData]) -> None:
    fig, axes = plt.subplots(len(runs), 1, figsize=(10, 10), sharex=True, sharey=True)
    for ax, run in zip(axes, runs):
        ax.plot(run.time_s, -run.front_actual_deg, label="Front (sign-normalized)", linewidth=1.8)
        ax.plot(run.time_s, run.rear_actual_deg, label="Rear", linewidth=1.8)
        ax.axhline(45.0, color="black", linestyle="--", linewidth=1.0, label="Target 45 deg")
        ax.axhspan(43.5, 46.5, color="tab:green", alpha=0.12, label="+/-1.5 deg band")
        ax.set_title(run.run)
        ax.set_ylabel("Steering angle [deg]")
        ax.grid(True, alpha=0.3)
    axes[0].legend(loc="lower right", ncol=2)
    axes[-1].set_xlabel("Time after target step [s]")
    axes[-1].set_xlim(0, min(6.0, max(run.time_s[-1] for run in runs)))
    axes[-1].set_ylim(-1, 47)
    fig.tight_layout()
    fig.savefig(out_dir / "steering_0_to_45_runs.png", dpi=180)
    fig.savefig(out_dir / "steering_0_to_45_runs.pdf")
    plt.close(fig)

    common_t = np.arange(0.0, min(6.0, min(run.time_s[-1] for run in runs)) + 1e-9, 0.02)
    front = np.vstack([np.interp(common_t, run.time_s, -run.front_actual_deg) for run in runs])
    rear = np.vstack([np.interp(common_t, run.time_s, run.rear_actual_deg) for run in runs])
    fig, ax = plt.subplots(figsize=(10, 5.5))
    for values, color, label in ((front, "tab:blue", "Front"), (rear, "tab:orange", "Rear")):
        mean = np.mean(values, axis=0)
        std = np.std(values, axis=0, ddof=1)
        ax.plot(common_t, mean, color=color, linewidth=2.0, label=f"{label} mean")
        ax.fill_between(common_t, mean - std, mean + std, color=color, alpha=0.2, label=f"{label} +/-1 SD")
    ax.axhline(45.0, color="black", linestyle="--", linewidth=1.0, label="Target 45 deg")
    ax.axhspan(43.5, 46.5, color="tab:green", alpha=0.12, label="+/-1.5 deg band")
    ax.set(xlabel="Time after target step [s]", ylabel="Steering angle [deg]", xlim=(0, common_t[-1]), ylim=(-1, 47))
    ax.grid(True, alpha=0.3)
    ax.legend(loc="lower right", ncol=2)
    fig.tight_layout()
    fig.savefig(out_dir / "steering_0_to_45_mean_std.png", dpi=180)
    fig.savefig(out_dir / "steering_0_to_45_mean_std.pdf")
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bags", type=Path, default=Path("motor_bag/real_ground"))
    parser.add_argument("--output", type=Path, default=Path("results/real_ground_analysis"))
    args = parser.parse_args()
    bags = sorted(path for path in args.bags.glob("steering_ground_run*") if path.is_dir())
    if not bags:
        raise SystemExit(f"No steering bags found below {args.bags}")
    args.output.mkdir(parents=True, exist_ok=True)
    runs = [read_step(path) for path in bags]
    for run in runs:
        write_samples(args.output, run)
    metrics = [axis_metrics(run, axis) for run in runs for axis in ("front", "rear")]
    write_metrics(args.output, metrics)
    make_plots(args.output, runs)
    print(f"Analysed {len(runs)} bags -> {args.output}")


if __name__ == "__main__":
    main()
