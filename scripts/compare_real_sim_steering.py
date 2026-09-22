#!/usr/bin/env python3
"""Compare one Isaac dual-steering CSV with the three real ground runs."""

from __future__ import annotations

import argparse
import csv
import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/patient-steering-matplotlib")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


BAND_DEG = 1.5
HOLD_S = 0.5
REAL_METRICS = {
    "front": {
        "t90_s": 3.474009590,
        "band_entry_s": 3.747528980,
        "settling_time_s": 3.747528980,
        "max_velocity_deg_s": 13.851541368,
        "final_error_deg": 0.000183105,
        "overshoot_deg": 0.001098633,
    },
    "rear": {
        "t90_s": 3.475330896,
        "band_entry_s": 3.748745746,
        "settling_time_s": 3.748745746,
        "max_velocity_deg_s": 13.867436239,
        "final_error_deg": 0.0,
        "overshoot_deg": 0.001098633,
    },
}
TOLERANCES = {
    "curve_rmse_deg": 0.5,
    "t90_abs_error_s": 0.10,
    "settling_abs_error_s": 0.10,
    "max_velocity_relative_error": 0.05,
    "final_error_deg": 0.1,
    "overshoot_deg": 0.2,
}


def _first_crossing(t: np.ndarray, value: np.ndarray, threshold: float) -> float:
    indices = np.flatnonzero(value >= threshold)
    if not len(indices):
        return float("nan")
    index = int(indices[0])
    if index == 0 or value[index] == value[index - 1]:
        return float(t[index])
    fraction = (threshold - value[index - 1]) / (value[index] - value[index - 1])
    return float(t[index - 1] + fraction * (t[index] - t[index - 1]))


def _settling_time(t: np.ndarray, error: np.ndarray) -> float:
    inside = error <= BAND_DEG
    for index in np.flatnonzero(inside):
        end = int(np.searchsorted(t, t[index] + HOLD_S, side="left"))
        if end < len(t) and np.all(inside[index : end + 1]):
            if index > 0 and error[index - 1] > BAND_DEG and error[index] != error[index - 1]:
                fraction = (BAND_DEG - error[index - 1]) / (error[index] - error[index - 1])
                return float(t[index - 1] + fraction * (t[index] - t[index - 1]))
            return float(t[index])
    return float("nan")


def _peak_velocity(t: np.ndarray, actual: np.ndarray) -> float:
    dt = np.diff(t)
    delta = np.diff(actual)
    valid = np.isfinite(dt) & np.isfinite(delta) & (dt > 0.0)
    if np.count_nonzero(valid) < 3:
        return float("nan")
    # The simulation position is already a clean joint-state signal. A
    # Savitzky-Golay derivative overshoots at the acceleration/cruise corners
    # of the trapezoidal profile (16.8 deg/s from a true 13.73 deg/s slope).
    # Segment slopes avoid that synthetic peak; the 99th percentile rejects a
    # single irregular sampling interval.
    segment_velocity = np.abs(delta[valid] / dt[valid])
    return float(np.quantile(segment_velocity, 0.99))


def _metrics(t: np.ndarray, signed_actual: np.ndarray, signed_target: float) -> dict[str, float]:
    direction = 1.0 if signed_target >= 0.0 else -1.0
    normalized = direction * signed_actual
    target_magnitude = abs(signed_target)
    error = np.abs(signed_actual - signed_target)
    final_mask = t >= max(0.0, t[-1] - 1.0)
    final_actual = float(np.median(signed_actual[final_mask]))
    return {
        "t90_s": _first_crossing(t, normalized, 0.9 * target_magnitude),
        "band_entry_s": _first_crossing(t, normalized, target_magnitude - BAND_DEG),
        "settling_time_s": _settling_time(t, error),
        "max_velocity_deg_s": _peak_velocity(t, signed_actual),
        "final_error_deg": abs(final_actual - signed_target),
        "overshoot_deg": max(0.0, float(np.max(direction * (signed_actual - signed_target)))),
    }


def _load_real_runs(real_dir: Path):
    paths = sorted(real_dir.glob("steering_ground_run*_steering.csv"))
    if len(paths) != 3:
        raise RuntimeError(f"Expected 3 real CSV files in {real_dir}, found {len(paths)}")
    return [pd.read_csv(path) for path in paths]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim", required=True, type=Path, help="Isaac CSV produced by the runner")
    parser.add_argument(
        "--real-dir",
        type=Path,
        default=Path("results/real_ground_analysis"),
    )
    parser.add_argument("--output-dir", type=Path, default=Path("results/isaac_comparison"))
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    sim = pd.read_csv(args.sim)
    required = {
        "time_s",
        "front_actual_deg",
        "rear_actual_deg",
        "front_target_deg",
        "rear_target_deg",
    }
    missing = required.difference(sim.columns)
    if missing:
        raise RuntimeError(f"{args.sim}: missing columns {sorted(missing)}")
    real_runs = _load_real_runs(args.real_dir)

    sim_t = sim["time_s"].to_numpy(float)
    sim_t -= sim_t[0]
    common_end = min(6.0, sim_t[-1], *(run["time_s"].iloc[-1] for run in real_runs))
    common_t = np.arange(0.0, common_end + 1e-9, 0.02)
    rows = []
    plot_data = {}

    for axis, direction, target in (("front", -1.0, -45.0), ("rear", 1.0, 45.0)):
        real_stack = np.vstack(
            [
                np.interp(common_t, run["time_s"], direction * run[f"{axis}_actual_deg"])
                for run in real_runs
            ]
        )
        real_mean = np.mean(real_stack, axis=0)
        real_std = np.std(real_stack, axis=0, ddof=1)
        sim_normalized = np.interp(
            common_t, sim_t, direction * sim[f"{axis}_actual_deg"].to_numpy(float)
        )
        curve_rmse = float(np.sqrt(np.mean((sim_normalized - real_mean) ** 2)))
        sim_metrics = _metrics(
            sim_t,
            sim[f"{axis}_actual_deg"].to_numpy(float),
            target,
        )
        reference = REAL_METRICS[axis]
        comparisons = {
            "curve_rmse_deg": curve_rmse,
            "t90_abs_error_s": abs(sim_metrics["t90_s"] - reference["t90_s"]),
            "settling_abs_error_s": abs(
                sim_metrics["settling_time_s"] - reference["settling_time_s"]
            ),
            "max_velocity_relative_error": abs(
                sim_metrics["max_velocity_deg_s"] - reference["max_velocity_deg_s"]
            )
            / reference["max_velocity_deg_s"],
            "final_error_deg": sim_metrics["final_error_deg"],
            "overshoot_deg": sim_metrics["overshoot_deg"],
        }
        passed = all(comparisons[name] <= limit for name, limit in TOLERANCES.items())
        row = {"axis": axis, **sim_metrics, **comparisons, "passed": passed}
        rows.append(row)
        plot_data[axis] = (real_mean, real_std, sim_normalized)

    output_stem = args.sim.stem
    metrics_path = args.output_dir / f"{output_stem}_comparison.csv"
    with metrics_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    fig, ax = plt.subplots(figsize=(10, 5.8))
    styles = {"front": ("tab:blue", "-"), "rear": ("tab:orange", "--")}
    for axis in ("front", "rear"):
        color, linestyle = styles[axis]
        real_mean, real_std, sim_curve = plot_data[axis]
        ax.plot(common_t, real_mean, color=color, linewidth=2.0, label=f"Real {axis} mean")
        ax.fill_between(common_t, real_mean - real_std, real_mean + real_std, color=color, alpha=0.15)
        ax.plot(
            common_t,
            sim_curve,
            color=color,
            linestyle=linestyle,
            linewidth=1.5,
            alpha=0.85,
            label=f"Isaac {axis}",
        )
    ax.axhline(45.0, color="black", linestyle=":", linewidth=1.0, label="Target 45 deg")
    ax.axhspan(43.5, 46.5, color="tab:green", alpha=0.10, label="+/-1.5 deg band")
    ax.set(
        xlabel="Time after steering command [s]",
        ylabel="Direction-normalized steering angle [deg]",
        xlim=(0.0, common_t[-1]),
        ylim=(-1.0, 47.0),
    )
    ax.grid(True, alpha=0.3)
    ax.legend(loc="lower right", ncol=2)
    fig.tight_layout()
    plot_path = args.output_dir / f"{output_stem}_comparison.png"
    fig.savefig(plot_path, dpi=180)
    plt.close(fig)

    print(f"Comparison metrics: {metrics_path}")
    print(f"Comparison plot: {plot_path}")
    for row in rows:
        print(
            f"{row['axis']}: passed={row['passed']}, "
            f"RMSE={row['curve_rmse_deg']:.3f} deg, "
            f"T90={row['t90_s']:.3f} s, "
            f"settling={row['settling_time_s']:.3f} s, "
            f"vmax={row['max_velocity_deg_s']:.3f} deg/s"
        )


if __name__ == "__main__":
    main()
