#!/usr/bin/env python3
"""Summarize three-run component-aware initial-motion experiments."""

import csv
import json
import statistics
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


RESULTS = Path("results/ros_reference_path")
GROUPS = {
    ("Same +25", "Simultaneous"): (
        "threshold_same_p25_simultaneous_coordinated_counterphase",
        "run02_same_p25_simultaneous_coordinated_counterphase",
        "run03_same_p25_simultaneous_coordinated_counterphase",
    ),
    ("Same +25", "Proposed"): (
        "componentaware_same_p25_coordinated_counterphase",
        "run02_same_p25_componentaware_coordinated_counterphase",
        "run03_same_p25_componentaware_coordinated_counterphase",
    ),
    ("Left counter +40", "Simultaneous"): (
        "cfgthreshold_counter_p40_simultaneous_coordinated_counterphase",
        "run02_left_counter_p40_simultaneous_coordinated_counterphase",
        "run03_left_counter_p40_simultaneous_coordinated_counterphase",
    ),
    ("Left counter +40", "Proposed"): (
        "componentaware_counter_p40_threshold35_coordinated_counterphase",
        "run02_left_counter_p40_componentaware_coordinated_counterphase",
        "run03_left_counter_p40_componentaware_coordinated_counterphase",
    ),
    ("Right counter -40", "Simultaneous"): (
        "right_counter_m40_simultaneous_coordinated_counterphase",
        "run02_right_counter_m40_simultaneous_coordinated_counterphase",
        "run03_right_counter_m40_simultaneous_coordinated_counterphase",
    ),
    ("Right counter -40", "Proposed"): (
        "right_counter_m40_componentaware_threshold35_coordinated_counterphase",
        "run02_right_counter_m40_componentaware_coordinated_counterphase",
        "run03_right_counter_m40_componentaware_coordinated_counterphase",
    ),
}

METRICS = {
    "rmse_mm": ("rmse_cross_track_m", 1000.0),
    "max_error_mm": ("max_cross_track_m", 1000.0),
    "goal_error_mm": ("final_goal_error_m", 1000.0),
    "yaw_disturbance_deg": ("motion_initial_max_yaw_disturbance_deg", 1.0),
    "curvature_saturation_percent": ("curvature_saturation_fraction", 100.0),
    "motion_start_delay_s": ("motion_start_delay_s", 1.0),
    "peak_steering_effort": ("initial_peak_steering_effort", 1.0),
}


def load(stem):
    return json.loads((RESULTS / f"{stem}.json").read_text(encoding="utf-8"))


def mean_std(values):
    return statistics.mean(values), statistics.stdev(values)


def main():
    raw_rows = []
    grouped = {}
    summary_rows = []
    for (condition, policy), stems in GROUPS.items():
        runs = [load(stem) for stem in stems]
        grouped[(condition, policy)] = runs
        for repeat, (stem, run) in enumerate(zip(stems, runs), start=1):
            row = {
                "condition": condition,
                "policy": policy,
                "repeat": repeat,
                "run_id": stem,
                "success": not run["timed_out"],
            }
            for output_name, (input_name, scale) in METRICS.items():
                row[output_name] = scale * run[input_name]
            raw_rows.append(row)

        summary = {
            "condition": condition,
            "policy": policy,
            "n": len(runs),
            "success_count": sum(not run["timed_out"] for run in runs),
            "success_rate_percent": 100.0
            * sum(not run["timed_out"] for run in runs)
            / len(runs),
        }
        for output_name, (input_name, scale) in METRICS.items():
            values = [scale * run[input_name] for run in runs]
            mean, std = mean_std(values)
            summary[f"{output_name}_mean"] = mean
            summary[f"{output_name}_std"] = std
        summary_rows.append(summary)

    raw_csv = RESULTS / "componentaware_repeats_raw.csv"
    with raw_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(raw_rows[0]))
        writer.writeheader()
        writer.writerows(raw_rows)

    summary_csv = RESULTS / "componentaware_repeats_summary.csv"
    with summary_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(summary_rows[0]))
        writer.writeheader()
        writer.writerows(summary_rows)

    conditions = ("Same +25", "Left counter +40", "Right counter -40")
    policies = ("Simultaneous", "Proposed")
    x = np.arange(len(conditions))
    width = 0.36
    plots = (
        ("success_rate_percent", "Success rate [%]", False),
        ("rmse_mm", "Path RMSE [mm]", True),
        ("max_error_mm", "Maximum path error [mm]", True),
        ("yaw_disturbance_deg", "Initial yaw disturbance [deg]", False),
        ("curvature_saturation_percent", "Curvature saturation [%]", False),
        ("motion_start_delay_s", "Motion-start delay [s]", False),
    )
    colors = {"Simultaneous": "tab:red", "Proposed": "tab:blue"}
    fig, axes = plt.subplots(2, 3, figsize=(15, 8.5))
    for axis, (metric, ylabel, log_scale) in zip(axes.flat, plots):
        for offset_index, policy in enumerate(policies):
            means = []
            stds = []
            for condition in conditions:
                runs = grouped[(condition, policy)]
                if metric == "success_rate_percent":
                    values = [100.0 * (not run["timed_out"]) for run in runs]
                else:
                    input_name, scale = METRICS[metric]
                    values = [scale * run[input_name] for run in runs]
                means.append(statistics.mean(values))
                stds.append(statistics.stdev(values))
            positions = x + (offset_index - 0.5) * width
            axis.bar(
                positions,
                means,
                width,
                yerr=stds,
                capsize=4,
                label=policy,
                color=colors[policy],
                alpha=0.85,
            )
        axis.set_xticks(x, conditions, rotation=15, ha="right")
        axis.set_ylabel(ylabel)
        axis.grid(axis="y", alpha=0.3)
        if log_scale:
            axis.set_yscale("log")
        axis.legend(fontsize=8)
    fig.suptitle(
        "Configuration-component-aware initial-motion control (mean +/- SD, n=3)\n"
        "same-phase threshold 20 deg; counterphase threshold 35 deg"
    )
    fig.tight_layout()
    output_plot = RESULTS / "componentaware_repeats_summary.png"
    fig.savefig(output_plot, dpi=180)
    plt.close(fig)

    print(f"Raw CSV: {raw_csv}")
    print(f"Summary CSV: {summary_csv}")
    print(f"Summary plot: {output_plot}")


if __name__ == "__main__":
    main()
