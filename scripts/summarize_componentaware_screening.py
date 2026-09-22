#!/usr/bin/env python3
"""Summarize the component-aware initial-motion screening runs."""

import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


RESULTS = Path("results/ros_reference_path")
RUNS = (
    ("Same +25\nSim.", "threshold_same_p25_simultaneous_coordinated_counterphase", "simultaneous"),
    ("Same +25\nProposed", "componentaware_same_p25_coordinated_counterphase", "proposed"),
    ("Counter +35\nSim.", "cfgthreshold_counter_p35_simultaneous_coordinated_counterphase", "simultaneous"),
    ("Counter +35\nProposed", "componentaware_counter_p35_threshold35_coordinated_counterphase", "proposed"),
    ("Left counter +40\nSim.", "cfgthreshold_counter_p40_simultaneous_coordinated_counterphase", "simultaneous"),
    ("Left counter +40\nProposed", "componentaware_counter_p40_threshold35_coordinated_counterphase", "proposed"),
    ("Right counter -40\nSim.", "right_counter_m40_simultaneous_coordinated_counterphase", "simultaneous"),
    ("Right counter -40\nProposed", "right_counter_m40_componentaware_threshold35_coordinated_counterphase", "proposed"),
)


def load(stem):
    summary = json.loads((RESULTS / f"{stem}.json").read_text(encoding="utf-8"))
    with (RESULTS / f"{stem}.csv").open(encoding="utf-8") as stream:
        samples = list(csv.DictReader(stream))
    summary["elapsed_s"] = float(samples[-1]["elapsed_s"])
    return summary


def main():
    loaded = [(label, policy, load(stem)) for label, stem, policy in RUNS]
    rows = []
    for label, policy, result in loaded:
        rows.append(
            {
                "condition": label.replace("\n", " "),
                "policy": policy,
                "success": not result["timed_out"],
                "rmse_mm": 1000.0 * result["rmse_cross_track_m"],
                "max_error_mm": 1000.0 * result["max_cross_track_m"],
                "goal_error_mm": 1000.0 * result["final_goal_error_m"],
                "motion_yaw_disturbance_deg": result[
                    "motion_initial_max_yaw_disturbance_deg"
                ],
                "curvature_saturation_percent": 100.0
                * result["curvature_saturation_fraction"],
                "motion_start_delay_s": result["motion_start_delay_s"],
                "elapsed_s": result["elapsed_s"],
            }
        )

    output_csv = RESULTS / "componentaware_screening_summary.csv"
    with output_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    specs = (
        ("rmse_cross_track_m", 1000.0, "Path RMSE [mm]"),
        ("motion_initial_max_yaw_disturbance_deg", 1.0, "Initial yaw disturbance [deg]"),
        ("curvature_saturation_fraction", 100.0, "Curvature saturation [%]"),
        ("motion_start_delay_s", 1.0, "Motion-start delay [s]"),
    )
    labels = [label for label, _, _ in loaded]
    colors = ["tab:red" if policy == "simultaneous" else "tab:blue" for _, policy, _ in loaded]
    x = np.arange(len(loaded))
    fig, axes = plt.subplots(2, 2, figsize=(14, 8.5))
    for axis, (metric, scale, ylabel) in zip(axes.flat, specs):
        values = [scale * result[metric] for _, _, result in loaded]
        bars = axis.bar(x, values, color=colors, alpha=0.85)
        axis.set_xticks(x, labels, rotation=25, ha="right")
        axis.set_ylabel(ylabel)
        axis.grid(axis="y", alpha=0.3)
        for bar, value in zip(bars, values):
            axis.text(
                bar.get_x() + bar.get_width() / 2.0,
                bar.get_height(),
                f"{value:.2f}",
                ha="center",
                va="bottom",
                fontsize=7,
            )
    fig.suptitle(
        "Component-aware initial-motion screening\n"
        "same-phase threshold 20 deg, counterphase threshold 35 deg (single runs)"
    )
    fig.tight_layout()
    output_plot = RESULTS / "componentaware_screening_summary.png"
    fig.savefig(output_plot, dpi=180)
    plt.close(fig)
    print(f"Summary CSV: {output_csv}")
    print(f"Summary plot: {output_plot}")


if __name__ == "__main__":
    main()
