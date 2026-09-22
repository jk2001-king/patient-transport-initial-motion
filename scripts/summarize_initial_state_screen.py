#!/usr/bin/env python3
"""Summarize the initial-steering-state policy screening experiments."""

import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt


RESULTS = Path("results/ros_reference_path")
EXPERIMENTS = (
    ("front 0\nrear 0", "simultaneous_aligned_coordinated_counterphase", "proposed_aligned_coordinated_counterphase"),
    ("front +45\nrear +45", "initial_crab45_coordinated_counterphase", "proposed_crab45_coordinated_counterphase"),
    ("front +45\nrear 0", "simultaneous_front45_coordinated_counterphase", "proposed_front45_coordinated_counterphase"),
    ("front 0\nrear +45", "simultaneous_rear45_coordinated_counterphase", "proposed_rear45_coordinated_counterphase"),
    ("front +45\nrear -45", "simultaneous_counter45_coordinated_counterphase", "proposed_counter45_coordinated_counterphase"),
)


def load(stem):
    result = json.loads((RESULTS / f"{stem}.json").read_text(encoding="utf-8"))
    samples = list(csv.DictReader((RESULTS / f"{stem}.csv").open(encoding="utf-8")))
    result["elapsed_s"] = float(samples[-1]["elapsed_s"])
    return result


def main():
    rows = []
    for condition, baseline_stem, proposed_stem in EXPERIMENTS:
        for policy, stem in (("simultaneous", baseline_stem), ("steer_then_drive", proposed_stem)):
            data = load(stem)
            rows.append(
                {
                    "condition": condition.replace("\n", "/"),
                    "policy": policy,
                    "success": not data["timed_out"],
                    "elapsed_s": data["elapsed_s"],
                    "rmse_cross_track_m": data["rmse_cross_track_m"],
                    "max_cross_track_m": data["max_cross_track_m"],
                    "final_goal_error_m": data["final_goal_error_m"],
                    "initial_rmse_cross_track_m": data["initial_rmse_cross_track_m"],
                    "initial_cross_track_iae_m_s": data["initial_cross_track_iae_m_s"],
                    "initial_max_yaw_disturbance_deg": data["initial_max_yaw_disturbance_deg"],
                    "initial_peak_steering_effort": data["initial_peak_steering_effort"],
                    "curvature_saturation_fraction": data["curvature_saturation_fraction"],
                }
            )

    output_csv = RESULTS / "initial_state_screen_summary.csv"
    with output_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    by_key = {(row["condition"], row["policy"]): row for row in rows}
    labels = [condition for condition, _, _ in EXPERIMENTS]
    keys = [label.replace("\n", "/") for label in labels]
    positions = list(range(len(labels)))
    fig, axes = plt.subplots(2, 2, figsize=(13, 8))
    specs = (
        (axes[0, 0], "rmse_cross_track_m", 1000.0, "Overall RMSE [mm]"),
        (axes[0, 1], "initial_cross_track_iae_m_s", 1000.0, "Initial CTE IAE [mm s]"),
        (axes[1, 0], "initial_max_yaw_disturbance_deg", 1.0, "Initial max yaw disturbance [deg]"),
        (axes[1, 1], "elapsed_s", 1.0, "Elapsed time [s]"),
    )
    for axis, metric, scale, ylabel in specs:
        for offset, policy, color, display in (
            (-0.19, "simultaneous", "tab:red", "simultaneous"),
            (+0.19, "steer_then_drive", "tab:blue", "steer-then-drive"),
        ):
            values = [scale * by_key[(key, policy)][metric] for key in keys]
            bars = axis.bar([x + offset for x in positions], values, width=0.38, color=color, label=display)
            if policy == "simultaneous":
                for bar, key in zip(bars, keys):
                    if not by_key[(key, policy)]["success"]:
                        bar.set_hatch("//")
                        axis.text(bar.get_x() + bar.get_width() / 2, bar.get_height(), " timeout", rotation=90,
                                  va="bottom", ha="center", fontsize=8)
        axis.set_xticks(positions, labels)
        axis.set_ylabel(ylabel)
        axis.grid(True, axis="y", alpha=0.4)
        axis.legend()

    fig.suptitle("Initial steering state: simultaneous vs steer-then-drive")
    fig.tight_layout()
    output_plot = RESULTS / "initial_state_screen_summary.png"
    fig.savefig(output_plot, dpi=180)
    plt.close(fig)
    print(f"Summary CSV: {output_csv}")
    print(f"Summary plot: {output_plot}")


if __name__ == "__main__":
    main()
