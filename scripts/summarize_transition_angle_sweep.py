#!/usr/bin/env python3
"""Summarize the coordinated same-phase 45--90 deg transition sweep."""

import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt


RESULTS = Path("results/ros_reference_path")
CASES = (
    (
        45,
        "initial_crab45_coordinated_counterphase",
        "cfg_coordinated_configadaptive_same45_coordinated_counterphase",
    ),
    (
        60,
        "transition_same_p60_simultaneous_coordinated_counterphase",
        "transition_same_p60_configadaptive_coordinated_counterphase",
    ),
    (
        75,
        "transition_same_p75_simultaneous_coordinated_counterphase",
        "transition_same_p75_configadaptive_coordinated_counterphase",
    ),
    (
        90,
        "transition_same_p90_simultaneous_coordinated_counterphase",
        "transition_same_p90_configadaptive_coordinated_counterphase",
    ),
)


def load(stem):
    summary = json.loads((RESULTS / f"{stem}.json").read_text(encoding="utf-8"))
    with (RESULTS / f"{stem}.csv").open(encoding="utf-8") as stream:
        samples = list(csv.DictReader(stream))
    summary["elapsed_s"] = float(samples[-1]["elapsed_s"])
    return summary


def reduction(before, after):
    return 100.0 * (1.0 - after / before) if before else float("nan")


def main():
    rows = []
    paired = []
    for angle, baseline_stem, adaptive_stem in CASES:
        baseline = load(baseline_stem)
        adaptive = load(adaptive_stem)
        paired.append((angle, baseline, adaptive))
        rows.append(
            {
                "initial_same_phase_angle_deg": angle,
                "baseline_success": not baseline["timed_out"],
                "adaptive_success": not adaptive["timed_out"],
                "baseline_elapsed_s": baseline["elapsed_s"],
                "adaptive_elapsed_s": adaptive["elapsed_s"],
                "baseline_rmse_mm": 1000.0 * baseline["rmse_cross_track_m"],
                "adaptive_rmse_mm": 1000.0 * adaptive["rmse_cross_track_m"],
                "rmse_reduction_pct": reduction(
                    baseline["rmse_cross_track_m"], adaptive["rmse_cross_track_m"]
                ),
                "baseline_max_error_mm": 1000.0 * baseline["max_cross_track_m"],
                "adaptive_max_error_mm": 1000.0 * adaptive["max_cross_track_m"],
                "max_error_reduction_pct": reduction(
                    baseline["max_cross_track_m"], adaptive["max_cross_track_m"]
                ),
                "baseline_goal_error_mm": 1000.0 * baseline["final_goal_error_m"],
                "adaptive_goal_error_mm": 1000.0 * adaptive["final_goal_error_m"],
                "baseline_abs_heading_error_deg": abs(
                    baseline["final_heading_error_deg"]
                ),
                "adaptive_abs_heading_error_deg": abs(
                    adaptive["final_heading_error_deg"]
                ),
            }
        )

    output_csv = RESULTS / "transition_angle_sweep_summary.csv"
    with output_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    angles = [angle for angle, _, _ in paired]
    specs = (
        ("rmse_cross_track_m", 1000.0, "RMSE [mm]"),
        ("max_cross_track_m", 1000.0, "Maximum cross-track error [mm]"),
        ("final_goal_error_m", 1000.0, "Final goal error [mm]"),
        ("final_heading_error_deg", 1.0, "Absolute final heading error [deg]"),
    )
    fig, axes = plt.subplots(2, 2, figsize=(11, 8))
    for axis, (metric, scale, title) in zip(axes.flat, specs):
        baseline_values = [
            scale * abs(baseline[metric]) for _, baseline, _ in paired
        ]
        adaptive_values = [
            scale * abs(adaptive[metric]) for _, _, adaptive in paired
        ]
        axis.plot(angles, baseline_values, "o-", color="tab:red", label="Simultaneous")
        axis.plot(
            angles,
            adaptive_values,
            "o-",
            color="tab:blue",
            label="Configuration-aware",
        )
        for angle, value, (_, baseline, _) in zip(angles, baseline_values, paired):
            if baseline["timed_out"]:
                axis.annotate(
                    "timeout",
                    (angle, value),
                    xytext=(0, 7),
                    textcoords="offset points",
                    ha="center",
                    fontsize=8,
                )
        axis.set_xticks(angles)
        axis.set_xlabel("Initial front/rear same-phase angle [deg]")
        axis.set_ylabel(title)
        axis.grid(True, alpha=0.35)
        axis.legend()

    fig.suptitle("Initial steering transition sweep (single-run screening)")
    fig.tight_layout()
    output_plot = RESULTS / "transition_angle_sweep_summary.png"
    fig.savefig(output_plot, dpi=180)
    plt.close(fig)

    print(f"Summary CSV: {output_csv}")
    print(f"Summary plot: {output_plot}")
    for row in rows:
        print(
            f"{row['initial_same_phase_angle_deg']:>2} deg: "
            f"baseline_success={row['baseline_success']}, "
            f"adaptive_success={row['adaptive_success']}, "
            f"RMSE reduction={row['rmse_reduction_pct']:.2f}%, "
            f"max reduction={row['max_error_reduction_pct']:.2f}%"
        )


if __name__ == "__main__":
    main()
