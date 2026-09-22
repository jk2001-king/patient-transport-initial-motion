#!/usr/bin/env python3
"""Summarize final configuration-aware adaptive initial-control experiments."""

import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt


RESULTS = Path("results/ros_reference_path")
CASES = (
    (
        "front_only\n(+45/0)",
        "cfg_frontonly_sim_front45_front_only",
        "cfg_frontonly_configadaptive_front45_front_only",
    ),
    (
        "rear_only\n(0/+45)",
        "cfg_rearonly_sim_rear45_rear_only",
        "cfg_rearonly_configadaptive_rear45_rear_only",
    ),
    (
        "coordinated\n(+45/+45)",
        "initial_crab45_coordinated_counterphase",
        "cfg_coordinated_configadaptive_same45_coordinated_counterphase",
    ),
    (
        "coordinated\n(+45/-45)",
        "simultaneous_counter45_coordinated_counterphase",
        "cfg_coordinated_configadaptive_counter45_coordinated_counterphase",
    ),
)


def load(stem):
    summary = json.loads((RESULTS / f"{stem}.json").read_text(encoding="utf-8"))
    samples = list(csv.DictReader((RESULTS / f"{stem}.csv").open(encoding="utf-8")))
    summary["elapsed_s"] = float(samples[-1]["elapsed_s"])
    return summary


def reduction(before, after):
    return 100.0 * (1.0 - after / before) if before else float("nan")


def main():
    rows = []
    loaded = []
    for condition, baseline_stem, adaptive_stem in CASES:
        baseline = load(baseline_stem)
        adaptive = load(adaptive_stem)
        loaded.append((condition, baseline, adaptive))
        rows.append(
            {
                "condition": condition.replace("\n", " "),
                "baseline_success": not baseline["timed_out"],
                "adaptive_success": not adaptive["timed_out"],
                "baseline_elapsed_s": baseline["elapsed_s"],
                "adaptive_elapsed_s": adaptive["elapsed_s"],
                "added_time_s": adaptive["elapsed_s"] - baseline["elapsed_s"],
                "baseline_rmse_m": baseline["rmse_cross_track_m"],
                "adaptive_rmse_m": adaptive["rmse_cross_track_m"],
                "rmse_reduction_pct": reduction(
                    baseline["rmse_cross_track_m"], adaptive["rmse_cross_track_m"]
                ),
                "baseline_max_error_m": baseline["max_cross_track_m"],
                "adaptive_max_error_m": adaptive["max_cross_track_m"],
                "max_error_reduction_pct": reduction(
                    baseline["max_cross_track_m"], adaptive["max_cross_track_m"]
                ),
                "baseline_initial_iae_m_s": baseline["initial_cross_track_iae_m_s"],
                "adaptive_initial_iae_m_s": adaptive["initial_cross_track_iae_m_s"],
                "initial_iae_reduction_pct": reduction(
                    baseline["initial_cross_track_iae_m_s"], adaptive["initial_cross_track_iae_m_s"]
                ),
                "baseline_yaw_disturbance_deg": baseline["initial_max_yaw_disturbance_deg"],
                "adaptive_yaw_disturbance_deg": adaptive["initial_max_yaw_disturbance_deg"],
                "yaw_reduction_pct": reduction(
                    baseline["initial_max_yaw_disturbance_deg"],
                    adaptive["initial_max_yaw_disturbance_deg"],
                ),
                "baseline_peak_effort": baseline["initial_peak_steering_effort"],
                "adaptive_peak_effort": adaptive["initial_peak_steering_effort"],
            }
        )

    output_csv = RESULTS / "configuration_adaptive_summary.csv"
    with output_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    specs = (
        ("rmse_cross_track_m", 1000.0, "RMSE [mm]"),
        ("max_cross_track_m", 1000.0, "Maximum cross-track error [mm]"),
        ("initial_cross_track_iae_m_s", 1000.0, "Initial CTE IAE [mm s]"),
        ("initial_max_yaw_disturbance_deg", 1.0, "Initial yaw disturbance [deg]"),
        ("elapsed_s", 1.0, "Elapsed time [s]"),
        ("initial_peak_steering_effort", 1.0, "Peak steering effort proxy"),
    )
    positions = list(range(len(CASES)))
    labels = [case[0] for case in CASES]
    fig, axes = plt.subplots(2, 3, figsize=(16, 8))
    for axis, (metric, scale, title) in zip(axes.flat, specs):
        baseline_values = [scale * baseline[metric] for _, baseline, _ in loaded]
        adaptive_values = [scale * adaptive[metric] for _, _, adaptive in loaded]
        baseline_bars = axis.bar(
            [position - 0.19 for position in positions],
            baseline_values,
            width=0.38,
            label="simultaneous",
            color="tab:red",
        )
        axis.bar(
            [position + 0.19 for position in positions],
            adaptive_values,
            width=0.38,
            label="configuration-adaptive",
            color="tab:blue",
        )
        for bar, (_, baseline, _) in zip(baseline_bars, loaded):
            if baseline["timed_out"]:
                bar.set_hatch("//")
                axis.text(
                    bar.get_x() + bar.get_width() / 2,
                    bar.get_height(),
                    "timeout",
                    ha="center",
                    va="bottom",
                    rotation=90,
                    fontsize=8,
                )
        axis.set_xticks(positions, labels)
        axis.set_title(title)
        axis.grid(True, axis="y", alpha=0.4)
        axis.legend(fontsize=8)

    fig.suptitle("Configuration-aware adaptive initial-control results")
    fig.tight_layout()
    output_plot = RESULTS / "configuration_adaptive_summary.png"
    fig.savefig(output_plot, dpi=180)
    plt.close(fig)
    print(f"Summary CSV: {output_csv}")
    print(f"Summary plot: {output_plot}")
    for row in rows:
        print(
            f"{row['condition']}: RMSE {row['rmse_reduction_pct']:+.2f}%, "
            f"max {row['max_error_reduction_pct']:+.2f}%, "
            f"IAE {row['initial_iae_reduction_pct']:+.2f}%, "
            f"yaw {row['yaw_reduction_pct']:+.2f}%, "
            f"added time {row['added_time_s']:+.3f} s"
        )


if __name__ == "__main__":
    main()
