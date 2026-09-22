#!/usr/bin/env python3
"""Summarize the same-phase threshold screening and 20-degree policy checks."""

import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt


RESULTS = Path("results/ros_reference_path")
BASELINE = {
    5.0: "threshold_same_p05_simultaneous_coordinated_counterphase",
    10.0: "threshold_same_p10_simultaneous_coordinated_counterphase",
    15.0: "threshold_same_p15_simultaneous_coordinated_counterphase",
    20.0: "threshold_same_p20_simultaneous_coordinated_counterphase",
    22.5: "threshold_same_p22p5_simultaneous_coordinated_counterphase",
    25.0: "threshold_same_p25_simultaneous_coordinated_counterphase",
    30.0: "threshold_same_p30_simultaneous_coordinated_counterphase",
}
THRESHOLD20 = {
    20.0: "threshold20_same_p20_configadaptive_coordinated_counterphase",
    22.5: "threshold20_same_p22p5_configadaptive_coordinated_counterphase",
    25.0: "threshold20_same_p25_configadaptive_coordinated_counterphase",
    30.0: "threshold20_same_p30_configadaptive_coordinated_counterphase",
}


def load(stem):
    result = json.loads((RESULTS / f"{stem}.json").read_text(encoding="utf-8"))
    with (RESULTS / f"{stem}.csv").open(encoding="utf-8") as stream:
        samples = list(csv.DictReader(stream))
    result["elapsed_s"] = float(samples[-1]["elapsed_s"])
    return result


def main():
    rows = []
    loaded = {"simultaneous": {}, "threshold20": {}}
    for policy, cases in (("simultaneous", BASELINE), ("threshold20", THRESHOLD20)):
        for angle, stem in cases.items():
            result = load(stem)
            loaded[policy][angle] = result
            rows.append(
                {
                    "policy": policy,
                    "initial_same_phase_angle_deg": angle,
                    "success": not result["timed_out"],
                    "rmse_mm": 1000.0 * result["rmse_cross_track_m"],
                    "max_error_mm": 1000.0 * result["max_cross_track_m"],
                    "goal_error_mm": 1000.0 * result["final_goal_error_m"],
                    "motion_yaw_disturbance_deg": result[
                        "motion_initial_max_yaw_disturbance_deg"
                    ],
                    "curvature_saturation_fraction": result[
                        "curvature_saturation_fraction"
                    ],
                    "motion_start_delay_s": result["motion_start_delay_s"],
                    "elapsed_s": result["elapsed_s"],
                }
            )

    output_csv = RESULTS / "threshold20_screening_summary.csv"
    with output_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    specs = (
        ("rmse_cross_track_m", 1000.0, "RMSE [mm]"),
        ("max_cross_track_m", 1000.0, "Maximum cross-track error [mm]"),
        ("motion_initial_max_yaw_disturbance_deg", 1.0, "Motion-aligned yaw disturbance [deg]"),
        ("elapsed_s", 1.0, "Elapsed time [s]"),
    )
    fig, axes = plt.subplots(2, 2, figsize=(11, 8))
    for axis, (metric, scale, ylabel) in zip(axes.flat, specs):
        for policy, color, label in (
            ("simultaneous", "tab:red", "Simultaneous"),
            ("threshold20", "tab:blue", "20-deg policy"),
        ):
            angles = sorted(loaded[policy])
            values = [scale * loaded[policy][angle][metric] for angle in angles]
            axis.plot(angles, values, "o-", color=color, label=label)
            for angle, value in zip(angles, values):
                if loaded[policy][angle]["timed_out"]:
                    axis.annotate(
                        "timeout",
                        (angle, value),
                        xytext=(0, 7),
                        textcoords="offset points",
                        ha="center",
                        fontsize=8,
                    )
        axis.axvline(20.0, color="black", linestyle="--", linewidth=1.0, label="candidate threshold")
        axis.set_xlabel("Initial front/rear same-phase angle [deg]")
        axis.set_ylabel(ylabel)
        axis.grid(True, alpha=0.35)
        axis.legend(fontsize=8)

    fig.suptitle("20-deg initial-steering threshold screening (single runs)")
    fig.tight_layout()
    output_plot = RESULTS / "threshold20_screening_summary.png"
    fig.savefig(output_plot, dpi=180)
    plt.close(fig)
    print(f"Summary CSV: {output_csv}")
    print(f"Summary plot: {output_plot}")


if __name__ == "__main__":
    main()
