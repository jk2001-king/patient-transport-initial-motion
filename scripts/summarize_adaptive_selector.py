#!/usr/bin/env python3
"""Summarize whether the adaptive selector reproduces each selected policy."""

import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt


RESULTS = Path("results/ros_reference_path")
CASES = (
    (
        "front +45\n(risk)",
        "scaled10",
        "scaled10_front45_coordinated_counterphase",
        "adaptive_front45_coordinated_counterphase",
    ),
    (
        "rear +45\n(safe)",
        "simultaneous",
        "simultaneous_rear45_coordinated_counterphase",
        "adaptive_rear45_coordinated_counterphase",
    ),
)


def load(stem):
    summary = json.loads((RESULTS / f"{stem}.json").read_text(encoding="utf-8"))
    samples = list(csv.DictReader((RESULTS / f"{stem}.csv").open(encoding="utf-8")))
    summary["elapsed_s"] = float(samples[-1]["elapsed_s"])
    return summary


def main():
    rows = []
    for condition, selected_policy, selected_stem, adaptive_stem in CASES:
        for implementation, stem in ((selected_policy, selected_stem), ("adaptive", adaptive_stem)):
            data = load(stem)
            rows.append(
                {
                    "condition": condition.replace("\n", " "),
                    "implementation": implementation,
                    "success": not data["timed_out"],
                    "elapsed_s": data["elapsed_s"],
                    "rmse_cross_track_m": data["rmse_cross_track_m"],
                    "max_cross_track_m": data["max_cross_track_m"],
                    "final_goal_error_m": data["final_goal_error_m"],
                    "initial_cross_track_iae_m_s": data["initial_cross_track_iae_m_s"],
                    "initial_max_yaw_disturbance_deg": data["initial_max_yaw_disturbance_deg"],
                }
            )

    output_csv = RESULTS / "adaptive_selector_validation.csv"
    with output_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    fig, axes = plt.subplots(1, 3, figsize=(13, 4))
    metrics = (
        ("elapsed_s", 1.0, "Elapsed time [s]"),
        ("rmse_cross_track_m", 1000.0, "RMSE [mm]"),
        ("initial_max_yaw_disturbance_deg", 1.0, "Initial yaw disturbance [deg]"),
    )
    positions = list(range(len(CASES)))
    for axis, (metric, scale, title) in zip(axes, metrics):
        selected = [scale * rows[2 * i][metric] for i in positions]
        adaptive = [scale * rows[2 * i + 1][metric] for i in positions]
        axis.bar([p - 0.18 for p in positions], selected, 0.36, label="selected component", color="tab:gray")
        axis.bar([p + 0.18 for p in positions], adaptive, 0.36, label="adaptive", color="tab:purple")
        axis.set_xticks(positions, [case[0] for case in CASES])
        axis.set_title(title)
        axis.grid(True, axis="y", alpha=0.4)
        axis.legend()
    fig.suptitle("Leading-axle adaptive selector validation")
    fig.tight_layout()
    output_plot = RESULTS / "adaptive_selector_validation.png"
    fig.savefig(output_plot, dpi=180)
    plt.close(fig)
    print(f"Summary CSV: {output_csv}")
    print(f"Summary plot: {output_plot}")


if __name__ == "__main__":
    main()
