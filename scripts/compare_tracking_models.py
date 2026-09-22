#!/usr/bin/env python3
"""Compare common-base and configuration-aware tracking for each steering mode."""

import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt


RESULTS = Path("results/ros_reference_path")
CONFIGURATIONS = ("front_only", "rear_only", "coordinated_counterphase")
LABELS = ("Front only", "Rear only", "Coordinated")


def load_json(run_id, configuration):
    path = RESULTS / f"{run_id}_{configuration}.json"
    return json.loads(path.read_text(encoding="utf-8"))


def load_csv(run_id, configuration):
    path = RESULTS / f"{run_id}_{configuration}.csv"
    rows = list(csv.DictReader(path.open(encoding="utf-8")))
    for row in rows:
        for key, value in list(row.items()):
            row[key] = float(value)
    return rows


def main():
    output_rows = []
    baseline_rows = {}
    aware_rows = {}
    for configuration in CONFIGURATIONS:
        baseline = load_json("curve01", configuration)
        aware = load_json("aware01", configuration)
        baseline_rows[configuration] = load_csv("curve01", configuration)
        aware_rows[configuration] = load_csv("aware01", configuration)
        reduction = 100.0 * (
            1.0 - aware["rmse_cross_track_m"] / baseline["rmse_cross_track_m"]
        )
        output_rows.append(
            {
                "configuration": configuration,
                "baseline_timeout": baseline["timed_out"],
                "aware_timeout": aware["timed_out"],
                "baseline_rmse_m": baseline["rmse_cross_track_m"],
                "aware_rmse_m": aware["rmse_cross_track_m"],
                "rmse_reduction_pct": reduction,
                "baseline_max_error_m": baseline["max_cross_track_m"],
                "aware_max_error_m": aware["max_cross_track_m"],
                "baseline_goal_error_m": baseline["final_goal_error_m"],
                "aware_goal_error_m": aware["final_goal_error_m"],
                "baseline_heading_error_deg": baseline["final_heading_error_deg"],
                "aware_heading_error_deg": aware["final_heading_error_deg"],
                "aware_curvature_saturation_fraction": aware.get(
                    "curvature_saturation_fraction"
                ),
            }
        )

    output_csv = RESULTS / "curve01_baseline_vs_aware.csv"
    with output_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(output_rows[0].keys()))
        writer.writeheader()
        writer.writerows(output_rows)

    fig, axes = plt.subplots(2, 3, figsize=(15, 8))
    for column, (configuration, label) in enumerate(zip(CONFIGURATIONS, LABELS)):
        baseline = baseline_rows[configuration]
        aware = aware_rows[configuration]
        axes[0, column].plot(
            [row["x_m"] - baseline[0]["x_m"] for row in baseline],
            [row["y_m"] - baseline[0]["y_m"] for row in baseline],
            label="common tracker",
            color="tab:red",
        )
        axes[0, column].plot(
            [row["x_m"] - aware[0]["x_m"] for row in aware],
            [row["y_m"] - aware[0]["y_m"] for row in aware],
            label="configuration-aware",
            color="tab:blue",
        )
        axes[0, column].set_title(label)
        axes[0, column].set_xlabel("x [m]")
        axes[0, column].set_ylabel("y [m]")
        axes[0, column].grid(True)
        axes[0, column].legend(fontsize=8)

        axes[1, column].plot(
            [row["elapsed_s"] for row in baseline],
            [100.0 * row["cross_track_error_m"] for row in baseline],
            label="common tracker",
            color="tab:red",
        )
        axes[1, column].plot(
            [row["elapsed_s"] for row in aware],
            [100.0 * row["cross_track_error_m"] for row in aware],
            label="configuration-aware",
            color="tab:blue",
        )
        axes[1, column].set_xlabel("time [s]")
        axes[1, column].set_ylabel("cross-track error [cm]")
        axes[1, column].grid(True)

    fig.tight_layout()
    output_plot = RESULTS / "curve01_baseline_vs_aware.png"
    fig.savefig(output_plot, dpi=180)
    plt.close(fig)

    print(f"Comparison CSV: {output_csv}")
    print(f"Comparison plot: {output_plot}")
    for row in output_rows:
        print(
            f"{row['configuration']}: "
            f"RMSE {row['baseline_rmse_m']:.4f} -> {row['aware_rmse_m']:.4f} m "
            f"({row['rmse_reduction_pct']:+.1f}%), "
            f"heading {row['baseline_heading_error_deg']:.3f} -> "
            f"{row['aware_heading_error_deg']:.3f} deg"
        )


if __name__ == "__main__":
    main()
