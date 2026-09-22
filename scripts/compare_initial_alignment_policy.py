#!/usr/bin/env python3
"""Compare simultaneous motion and steer-then-drive experiment results."""

import argparse
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt


RESULTS = Path("results/ros_reference_path")
DEFAULT_BASE_STEM = "initial_crab45_coordinated_counterphase"
DEFAULT_PROPOSED_STEM = "proposed_crab45_coordinated_counterphase"


def load_rows(stem):
    rows = list(csv.DictReader((RESULTS / f"{stem}.csv").open(encoding="utf-8")))
    for row in rows:
        for key, value in list(row.items()):
            row[key] = float(value)
    return rows


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline", default=DEFAULT_BASE_STEM)
    parser.add_argument("--proposed", default=DEFAULT_PROPOSED_STEM)
    parser.add_argument("--output-prefix", default="initial_crab45_policy")
    return parser.parse_args()


def main():
    args = parse_args()
    baseline = json.loads((RESULTS / f"{args.baseline}.json").read_text(encoding="utf-8"))
    proposed = json.loads((RESULTS / f"{args.proposed}.json").read_text(encoding="utf-8"))
    baseline_rows = load_rows(args.baseline)
    proposed_rows = load_rows(args.proposed)

    metrics = (
        "rmse_cross_track_m",
        "max_cross_track_m",
        "final_goal_error_m",
        "initial_rmse_cross_track_m",
        "initial_max_cross_track_m",
        "initial_cross_track_iae_m_s",
        "initial_max_yaw_disturbance_deg",
        "initial_peak_steering_effort",
    )
    output_rows = []
    for metric in metrics:
        before, after = baseline[metric], proposed[metric]
        output_rows.append(
            {
                "metric": metric,
                "simultaneous": before,
                "steer_then_drive": after,
                "reduction_pct": 100.0 * (1.0 - after / before) if before else float("nan"),
            }
        )
    output_csv = RESULTS / f"{args.output_prefix}_comparison.csv"
    with output_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(output_rows[0].keys()))
        writer.writeheader()
        writer.writerows(output_rows)

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    for rows, label, color in (
        (baseline_rows, "simultaneous", "tab:red"),
        (proposed_rows, "steer-then-drive", "tab:blue"),
    ):
        x0, y0 = rows[0]["x_m"], rows[0]["y_m"]
        axes[0, 0].plot(
            [row["x_m"] - x0 for row in rows],
            [row["y_m"] - y0 for row in rows],
            label=label,
            color=color,
        )
        axes[0, 1].plot(
            [row["elapsed_s"] for row in rows],
            [100.0 * row["cross_track_error_m"] for row in rows],
            label=label,
            color=color,
        )
        axes[1, 0].plot(
            [row["elapsed_s"] for row in rows],
            [row["front_steering_deg"] for row in rows],
            label=f"{label}: front",
            color=color,
        )
        axes[1, 0].plot(
            [row["elapsed_s"] for row in rows],
            [row["rear_steering_deg"] for row in rows],
            linestyle="--",
            label=f"{label}: rear",
            color=color,
        )

    axes[0, 0].set_title("Ground-truth trajectory")
    axes[0, 0].set_xlabel("x [m]")
    axes[0, 0].set_ylabel("y [m]")
    axes[0, 0].grid(True)
    axes[0, 0].legend()
    axes[0, 1].set_title("Cross-track error")
    axes[0, 1].set_xlabel("time [s]")
    axes[0, 1].set_ylabel("error [cm]")
    axes[0, 1].grid(True)
    axes[0, 1].legend()
    axes[1, 0].set_title("Measured steering")
    axes[1, 0].set_xlabel("time [s]")
    axes[1, 0].set_ylabel("physical steering [deg]")
    axes[1, 0].grid(True)
    axes[1, 0].legend(fontsize=8)

    labels = ["initial RMSE", "initial max", "overall RMSE"]
    before = [
        1000.0 * baseline["initial_rmse_cross_track_m"],
        1000.0 * baseline["initial_max_cross_track_m"],
        1000.0 * baseline["rmse_cross_track_m"],
    ]
    after = [
        1000.0 * proposed["initial_rmse_cross_track_m"],
        1000.0 * proposed["initial_max_cross_track_m"],
        1000.0 * proposed["rmse_cross_track_m"],
    ]
    positions = range(len(labels))
    axes[1, 1].bar([p - 0.18 for p in positions], before, width=0.36, label="simultaneous", color="tab:red")
    axes[1, 1].bar([p + 0.18 for p in positions], after, width=0.36, label="steer-then-drive", color="tab:blue")
    axes[1, 1].set_xticks(list(positions), labels)
    axes[1, 1].set_ylabel("error [mm]")
    axes[1, 1].set_title("Error comparison")
    axes[1, 1].grid(True, axis="y")
    axes[1, 1].legend()

    fig.tight_layout()
    output_plot = RESULTS / f"{args.output_prefix}_comparison.png"
    fig.savefig(output_plot, dpi=180)
    plt.close(fig)
    print(f"Comparison CSV: {output_csv}")
    print(f"Comparison plot: {output_plot}")
    for row in output_rows:
        print(
            f"{row['metric']}: {row['simultaneous']:.6f} -> "
            f"{row['steer_then_drive']:.6f} ({row['reduction_pct']:+.2f}%)"
        )


if __name__ == "__main__":
    main()
