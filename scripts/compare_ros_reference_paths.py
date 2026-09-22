#!/usr/bin/env python3
"""Compare front-only, rear-only, and coordinated ROS path-tracking runs."""

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib.pyplot as plt


CONFIGURATIONS = (
    "front_only",
    "rear_only",
    "coordinated_counterphase",
)
COLORS = {
    "front_only": "tab:orange",
    "rear_only": "tab:red",
    "coordinated_counterphase": "tab:blue",
}


def load_rows(path: Path):
    rows = list(csv.DictReader(path.open(encoding="utf-8")))
    for row in rows:
        for key, value in list(row.items()):
            row[key] = float(value)
    return rows


def localize(rows):
    x0, y0, yaw0 = rows[0]["x_m"], rows[0]["y_m"], rows[0]["yaw_rad"]
    c, s = math.cos(yaw0), math.sin(yaw0)
    localized = []
    for row in rows:
        dx, dy = row["x_m"] - x0, row["y_m"] - y0
        localized.append((c * dx + s * dy, -s * dx + c * dy))
    return localized


def reference_path():
    points = []
    for index in range(61):
        path_s = index * 0.01
        if path_s <= 0.2:
            x, y = path_s, 0.0
        else:
            arc_s = path_s - 0.2
            x = 0.2 + 2.0 * math.sin(0.5 * arc_s)
            y = 2.0 * (1.0 - math.cos(0.5 * arc_s))
        points.append((x, y))
    return points


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-id", default="curve01")
    parser.add_argument("--results-dir", type=Path, default=Path("results/ros_reference_path"))
    args = parser.parse_args()

    runs = {}
    summaries = {}
    for configuration in CONFIGURATIONS:
        stem = f"{args.run_id}_{configuration}"
        runs[configuration] = load_rows(args.results_dir / f"{stem}.csv")
        summaries[configuration] = json.loads(
            (args.results_dir / f"{stem}.json").read_text(encoding="utf-8")
        )

    output_rows = []
    coordinated_rmse = summaries["coordinated_counterphase"]["rmse_cross_track_m"]
    for configuration in CONFIGURATIONS:
        summary = summaries[configuration]
        output_rows.append(
            {
                "configuration": configuration,
                "timed_out": summary["timed_out"],
                "rmse_cross_track_m": summary["rmse_cross_track_m"],
                "max_cross_track_m": summary["max_cross_track_m"],
                "final_goal_error_m": summary["final_goal_error_m"],
                "final_heading_error_deg": summary["final_heading_error_deg"],
                "rmse_vs_coordinated_ratio": summary["rmse_cross_track_m"] / coordinated_rmse,
            }
        )

    comparison_csv = args.results_dir / f"{args.run_id}_configuration_comparison.csv"
    with comparison_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(output_rows[0].keys()))
        writer.writeheader()
        writer.writerows(output_rows)

    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5))
    ref = reference_path()
    axes[0].plot([p[0] for p in ref], [p[1] for p in ref], "k--", label="reference")
    for configuration in CONFIGURATIONS:
        xy = localize(runs[configuration])
        axes[0].plot(
            [p[0] for p in xy],
            [p[1] for p in xy],
            color=COLORS[configuration],
            label=configuration,
        )
    axes[0].set_title("Ground-truth trajectory")
    axes[0].set_xlabel("longitudinal x [m]")
    axes[0].set_ylabel("lateral y [m]")
    axes[0].axis("equal")
    axes[0].grid(True)
    axes[0].legend(fontsize=8)

    for configuration in CONFIGURATIONS:
        rows = runs[configuration]
        axes[1].plot(
            [row["elapsed_s"] for row in rows],
            [100.0 * row["cross_track_error_m"] for row in rows],
            color=COLORS[configuration],
            label=configuration,
        )
    axes[1].set_title("Cross-track error")
    axes[1].set_xlabel("time [s]")
    axes[1].set_ylabel("absolute error [cm]")
    axes[1].grid(True)

    labels = ["front", "rear", "coordinated"]
    rmse_cm = [100.0 * summaries[c]["rmse_cross_track_m"] for c in CONFIGURATIONS]
    axes[2].bar(labels, rmse_cm, color=[COLORS[c] for c in CONFIGURATIONS])
    axes[2].set_title("Path RMSE")
    axes[2].set_ylabel("RMSE [cm]")
    axes[2].grid(True, axis="y")

    fig.tight_layout()
    comparison_plot = args.results_dir / f"{args.run_id}_configuration_comparison.png"
    fig.savefig(comparison_plot, dpi=180)
    plt.close(fig)

    print(f"Comparison CSV: {comparison_csv}")
    print(f"Comparison plot: {comparison_plot}")
    for row in output_rows:
        print(
            f"{row['configuration']}: timeout={row['timed_out']}, "
            f"RMSE={row['rmse_cross_track_m']:.4f} m, "
            f"max={row['max_cross_track_m']:.4f} m, "
            f"goal={row['final_goal_error_m']:.4f} m, "
            f"heading={row['final_heading_error_deg']:.3f} deg, "
            f"RMSE ratio={row['rmse_vs_coordinated_ratio']:.2f}x"
        )


if __name__ == "__main__":
    main()
