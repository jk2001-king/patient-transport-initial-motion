#!/usr/bin/env python3
"""Plot the representative +45/+45 configuration-adaptive comparison."""

import csv
import json
import math
from pathlib import Path

import matplotlib.pyplot as plt


RESULTS = Path("results/ros_reference_path")
BASELINE_STEM = "initial_crab45_coordinated_counterphase"
ADAPTIVE_STEM = "cfg_coordinated_configadaptive_same45_coordinated_counterphase"
OUTPUT = RESULTS / "coordinated_same45_configuration_adaptive_trajectory.png"


def load_samples(stem):
    with (RESULTS / f"{stem}.csv").open(encoding="utf-8") as stream:
        return [
            {key: float(value) for key, value in row.items()}
            for row in csv.DictReader(stream)
        ]


def load_summary(stem):
    return json.loads((RESULTS / f"{stem}.json").read_text(encoding="utf-8"))


def localize(rows):
    x0 = rows[0]["x_m"]
    y0 = rows[0]["y_m"]
    yaw0 = rows[0]["yaw_rad"]
    cosine = math.cos(yaw0)
    sine = math.sin(yaw0)
    points = []
    for row in rows:
        dx = row["x_m"] - x0
        dy = row["y_m"] - y0
        points.append((cosine * dx + sine * dy, -sine * dx + cosine * dy))
    return points


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
    baseline = load_samples(BASELINE_STEM)
    adaptive = load_samples(ADAPTIVE_STEM)
    baseline_summary = load_summary(BASELINE_STEM)
    adaptive_summary = load_summary(ADAPTIVE_STEM)

    reference = reference_path()
    baseline_xy = localize(baseline)
    adaptive_xy = localize(adaptive)

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.8))

    axes[0].plot(
        [point[0] for point in reference],
        [point[1] for point in reference],
        "k--",
        linewidth=2.0,
        label="reference",
    )
    axes[0].plot(
        [point[0] for point in baseline_xy],
        [point[1] for point in baseline_xy],
        color="tab:red",
        linewidth=2.0,
        label="simultaneous",
    )
    axes[0].plot(
        [point[0] for point in adaptive_xy],
        [point[1] for point in adaptive_xy],
        color="tab:blue",
        linewidth=2.0,
        label="configuration-adaptive",
    )
    axes[0].set_title("Ground-truth trajectory")
    axes[0].set_xlabel("longitudinal x [m]")
    axes[0].set_ylabel("lateral y [m]")
    axes[0].axis("equal")
    axes[0].grid(True, alpha=0.4)
    axes[0].legend()

    axes[1].plot(
        [row["elapsed_s"] for row in baseline],
        [1000.0 * abs(row["cross_track_error_m"]) for row in baseline],
        color="tab:red",
        linewidth=2.0,
        label="simultaneous",
    )
    axes[1].plot(
        [row["elapsed_s"] for row in adaptive],
        [1000.0 * abs(row["cross_track_error_m"]) for row in adaptive],
        color="tab:blue",
        linewidth=2.0,
        label="configuration-adaptive",
    )
    axes[1].set_title("Absolute cross-track error")
    axes[1].set_xlabel("elapsed time [s]")
    axes[1].set_ylabel("error [mm]")
    axes[1].grid(True, alpha=0.4)
    axes[1].legend()

    baseline_rmse = 1000.0 * baseline_summary["rmse_cross_track_m"]
    adaptive_rmse = 1000.0 * adaptive_summary["rmse_cross_track_m"]
    baseline_max = 1000.0 * baseline_summary["max_cross_track_m"]
    adaptive_max = 1000.0 * adaptive_summary["max_cross_track_m"]
    fig.suptitle(
        "Coordinated +45°/+45° initial steering\n"
        f"RMSE: {baseline_rmse:.1f} → {adaptive_rmse:.2f} mm, "
        f"max error: {baseline_max:.1f} → {adaptive_max:.2f} mm"
    )
    fig.tight_layout()
    fig.savefig(OUTPUT, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved: {OUTPUT}")


if __name__ == "__main__":
    main()
