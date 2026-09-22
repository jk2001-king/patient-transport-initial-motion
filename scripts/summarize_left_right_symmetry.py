#!/usr/bin/env python3
"""Compare mirrored left/right front-axle initial-state experiments."""

import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt


RESULTS = Path("results/ros_reference_path")
RUNS = (
    ("left simultaneous", "simultaneous_front45_coordinated_counterphase", "tab:red", "--"),
    ("right simultaneous", "right_simultaneous_frontm45_coordinated_counterphase", "tab:orange", "--"),
    ("left adaptive", "adaptive_front45_coordinated_counterphase", "tab:blue", "-"),
    ("right adaptive", "right_adaptive_frontm45_coordinated_counterphase", "tab:green", "-"),
)


def load(stem):
    summary = json.loads((RESULTS / f"{stem}.json").read_text(encoding="utf-8"))
    samples = list(csv.DictReader((RESULTS / f"{stem}.csv").open(encoding="utf-8")))
    for sample in samples:
        for key, value in sample.items():
            sample[key] = float(value)
    summary["elapsed_s"] = samples[-1]["elapsed_s"]
    return summary, samples


def main():
    loaded = [(label, *load(stem), color, linestyle) for label, stem, color, linestyle in RUNS]
    output_rows = []
    for label, summary, _, _, _ in loaded:
        output_rows.append(
            {
                "run": label,
                "success": not summary["timed_out"],
                "elapsed_s": summary["elapsed_s"],
                "rmse_cross_track_m": summary["rmse_cross_track_m"],
                "max_cross_track_m": summary["max_cross_track_m"],
                "final_goal_error_m": summary["final_goal_error_m"],
                "initial_cross_track_iae_m_s": summary["initial_cross_track_iae_m_s"],
                "initial_max_yaw_disturbance_deg": summary["initial_max_yaw_disturbance_deg"],
            }
        )
    output_csv = RESULTS / "left_right_front45_symmetry.csv"
    with output_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(output_rows[0]))
        writer.writeheader()
        writer.writerows(output_rows)

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    for label, _, samples, color, linestyle in loaded:
        x0, y0 = samples[0]["x_m"], samples[0]["y_m"]
        axes[0].plot(
            [sample["x_m"] - x0 for sample in samples],
            [sample["y_m"] - y0 for sample in samples],
            label=label,
            color=color,
            linestyle=linestyle,
        )
        axes[1].plot(
            [sample["elapsed_s"] for sample in samples],
            [100.0 * sample["cross_track_error_m"] for sample in samples],
            label=label,
            color=color,
            linestyle=linestyle,
        )
    axes[0].set(title="Mirrored ground-truth trajectories", xlabel="x [m]", ylabel="y [m]")
    axes[1].set(title="Cross-track error", xlabel="time [s]", ylabel="error [cm]")
    for axis in axes:
        axis.grid(True)
        axis.legend(fontsize=8)
    fig.tight_layout()
    output_plot = RESULTS / "left_right_front45_symmetry.png"
    fig.savefig(output_plot, dpi=180)
    plt.close(fig)
    print(f"Summary CSV: {output_csv}")
    print(f"Summary plot: {output_plot}")


if __name__ == "__main__":
    main()
