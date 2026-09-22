#!/usr/bin/env python3
"""Compare three initial-motion policies for one initial steering condition."""

import argparse
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt


RESULTS = Path("results/ros_reference_path")


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--simultaneous", default="simultaneous_front45_coordinated_counterphase")
    parser.add_argument("--steer-then-drive", default="proposed_front45_coordinated_counterphase")
    parser.add_argument("--scaled", default="scaled10_front45_coordinated_counterphase")
    parser.add_argument("--output-prefix", default="front45_three_policy")
    parser.add_argument("--title", default="Front +45 deg initial-state policy comparison")
    return parser.parse_args()


def load(stem):
    summary = json.loads((RESULTS / f"{stem}.json").read_text(encoding="utf-8"))
    samples = list(csv.DictReader((RESULTS / f"{stem}.csv").open(encoding="utf-8")))
    for sample in samples:
        for key, value in sample.items():
            sample[key] = float(value)
    summary["elapsed_s"] = samples[-1]["elapsed_s"]
    return summary, samples


def main():
    args = parse_args()
    policies = (
        ("simultaneous", args.simultaneous, "tab:red"),
        ("steer-then-drive", args.steer_then_drive, "tab:blue"),
        ("scaled10", args.scaled, "tab:green"),
    )
    loaded = [(label, *load(stem), color) for label, stem, color in policies]
    output_rows = []
    for label, summary, _, _ in loaded:
        output_rows.append(
            {
                "policy": label,
                "success": not summary["timed_out"],
                "elapsed_s": summary["elapsed_s"],
                "rmse_cross_track_m": summary["rmse_cross_track_m"],
                "max_cross_track_m": summary["max_cross_track_m"],
                "final_goal_error_m": summary["final_goal_error_m"],
                "initial_cross_track_iae_m_s": summary["initial_cross_track_iae_m_s"],
                "initial_max_yaw_disturbance_deg": summary["initial_max_yaw_disturbance_deg"],
                "initial_peak_steering_effort": summary["initial_peak_steering_effort"],
                "curvature_saturation_fraction": summary["curvature_saturation_fraction"],
            }
        )
    output_csv = RESULTS / f"{args.output_prefix}_comparison.csv"
    with output_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(output_rows[0]))
        writer.writeheader()
        writer.writerows(output_rows)

    fig, axes = plt.subplots(2, 2, figsize=(13, 8))
    for label, summary, samples, color in loaded:
        x0, y0 = samples[0]["x_m"], samples[0]["y_m"]
        axes[0, 0].plot(
            [row["x_m"] - x0 for row in samples],
            [row["y_m"] - y0 for row in samples],
            label=label,
            color=color,
        )
        axes[0, 1].plot(
            [row["elapsed_s"] for row in samples],
            [100.0 * row["cross_track_error_m"] for row in samples],
            label=label,
            color=color,
        )
    axes[0, 0].set(title="Ground-truth trajectory", xlabel="x [m]", ylabel="y [m]")
    axes[0, 1].set(title="Cross-track error", xlabel="time [s]", ylabel="error [cm]")
    for axis in axes[0]:
        axis.grid(True)
        axis.legend()

    labels = [item[0] for item in loaded]
    summaries = [item[1] for item in loaded]
    metric_specs = (
        ("RMSE [mm]", "rmse_cross_track_m", 1000.0),
        ("initial IAE [mm s]", "initial_cross_track_iae_m_s", 1000.0),
        ("yaw disturbance [deg]", "initial_max_yaw_disturbance_deg", 1.0),
    )
    x = list(range(len(metric_specs)))
    width = 0.24
    for policy_index, (label, summary, _, color) in enumerate(loaded):
        values = [summary[key] * scale for _, key, scale in metric_specs]
        bars = axes[1, 0].bar(
            [position + (policy_index - 1) * width for position in x],
            values,
            width=width,
            label=label,
            color=color,
        )
        if summary["timed_out"]:
            for bar in bars:
                bar.set_hatch("//")
    axes[1, 0].set_xticks(x, [item[0] for item in metric_specs])
    axes[1, 0].set_title("Error and disturbance")
    axes[1, 0].grid(True, axis="y", alpha=0.4)
    axes[1, 0].legend()

    elapsed_bars = axes[1, 1].bar(
        labels,
        [summary["elapsed_s"] for summary in summaries],
        color=[item[3] for item in loaded],
    )
    for bar, summary in zip(elapsed_bars, summaries):
        if summary["timed_out"]:
            bar.set_hatch("//")
            axes[1, 1].text(
                bar.get_x() + bar.get_width() / 2,
                bar.get_height(),
                "timeout",
                ha="center",
                va="bottom",
            )
    axes[1, 1].set(title="Run duration", ylabel="elapsed time [s]")
    axes[1, 1].grid(True, axis="y", alpha=0.4)

    fig.suptitle(args.title)
    fig.tight_layout()
    output_plot = RESULTS / f"{args.output_prefix}_comparison.png"
    fig.savefig(output_plot, dpi=180)
    plt.close(fig)
    print(f"Comparison CSV: {output_csv}")
    print(f"Comparison plot: {output_plot}")


if __name__ == "__main__":
    main()
