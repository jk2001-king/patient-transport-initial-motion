#!/usr/bin/env python3
"""Summarize repeated Nav2 initial-motion experiments for reporting."""

import csv
import json
import statistics as stats
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / "results" / "nav2_initial_motion"

CASES = {
    "Coordinated\n+25°/+25°": {
        "simultaneous": "nav2_same25_simultaneous_run0{}.json",
        "proposed": "nav2_same25_proposed_run0{}.json",
    },
    "Front only\n+40°/0°": {
        "simultaneous": "nav2_frontonly_p40_simultaneous_modelaware_run0{}.json",
        "proposed": "nav2_frontonly_p40_proposed_modelaware_run0{}.json",
    },
    "Rear only\n0°/+40°": {
        "simultaneous": "nav2_rearonly_p40_simultaneous_modelaware_run0{}.json",
        "proposed": "nav2_rearonly_p40_proposed_modelaware_run0{}.json",
    },
}

METRICS = {
    "rmse_lateral_m": ("Lateral RMSE", 1000.0, "mm"),
    "max_abs_lateral_m": ("Max. lateral deviation", 1000.0, "mm"),
    "max_abs_yaw_disturbance_deg": ("Max. yaw disturbance", 1.0, "deg"),
    "duration_s": ("Navigation time", 1.0, "s"),
}


def load_repeats(pattern):
    return [json.loads((RESULTS / pattern.format(i)).read_text()) for i in range(1, 4)]


def mean_sd(rows, key):
    values = [float(row[key]) for row in rows]
    return stats.mean(values), stats.stdev(values)


def write_summary(all_rows):
    output = RESULTS / "nav2_configuration_repeats_summary.csv"
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(all_rows[0]))
        writer.writeheader()
        writer.writerows(all_rows)
    return output


def plot_summary(data):
    labels = list(data)
    methods = ("simultaneous", "proposed")
    colors = {"simultaneous": "#d62728", "proposed": "#1f77b4"}
    display = {"simultaneous": "Simultaneous", "proposed": "Proposed"}
    fig, axes = plt.subplots(2, 2, figsize=(12.0, 7.2), constrained_layout=True)
    x = np.arange(len(labels))
    width = 0.36
    for axis, (key, (title, scale, unit)) in zip(axes.flat, METRICS.items()):
        for index, method in enumerate(methods):
            means, errors = [], []
            for label in labels:
                mean, sd = mean_sd(data[label][method], key)
                means.append(mean * scale)
                errors.append(sd * scale)
            axis.bar(
                x + (index - 0.5) * width,
                means,
                width,
                yerr=errors,
                capsize=4,
                color=colors[method],
                label=display[method],
                alpha=0.88,
            )
        axis.set_title(title)
        axis.set_ylabel(unit)
        axis.set_xticks(x, labels)
        axis.grid(axis="y", alpha=0.25)
    axes[0, 0].legend(frameon=False)
    fig.suptitle("Nav2 initial-motion control: mean ± SD (n=3)", fontsize=15)
    output = RESULTS / "nav2_configuration_repeats_summary.png"
    fig.savefig(output, dpi=220)
    plt.close(fig)
    return output


def read_csv(name):
    with (RESULTS / name).open(encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def plot_transients():
    cases = [
        (
            "Front only +40°/0°",
            "nav2_frontonly_p40_simultaneous_modelaware_run01.csv",
            "nav2_frontonly_p40_proposed_modelaware_run01.csv",
        ),
        (
            "Rear only 0°/+40°",
            "nav2_rearonly_p40_simultaneous_modelaware_run01.csv",
            "nav2_rearonly_p40_proposed_modelaware_run01.csv",
        ),
    ]
    fig, axes = plt.subplots(2, 2, figsize=(12.0, 7.0), constrained_layout=True)
    for row_index, (title, simultaneous_name, proposed_name) in enumerate(cases):
        for name, label, color in (
            (simultaneous_name, "Simultaneous", "#d62728"),
            (proposed_name, "Proposed", "#1f77b4"),
        ):
            rows = [r for r in read_csv(name) if float(r["elapsed_s"]) <= 8.0]
            time = [float(r["elapsed_s"]) for r in rows]
            lateral = [1000.0 * float(r["lateral_error_m"]) for r in rows]
            yaw = [float(r["yaw_disturbance_deg"]) for r in rows]
            axes[row_index, 0].plot(time, lateral, color=color, label=label, linewidth=2)
            axes[row_index, 1].plot(time, yaw, color=color, label=label, linewidth=2)
        axes[row_index, 0].set_title(f"{title}: lateral deviation")
        axes[row_index, 1].set_title(f"{title}: yaw disturbance")
        axes[row_index, 0].set_ylabel("mm")
        axes[row_index, 1].set_ylabel("deg")
        for axis in axes[row_index]:
            axis.set_xlabel("Time (s)")
            axis.grid(alpha=0.25)
            axis.legend(frameon=False)
    output = RESULTS / "nav2_front_rear_transient_comparison.png"
    fig.savefig(output, dpi=220)
    plt.close(fig)
    return output


def main():
    data = {
        label: {
            method: load_repeats(pattern)
            for method, pattern in methods.items()
        }
        for label, methods in CASES.items()
    }
    rows = []
    for label, methods in data.items():
        for method, repeats in methods.items():
            row = {
                "condition": label.replace("\n", " "),
                "method": method,
                "success_count": sum(item["success"] for item in repeats),
                "repeat_count": len(repeats),
            }
            for key in METRICS:
                mean, sd = mean_sd(repeats, key)
                row[f"{key}_mean"] = mean
                row[f"{key}_sd"] = sd
            rows.append(row)
    print(f"Summary CSV: {write_summary(rows)}")
    print(f"Summary plot: {plot_summary(data)}")
    print(f"Transient plot: {plot_transients()}")


if __name__ == "__main__":
    main()
