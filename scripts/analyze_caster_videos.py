#!/usr/bin/env python3
"""Track the body-centre marker in all caster Go/No-Go videos.

The blue tape on the rigid rear body centre and the two blue endpoints of the
near 1 m transverse ground line are detected.  Normalising by the endpoint
spacing cancels most hand-held camera translation and scale changes.  Because
the body marker is above the floor plane, the result is intentionally called a
video proxy rather than a metric ground-plane trajectory.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/patient-caster-video-matplotlib")

import cv2
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.ndimage import median_filter


def parse_name(name: str) -> tuple[str, str, float]:
    stem = name.rsplit("_run", 1)[0]
    if stem.endswith("_fixed"):
        return stem[:-6], "fixed", 4.0
    if stem.endswith("_long"):
        return stem[:-5], "long", 10.0
    return stem, "normal", 4.0


def components(frame: np.ndarray) -> list[tuple[float, float, int, int, int]]:
    blue, green, red = cv2.split(frame)
    blue_i, green_i, red_i = blue.astype(np.int16), green.astype(np.int16), red.astype(np.int16)
    mask = (
        (blue_i > red_i + 10)
        & (green_i > red_i + 3)
        & (blue > 85)
    ).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
    count, _, stats, centroids = cv2.connectedComponentsWithStats(mask)
    result = []
    for index in range(1, count):
        x, y, width, height, area = stats[index]
        if area >= 6:
            result.append((*centroids[index], int(width), int(height), int(area)))
    return result


def detect_markers(frame: np.ndarray) -> tuple[np.ndarray | None, np.ndarray | None, np.ndarray | None]:
    height, width = frame.shape[:2]
    found = components(frame)
    body_candidates = [
        item for item in found
        if 0.37 * width < item[0] < 0.63 * width
        and 0.35 * height < item[1] < 0.72 * height
        and item[4] >= 18 and item[3] >= 7 and item[3] > 1.3 * item[2]
    ]
    body = max(body_candidates, key=lambda item: item[4] * item[3] / max(item[2], 1), default=None)
    if body is None:
        return None, None, None
    body_point = np.asarray(body[:2], dtype=float)
    endpoint_candidates = [
        item for item in found
        if item[1] > body_point[1] + 0.055 * height
        and item[1] < 0.78 * height and item[4] >= 18
    ]
    left = [item for item in endpoint_candidates if item[0] < 0.47 * width]
    right = [item for item in endpoint_candidates if item[0] > 0.53 * width]
    left_item = max(left, key=lambda item: item[4], default=None)
    right_item = max(right, key=lambda item: item[4], default=None)
    if left_item is None or right_item is None:
        return body_point, None, None
    return body_point, np.asarray(left_item[:2]), np.asarray(right_item[:2])


def interpolate_valid(time: np.ndarray, values: np.ndarray) -> np.ndarray:
    valid = np.isfinite(values)
    if np.count_nonzero(valid) < 2:
        return np.full_like(values, np.nan)
    result = np.interp(time, time[valid], values[valid])
    # Do not fabricate long marker-loss intervals.
    indices = np.flatnonzero(valid)
    for first, second in zip(indices[:-1], indices[1:]):
        if time[second] - time[first] > 1.0:
            result[first + 1 : second] = np.nan
    return result


def estimate_centerline_slope(
    frame: np.ndarray, body: np.ndarray, left: np.ndarray, right: np.ndarray
) -> float:
    """Estimate dx/dy of the taped centreline below the body marker."""
    midpoint = (left + right) / 2.0
    y0 = int(body[1] + 0.25 * (midpoint[1] - body[1]))
    y1 = int(midpoint[1] - 5)
    x0 = int(midpoint[0])
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    rows = []
    for y in range(max(0, y0), min(gray.shape[0], y1)):
        lo, hi = max(0, x0 - 28), min(gray.shape[1], x0 + 29)
        xs = np.flatnonzero(gray[y, lo:hi] < 75) + lo
        if len(xs) >= 3:
            # The tape is the dark cluster closest to the reference midpoint.
            close = xs[np.abs(xs - x0) <= 16]
            if len(close) >= 3:
                rows.append((y, float(np.median(close))))
    if len(rows) < 8:
        return 0.0
    values = np.asarray(rows)
    slope, _ = np.polyfit(values[:, 0], values[:, 1], 1)
    return float(np.clip(slope, -0.25, 0.25))


def analyse_video(path: Path, output: Path, sample_hz: float) -> tuple[dict[str, object], dict[str, np.ndarray]]:
    state, method, expected = parse_name(path.stem)
    capture = cv2.VideoCapture(str(path))
    fps = float(capture.get(cv2.CAP_PROP_FPS))
    frame_count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    stride = max(1, round(fps / sample_hz))
    times, bodies, lefts, rights = [], [], [], []
    index = 0
    while True:
        ok, full = capture.read()
        if not ok:
            break
        if index % stride == 0:
            frame = cv2.resize(full, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
            body, left, right = detect_markers(frame)
            times.append(index / fps)
            bodies.append(body if body is not None else [np.nan, np.nan])
            lefts.append(left if left is not None else [np.nan, np.nan])
            rights.append(right if right is not None else [np.nan, np.nan])
        index += 1
    capture.release()

    time = np.asarray(times)
    body = np.asarray(bodies)
    left = np.asarray(lefts)
    right = np.asarray(rights)
    for array in (body, left, right):
        for column in range(2):
            array[:, column] = interpolate_valid(time, array[:, column])
    near_mid = (left + right) / 2.0
    near_width = np.linalg.norm(right - left, axis=1)
    valid = np.isfinite(body[:, 0]) & np.isfinite(near_width) & (near_width > 50)
    norm_x = np.full(len(time), np.nan)
    norm_y = np.full(len(time), np.nan)
    norm_x[valid] = (body[valid, 0] - near_mid[valid, 0]) / near_width[valid]
    norm_y[valid] = (body[valid, 1] - near_mid[valid, 1]) / near_width[valid]
    norm_x = interpolate_valid(time, norm_x)
    norm_y = interpolate_valid(time, norm_y)
    smooth_x = median_filter(norm_x, size=5, mode="nearest")
    smooth_y = median_filter(norm_y, size=5, mode="nearest")

    # Locate the fixed-duration window with the largest approach toward camera.
    steps = max(1, round(expected / np.median(np.diff(time))))
    gains = smooth_y[steps:] - smooth_y[:-steps]
    finite = np.isfinite(gains)
    if not np.any(finite):
        start_index = 0
    else:
        start_index = int(np.nanargmax(np.where(finite, gains, np.nan)))
    end_index = min(len(time) - 1, start_index + steps)
    start_time, end_time = float(time[start_index]), float(time[end_index])
    window = (time >= start_time) & (time <= end_time)
    capture = cv2.VideoCapture(str(path))
    capture.set(cv2.CAP_PROP_POS_MSEC, start_time * 1000.0)
    ok, start_frame_full = capture.read()
    capture.release()
    if ok and np.all(np.isfinite([*body[start_index], *left[start_index], *right[start_index]])):
        start_frame = cv2.resize(start_frame_full, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
        centerline_slope = estimate_centerline_slope(
            start_frame, body[start_index], left[start_index], right[start_index]
        )
    else:
        centerline_slope = 0.0
    centerline_relative_offset = (
        body[:, 0] - near_mid[:, 0]
        - centerline_slope * (body[:, 1] - near_mid[:, 1])
    ) / near_width
    centerline_relative_offset = median_filter(centerline_relative_offset, size=5, mode="nearest")
    lateral = centerline_relative_offset - centerline_relative_offset[start_index]
    approach = smooth_y - smooth_y[start_index]
    detection_fraction = float(np.mean(valid))
    # Endpoint motion is a direct indicator of hand-held camera instability.
    camera_mid_motion_px = float(np.nanmax(np.linalg.norm(near_mid[window] - near_mid[start_index], axis=1)))
    camera_scale_change = float(
        np.nanmax(np.abs(near_width[window] / near_width[start_index] - 1.0))
    )
    # Normalised by a known 1 m line, but not a true planar metric because the
    # tracked body marker is elevated.  Keep the conservative proxy label.
    max_lateral_proxy = float(np.nanmax(np.abs(lateral[window])))
    final_lateral_proxy = float(lateral[end_index])
    qc = (
        detection_fraction >= 0.75
        and camera_scale_change <= 0.02
        and camera_mid_motion_px <= 3.0
    )
    row: dict[str, object] = {
        "run": path.stem,
        "initial_state": state,
        "method": method,
        "video_duration_s": frame_count / fps,
        "fps": fps,
        "marker_detection_fraction": detection_fraction,
        "motion_window_start_s": start_time,
        "motion_window_end_s": end_time,
        "approach_proxy_change": float(approach[end_index]),
        "max_abs_lateral_nearline_proxy_m": max_lateral_proxy,
        "final_lateral_nearline_proxy_m": final_lateral_proxy,
        "camera_near_mid_max_motion_px_halfres": camera_mid_motion_px,
        "camera_near_width_max_relative_change": camera_scale_change,
        "fitted_centerline_dx_dy": centerline_slope,
        "video_proxy_qc_pass": qc,
    }

    # Save visual start/end verification at full resolution.
    capture = cv2.VideoCapture(str(path))
    panels = []
    for label, sample_index in (("START", start_index), ("END", end_index)):
        capture.set(cv2.CAP_PROP_POS_MSEC, float(time[sample_index]) * 1000.0)
        ok, frame = capture.read()
        if not ok:
            continue
        scale = 2.0
        for point, color in (
            (body[sample_index], (255, 0, 255)),
            (left[sample_index], (0, 255, 0)),
            (right[sample_index], (0, 255, 0)),
        ):
            if np.all(np.isfinite(point)):
                cv2.circle(frame, tuple(np.round(scale * point).astype(int)), 12, color, 3)
        cv2.putText(frame, f"{label} {time[sample_index]:.2f}s", (25, 55), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 0, 255), 3)
        frame = cv2.resize(frame, (360, 640))
        panels.append(frame)
    capture.release()
    if len(panels) == 2:
        cv2.imwrite(str(output / "overlays" / f"{path.stem}_start_end.jpg"), cv2.hconcat(panels))

    series = {
        "time": time,
        "norm_x": norm_x,
        "norm_y": norm_y,
        "lateral": lateral,
        "approach": approach,
        "near_mid_x": near_mid[:, 0],
        "near_mid_y": near_mid[:, 1],
        "near_width": near_width,
        "centerline_relative_offset": centerline_relative_offset,
    }
    return row, series


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--videos", type=Path, default=Path("motor_bag/caster_go_no_go/bag_video"))
    parser.add_argument("--output", type=Path, default=Path("results/real_caster_go_no_go/video_analysis"))
    parser.add_argument("--sample-hz", type=float, default=5.0)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "overlays").mkdir(exist_ok=True)
    (args.output / "timeseries").mkdir(exist_ok=True)
    paths = sorted(args.videos.glob("*.mp4"))
    rows, all_series = [], {}
    for path in paths:
        row, series = analyse_video(path, args.output, args.sample_hz)
        rows.append(row)
        all_series[path.stem] = series
        with (args.output / "timeseries" / f"{path.stem}_video_track.csv").open(
            "w", newline="", encoding="utf-8"
        ) as stream:
            writer = csv.writer(stream)
            writer.writerow(
                ["video_time_s", "body_x_normalized", "body_y_normalized", "approach_proxy", "lateral_proxy_m"]
            )
            writer.writerows(
                zip(series["time"], series["norm_x"], series["norm_y"], series["approach"], series["lateral"])
            )
        print(
            f"{path.stem:20s} qc={str(row['video_proxy_qc_pass']):5s} "
            f"window={row['motion_window_start_s']:.1f}-{row['motion_window_end_s']:.1f}s "
            f"lat_proxy={1000*float(row['max_abs_lateral_nearline_proxy_m']):.1f}mm"
        )
    with (args.output / "video_metrics_per_run.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    bag_metrics_path = args.output.parent / "bag_analysis" / "metrics_per_run.csv"
    if bag_metrics_path.exists():
        with bag_metrics_path.open(encoding="utf-8") as stream:
            bag_rows = {row["run"]: row for row in csv.DictReader(stream)}
        combined = []
        for video_row in rows:
            bag_row = bag_rows.get(str(video_row["run"]), {})
            merged = {**bag_row, **video_row}
            try:
                merged["initial_imu_qc_pass"] = (
                    int(float(merged["imu_count"])) == 3
                    and float(merged["command_active_duration_s"]) >= 1.9
                    and float(merged["imu_pre_max_abs_integrated_yaw_2s_deg"]) <= 0.06
                )
                merged["active_steering_confound"] = max(
                    abs(float(merged["steer_front_max_abs_deg"])),
                    abs(float(merged["steer_rear_max_abs_deg"])),
                ) > 0.5
            except (KeyError, TypeError, ValueError):
                merged["initial_imu_qc_pass"] = False
                merged["active_steering_confound"] = False
            merged["external_localization_usable"] = False
            combined.append(merged)
        with (args.output.parent / "combined_run_qc.csv").open(
            "w", newline="", encoding="utf-8"
        ) as stream:
            writer = csv.DictWriter(stream, fieldnames=list(combined[0]))
            writer.writeheader()
            writer.writerows(combined)

    fig, axes = plt.subplots(2, 1, figsize=(11, 8), sharex=False)
    colors = {"A": "tab:green", "BL": "tab:blue", "BR": "tab:orange"}
    for row in rows:
        series = all_series[str(row["run"])]
        start = float(row["motion_window_start_s"])
        elapsed = series["time"] - start
        mask = (elapsed >= -1) & (elapsed <= 6)
        alpha = 0.75 if row["video_proxy_qc_pass"] else 0.2
        axes[0].plot(elapsed[mask], series["approach"][mask], color=colors[str(row["initial_state"])], alpha=alpha, label=str(row["run"]))
        axes[1].plot(elapsed[mask], 1000 * series["lateral"][mask], color=colors[str(row["initial_state"])], alpha=alpha)
    axes[0].set_ylabel("approach proxy [near-line widths]")
    axes[1].set_ylabel("lateral proxy [mm]")
    axes[1].set_xlabel("time from detected motion-window start [s]")
    for axis in axes:
        axis.grid(True, alpha=0.25)
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="center right", fontsize=7)
    fig.tight_layout(rect=(0, 0, 0.8, 1))
    fig.savefig(args.output / "video_marker_tracks.png", dpi=180)
    plt.close(fig)
    print(f"Analysed {len(rows)} videos -> {args.output}")


if __name__ == "__main__":
    main()
