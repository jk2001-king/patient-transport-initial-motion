import pytest

from patient_initial_stability.reference_path_tracker import (
    configuration_aware_curvature,
    make_straight_arc_path,
    metric_window,
    motion_start_time,
    sideslip_beta,
)


def test_straight_arc_path_geometry():
    path = make_straight_arc_path(0.2, 0.4, 0.5, spacing_m=0.01)
    end = path[-1]
    assert end.s == pytest.approx(0.6)
    assert end.x == pytest.approx(0.2 + 2.0 * 0.1986693308, abs=1.0e-8)
    assert end.y == pytest.approx(2.0 * (1.0 - 0.9800665778), abs=1.0e-8)
    assert end.yaw == pytest.approx(0.2)


def test_sideslip_sign_changes_between_front_and_rear_steering():
    front = sideslip_beta("front_only", 0.5, 0.645, 0.645)
    rear = sideslip_beta("rear_only", 0.5, 0.645, 0.645)
    coordinated = sideslip_beta("coordinated_counterphase", 0.5, 0.645, 0.645)
    assert front > 0.0
    assert rear < 0.0
    assert coordinated == 0.0
    assert front == pytest.approx(-rear)


def test_aware_curvature_countersteers_rear_to_create_leftward_velocity():
    curvature = configuration_aware_curvature(
        target_bearing_rad=0.10,
        target_distance_m=0.12,
        configuration="rear_only",
        maximum_curvature=0.7,
        front_length_m=0.645,
        rear_length_m=0.645,
    )
    assert curvature < 0.0
    beta = sideslip_beta("rear_only", curvature, 0.645, 0.645)
    assert beta > 0.0
    assert abs(curvature) <= 0.7


def test_motion_relative_metric_window_starts_after_alignment_delay():
    rows = [
        {"elapsed_s": 0.0, "x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0, "cross_track_error_m": 0.0},
        {"elapsed_s": 2.0, "x_m": 0.0005, "y_m": 0.0, "yaw_rad": 0.0, "cross_track_error_m": 0.0},
        {"elapsed_s": 3.0, "x_m": 0.0020, "y_m": 0.0, "yaw_rad": 0.0, "cross_track_error_m": 0.01},
        {"elapsed_s": 4.0, "x_m": 0.0100, "y_m": 0.0, "yaw_rad": 0.1, "cross_track_error_m": 0.02},
        {"elapsed_s": 8.0, "x_m": 0.0200, "y_m": 0.0, "yaw_rad": 0.2, "cross_track_error_m": 0.03},
    ]
    start = motion_start_time(rows, 0.001)
    metrics = metric_window(rows, start, 4.0)
    assert start == pytest.approx(3.0)
    assert metrics["samples"] == 2
    assert metrics["cross_track_iae_m_s"] == pytest.approx(0.015)
    assert metrics["max_yaw_disturbance_deg"] == pytest.approx(5.72957795)
