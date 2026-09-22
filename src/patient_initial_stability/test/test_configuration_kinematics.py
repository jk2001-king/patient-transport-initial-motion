import math

import pytest

from patient_initial_stability.configuration_controller import (
    configuration_targets,
    configuration_risk_alignment_error,
    configuration_alignment_severity,
    configuration_aware_curvature_from_common_command,
    coordinated_alignment_components,
    initial_alignment_speed_scale,
    leading_axle_alignment_error,
)


def test_three_configurations_match_half_inverse_meter_curvature():
    common = dict(
        linear_x=0.1,
        angular_z=0.05,
        wheelbase_m=1.29,
        front_length_m=0.645,
        rear_length_m=0.645,
        wheel_radius_m=0.075,
        steering_limit_rad=math.pi / 4.0,
    )

    front = configuration_targets(configuration="front_only", **common)
    rear = configuration_targets(configuration="rear_only", **common)
    coordinated = configuration_targets(configuration="coordinated_counterphase", **common)

    assert math.degrees(front.front_steering_rad) == pytest.approx(32.822016, abs=1.0e-5)
    assert front.rear_steering_rad == 0.0
    assert rear.front_steering_rad == 0.0
    assert math.degrees(rear.rear_steering_rad) == pytest.approx(-32.822016, abs=1.0e-5)
    assert math.degrees(coordinated.front_steering_rad) == pytest.approx(17.874511, abs=1.0e-5)
    assert math.degrees(coordinated.rear_steering_rad) == pytest.approx(-17.874511, abs=1.0e-5)
    assert front.front_wheel_rad_s > 0.0
    assert front.rear_wheel_rad_s < 0.0


def test_initial_alignment_speed_scale():
    zero_speed_error = math.radians(45.0)
    assert initial_alignment_speed_scale(0.0, zero_speed_error) == 1.0
    assert initial_alignment_speed_scale(math.radians(22.5), zero_speed_error) == pytest.approx(0.5)
    assert initial_alignment_speed_scale(math.radians(45.0), zero_speed_error) == 0.0
    assert initial_alignment_speed_scale(math.radians(60.0), zero_speed_error) == 0.0


def test_leading_axle_alignment_error_tracks_motion_direction():
    front = math.radians(30.0)
    rear = math.radians(-20.0)
    assert leading_axle_alignment_error(front, rear, 0.1) == front
    assert leading_axle_alignment_error(front, rear, -0.1) == rear


def test_configuration_risk_alignment_error_uses_actuated_or_leading_axle():
    front = math.radians(30.0)
    rear = math.radians(-20.0)
    assert configuration_risk_alignment_error("front_only", front, rear, 0.1) == front
    assert configuration_risk_alignment_error("rear_only", front, rear, 0.1) == rear
    assert configuration_risk_alignment_error(
        "coordinated_counterphase", front, rear, 0.1
    ) == front
    assert configuration_risk_alignment_error(
        "coordinated_counterphase", front, rear, -0.1
    ) == rear


def test_coordinated_components_separate_same_and_counterphase_errors():
    common, counter = coordinated_alignment_components(
        math.radians(25.0), math.radians(25.0)
    )
    assert math.degrees(common) == pytest.approx(25.0)
    assert math.degrees(counter) == pytest.approx(0.0)
    common, counter = coordinated_alignment_components(
        math.radians(35.0), math.radians(-35.0)
    )
    assert math.degrees(common) == pytest.approx(0.0)
    assert math.degrees(counter) == pytest.approx(35.0)


def test_configuration_alignment_severity_uses_configuration_specific_thresholds():
    thresholds = dict(
        front_only_threshold_rad=math.radians(35.0),
        rear_only_threshold_rad=math.radians(35.0),
        coordinated_common_threshold_rad=math.radians(20.0),
        coordinated_counter_threshold_rad=math.radians(35.0),
    )
    assert configuration_alignment_severity(
        "front_only", math.radians(35.0), 0.0, **thresholds
    ) == pytest.approx(1.0)
    assert configuration_alignment_severity(
        "rear_only", 0.0, math.radians(35.0), **thresholds
    ) == pytest.approx(1.0)
    assert configuration_alignment_severity(
        "coordinated_counterphase",
        math.radians(25.0),
        math.radians(25.0),
        **thresholds,
    ) == pytest.approx(1.25)
    assert configuration_alignment_severity(
        "coordinated_counterphase",
        math.radians(35.0),
        math.radians(-35.0),
        **thresholds,
    ) == pytest.approx(1.0)


def test_configuration_aware_curvature_accounts_for_rear_steering_sideslip():
    common = dict(
        lookahead_m=0.20,
        wheelbase_m=1.29,
        front_length_m=0.645,
        rear_length_m=0.645,
        steering_limit_rad=math.pi / 4.0,
    )
    rear = configuration_aware_curvature_from_common_command(
        common_curvature=-0.5,
        configuration="rear_only",
        **common,
    )
    front = configuration_aware_curvature_from_common_command(
        common_curvature=-0.5,
        configuration="front_only",
        **common,
    )
    coordinated = configuration_aware_curvature_from_common_command(
        common_curvature=-0.5,
        configuration="coordinated_counterphase",
        **common,
    )

    assert rear > 0.0
    assert front < 0.0
    assert coordinated == pytest.approx(-0.5)
