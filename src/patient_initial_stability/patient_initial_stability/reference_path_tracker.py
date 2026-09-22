import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as PathMessage
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .configuration_controller import VALID_CONFIGURATIONS, clamp


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_odometry(message: Odometry) -> float:
    q = message.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def stamp_seconds(message: Odometry) -> float:
    return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1.0e-9


@dataclass(frozen=True)
class PathPoint:
    s: float
    x: float
    y: float
    yaw: float


def make_straight_arc_path(
    straight_length_m: float,
    arc_length_m: float,
    curvature_1_m: float,
    spacing_m: float = 0.01,
) -> List[PathPoint]:
    total = straight_length_m + arc_length_m
    count = max(2, int(math.ceil(total / spacing_m)) + 1)
    points = []
    radius = 1.0 / curvature_1_m
    for index in range(count):
        s = min(total, index * spacing_m)
        if s <= straight_length_m:
            x, y, yaw = s, 0.0, 0.0
        else:
            arc_s = s - straight_length_m
            angle = arc_s * curvature_1_m
            x = straight_length_m + radius * math.sin(angle)
            y = radius * (1.0 - math.cos(angle))
            yaw = angle
        points.append(PathPoint(s=s, x=x, y=y, yaw=yaw))
    if points[-1].s < total:
        arc_s = arc_length_m
        angle = arc_s * curvature_1_m
        points.append(
            PathPoint(
                s=total,
                x=straight_length_m + radius * math.sin(angle),
                y=radius * (1.0 - math.cos(angle)),
                yaw=angle,
            )
        )
    return points


def sideslip_beta(configuration: str, yaw_curvature: float, front_length_m: float, rear_length_m: float) -> float:
    """Return the body-center velocity sideslip implied by a steering configuration."""
    if configuration == "front_only":
        lateral_velocity_ratio = rear_length_m * yaw_curvature
    elif configuration == "rear_only":
        lateral_velocity_ratio = -front_length_m * yaw_curvature
    elif configuration == "coordinated_counterphase":
        lateral_velocity_ratio = 0.0
    else:
        raise ValueError(f"Unsupported steering configuration: {configuration}")
    return math.atan(lateral_velocity_ratio)


def configuration_aware_curvature(
    target_bearing_rad: float,
    target_distance_m: float,
    configuration: str,
    maximum_curvature: float,
    front_length_m: float,
    rear_length_m: float,
) -> float:
    """Solve the pure-pursuit relation in the velocity-direction frame."""
    distance = max(target_distance_m, 1.0e-6)
    candidates = [
        -maximum_curvature + 2.0 * maximum_curvature * index / 280.0
        for index in range(281)
    ]

    def residual(curvature: float) -> float:
        beta = sideslip_beta(configuration, curvature, front_length_m, rear_length_m)
        requested = 2.0 * math.sin(target_bearing_rad - beta) / distance
        return abs(curvature - requested)

    return min(candidates, key=residual)


def metric_window(rows, start_s: float, duration_s: float):
    """Return path-error metrics over a time window relative to ``start_s``."""
    selected = [
        row
        for row in rows
        if start_s <= row["elapsed_s"] <= start_s + duration_s
    ]
    errors = [row["cross_track_error_m"] for row in selected]
    iae = 0.0
    for previous, current in zip(selected, selected[1:]):
        dt = current["elapsed_s"] - previous["elapsed_s"]
        iae += 0.5 * dt * (
            previous["cross_track_error_m"] + current["cross_track_error_m"]
        )
    reference_yaw = selected[0]["yaw_rad"] if selected else math.nan
    return {
        "samples": len(selected),
        "rmse_cross_track_m": (
            math.sqrt(sum(error * error for error in errors) / len(errors))
            if errors
            else math.nan
        ),
        "max_cross_track_m": max(errors) if errors else math.nan,
        "cross_track_iae_m_s": iae,
        "max_yaw_disturbance_deg": (
            max(
                abs(math.degrees(wrap_angle(row["yaw_rad"] - reference_yaw)))
                for row in selected
            )
            if selected
            else math.nan
        ),
    }


def motion_start_time(rows, displacement_threshold_m: float) -> float:
    """Find the first sample whose chassis displacement exceeds the threshold."""
    if not rows:
        return math.nan
    x0, y0 = rows[0]["x_m"], rows[0]["y_m"]
    for row in rows:
        if math.hypot(row["x_m"] - x0, row["y_m"] - y0) >= displacement_threshold_m:
            return row["elapsed_s"]
    return math.nan


class ReferencePathTracker(Node):
    def __init__(self) -> None:
        super().__init__("reference_path_tracker")
        self.declare_parameter("configuration", "coordinated_counterphase")
        self.declare_parameter("tracking_model", "common_base")
        self.declare_parameter("run_id", "pilot01")
        self.declare_parameter("nominal_speed_m_s", 0.06)
        self.declare_parameter("straight_length_m", 0.20)
        self.declare_parameter("arc_length_m", 0.40)
        self.declare_parameter("arc_curvature_1_m", 0.50)
        self.declare_parameter("lookahead_m", 0.12)
        self.declare_parameter("max_curvature_1_m", 0.70)
        self.declare_parameter("goal_tolerance_m", 0.025)
        self.declare_parameter("maximum_run_time_s", 20.0)
        self.declare_parameter("front_length_m", 0.645)
        self.declare_parameter("rear_length_m", 0.645)
        self.declare_parameter("initial_metric_window_s", 4.0)
        self.declare_parameter("motion_start_displacement_m", 0.001)

        self.configuration = str(self.get_parameter("configuration").value)
        if self.configuration not in VALID_CONFIGURATIONS:
            raise ValueError(f"Invalid configuration: {self.configuration}")
        self.tracking_model = str(self.get_parameter("tracking_model").value)
        if self.tracking_model not in ("common_base", "configuration_aware"):
            raise ValueError(f"Invalid tracking_model: {self.tracking_model}")
        self.run_id = str(self.get_parameter("run_id").value)
        self.nominal_speed = float(self.get_parameter("nominal_speed_m_s").value)
        self.lookahead = float(self.get_parameter("lookahead_m").value)
        self.max_curvature = float(self.get_parameter("max_curvature_1_m").value)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance_m").value)
        self.maximum_run_time = float(self.get_parameter("maximum_run_time_s").value)
        self.front_length_m = float(self.get_parameter("front_length_m").value)
        self.rear_length_m = float(self.get_parameter("rear_length_m").value)
        self.initial_metric_window = float(self.get_parameter("initial_metric_window_s").value)
        self.motion_start_displacement = float(
            self.get_parameter("motion_start_displacement_m").value
        )
        self.local_path = make_straight_arc_path(
            float(self.get_parameter("straight_length_m").value),
            float(self.get_parameter("arc_length_m").value),
            float(self.get_parameter("arc_curvature_1_m").value),
        )

        self.cmd_pub = self.create_publisher(Twist, "/sim/cmd_vel", 10)
        self.configuration_pub = self.create_publisher(String, "/sim/steering_configuration", 10)
        self.path_pub = self.create_publisher(PathMessage, "/sim/reference_path", 10)
        self.create_subscription(Odometry, "/sim/ground_truth/odom", self._odom_callback, 10)
        self.create_subscription(JointState, "/joint_states", self._joint_callback, 10)
        self.create_timer(0.05, self._control_tick)

        self.latest_odom: Optional[Odometry] = None
        self.initial_odom: Optional[Odometry] = None
        self.actual_front = math.nan
        self.actual_rear = math.nan
        self.actual_front_effort = math.nan
        self.actual_rear_effort = math.nan
        self.start_time: Optional[float] = None
        self.settle_start: Optional[float] = None
        self.done = False
        self.timed_out = False
        self.rows = []
        self.world_path: List[PathPoint] = []
        self.get_logger().info(
            f"Waiting for odometry: configuration={self.configuration}, "
            f"tracking_model={self.tracking_model}, run_id={self.run_id}"
        )

    def _joint_callback(self, message: JointState) -> None:
        index = {name: i for i, name in enumerate(message.name)}
        if "front_steering_joint" in index and len(message.position) > index["front_steering_joint"]:
            self.actual_front = message.position[index["front_steering_joint"]] - math.pi / 2.0
        if "rear_steering_joint" in index and len(message.position) > index["rear_steering_joint"]:
            self.actual_rear = message.position[index["rear_steering_joint"]] - math.pi / 2.0
        if len(message.effort) > index.get("front_steering_joint", len(message.effort)):
            self.actual_front_effort = message.effort[index["front_steering_joint"]]
        if len(message.effort) > index.get("rear_steering_joint", len(message.effort)):
            self.actual_rear_effort = message.effort[index["rear_steering_joint"]]

    def _odom_callback(self, message: Odometry) -> None:
        self.latest_odom = message
        if self.initial_odom is None:
            self.initial_odom = message
            self.start_time = stamp_seconds(message) + 0.5
            self._make_world_path()
            self._publish_reference_path()

    def _make_world_path(self) -> None:
        origin = self.initial_odom.pose.pose.position
        yaw0 = yaw_from_odometry(self.initial_odom)
        c, s = math.cos(yaw0), math.sin(yaw0)
        self.world_path = [
            PathPoint(
                point.s,
                origin.x + c * point.x - s * point.y,
                origin.y + s * point.x + c * point.y,
                yaw0 + point.yaw,
            )
            for point in self.local_path
        ]

    def _publish_reference_path(self) -> None:
        message = PathMessage()
        message.header.frame_id = "odom"
        message.header.stamp = self.latest_odom.header.stamp
        for point in self.world_path:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = point.x
            pose.pose.position.y = point.y
            pose.pose.orientation.z = math.sin(point.yaw / 2.0)
            pose.pose.orientation.w = math.cos(point.yaw / 2.0)
            message.poses.append(pose)
        self.path_pub.publish(message)

    def _publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def _nearest_index(self, x: float, y: float) -> int:
        return min(
            range(len(self.world_path)),
            key=lambda i: (self.world_path[i].x - x) ** 2 + (self.world_path[i].y - y) ** 2,
        )

    def _control_tick(self) -> None:
        if self.done or self.latest_odom is None or self.start_time is None:
            return
        now = stamp_seconds(self.latest_odom)
        mode = String()
        mode.data = self.configuration
        self.configuration_pub.publish(mode)
        self._publish_reference_path()

        if now < self.start_time:
            self._publish_stop()
            return

        pose = self.latest_odom.pose.pose.position
        yaw = yaw_from_odometry(self.latest_odom)
        nearest = self._nearest_index(pose.x, pose.y)
        nearest_point = self.world_path[nearest]
        cross_track = math.hypot(pose.x - nearest_point.x, pose.y - nearest_point.y)
        goal = self.world_path[-1]
        goal_distance = math.hypot(goal.x - pose.x, goal.y - pose.y)
        elapsed = now - self.start_time

        if self.settle_start is not None:
            self._publish_stop()
            if now - self.settle_start >= 1.0:
                self._finish(goal_distance, yaw)
            return

        if goal_distance <= self.goal_tolerance or elapsed >= self.maximum_run_time:
            self.timed_out = elapsed >= self.maximum_run_time
            self.settle_start = now
            self._publish_stop()
            return

        target_s = nearest_point.s + self.lookahead
        target = min(self.world_path, key=lambda point: abs(point.s - target_s))
        dx = target.x - pose.x
        dy = target.y - pose.y
        target_x_body = math.cos(yaw) * dx + math.sin(yaw) * dy
        target_y_body = -math.sin(yaw) * dx + math.cos(yaw) * dy
        distance_sq = max(target_x_body**2 + target_y_body**2, 1.0e-6)
        target_distance = math.sqrt(distance_sq)
        target_bearing = math.atan2(target_y_body, target_x_body)
        if self.tracking_model == "configuration_aware":
            curvature = configuration_aware_curvature(
                target_bearing_rad=target_bearing,
                target_distance_m=target_distance,
                configuration=self.configuration,
                maximum_curvature=self.max_curvature,
                front_length_m=self.front_length_m,
                rear_length_m=self.rear_length_m,
            )
        else:
            curvature = clamp(2.0 * target_y_body / distance_sq, -self.max_curvature, self.max_curvature)
        beta = sideslip_beta(
            self.configuration, curvature, self.front_length_m, self.rear_length_m
        )
        speed = min(self.nominal_speed, max(0.02, 0.6 * goal_distance))
        command = Twist()
        command.linear.x = speed
        command.angular.z = speed * curvature
        self.cmd_pub.publish(command)

        self.rows.append(
            {
                "sim_time_s": now,
                "elapsed_s": elapsed,
                "x_m": pose.x,
                "y_m": pose.y,
                "yaw_rad": yaw,
                "nearest_s_m": nearest_point.s,
                "cross_track_error_m": cross_track,
                "goal_distance_m": goal_distance,
                "cmd_linear_m_s": speed,
                "cmd_angular_rad_s": command.angular.z,
                "cmd_curvature_1_m": curvature,
                "sideslip_beta_deg": math.degrees(beta),
                "effective_velocity_heading_deg": math.degrees(yaw + beta),
                "front_steering_deg": math.degrees(self.actual_front),
                "rear_steering_deg": math.degrees(self.actual_rear),
                "front_steering_effort": self.actual_front_effort,
                "rear_steering_effort": self.actual_rear_effort,
            }
        )

    def _finish(self, goal_distance: float, final_yaw: float) -> None:
        self._publish_stop()
        errors = [row["cross_track_error_m"] for row in self.rows]
        squared = [error * error for error in errors]
        goal = self.world_path[-1]
        initial_metrics = metric_window(self.rows, 0.0, self.initial_metric_window)
        movement_start = motion_start_time(self.rows, self.motion_start_displacement)
        motion_metrics = metric_window(
            self.rows, movement_start, self.initial_metric_window
        )
        initial_rows = [
            row for row in self.rows if row["elapsed_s"] <= self.initial_metric_window
        ]
        summary = {
            "run_id": self.run_id,
            "configuration": self.configuration,
            "tracking_model": self.tracking_model,
            "timed_out": self.timed_out,
            "samples": len(self.rows),
            "rmse_cross_track_m": math.sqrt(sum(squared) / len(squared)) if squared else math.nan,
            "max_cross_track_m": max(errors) if errors else math.nan,
            "final_goal_error_m": goal_distance,
            "final_heading_error_deg": math.degrees(wrap_angle(final_yaw - goal.yaw)),
            "curvature_saturation_fraction": (
                sum(
                    abs(row["cmd_curvature_1_m"]) >= self.max_curvature - 1.0e-9
                    for row in self.rows
                )
                / len(self.rows)
                if self.rows
                else math.nan
            ),
            "initial_metric_window_s": self.initial_metric_window,
            "motion_start_displacement_m": self.motion_start_displacement,
            "motion_start_delay_s": movement_start,
            "motion_initial_samples": motion_metrics["samples"],
            "motion_initial_rmse_cross_track_m": motion_metrics["rmse_cross_track_m"],
            "motion_initial_max_cross_track_m": motion_metrics["max_cross_track_m"],
            "motion_initial_cross_track_iae_m_s": motion_metrics[
                "cross_track_iae_m_s"
            ],
            "motion_initial_max_yaw_disturbance_deg": motion_metrics[
                "max_yaw_disturbance_deg"
            ],
            "initial_front_steering_deg": self.rows[0]["front_steering_deg"] if self.rows else math.nan,
            "initial_rear_steering_deg": self.rows[0]["rear_steering_deg"] if self.rows else math.nan,
            "initial_rmse_cross_track_m": initial_metrics["rmse_cross_track_m"],
            "initial_max_cross_track_m": initial_metrics["max_cross_track_m"],
            "initial_cross_track_iae_m_s": initial_metrics["cross_track_iae_m_s"],
            "initial_max_yaw_disturbance_deg": initial_metrics[
                "max_yaw_disturbance_deg"
            ],
            "initial_peak_steering_effort": (
                max(
                    max(abs(row["front_steering_effort"]), abs(row["rear_steering_effort"]))
                    for row in initial_rows
                )
                if initial_rows
                else math.nan
            ),
        }
        output_dir = Path.cwd() / "results" / "ros_reference_path"
        output_dir.mkdir(parents=True, exist_ok=True)
        stem = f"{self.run_id}_{self.configuration}"
        with (output_dir / f"{stem}.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(self.rows[0].keys()))
            writer.writeheader()
            writer.writerows(self.rows)
        (output_dir / f"{stem}.json").write_text(
            json.dumps(summary, indent=2) + "\n", encoding="utf-8"
        )
        self.get_logger().info(
            "REFERENCE PATH RESULT: "
            f"configuration={self.configuration}, model={self.tracking_model}, "
            f"timed_out={self.timed_out}, "
            f"RMSE={summary['rmse_cross_track_m']:.4f} m, "
            f"max={summary['max_cross_track_m']:.4f} m, "
            f"goal={goal_distance:.4f} m, heading={summary['final_heading_error_deg']:.3f} deg"
        )
        self.get_logger().info(f"Saved: {output_dir / (stem + '.json')}")
        self.done = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ReferencePathTracker()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_stop()
        rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
