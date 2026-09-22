#!/usr/bin/env python3
"""Send a relative Nav2 goal and record initial-motion ground-truth metrics."""

import csv
import json
import math
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class Nav2GoalRunner(Node):
    def __init__(self):
        super().__init__("nav2_goal_runner")
        self.declare_parameter("run_id", "nav2_run01")
        self.declare_parameter("goal_forward_m", 1.0)
        self.declare_parameter("goal_lateral_m", 0.0)
        self.declare_parameter("goal_yaw_offset_deg", 0.0)
        self.declare_parameter("maximum_run_time_s", 45.0)
        self.declare_parameter("motion_start_displacement_m", 0.001)
        self.declare_parameter("transient_window_s", 8.0)

        self.run_id = str(self.get_parameter("run_id").value)
        self.goal_forward = float(self.get_parameter("goal_forward_m").value)
        self.goal_lateral = float(self.get_parameter("goal_lateral_m").value)
        self.goal_yaw_offset = math.radians(
            float(self.get_parameter("goal_yaw_offset_deg").value)
        )
        self.maximum_run_time = float(self.get_parameter("maximum_run_time_s").value)
        self.motion_start_displacement = float(
            self.get_parameter("motion_start_displacement_m").value
        )
        self.transient_window = float(self.get_parameter("transient_window_s").value)

        self.action_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.create_subscription(
            Odometry, "/sim/ground_truth/odom", self._odom_callback, 20
        )
        self.create_subscription(Twist, "/cmd_vel", self._cmd_callback, 20)
        self.create_subscription(JointState, "/joint_states", self._joint_callback, 20)
        self.create_subscription(
            String,
            "/sim/steering_configuration/status",
            self._status_callback,
            20,
        )
        self.create_timer(0.1, self._tick)

        self.latest_pose = None
        self.latest_cmd = Twist()
        self.front_steering_deg = math.nan
        self.rear_steering_deg = math.nan
        self.front_steering_effort = math.nan
        self.rear_steering_effort = math.nan
        self.front_drive_effort = math.nan
        self.rear_drive_effort = math.nan
        self.alignment_scale = math.nan
        self.applied_linear_m_s = math.nan
        self.applied_angular_rad_s = math.nan
        self.start_pose = None
        self.start_time = None
        self.goal_xy = None
        self.rows = []
        self.goal_sent = False
        self.goal_handle = None
        self.finished = False
        self.result_status = None
        self.get_logger().info(
            f"Waiting for Nav2 and odometry: run_id={self.run_id}, "
            f"relative_goal=({self.goal_forward:.3f}, {self.goal_lateral:.3f}) m"
        )

    def _odom_callback(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        yaw = yaw_from_quaternion(pose.orientation)
        self.latest_pose = (pose.position.x, pose.position.y, yaw)
        if not self.goal_sent or self.start_pose is None:
            return
        now = self.get_clock().now().nanoseconds / 1.0e9
        x0, y0, yaw0 = self.start_pose
        dx = pose.position.x - x0
        dy = pose.position.y - y0
        longitudinal = math.cos(yaw0) * dx + math.sin(yaw0) * dy
        lateral = -math.sin(yaw0) * dx + math.cos(yaw0) * dy
        self.rows.append(
            {
                "elapsed_s": now - self.start_time,
                "x_m": pose.position.x,
                "y_m": pose.position.y,
                "yaw_deg": math.degrees(yaw),
                "longitudinal_m": longitudinal,
                "lateral_error_m": lateral,
                "yaw_disturbance_deg": math.degrees(wrap_angle(yaw - yaw0)),
                "cmd_linear_m_s": self.latest_cmd.linear.x,
                "cmd_angular_rad_s": self.latest_cmd.angular.z,
                "body_linear_x_m_s": msg.twist.twist.linear.x,
                "body_lateral_velocity_m_s": msg.twist.twist.linear.y,
                "body_yaw_rate_deg_s": math.degrees(msg.twist.twist.angular.z),
                "front_steering_deg": self.front_steering_deg,
                "rear_steering_deg": self.rear_steering_deg,
                "front_steering_effort": self.front_steering_effort,
                "rear_steering_effort": self.rear_steering_effort,
                "front_drive_effort": self.front_drive_effort,
                "rear_drive_effort": self.rear_drive_effort,
                "alignment_scale": self.alignment_scale,
                "applied_linear_m_s": self.applied_linear_m_s,
                "applied_angular_rad_s": self.applied_angular_rad_s,
            }
        )

    def _cmd_callback(self, msg: Twist) -> None:
        self.latest_cmd = msg

    def _joint_callback(self, msg: JointState) -> None:
        index = {name: i for i, name in enumerate(msg.name)}
        front = index.get("front_steering_joint")
        rear = index.get("rear_steering_joint")
        if front is not None and front < len(msg.position):
            self.front_steering_deg = math.degrees(msg.position[front] - math.pi / 2.0)
        if rear is not None and rear < len(msg.position):
            self.rear_steering_deg = math.degrees(msg.position[rear] - math.pi / 2.0)
        effort_values = {
            name: msg.effort[i]
            for name, i in index.items()
            if i < len(msg.effort)
        }
        self.front_steering_effort = effort_values.get(
            "front_steering_joint", math.nan
        )
        self.rear_steering_effort = effort_values.get(
            "rear_steering_joint", math.nan
        )
        self.front_drive_effort = effort_values.get("front_wheel_joint", math.nan)
        self.rear_drive_effort = effort_values.get("rear_wheel_joint", math.nan)

    def _status_callback(self, msg: String) -> None:
        fields = {}
        for item in msg.data.split(","):
            if "=" in item:
                key, value = item.split("=", 1)
                fields[key.strip()] = value.strip()
        try:
            self.alignment_scale = float(fields.get("alignment_scale", "nan"))
        except ValueError:
            self.alignment_scale = math.nan
        try:
            self.applied_linear_m_s = float(fields.get("v", "nan"))
            self.applied_angular_rad_s = float(fields.get("omega", "nan"))
        except ValueError:
            self.applied_linear_m_s = math.nan
            self.applied_angular_rad_s = math.nan

    def _tick(self) -> None:
        if self.finished:
            return
        if not self.goal_sent:
            if self.latest_pose is None or not self.action_client.server_is_ready():
                return
            self._send_goal()
            return
        now = self.get_clock().now().nanoseconds / 1.0e9
        if self.start_time is not None and now - self.start_time > self.maximum_run_time:
            self.get_logger().error("Nav2 trial timed out; requesting goal cancellation")
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
            self.result_status = GoalStatus.STATUS_CANCELED
            self._finish()

    def _send_goal(self) -> None:
        x0, y0, yaw0 = self.latest_pose
        goal_x = x0 + math.cos(yaw0) * self.goal_forward - math.sin(yaw0) * self.goal_lateral
        goal_y = y0 + math.sin(yaw0) * self.goal_forward + math.cos(yaw0) * self.goal_lateral
        goal_yaw = yaw0 + self.goal_yaw_offset
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = goal_x
        goal.pose.pose.position.y = goal_y
        goal.pose.pose.orientation.z = math.sin(goal_yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(goal_yaw / 2.0)
        self.start_pose = self.latest_pose
        self.goal_xy = (goal_x, goal_y)
        self.start_time = self.get_clock().now().nanoseconds / 1.0e9
        self.goal_sent = True
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)
        self.get_logger().info(
            f"Sending Nav2 goal: x={goal_x:.3f}, y={goal_y:.3f}, "
            f"yaw={math.degrees(goal_yaw):.3f} deg"
        )

    def _goal_response_callback(self, future) -> None:
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error("Nav2 goal was rejected")
            self.result_status = GoalStatus.STATUS_ABORTED
            self._finish()
            return
        self.get_logger().info("Nav2 goal accepted")
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future) -> None:
        self.result_status = future.result().status
        self._finish()

    def _finish(self) -> None:
        if self.finished:
            return
        self.finished = True
        if not self.rows:
            self.get_logger().error("No odometry samples were recorded")
            return
        lateral = [row["lateral_error_m"] for row in self.rows]
        yaw = [abs(row["yaw_disturbance_deg"]) for row in self.rows]
        transient_rows = [
            row for row in self.rows if row["elapsed_s"] <= self.transient_window
        ]
        if not transient_rows:
            transient_rows = self.rows[:1]
        transient_lateral = [row["lateral_error_m"] for row in transient_rows]

        def finite_abs_max(key: str) -> float:
            values = [
                abs(row[key])
                for row in transient_rows
                if math.isfinite(row[key])
            ]
            return max(values, default=math.nan)

        transient_lateral_iae = 0.0
        for previous, current in zip(transient_rows, transient_rows[1:]):
            dt = current["elapsed_s"] - previous["elapsed_s"]
            if dt > 0.0:
                transient_lateral_iae += 0.5 * dt * (
                    abs(previous["lateral_error_m"])
                    + abs(current["lateral_error_m"])
                )
        movement_start = math.nan
        for row in self.rows:
            if math.hypot(row["longitudinal_m"], row["lateral_error_m"]) >= self.motion_start_displacement:
                movement_start = row["elapsed_s"]
                break
        x, y, current_yaw = self.latest_pose
        goal_x, goal_y = self.goal_xy
        status_name = {
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
        }.get(self.result_status, str(self.result_status))
        summary = {
            "run_id": self.run_id,
            "status": status_name,
            "success": self.result_status == GoalStatus.STATUS_SUCCEEDED,
            "samples": len(self.rows),
            "duration_s": self.rows[-1]["elapsed_s"],
            "motion_start_delay_s": movement_start,
            "transient_window_s": self.transient_window,
            "transient_rmse_lateral_m": math.sqrt(
                sum(value * value for value in transient_lateral)
                / len(transient_lateral)
            ),
            "transient_max_abs_lateral_m": max(
                abs(value) for value in transient_lateral
            ),
            "transient_lateral_iae_m_s": transient_lateral_iae,
            "transient_max_abs_yaw_disturbance_deg": finite_abs_max(
                "yaw_disturbance_deg"
            ),
            "transient_max_abs_yaw_rate_deg_s": finite_abs_max(
                "body_yaw_rate_deg_s"
            ),
            "transient_max_abs_lateral_velocity_m_s": finite_abs_max(
                "body_lateral_velocity_m_s"
            ),
            "transient_peak_abs_steering_effort": max(
                finite_abs_max("front_steering_effort"),
                finite_abs_max("rear_steering_effort"),
            ),
            "transient_peak_abs_drive_effort": max(
                finite_abs_max("front_drive_effort"),
                finite_abs_max("rear_drive_effort"),
            ),
            "rmse_lateral_m": math.sqrt(sum(value * value for value in lateral) / len(lateral)),
            "max_abs_lateral_m": max(abs(value) for value in lateral),
            "max_abs_yaw_disturbance_deg": max(yaw),
            "final_goal_error_m": math.hypot(goal_x - x, goal_y - y),
            "final_yaw_disturbance_deg": math.degrees(
                wrap_angle(current_yaw - self.start_pose[2])
            ),
            "min_alignment_scale": min(
                (row["alignment_scale"] for row in self.rows if math.isfinite(row["alignment_scale"])),
                default=math.nan,
            ),
        }
        output_dir = Path.cwd() / "results" / "nav2_initial_motion"
        output_dir.mkdir(parents=True, exist_ok=True)
        csv_path = output_dir / f"{self.run_id}.csv"
        json_path = output_dir / f"{self.run_id}.json"
        with csv_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(self.rows[0]))
            writer.writeheader()
            writer.writerows(self.rows)
        json_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
        self.get_logger().info(
            "NAV2 INITIAL-MOTION RESULT: "
            f"status={status_name}, RMSE={summary['rmse_lateral_m']:.4f} m, "
            f"max={summary['max_abs_lateral_m']:.4f} m, "
            f"yaw={summary['max_abs_yaw_disturbance_deg']:.3f} deg, "
            f"goal={summary['final_goal_error_m']:.4f} m"
        )
        self.get_logger().info(f"Saved: {json_path}")


def main(args=None):
    rclpy.init(args=args)
    node = Nav2GoalRunner()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
