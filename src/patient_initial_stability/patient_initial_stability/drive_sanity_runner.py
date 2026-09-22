import json
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def stamp_seconds(message: Odometry) -> float:
    return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1.0e-9


def yaw_from_odometry(message: Odometry) -> float:
    q = message.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class DriveSanityRunner(Node):
    def __init__(self) -> None:
        super().__init__("drive_sanity_runner")
        self.declare_parameter("speed_m_s", 0.03)
        self.declare_parameter("command_duration_s", 4.0)
        self.declare_parameter("settle_duration_s", 2.0)
        self.declare_parameter("expected_acceleration_m_s2", 0.02)

        self.speed = float(self.get_parameter("speed_m_s").value)
        self.command_duration = float(self.get_parameter("command_duration_s").value)
        self.settle_duration = float(self.get_parameter("settle_duration_s").value)
        self.acceleration = float(self.get_parameter("expected_acceleration_m_s2").value)

        self.publisher = self.create_publisher(Twist, "/sim/cmd_vel", 10)
        self.create_subscription(Odometry, "/sim/ground_truth/odom", self._odom_callback, 10)
        self.create_timer(0.05, self._timer_callback)

        self.latest_odom = None
        self.start_odom = None
        self.start_time = None
        self.done = False
        self.result = None
        self.get_logger().info("Waiting for /sim/ground_truth/odom; keep Isaac Timeline playing")

    def _odom_callback(self, message: Odometry) -> None:
        self.latest_odom = message
        if self.start_odom is None:
            self.start_odom = message
            self.start_time = stamp_seconds(message)
            self.get_logger().info(
                f"Starting {self.command_duration:.1f} s drive sanity run at {self.speed:.3f} m/s"
            )

    def _publish(self, speed: float) -> None:
        command = Twist()
        command.linear.x = speed
        self.publisher.publish(command)

    def _expected_distance(self) -> float:
        ramp_time = abs(self.speed) / self.acceleration
        if self.command_duration <= ramp_time:
            command_distance = 0.5 * self.acceleration * self.command_duration**2
            final_speed = self.acceleration * self.command_duration
        else:
            command_distance = 0.5 * abs(self.speed) * ramp_time
            command_distance += abs(self.speed) * (self.command_duration - ramp_time)
            final_speed = abs(self.speed)
        stopping_distance = 0.5 * final_speed * final_speed / self.acceleration
        return math.copysign(command_distance + stopping_distance, self.speed)

    def _timer_callback(self) -> None:
        if self.done or self.latest_odom is None or self.start_time is None:
            return
        elapsed = stamp_seconds(self.latest_odom) - self.start_time
        if elapsed < self.command_duration:
            self._publish(self.speed)
            return

        self._publish(0.0)
        if elapsed < self.command_duration + self.settle_duration:
            return

        start = self.start_odom.pose.pose.position
        end = self.latest_odom.pose.pose.position
        yaw0 = yaw_from_odometry(self.start_odom)
        yaw1 = yaw_from_odometry(self.latest_odom)
        dx = end.x - start.x
        dy = end.y - start.y
        longitudinal = math.cos(yaw0) * dx + math.sin(yaw0) * dy
        lateral = -math.sin(yaw0) * dx + math.cos(yaw0) * dy
        expected = self._expected_distance()
        ratio = longitudinal / expected if abs(expected) > 1.0e-9 else math.nan

        self.result = {
            "speed_m_s": self.speed,
            "command_duration_s": self.command_duration,
            "settle_duration_s": self.settle_duration,
            "expected_acceleration_m_s2": self.acceleration,
            "expected_longitudinal_m": expected,
            "measured_longitudinal_m": longitudinal,
            "measured_lateral_m": lateral,
            "measured_yaw_deg": math.degrees(wrap_angle(yaw1 - yaw0)),
            "distance_ratio": ratio,
        }
        output_dir = Path.cwd() / "results" / "ros_drive_sanity"
        output_dir.mkdir(parents=True, exist_ok=True)
        output_path = output_dir / "latest.json"
        output_path.write_text(json.dumps(self.result, indent=2) + "\n", encoding="utf-8")
        self.get_logger().info(
            "DRIVE SANITY RESULT: "
            f"expected={expected:.4f} m, longitudinal={longitudinal:.4f} m, "
            f"ratio={ratio:.3f}, lateral={lateral:.4f} m, "
            f"yaw={self.result['measured_yaw_deg']:.3f} deg"
        )
        self.get_logger().info(f"Saved: {output_path}")
        self.done = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DriveSanityRunner()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish(0.0)
        rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
