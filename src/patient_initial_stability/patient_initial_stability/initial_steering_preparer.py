import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class InitialSteeringPreparer(Node):
    def __init__(self) -> None:
        super().__init__("initial_steering_preparer")
        self.declare_parameter("front_deg", 0.0)
        self.declare_parameter("rear_deg", 0.0)
        self.declare_parameter("tolerance_deg", 0.25)
        self.declare_parameter("settle_time_s", 0.5)
        self.declare_parameter("maximum_time_s", 10.0)
        self.declare_parameter("maximum_abs_angle_deg", 45.0)

        self.front_deg = float(self.get_parameter("front_deg").value)
        self.rear_deg = float(self.get_parameter("rear_deg").value)
        self.tolerance_deg = float(self.get_parameter("tolerance_deg").value)
        self.settle_time = float(self.get_parameter("settle_time_s").value)
        self.maximum_time = float(self.get_parameter("maximum_time_s").value)
        self.maximum_abs_angle_deg = float(
            self.get_parameter("maximum_abs_angle_deg").value
        )
        if not 0.0 < self.maximum_abs_angle_deg <= 90.0:
            raise ValueError("maximum_abs_angle_deg must be in (0, 90]")
        if (
            abs(self.front_deg) > self.maximum_abs_angle_deg
            or abs(self.rear_deg) > self.maximum_abs_angle_deg
        ):
            raise ValueError(
                "Initial physical steering angles must be within "
                f"+/-{self.maximum_abs_angle_deg:.1f} deg"
            )

        self.publisher = self.create_publisher(JointState, "/sim/joint_command", 10)
        self.create_subscription(JointState, "/joint_states", self._joint_callback, 10)
        self.create_timer(0.05, self._timer_callback)
        self.start_ns = self.get_clock().now().nanoseconds
        self.within_since_ns = None
        self.actual_front_deg = None
        self.actual_rear_deg = None
        self.done = False
        self.success = False
        self.get_logger().info(
            f"Preparing initial steering: front={self.front_deg:+.3f} deg, "
            f"rear={self.rear_deg:+.3f} deg"
        )

    def _joint_callback(self, message: JointState) -> None:
        index = {name: i for i, name in enumerate(message.name)}
        if "front_steering_joint" not in index or "rear_steering_joint" not in index:
            return
        if len(message.position) <= max(index["front_steering_joint"], index["rear_steering_joint"]):
            return
        self.actual_front_deg = math.degrees(
            message.position[index["front_steering_joint"]] - math.pi / 2.0
        )
        self.actual_rear_deg = math.degrees(
            message.position[index["rear_steering_joint"]] - math.pi / 2.0
        )

    def _timer_callback(self) -> None:
        command = JointState()
        command.header.stamp = self.get_clock().now().to_msg()
        command.name = ["front_steering_joint", "rear_steering_joint"]
        command.position = [
            math.pi / 2.0 + math.radians(self.front_deg),
            math.pi / 2.0 + math.radians(self.rear_deg),
        ]
        self.publisher.publish(command)

        now_ns = self.get_clock().now().nanoseconds
        elapsed = (now_ns - self.start_ns) / 1.0e9
        if self.actual_front_deg is not None and self.actual_rear_deg is not None:
            within = (
                abs(self.actual_front_deg - self.front_deg) <= self.tolerance_deg
                and abs(self.actual_rear_deg - self.rear_deg) <= self.tolerance_deg
            )
            if within:
                if self.within_since_ns is None:
                    self.within_since_ns = now_ns
                elif (now_ns - self.within_since_ns) / 1.0e9 >= self.settle_time:
                    self.success = True
                    self.done = True
                    self.get_logger().info(
                        "INITIAL STEERING READY: "
                        f"front={self.actual_front_deg:+.3f} deg, "
                        f"rear={self.actual_rear_deg:+.3f} deg"
                    )
                    return
            else:
                self.within_since_ns = None

        if elapsed >= self.maximum_time:
            self.done = True
            self.get_logger().error(
                "Initial steering preparation timed out: "
                f"front={self.actual_front_deg}, rear={self.actual_rear_deg}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InitialSteeringPreparer()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    success = node.success
    node.destroy_node()
    rclpy.shutdown()
    if not success:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
