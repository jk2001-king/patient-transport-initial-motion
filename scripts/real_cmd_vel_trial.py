#!/usr/bin/env python3
"""Publish a reproducible, safety-limited straight cmd_vel trial on ROS 2."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


def _arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", choices=("step", "fixed-ramp"), required=True)
    parser.add_argument("--speed", type=float, default=0.075)
    parser.add_argument("--duration", type=float, default=4.0)
    parser.add_argument("--ramp-duration", type=float, default=1.0)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--run",
        action="store_true",
        help="Required physical-motion acknowledgement.",
    )
    args = parser.parse_args()
    if not args.run:
        parser.error("Refusing to move without --run")
    if not 0.0 < args.speed <= 0.1:
        parser.error("--speed must be in (0, 0.1] m/s for this screening test")
    if not 1.0 <= args.duration <= 6.0:
        parser.error("--duration must be in [1, 6] s")
    if not 10.0 <= args.rate <= 50.0:
        parser.error("--rate must be in [10, 50] Hz")
    if args.profile == "fixed-ramp" and not 0.1 <= args.ramp_duration <= args.duration:
        parser.error("--ramp-duration must be in [0.1, duration]")
    return args


def _smoothstep(progress):
    p = min(1.0, max(0.0, float(progress)))
    return p * p * (3.0 - 2.0 * p)


class TrialPublisher(Node):
    def __init__(self):
        super().__init__("patient_initial_motion_trial_publisher")
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)

    def publish_speed(self, speed):
        message = Twist()
        message.linear.x = float(speed)
        message.angular.z = 0.0
        self.publisher.publish(message)


def main():
    args = _arguments()
    rclpy.init()
    node = TrialPublisher()
    period = 1.0 / args.rate
    rows = []
    args.output.parent.mkdir(parents=True, exist_ok=True)
    try:
        deadline = time.monotonic() + 5.0
        while node.publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.publisher.get_subscription_count() == 0:
            raise RuntimeError("No /cmd_vel subscriber found; bringup is not ready")

        print("Neutral command for 0.5 s. Confirm the lane is clear and E-stop operator is ready.")
        for _ in range(max(1, math.ceil(0.5 * args.rate))):
            node.publish_speed(0.0)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(period)

        print(
            f"START profile={args.profile}, speed={args.speed:.3f} m/s, "
            f"duration={args.duration:.3f} s"
        )
        start = time.monotonic()
        samples = math.ceil(args.duration * args.rate)
        for sample in range(samples):
            target_time = sample * period
            now = time.monotonic() - start
            if now < target_time:
                time.sleep(target_time - now)
            elapsed = time.monotonic() - start
            if args.profile == "step":
                scale = 1.0
            else:
                scale = _smoothstep(elapsed / args.ramp_duration)
            command = args.speed * scale
            node.publish_speed(command)
            stamp = node.get_clock().now().nanoseconds * 1e-9
            rows.append(
                {
                    "elapsed_s": elapsed,
                    "ros_time_s": stamp,
                    "profile": args.profile,
                    "speed_command_m_s": command,
                    "yaw_rate_command_rad_s": 0.0,
                }
            )
            rclpy.spin_once(node, timeout_sec=0.0)

    except KeyboardInterrupt:
        print("Interrupted; sending stop command.")
    finally:
        # Publish zero repeatedly so shutdown never leaves the last non-zero
        # command relying only on motor_node's timeout.
        for _ in range(10):
            node.publish_speed(0.0)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.05)
        if rows:
            with args.output.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
            print(f"Saved command log: {args.output}")
        print("STOP command published.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
