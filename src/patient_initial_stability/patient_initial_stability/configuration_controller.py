import math
from dataclasses import dataclass
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


VALID_CONFIGURATIONS = ("front_only", "rear_only", "coordinated_counterphase")


@dataclass(frozen=True)
class JointTargets:
    front_steering_rad: float
    rear_steering_rad: float
    front_wheel_rad_s: float
    rear_wheel_rad_s: float


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def configuration_targets(
    linear_x: float,
    angular_z: float,
    configuration: str,
    wheelbase_m: float,
    front_length_m: float,
    rear_length_m: float,
    wheel_radius_m: float,
    steering_limit_rad: float,
) -> JointTargets:
    """Map a planar body command to physical steering and wheel-joint targets."""
    if configuration not in VALID_CONFIGURATIONS:
        raise ValueError(f"Unsupported steering configuration: {configuration}")
    if abs(linear_x) < 1.0e-6:
        return JointTargets(0.0, 0.0, 0.0, 0.0)

    curvature = angular_z / linear_x
    if configuration == "front_only":
        front = math.atan(wheelbase_m * curvature)
        rear = 0.0
    elif configuration == "rear_only":
        front = 0.0
        rear = -math.atan(wheelbase_m * curvature)
    else:
        front = math.atan(front_length_m * curvature)
        rear = -math.atan(rear_length_m * curvature)

    front = clamp(front, -steering_limit_rad, steering_limit_rad)
    rear = clamp(rear, -steering_limit_rad, steering_limit_rad)

    front_roll_m_s = linear_x / max(math.cos(front), 1.0e-6)
    rear_roll_m_s = linear_x / max(math.cos(rear), 1.0e-6)
    return JointTargets(
        front_steering_rad=front,
        rear_steering_rad=rear,
        front_wheel_rad_s=front_roll_m_s / wheel_radius_m,
        rear_wheel_rad_s=-rear_roll_m_s / wheel_radius_m,
    )


def move_toward(current: float, target: float, maximum_step: float) -> float:
    return current + clamp(target - current, -maximum_step, maximum_step)


def initial_alignment_speed_scale(
    maximum_alignment_error_rad: float,
    zero_speed_error_rad: float,
) -> float:
    """Return a continuous 0..1 initial drive scale from steering error."""
    if zero_speed_error_rad <= 0.0:
        raise ValueError("zero_speed_error_rad must be positive")
    return clamp(1.0 - abs(maximum_alignment_error_rad) / zero_speed_error_rad, 0.0, 1.0)


def leading_axle_alignment_error(
    front_alignment_error_rad: float,
    rear_alignment_error_rad: float,
    requested_linear_x: float,
) -> float:
    """Select the steering error of the axle leading the requested motion."""
    return front_alignment_error_rad if requested_linear_x >= 0.0 else rear_alignment_error_rad


def configuration_risk_alignment_error(
    configuration: str,
    front_alignment_error_rad: float,
    rear_alignment_error_rad: float,
    requested_linear_x: float,
) -> float:
    """Select the initial-risk steering error for the active configuration."""
    if configuration == "front_only":
        return front_alignment_error_rad
    if configuration == "rear_only":
        return rear_alignment_error_rad
    if configuration == "coordinated_counterphase":
        return leading_axle_alignment_error(
            front_alignment_error_rad,
            rear_alignment_error_rad,
            requested_linear_x,
        )
    raise ValueError(f"Unsupported steering configuration: {configuration}")


def coordinated_alignment_components(
    front_alignment_error_rad: float,
    rear_alignment_error_rad: float,
):
    """Return common-mode (crab) and counterphase (curvature) errors."""
    return (
        0.5 * (front_alignment_error_rad + rear_alignment_error_rad),
        0.5 * (front_alignment_error_rad - rear_alignment_error_rad),
    )


def sideslip_beta(
    configuration: str,
    yaw_curvature: float,
    front_length_m: float,
    rear_length_m: float,
) -> float:
    """Return the body-center velocity sideslip for a steering configuration."""
    if configuration == "front_only":
        lateral_velocity_ratio = rear_length_m * yaw_curvature
    elif configuration == "rear_only":
        lateral_velocity_ratio = -front_length_m * yaw_curvature
    elif configuration == "coordinated_counterphase":
        lateral_velocity_ratio = 0.0
    else:
        raise ValueError(f"Unsupported steering configuration: {configuration}")
    return math.atan(lateral_velocity_ratio)


def configuration_aware_curvature_from_common_command(
    common_curvature: float,
    configuration: str,
    lookahead_m: float,
    wheelbase_m: float,
    front_length_m: float,
    rear_length_m: float,
    steering_limit_rad: float,
) -> float:
    """Convert a common-base pure-pursuit command for the steering layout.

    Nav2's standard pure-pursuit command assumes that the velocity direction
    equals the body heading.  Front- and rear-only steering introduce a
    configuration-dependent velocity sideslip at the body center.  Recover the
    implied lookahead bearing from the common command and solve the same
    pursuit relation in that velocity-direction frame.
    """
    if lookahead_m <= 0.0:
        raise ValueError("lookahead_m must be positive")
    if configuration == "coordinated_counterphase":
        maximum_curvature = math.tan(steering_limit_rad) / min(
            front_length_m, rear_length_m
        )
        return clamp(common_curvature, -maximum_curvature, maximum_curvature)
    if configuration not in ("front_only", "rear_only"):
        raise ValueError(f"Unsupported steering configuration: {configuration}")

    maximum_curvature = math.tan(steering_limit_rad) / wheelbase_m
    target_bearing = math.asin(
        clamp(0.5 * common_curvature * lookahead_m, -1.0, 1.0)
    )
    candidates = [
        -maximum_curvature + 2.0 * maximum_curvature * index / 400.0
        for index in range(401)
    ]

    def residual(curvature: float) -> float:
        beta = sideslip_beta(
            configuration, curvature, front_length_m, rear_length_m
        )
        requested = 2.0 * math.sin(target_bearing - beta) / lookahead_m
        return abs(curvature - requested)

    return min(candidates, key=residual)


def configuration_alignment_severity(
    configuration: str,
    front_alignment_error_rad: float,
    rear_alignment_error_rad: float,
    front_only_threshold_rad: float,
    rear_only_threshold_rad: float,
    coordinated_common_threshold_rad: float,
    coordinated_counter_threshold_rad: float,
) -> float:
    """Return normalized initial mismatch; values above one require scaling."""
    thresholds = (
        front_only_threshold_rad,
        rear_only_threshold_rad,
        coordinated_common_threshold_rad,
        coordinated_counter_threshold_rad,
    )
    if any(value <= 0.0 for value in thresholds):
        raise ValueError("All configuration-aware thresholds must be positive")
    if configuration == "front_only":
        return abs(front_alignment_error_rad) / front_only_threshold_rad
    if configuration == "rear_only":
        return abs(rear_alignment_error_rad) / rear_only_threshold_rad
    if configuration == "coordinated_counterphase":
        common_error, counter_error = coordinated_alignment_components(
            front_alignment_error_rad, rear_alignment_error_rad
        )
        return max(
            abs(common_error) / coordinated_common_threshold_rad,
            abs(counter_error) / coordinated_counter_threshold_rad,
        )
    raise ValueError(f"Unsupported steering configuration: {configuration}")


class ConfigurationController(Node):
    def __init__(self) -> None:
        super().__init__("steering_configuration_controller")

        self.declare_parameter("configuration", "coordinated_counterphase")
        self.declare_parameter("control_rate_hz", 30.0)
        self.declare_parameter("command_timeout_s", 0.5)
        self.declare_parameter("wheelbase_m", 1.29)
        self.declare_parameter("front_length_m", 0.645)
        self.declare_parameter("rear_length_m", 0.645)
        self.declare_parameter("wheel_radius_m", 0.075)
        self.declare_parameter("steering_raw_zero_rad", math.pi / 2.0)
        self.declare_parameter("steering_limit_rad", math.pi / 4.0)
        self.declare_parameter("max_steering_rate_rad_s", math.radians(13.732910))
        self.declare_parameter("max_linear_acceleration_m_s2", 0.02)
        self.declare_parameter("max_linear_speed_m_s", 0.30)
        self.declare_parameter("max_angular_speed_rad_s", 0.50)
        self.declare_parameter("path_tracking_model", "common_base")
        self.declare_parameter("path_tracking_lookahead_m", 0.20)
        self.declare_parameter("initial_alignment_policy", "simultaneous")
        self.declare_parameter("cmd_vel_topic", "/sim/cmd_vel")
        self.declare_parameter("initial_reference_cmd_vel_topic", "")
        self.declare_parameter("initial_alignment_tolerance_deg", 1.5)
        self.declare_parameter("initial_alignment_hold_s", 0.25)
        self.declare_parameter("initial_alignment_zero_speed_error_deg", 20.0)
        self.declare_parameter("front_only_alignment_threshold_deg", 35.0)
        self.declare_parameter("rear_only_alignment_threshold_deg", 35.0)
        self.declare_parameter("coordinated_common_threshold_deg", 20.0)
        self.declare_parameter("coordinated_counter_threshold_deg", 35.0)

        self._configuration = str(self.get_parameter("configuration").value)
        self._cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._initial_reference_cmd_vel_topic = str(
            self.get_parameter("initial_reference_cmd_vel_topic").value
        )
        if not self._cmd_vel_topic:
            raise ValueError("cmd_vel_topic must not be empty")
        if self._configuration not in VALID_CONFIGURATIONS:
            raise ValueError(f"Invalid initial configuration: {self._configuration}")

        self._rate_hz = float(self.get_parameter("control_rate_hz").value)
        self._period_s = 1.0 / self._rate_hz
        self._timeout_s = float(self.get_parameter("command_timeout_s").value)
        self._wheelbase_m = float(self.get_parameter("wheelbase_m").value)
        self._front_length_m = float(self.get_parameter("front_length_m").value)
        self._rear_length_m = float(self.get_parameter("rear_length_m").value)
        self._wheel_radius_m = float(self.get_parameter("wheel_radius_m").value)
        self._raw_zero_rad = float(self.get_parameter("steering_raw_zero_rad").value)
        self._steering_limit_rad = float(self.get_parameter("steering_limit_rad").value)
        self._max_steering_step = (
            float(self.get_parameter("max_steering_rate_rad_s").value) * self._period_s
        )
        self._max_linear_step = (
            float(self.get_parameter("max_linear_acceleration_m_s2").value) * self._period_s
        )
        self._max_linear_speed = float(self.get_parameter("max_linear_speed_m_s").value)
        self._max_angular_speed = float(self.get_parameter("max_angular_speed_rad_s").value)
        self._path_tracking_model = str(
            self.get_parameter("path_tracking_model").value
        )
        if self._path_tracking_model not in ("common_base", "configuration_aware"):
            raise ValueError(
                f"Invalid path_tracking_model: {self._path_tracking_model}"
            )
        self._path_tracking_lookahead = float(
            self.get_parameter("path_tracking_lookahead_m").value
        )
        if self._path_tracking_lookahead <= 0.0:
            raise ValueError("path_tracking_lookahead_m must be positive")
        self._alignment_policy = str(self.get_parameter("initial_alignment_policy").value)
        if self._alignment_policy not in (
            "simultaneous",
            "steer_then_drive",
            "misalignment_scaled",
            "leading_axle_adaptive",
            "configuration_adaptive",
        ):
            raise ValueError(f"Invalid initial_alignment_policy: {self._alignment_policy}")
        self._alignment_tolerance = math.radians(
            float(self.get_parameter("initial_alignment_tolerance_deg").value)
        )
        self._alignment_hold_s = float(self.get_parameter("initial_alignment_hold_s").value)
        self._alignment_zero_speed_error = math.radians(
            float(self.get_parameter("initial_alignment_zero_speed_error_deg").value)
        )
        self._front_only_threshold = math.radians(
            float(self.get_parameter("front_only_alignment_threshold_deg").value)
        )
        self._rear_only_threshold = math.radians(
            float(self.get_parameter("rear_only_alignment_threshold_deg").value)
        )
        self._coordinated_common_threshold = math.radians(
            float(self.get_parameter("coordinated_common_threshold_deg").value)
        )
        self._coordinated_counter_threshold = math.radians(
            float(self.get_parameter("coordinated_counter_threshold_deg").value)
        )
        if self._alignment_zero_speed_error <= self._alignment_tolerance:
            raise ValueError(
                "initial_alignment_zero_speed_error_deg must exceed "
                "initial_alignment_tolerance_deg"
            )
        if min(
            self._front_only_threshold,
            self._rear_only_threshold,
            self._coordinated_common_threshold,
            self._coordinated_counter_threshold,
        ) <= self._alignment_tolerance:
            raise ValueError(
                "All configuration-aware thresholds must exceed "
                "initial_alignment_tolerance_deg"
            )

        self._requested_linear_x = 0.0
        self._requested_angular_z = 0.0
        self._initial_reference_linear_x = 0.0
        self._initial_reference_angular_z = 0.0
        self._have_initial_reference_command = False
        self._commanded_linear_x = 0.0
        self._front_physical = 0.0
        self._rear_physical = 0.0
        self._last_command_ns = None
        self._have_steering_state = False
        self._motion_initialized = False
        self._actual_front_physical = 0.0
        self._actual_rear_physical = 0.0
        self._initial_alignment_complete = self._alignment_policy == "simultaneous"
        self._alignment_within_since_ns = None
        self._adaptive_requires_alignment = None
        self._alignment_severity = 0.0

        self._command_pub = self.create_publisher(JointState, "/sim/joint_command", 10)
        self._status_pub = self.create_publisher(String, "/sim/steering_configuration/status", 10)
        self.create_subscription(Twist, self._cmd_vel_topic, self._cmd_vel_callback, 10)
        if self._initial_reference_cmd_vel_topic:
            self.create_subscription(
                Twist,
                self._initial_reference_cmd_vel_topic,
                self._initial_reference_cmd_vel_callback,
                10,
            )
        self.create_subscription(String, "/sim/steering_configuration", self._configuration_callback, 10)
        self.create_subscription(JointState, "/joint_states", self._joint_state_callback, 10)
        self.create_timer(self._period_s, self._control_tick)

        self.get_logger().info(
            f"Ready: configuration={self._configuration}, command={self._cmd_vel_topic}, "
            f"initial_reference={self._initial_reference_cmd_vel_topic or 'command'}, "
            f"tracking_model={self._path_tracking_model}, "
            f"joint output=/sim/joint_command, alignment_policy={self._alignment_policy}, "
            f"zero_speed_error={math.degrees(self._alignment_zero_speed_error):.3f} deg"
        )

    def _initial_reference_cmd_vel_callback(self, message: Twist) -> None:
        """Store the unsmoothed Nav2 command used only for the initial decision.

        The smoothed command starts with an almost-zero linear speed.  Its
        angular/linear ratio is therefore not a reliable initial path-curvature
        estimate.  Keep actuation on the smoothed command, but classify the
        initial steering state from the controller command before smoothing.
        """
        linear_x = clamp(
            message.linear.x, -self._max_linear_speed, self._max_linear_speed
        )
        angular_z = clamp(
            message.angular.z, -self._max_angular_speed, self._max_angular_speed
        )
        if abs(linear_x) < 1.0e-6:
            return
        self._initial_reference_linear_x = linear_x
        self._initial_reference_angular_z = angular_z
        self._have_initial_reference_command = True

    def _adapt_angular_command(self, linear_x: float, angular_z: float) -> float:
        if (
            self._path_tracking_model != "configuration_aware"
            or abs(linear_x) < 1.0e-6
        ):
            return angular_z
        common_curvature = angular_z / linear_x
        adapted_curvature = configuration_aware_curvature_from_common_command(
            common_curvature=common_curvature,
            configuration=self._configuration,
            lookahead_m=self._path_tracking_lookahead,
            wheelbase_m=self._wheelbase_m,
            front_length_m=self._front_length_m,
            rear_length_m=self._rear_length_m,
            steering_limit_rad=self._steering_limit_rad,
        )
        return clamp(
            linear_x * adapted_curvature,
            -self._max_angular_speed,
            self._max_angular_speed,
        )

    def _cmd_vel_callback(self, message: Twist) -> None:
        self._requested_linear_x = clamp(message.linear.x, -self._max_linear_speed, self._max_linear_speed)
        self._requested_angular_z = clamp(message.angular.z, -self._max_angular_speed, self._max_angular_speed)
        self._last_command_ns = self.get_clock().now().nanoseconds
        if (
            not self._motion_initialized
            and self._have_steering_state
            and (abs(self._requested_linear_x) >= 1.0e-6 or abs(self._requested_angular_z) >= 1.0e-6)
        ):
            self._front_physical = self._actual_front_physical
            self._rear_physical = self._actual_rear_physical
            self._motion_initialized = True
            self._initial_alignment_complete = self._alignment_policy == "simultaneous"
            self._alignment_within_since_ns = None
            self._adaptive_requires_alignment = None
            self._alignment_severity = 0.0
            self.get_logger().info(
                "Motion initialized from measured steering: "
                f"front={math.degrees(self._front_physical):.3f} deg, "
                f"rear={math.degrees(self._rear_physical):.3f} deg"
            )

    def _joint_state_callback(self, message: JointState) -> None:
        index = {name: i for i, name in enumerate(message.name)}
        front_index = index.get("front_steering_joint")
        rear_index = index.get("rear_steering_joint")
        if front_index is None or rear_index is None:
            return
        if len(message.position) <= max(front_index, rear_index):
            return
        self._actual_front_physical = message.position[front_index] - self._raw_zero_rad
        self._actual_rear_physical = message.position[rear_index] - self._raw_zero_rad
        self._have_steering_state = True

    def _configuration_callback(self, message: String) -> None:
        requested = message.data.strip()
        if requested not in VALID_CONFIGURATIONS:
            self.get_logger().error(
                f"Rejected configuration '{requested}'. Valid values: {VALID_CONFIGURATIONS}"
            )
            return
        if requested != self._configuration:
            self._configuration = requested
            self.get_logger().info(f"Configuration changed to {requested}")

    def _command_is_fresh(self) -> bool:
        if self._last_command_ns is None:
            return False
        age_s = (self.get_clock().now().nanoseconds - self._last_command_ns) / 1.0e9
        return 0.0 <= age_s <= self._timeout_s

    def _control_tick(self) -> None:
        if not self._motion_initialized:
            drive = JointState()
            drive.header.stamp = self.get_clock().now().to_msg()
            drive.name = ["front_wheel_joint", "rear_wheel_joint"]
            drive.velocity = [0.0, 0.0]
            self._command_pub.publish(drive)
            status = String()
            status.data = (
                f"configuration={self._configuration},v=0.0000,omega=0.0000,"
                f"front_deg={math.degrees(self._actual_front_physical):.3f},"
                f"rear_deg={math.degrees(self._actual_rear_physical):.3f},"
                f"fresh=False,initialized=False"
            )
            self._status_pub.publish(status)
            return

        if self._command_is_fresh():
            requested_linear = self._requested_linear_x
            requested_angular = self._requested_angular_z
        else:
            requested_linear = 0.0
            requested_angular = 0.0

        adapted_requested_angular = self._adapt_angular_command(
            requested_linear, requested_angular
        )
        requested_targets = configuration_targets(
            linear_x=requested_linear,
            angular_z=adapted_requested_angular,
            configuration=self._configuration,
            wheelbase_m=self._wheelbase_m,
            front_length_m=self._front_length_m,
            rear_length_m=self._rear_length_m,
            wheel_radius_m=self._wheel_radius_m,
            steering_limit_rad=self._steering_limit_rad,
        )
        if abs(requested_linear) >= 1.0e-6:
            self._front_physical = move_toward(
                self._front_physical,
                requested_targets.front_steering_rad,
                self._max_steering_step,
            )
            self._rear_physical = move_toward(
                self._rear_physical,
                requested_targets.rear_steering_rad,
                self._max_steering_step,
            )

        front_alignment_error = requested_targets.front_steering_rad - self._actual_front_physical
        rear_alignment_error = requested_targets.rear_steering_rad - self._actual_rear_physical
        if (
            self._alignment_policy in ("leading_axle_adaptive", "configuration_adaptive")
            and self._adaptive_requires_alignment is None
            and abs(requested_linear) >= 1.0e-6
        ):
            decision_targets = requested_targets
            if self._have_initial_reference_command:
                adapted_reference_angular = self._adapt_angular_command(
                    self._initial_reference_linear_x,
                    self._initial_reference_angular_z,
                )
                decision_targets = configuration_targets(
                    linear_x=self._initial_reference_linear_x,
                    angular_z=adapted_reference_angular,
                    configuration=self._configuration,
                    wheelbase_m=self._wheelbase_m,
                    front_length_m=self._front_length_m,
                    rear_length_m=self._rear_length_m,
                    wheel_radius_m=self._wheel_radius_m,
                    steering_limit_rad=self._steering_limit_rad,
                )
            decision_front_error = (
                decision_targets.front_steering_rad - self._actual_front_physical
            )
            decision_rear_error = (
                decision_targets.rear_steering_rad - self._actual_rear_physical
            )
            if self._alignment_policy == "configuration_adaptive":
                self._alignment_severity = configuration_alignment_severity(
                    self._configuration,
                    decision_front_error,
                    decision_rear_error,
                    self._front_only_threshold,
                    self._rear_only_threshold,
                    self._coordinated_common_threshold,
                    self._coordinated_counter_threshold,
                )
                common_error, counter_error = coordinated_alignment_components(
                    decision_front_error, decision_rear_error
                )
                risk_text = (
                    f"severity={self._alignment_severity:.3f}, "
                    f"common_error={math.degrees(common_error):+.3f} deg, "
                    f"counter_error={math.degrees(counter_error):+.3f} deg"
                )
            else:
                risk_error = leading_axle_alignment_error(
                    decision_front_error,
                    decision_rear_error,
                    self._initial_reference_linear_x
                    if self._have_initial_reference_command
                    else requested_linear,
                )
                self._alignment_severity = (
                    abs(risk_error) / self._alignment_zero_speed_error
                )
                risk_text = f"risk_error={math.degrees(risk_error):+.3f} deg"
            self._adaptive_requires_alignment = self._alignment_severity > 1.0
            if not self._adaptive_requires_alignment:
                self._initial_alignment_complete = True
                self.get_logger().info(
                    "Adaptive initial policy selected immediate drive: "
                    f"{risk_text}, "
                    f"configuration={self._configuration}"
                )
            else:
                self.get_logger().info(
                    "Adaptive initial policy selected misalignment scaling: "
                    f"{risk_text}, "
                    f"configuration={self._configuration}"
                )
        if self._alignment_policy != "simultaneous" and not self._initial_alignment_complete:
            within = (
                abs(front_alignment_error) <= self._alignment_tolerance
                and abs(rear_alignment_error) <= self._alignment_tolerance
            )
            now_ns = self.get_clock().now().nanoseconds
            if within:
                if self._alignment_within_since_ns is None:
                    self._alignment_within_since_ns = now_ns
                elif (now_ns - self._alignment_within_since_ns) / 1.0e9 >= self._alignment_hold_s:
                    self._initial_alignment_complete = True
                    self.get_logger().info(
                        "Initial steering alignment complete; drive released: "
                        f"front_error={math.degrees(front_alignment_error):+.3f} deg, "
                        f"rear_error={math.degrees(rear_alignment_error):+.3f} deg"
                    )
            else:
                self._alignment_within_since_ns = None

        alignment_scale = 1.0
        if not self._initial_alignment_complete:
            if self._alignment_policy == "steer_then_drive":
                alignment_scale = 0.0
            elif self._alignment_policy == "misalignment_scaled":
                alignment_scale = initial_alignment_speed_scale(
                    max(abs(front_alignment_error), abs(rear_alignment_error)),
                    self._alignment_zero_speed_error,
                )
            elif self._alignment_policy in (
                "leading_axle_adaptive",
                "configuration_adaptive",
            ):
                if self._alignment_policy == "configuration_adaptive":
                    self._alignment_severity = configuration_alignment_severity(
                        self._configuration,
                        front_alignment_error,
                        rear_alignment_error,
                        self._front_only_threshold,
                        self._rear_only_threshold,
                        self._coordinated_common_threshold,
                        self._coordinated_counter_threshold,
                    )
                    alignment_scale = clamp(1.0 - self._alignment_severity, 0.0, 1.0)
                else:
                    alignment_scale = initial_alignment_speed_scale(
                        max(abs(front_alignment_error), abs(rear_alignment_error)),
                        self._alignment_zero_speed_error,
                    )
        drive_requested_linear = requested_linear * alignment_scale
        self._commanded_linear_x = move_toward(
            self._commanded_linear_x, drive_requested_linear, self._max_linear_step
        )
        if abs(requested_linear) >= 1.0e-6:
            commanded_angular = (
                adapted_requested_angular
                * self._commanded_linear_x
                / requested_linear
            )
        else:
            commanded_angular = 0.0
        targets = configuration_targets(
            linear_x=self._commanded_linear_x,
            angular_z=commanded_angular,
            configuration=self._configuration,
            wheelbase_m=self._wheelbase_m,
            front_length_m=self._front_length_m,
            rear_length_m=self._rear_length_m,
            wheel_radius_m=self._wheel_radius_m,
            steering_limit_rad=self._steering_limit_rad,
        )

        stamp = self.get_clock().now().to_msg()
        steering = JointState()
        steering.header.stamp = stamp
        steering.name = ["front_steering_joint", "rear_steering_joint"]
        steering.position = [
            self._raw_zero_rad + self._front_physical,
            self._raw_zero_rad + self._rear_physical,
        ]
        self._command_pub.publish(steering)

        drive = JointState()
        drive.header.stamp = stamp
        drive.name = ["front_wheel_joint", "rear_wheel_joint"]
        drive.velocity = [targets.front_wheel_rad_s, targets.rear_wheel_rad_s]
        self._command_pub.publish(drive)

        status = String()
        status.data = (
            f"configuration={self._configuration},v={self._commanded_linear_x:.4f},"
            f"omega={commanded_angular:.4f},"
            f"front_deg={math.degrees(self._front_physical):.3f},"
            f"rear_deg={math.degrees(self._rear_physical):.3f},fresh={self._command_is_fresh()}"
            f",initialized={self._motion_initialized},aligned={self._initial_alignment_complete},"
            f"alignment_policy={self._alignment_policy},alignment_scale={alignment_scale:.4f},"
            f"alignment_severity={self._alignment_severity:.4f}"
        )
        self._status_pub.publish(status)

    def stop_robot(self) -> None:
        """Publish an explicit zero drive command before node shutdown."""
        drive = JointState()
        drive.header.stamp = self.get_clock().now().to_msg()
        drive.name = ["front_wheel_joint", "rear_wheel_joint"]
        drive.velocity = [0.0, 0.0]
        self._command_pub.publish(drive)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ConfigurationController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
