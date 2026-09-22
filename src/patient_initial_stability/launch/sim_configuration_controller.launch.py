from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("patient_initial_stability"),
        "config",
        "controller.yaml",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "initial_alignment_policy",
                default_value="simultaneous",
                description=(
                    "simultaneous, steer_then_drive, misalignment_scaled, "
                    "leading_axle_adaptive, or configuration_adaptive"
                ),
            ),
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="/sim/cmd_vel",
                description="Twist command input; use /cmd_vel for Nav2 integration",
            ),
            DeclareLaunchArgument(
                "initial_alignment_zero_speed_error_deg",
                default_value="20.0",
                description=(
                    "Screening threshold: select scaled start above this "
                    "configuration-dependent steering error"
                ),
            ),
            DeclareLaunchArgument(
                "front_only_alignment_threshold_deg",
                default_value="35.0",
                description="Front-only steering mismatch threshold",
            ),
            DeclareLaunchArgument(
                "rear_only_alignment_threshold_deg",
                default_value="35.0",
                description="Rear-only steering mismatch threshold",
            ),
            DeclareLaunchArgument(
                "coordinated_common_threshold_deg",
                default_value="20.0",
                description="Coordinated same-phase (crab) mismatch threshold",
            ),
            DeclareLaunchArgument(
                "coordinated_counter_threshold_deg",
                default_value="35.0",
                description="Coordinated counterphase curvature mismatch threshold",
            ),
            Node(
                package="patient_initial_stability",
                executable="configuration_controller",
                name="steering_configuration_controller",
                output="screen",
                parameters=[
                    config,
                    {
                        "initial_alignment_policy": LaunchConfiguration(
                            "initial_alignment_policy"
                        ),
                        "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                        "initial_alignment_zero_speed_error_deg": LaunchConfiguration(
                            "initial_alignment_zero_speed_error_deg"
                        ),
                        "front_only_alignment_threshold_deg": LaunchConfiguration(
                            "front_only_alignment_threshold_deg"
                        ),
                        "rear_only_alignment_threshold_deg": LaunchConfiguration(
                            "rear_only_alignment_threshold_deg"
                        ),
                        "coordinated_common_threshold_deg": LaunchConfiguration(
                            "coordinated_common_threshold_deg"
                        ),
                        "coordinated_counter_threshold_deg": LaunchConfiguration(
                            "coordinated_counter_threshold_deg"
                        ),
                    },
                ],
            )
        ]
    )
