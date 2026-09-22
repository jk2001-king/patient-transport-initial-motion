import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("patient_initial_stability")
    nav2_share = get_package_share_directory("nav2_bringup")
    params_file = os.path.join(package_share, "config", "nav2_sim.yaml")
    map_file = os.path.join(package_share, "maps", "empty_lab.yaml")
    controller_config = os.path.join(package_share, "config", "controller.yaml")

    policy = LaunchConfiguration("initial_alignment_policy")
    configuration = LaunchConfiguration("configuration")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "initial_alignment_policy", default_value="configuration_adaptive"
            ),
            DeclareLaunchArgument(
                "configuration", default_value="coordinated_counterphase"
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom_static_tf",
                arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
                parameters=[{"use_sim_time": True}],
                output="screen",
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[params_file, {"yaml_filename": map_file}],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_map",
                output="screen",
                parameters=[
                    {"use_sim_time": True},
                    {"autostart": True},
                    {"node_names": ["map_server"]},
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_share, "launch", "navigation_launch.py")
                ),
                launch_arguments={
                    "use_sim_time": "true",
                    "autostart": "true",
                    "params_file": params_file,
                    "use_composition": "False",
                }.items(),
            ),
            Node(
                package="patient_initial_stability",
                executable="configuration_controller",
                name="steering_configuration_controller",
                output="screen",
                parameters=[
                    controller_config,
                    {
                        "configuration": configuration,
                        "cmd_vel_topic": "/cmd_vel",
                        "initial_reference_cmd_vel_topic": "/cmd_vel_nav",
                        "path_tracking_model": "configuration_aware",
                        "path_tracking_lookahead_m": 0.20,
                        "initial_alignment_policy": policy,
                        "front_only_alignment_threshold_deg": 35.0,
                        "rear_only_alignment_threshold_deg": 35.0,
                        "coordinated_common_threshold_deg": 20.0,
                        "coordinated_counter_threshold_deg": 35.0,
                    },
                ],
            ),
        ]
    )
