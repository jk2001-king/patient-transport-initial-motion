from setuptools import find_packages, setup


package_name = "patient_initial_stability"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/sim_configuration_controller.launch.py",
                "launch/sim_nav2_bringup.launch.py",
            ],
        ),
        (
            "share/" + package_name + "/config",
            ["config/controller.yaml", "config/nav2_sim.yaml"],
        ),
        (
            "share/" + package_name + "/maps",
            ["maps/empty_lab.pgm", "maps/empty_lab.yaml"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jk",
    maintainer_email="jk@example.com",
    description="Initial steering-configuration control experiments.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "configuration_controller = patient_initial_stability.configuration_controller:main",
            "drive_sanity_runner = patient_initial_stability.drive_sanity_runner:main",
            "reference_path_tracker = patient_initial_stability.reference_path_tracker:main",
            "initial_steering_preparer = patient_initial_stability.initial_steering_preparer:main",
            "nav2_goal_runner = patient_initial_stability.nav2_goal_runner:main",
        ],
    },
)
