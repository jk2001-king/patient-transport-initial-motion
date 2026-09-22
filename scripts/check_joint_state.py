"""Print the patient transport robot's articulation DOF names and states.

Run this file inside Isaac Sim's Script Editor after pressing Play:

    exec(open("/home/jk/Research/patient_initial_ws/scripts/check_joint_state.py").read())
"""

from isaacsim.core.prims import SingleArticulation


ARTICULATION_ROOT = "/World/patient_transport/base_link"
JOINTS_TO_REPORT = (
    "front_steering_joint",
    "rear_steering_joint",
    "front_wheel_joint",
    "rear_wheel_joint",
)


robot = SingleArticulation(ARTICULATION_ROOT)
robot.initialize()

dof_names = list(robot.dof_names)
positions = robot.get_joint_positions()
velocities = robot.get_joint_velocities()
name_to_index = {name: index for index, name in enumerate(dof_names)}

print("\n=== Patient transport articulation ===")
print("articulation root:", ARTICULATION_ROOT)
print("DOF count:", len(dof_names))
print("DOF names:", dof_names)
print("position [rad]:", positions)
print("velocity [rad/s]:", velocities)

print("\n=== Active joint states ===")
for joint_name in JOINTS_TO_REPORT:
    if joint_name not in name_to_index:
        print(f"MISSING: {joint_name}")
        continue

    joint_index = name_to_index[joint_name]
    print(
        f"{joint_name}: index={joint_index}, "
        f"position={positions[joint_index]:.6f} rad, "
        f"velocity={velocities[joint_index]:.6f} rad/s"
    )

print("\nCopy everything from the first === line through this line.")
