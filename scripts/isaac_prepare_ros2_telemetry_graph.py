"""Create a temporary ROS 2 telemetry graph for the patient transport robot.

Run this file in Isaac Sim's Script Editor while the Timeline is stopped.
The graph is authored only in the USD session layer and must not be saved.
"""

import omni.graph.core as og
import omni.kit.app
import omni.timeline
import omni.usd
import usdrt.Sdf
from pxr import Usd


GRAPH_PATH = "/World/PatientInitialMotionROSGraph"
ARTICULATION_PATH = "/World/patient_transport/base_link"


def prepare_ros2_telemetry_graph():
    timeline = omni.timeline.get_timeline_interface()
    if timeline.is_playing():
        raise RuntimeError("Press Stop before preparing the ROS 2 telemetry graph")

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("No USD stage is open")
    if not stage.GetPrimAtPath(ARTICULATION_PATH).IsValid():
        raise RuntimeError(f"Missing articulation root: {ARTICULATION_PATH}")

    extension_manager = omni.kit.app.get_app().get_extension_manager()
    if not extension_manager.is_extension_enabled("isaacsim.ros2.bridge"):
        raise RuntimeError(
            "ROS 2 Bridge is disabled. Enable isaacsim.ros2.bridge in "
            "Window > Extensions, then run this script again."
        )

    session_layer = stage.GetSessionLayer()
    with Usd.EditContext(stage, session_layer):
        if stage.GetPrimAtPath(GRAPH_PATH).IsValid():
            stage.RemovePrim(GRAPH_PATH)

        keys = og.Controller.Keys
        og.Controller.edit(
            {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
                    ("ComputeOdometry", "isaacsim.core.nodes.IsaacComputeOdometry"),
                    ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                    ("PublishOdometry", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                    ("PublishOdomTF", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                    ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                    ("SubscribeJointCommand", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                    ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                ],
                keys.SET_VALUES: [
                    ("ROS2Context.inputs:useDomainIDEnvVar", True),
                    (
                        "ComputeOdometry.inputs:chassisPrim",
                        [usdrt.Sdf.Path(ARTICULATION_PATH)],
                    ),
                    ("PublishClock.inputs:topicName", "/clock"),
                    ("PublishOdometry.inputs:topicName", "/sim/ground_truth/odom"),
                    ("PublishOdometry.inputs:odomFrameId", "odom"),
                    ("PublishOdometry.inputs:chassisFrameId", "base_link"),
                    ("PublishOdometry.inputs:robotFront", [1.0, 0.0, 0.0]),
                    ("PublishOdometry.inputs:publishRawVelocities", False),
                    ("PublishOdomTF.inputs:topicName", "/tf"),
                    ("PublishOdomTF.inputs:parentFrameId", "odom"),
                    ("PublishOdomTF.inputs:childFrameId", "base_link"),
                    ("PublishOdomTF.inputs:staticPublisher", False),
                    ("PublishJointState.inputs:topicName", "/joint_states"),
                    (
                        "PublishJointState.inputs:targetPrim",
                        [usdrt.Sdf.Path(ARTICULATION_PATH)],
                    ),
                    ("SubscribeJointCommand.inputs:topicName", "/sim/joint_command"),
                    ("ArticulationController.inputs:robotPath", ARTICULATION_PATH),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "ComputeOdometry.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "SubscribeJointCommand.inputs:execIn"),
                    ("ComputeOdometry.outputs:execOut", "PublishOdometry.inputs:execIn"),
                    ("ComputeOdometry.outputs:execOut", "PublishOdomTF.inputs:execIn"),
                    ("ROS2Context.outputs:context", "PublishClock.inputs:context"),
                    ("ROS2Context.outputs:context", "PublishOdometry.inputs:context"),
                    ("ROS2Context.outputs:context", "PublishOdomTF.inputs:context"),
                    ("ROS2Context.outputs:context", "PublishJointState.inputs:context"),
                    ("ROS2Context.outputs:context", "SubscribeJointCommand.inputs:context"),
                    ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                    ("ReadSimTime.outputs:simulationTime", "PublishOdometry.inputs:timeStamp"),
                    ("ReadSimTime.outputs:simulationTime", "PublishOdomTF.inputs:timeStamp"),
                    ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                    ("ComputeOdometry.outputs:position", "PublishOdometry.inputs:position"),
                    ("ComputeOdometry.outputs:orientation", "PublishOdometry.inputs:orientation"),
                    ("ComputeOdometry.outputs:position", "PublishOdomTF.inputs:translation"),
                    ("ComputeOdometry.outputs:orientation", "PublishOdomTF.inputs:rotation"),
                    (
                        "ComputeOdometry.outputs:linearVelocity",
                        "PublishOdometry.inputs:linearVelocity",
                    ),
                    (
                        "ComputeOdometry.outputs:angularVelocity",
                        "PublishOdometry.inputs:angularVelocity",
                    ),
                    (
                        "SubscribeJointCommand.outputs:execOut",
                        "ArticulationController.inputs:execIn",
                    ),
                    (
                        "SubscribeJointCommand.outputs:jointNames",
                        "ArticulationController.inputs:jointNames",
                    ),
                    (
                        "SubscribeJointCommand.outputs:positionCommand",
                        "ArticulationController.inputs:positionCommand",
                    ),
                    (
                        "SubscribeJointCommand.outputs:velocityCommand",
                        "ArticulationController.inputs:velocityCommand",
                    ),
                    (
                        "SubscribeJointCommand.outputs:effortCommand",
                        "ArticulationController.inputs:effortCommand",
                    ),
                ],
            },
        )

    print("ROS 2 telemetry graph prepared in the session layer.")
    print(f"graph: {GRAPH_PATH}")
    print("ROS_DOMAIN_ID is read from Isaac Sim's launch environment.")
    print("publishers: /clock, /sim/ground_truth/odom, /joint_states, /tf")
    print("subscriber: /sim/joint_command (sensor_msgs/msg/JointState)")
    print("Press Play, then verify the topics from a ROS 2 terminal.")
    print("Do not save the USD file.")


print("ROS 2 telemetry graph helper loaded. Timeline must be stopped.")
