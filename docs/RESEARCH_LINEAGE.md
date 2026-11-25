# Research scope and successor projects

## Why these repositories remain separate

The research expanded from initial steering alignment into two different problems: local direction-reversal initiation control and continuous multi-steering path planning. A shared platform or motivation does not make the controllers, experiments or measured results interchangeable.

| Project | Core question | Implementation and evidence |
|---|---|---|
| [patient-transport-initial-motion](https://github.com/jk2001-king/patient-transport-initial-motion) | When may drive begin under a mismatch between current active steering and the first commanded motion? | Configuration-aware alignment/departure policies and ROS/Nav2 prototype |
| [micro-yaw-initial-motion-control](https://github.com/jk2001-king/micro-yaw-initial-motion-control) | Can micro-yaw initiation and body-yaw recovery reduce caster-realignment departure errors under load/CoM variation? | CICS: two laboratory platform models, fixed per-platform realization, original trial analysis |
| [multi-steering-motion-planning](https://github.com/jk2001-king/multi-steering-motion-planning) | Which hospital path and steering sequence remain executable when transitions and body response carry over? | IJCAS: state-history propagation, full planar envelope, speed refinement and H1–H6 comparison; separate H5/H6 sensor cases |

The separately maintained [transfer_robot_nav2](https://github.com/jk2001-king/transfer_robot_nav2) is a hardware navigation integration project, not proof that the simulation research results transfer unchanged to hardware.

## What is preserved here

The predecessor implementation at commit `2c70c92805a011680d3803a292734f3584dd58d8` includes:

- [`configuration_controller.py`](../src/patient_initial_stability/patient_initial_stability/configuration_controller.py): front/rear/counterphase kinematics and configuration-dependent initial-alignment policy.
- [`sim_nav2_bringup.launch.py`](../src/patient_initial_stability/launch/sim_nav2_bringup.launch.py): ROS/Nav2 prototype connection.
- [`robot.yaml`](../config/robot.yaml): this prototype's model/calibration settings.
- Earlier model inspection, steering calibration and research-plan documents in this directory.

The current documentation update retains the source and historical measurements. It does not import the final micro-yaw kernel, add IJCAS planning code, rerun the prototype experiments or retroactively validate its thresholds.

## Do not mix these claims

1. Lack of a repeatable caster effect in an early screening setup is not evidence that caster effects vanish under later platform/contact/load conditions.
2. CICS `P` is a response-terminated departure controller. IJCAS `P` denotes a path-planning configuration. Identical short labels do not mean identical algorithms.
3. The predecessor's geometry, steering calibration and screening percentages belong to that prototype. Use each successor's own models and measurements.
4. CICS trial counts and IJCAS task completion counts have different denominators; do not combine them into one success rate.
5. A physical platform photograph or a separate hardware navigation repository does not establish real-platform validation of either simulation paper.

Use this repository to inspect the precursor. Use the CICS repository for final micro-yaw experiments, and the IJCAS repository for final hospital planning experiments. Keeping all three preserves reproducibility and avoids silently changing what earlier results refer to.
