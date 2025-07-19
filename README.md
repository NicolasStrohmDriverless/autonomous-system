# Autonomous System

Repository for the Autonomous System of Strohm und Söhne e.V.

## Structure of Autonomous System Repositories
The codebase of the Autonomous System is divided into different subfolders dedicated to documentation (docs), environment setups (.devcontainer) and source code (src). Last can be distinguished into simulator nodes (fsds), autonomous logic (subsystems) and peripheral connectors (drivers). Source code is mainly consistent of ROS2 packages, since we use it as out middleware. The single package descriptions can be found below.

### Lidar Cone Detection

Contains the implementation for the --INSERT LIDAR-- Lidar for cone recognition, which is used for the cone recognition of the test vehicle. This ROS node outputs the perceived pointcloud in topic --INSERT TOPIC--.

### FSDS rosbridge

The Formula Student Driverless Simulator (FSDS) is our main tool of simulation. The interface is realized with rosbridge, which connects it the other modules.

### Camera Cone Detection

Contains the implementation for the --INSERT CAMERA-- Stereo camera. This ROS node outputs the --??--.

### Cone Fusion

This ROS node fuses the Lidar data with the camera data and publishes the result as --FORMAT?-- in topic --TOPIC?--

### Path Planning

This ROS node uses the CAR_STATE from the SLAM and calculates which path the vehicle should take. It outputs the result as --FORMAT?-- in topic --TOPIC?--

## Architecture

![](.drawio/architecture.png)

## Example System Workflow

![](.drawio/system_workflow.png)

## Quick Start

To run the provided ROS2 nodes, first build and source the workspace:

```bash
cd ~/autonomous-system
colcon build
source install/setup.bash
```

Afterwards you can start the multi-node launcher:

```bash
ros2 run random_cone_detect multi_node_main
```

The `car_state_node` provides a `max_yaw_accel` parameter which limits the
angular acceleration used when updating the vehicle pose. Adjust this parameter
when launching the node to modify the vehicle's turning behaviour.

To avoid conflicting commands, braking pressure is only applied when the
desired speed is zero. When a path ends, the controller gradually reduces the
actual speed until it reaches exactly zero before releasing the brake.

The cone detection node offers a `DISTANCE_NOISE` toggle to enable
distance-dependent position noise. The amount of deviation scales with the
detected cone's range and is capped by the `MAX_DISTORTION_30M` setting,
representing the maximum offset applied at a distance of 30&nbsp;m.

If `colcon build` is not executed or the `setup.bash` file is not sourced, you
may see import errors such as ``ModuleNotFoundError: No module named
'ebs_active'``.

## Documentation

If you have access to this Repository you also can have a look at the documentation [here](strohm-und-soehne.gitlab.io/driverless/autonomous-system/). This includes Onboarding and installation.
