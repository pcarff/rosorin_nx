# ROS2 Mecanum Simulation & Navigation

This repository contains the ROS2 workspace for simulating and controlling a Mecanum-wheeled robot in a Gazebo environment. It includes configurations for the robot model, simulation worlds, joystick teleoperation, and visualization tools.

## Prerequisites

- **ROS 2 Distribution**: Humble Hawksbill (or compatible)
- **OS**: Ubuntu 22.04 LTS (Jammy Jellyfish)
- **Gazebo**: Ignition Fortress / Gazebo Sim

## Getting Started

### 1. Build the Workspace

Clone the repository and build the packages:

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 2. Launch Simulation

To start the Gazebo simulation environment with the `robocup_home` world (which includes obstacles for Lidar/Camera testing):

```bash
# Set the robot type
export MACHINE_TYPE=ROSOrin_Mecanum
# Add local resources to the Gazebo path
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(pwd)/install/rosorin_description/share

# Launch the world and robot spawners
ros2 launch robot_gazebo worlds.launch.py
```

### 3. Visualization (RViz)

To visualize the robot, Lidar scan, and Camera feed, open a new terminal:

```bash
source install/setup.bash
# Launches RViz configured with the robot model, LaserScan, and Camera display
ros2 launch robot_gazebo visualize.launch.py
```

### 4. Joystick Control

To drive the robot using a connected USB gamepad/joystick:

1.  Connect your joystick.
2.  Open a new terminal and run:

```bash
source install/setup.bash
export MACHINE_TYPE=ROSOrin_Mecanum

# Launches joy_node and the teleop translator
ros2 launch peripherals joystick_teleop.launch.py
```

> **Note**: If the robot does not move, ensure your joystick is detected on `/dev/input/js0` and that you are holding the "enable" button if one is configured (usually L1/LB or similar depending on the mapping).

## Key Features & Fixes

- **Simulation**: Configured `robot_gazebo` to use `ros_gz_sim`. Implemented robust holonomic movement using `VelocityControl` plugin and simplified wheel physics (frictionless) for reliable strafing and rotation.
- **Hardware Interface**: Enabled `gz_ros2_control` for simulated hardware abstraction.
- **Teleoperation**: Added `joystick_teleop.launch.py` and patched `joystick_control.py` to ensure continuous command publishing (heartbeat).
- **Visualization**: Fixed camera frame transformation issues in RViz using static publisher updates.

## Repository Structure

- `src/robot_gazebo`: Simulation worlds, launch files, and URDF configurations.
- `src/rosorin_description`: Robot mesh and physical description files.
- `src/peripherals`: Hardware drivers and teleoperation nodes (Joystick, Lidar, Camera).
- `src/driver`: Low-level SDKs and controllers.

## License

[Add License Here]
