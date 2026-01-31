# Simulation Usage Guide

This guide details how to run the full Gazebo simulation, control the robot, and visualize sensor data.

## Prerequisites

Ensure you have built the workspace and sourced the setup script:

```bash
cd /workspaces/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Running the Components

Run each of these commands in a **separate terminal window**.

### 1. Launch Simulation (The Robot)
This starts Gazebo, spawns the robot in the `robocup_home` world, and enables the physics and sensor plugins.

```bash
source install/setup.bash
export MACHINE_TYPE=ROSOrin_Mecanum

# Fix for Gazebo resource path if models are missing
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(pwd)/install/rosorin_description/share

ros2 launch robot_gazebo worlds.launch.py
```
*Wait for the Gazebo window to open and the simulation to start.*

### 2. Launch Joystick Control
This starts the `joy_node` driver and our custom `joystick_control` node to translate gamepad inputs into robot velocity commands.

```bash
source install/setup.bash
export MACHINE_TYPE=ROSOrin_Mecanum

ros2 launch peripherals joystick_teleop.launch.py
```
*Move the left joystick to drive (strafe enabled) and right joystick to rotate.*

### 3. Launch Visualization (RViz)
This opens RViz2 with a pre-configured view showing:
- **Robot Model**: The URDF visualization.
- **Lidar**: Red laser scan lines (`/scan`).
- **Camera**: Live video feed from the robot's depth camera (`/depth_cam/depth_cam`).
- **TF**: Coordinate frames.

```bash
source install/setup.bash

ros2 launch robot_gazebo visualize.launch.py
```

## Troubleshooting

- **Robot not moving?** 
    - Ensure `joystick_teleop.launch.py` is running and your joystick is active (press the "Mode" or "Enable" button if applicable).
    - Check if the simulation is paused in Gazebo (bottom left play button).
- **No Camera image in RViz?**
    - The camera simulation uses Ignition Gazebo sensors. Ensure the `ros_ign_bridge` is running (it is included in `worlds.launch.py`).
    - Verify the static transform for `camera_link0` is being published (check terminal output of the simulation window).
