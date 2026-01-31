# Real Robot Usage Guide

This guide details how to run the software on the real physical robot and ensure visualization on your remote PC (laptop).

## Prerequisites

### 1. Networking Setup
Ensure both the **Robot** (ROSOrin) and your **Laptop** are on the same Wi-Fi network.
Check the `ROS_DOMAIN_ID` on both machines to match (usually `0` or `30`).

**On Robot & Laptop:**
```bash
echo $ROS_DOMAIN_ID
# If empty, it defaults to 0. If set, they must match.
```

## Running on the Robot
SSH into the robot (e.g., `ssh ubuntu@<robot_ip>`) and run:

```bash
# 1. Bringup (Hardware Drivers)
# This launches: Controller, Lidar, Camera, Web Server, App, and Joystick Control
# First stop the app that is running when the robot boots up
sudo systemctl stop start_app_node.service
ros2 launch bringup bringup.launch.py
```

> **Note**: The `bringup.launch.py` automatically includes `joystick_control.launch.py`, so if your joystick is plugged into the robot, it should work immediately.

## Running on the Laptop (Remote Visualization)

To visualize the robot's data (Lidar/Camera) on your laptop without running the heavy simulation:

### 1. Install & Build
Ensure you have the `rosorin_description` package (at minimum) on your laptop to visualize the robot model correctly.

```bash
cd ~/ros2_ws
colcon build --packages-select rosorin_description
source install/setup.bash
```

### 2. Launch RViz
You can use the same visualization launch file we used for simulation, as it just launches RViz with a config.

```bash
source install/setup.bash

# Ensure accurate time sync with the robot
sudo ntpdate -u <robot_ip> 

# Launch RViz
ros2 launch robot_gazebo visualize.launch.py
```
*You should see the live Lidar scan and Camera stream from the real robot.*

## Joystick Control

- **Connected to Robot**: If the joystick is plugged into the robot USB, it runs automatically with `bringup.launch.py`.
- **Connected to Laptop**: If you want to drive from your laptop's joystick:
    1.  Ensure `bringup` is running on the robot.
    2.  On laptop:
        ```bash
        sudo apt install ros-humble-teleop-twist-joy
        ros2 launch teleop_twist_joy teleop-launch.py joy_config:='ps3' # or xbox
        ```
        *(Note: You may need to remap the output topic to `/controller/cmd_vel` or `/cmd_vel` depending on the robot's configuration).*
