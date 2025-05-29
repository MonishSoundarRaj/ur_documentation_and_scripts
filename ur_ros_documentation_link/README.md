# UR5e ROS2 Driver Setup Guide

## Overview

This guide provides step-by-step instructions for setting up and using the Universal Robots ROS2 driver with a UR5e robot. This approach is recommended for advanced robotics applications requiring motion planning, simulation, and integration with the broader ROS ecosystem.

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [System Setup](#system-setup)
3. [ROS2 Installation](#ros2-installation)
4. [UR Driver Installation](#ur-driver-installation)
5. [Basic Operation](#basic-operation)
6. [MoveIt Integration](#moveit-integration)
7. [Troubleshooting](#troubleshooting)

## Prerequisites

### Knowledge Requirements

- Basic Linux command line usage
- Understanding of ROS2 concepts (nodes, topics, services)
- Python programming fundamentals

### Software Dependencies

- Ubuntu 22.04 LTS
- ROS2 Humble (recommended) or Galactic
- Git and build tools

## System Setup

### 1. Update System

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install curl gnupg lsb-release software-properties-common
```

### 2. Network Configuration

Follow the network setup from the [main README](../README.md#network-configuration), then verify connectivity:

```bash
# Test robot connection
ping 192.168.1.102

# Test URCap port (should show connection refused - that's normal)
telnet 192.168.1.102 50002
```

## ROS2 Installation

### Option A: Binary Installation (Recommended)

```bash
# Add ROS2 repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop python3-argcomplete python3-colcon-common-extensions python3-rosdep python3-vcstool

# Initialize rosdep
sudo rosdep init
rosdep update
```

### Option B: Source Installation (Advanced Users)

Follow the detailed instructions at [ROS2 Humble Source Installation](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html).

### 3. Environment Setup

```bash
# Add to ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify installation
ros2 --help
```

## UR Driver Installation

### 1. Create Workspace

```bash
mkdir -p ~/ur_ws/src
cd ~/ur_ws/src
```

### 2. Download UR Driver

```bash
# Clone the UR driver repository
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git

# Install dependencies
cd ~/ur_ws
rosdep install --ignore-src --from-paths src -y -r
```

### 3. Build the Workspace

```bash
cd ~/ur_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace
echo "source ~/ur_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 4. Verify Installation

```bash
# Check if packages are available
ros2 pkg list | grep ur_

# Expected output should include:
# ur_bringup
# ur_calibration
# ur_controllers
# ur_dashboard_msgs
# ur_moveit_config
# ur_robot_driver
```

## Basic Operation

### 1. Prepare Robot

1. **Power on the UR5e robot**
2. **Release emergency stop**
3. **Initialize robot** (if required)

### 2. Start ROS2 Driver

```bash
# Terminal 1: Start the driver
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    robot_ip:=192.168.1.102 \
    launch_rviz:=true \
    kinematics_config:="${HOME}/ur_calibration/ur5e_calibration.yaml"
```

### 3. Start Robot Program

On the UR teach pendant:

1. **Load External Control program**:
   - Go to `Program` in pendant and add `External Control` (under `URCaps`) to `Robot Programs`
   - Set Host IP to: `192.168.1.101` (your PC IP)
2. Press **Play** to start external control

### 4. Verify Connection

```bash
# Terminal 2: Check robot state
ros2 topic echo /joint_states

# Terminal 3: Check available controllers
ros2 control list_controllers
```

You should see output showing joint positions and active controllers.

## MoveIt Integration

### 1. Install MoveIt2

```bash
sudo apt install ros-humble-moveit
```

### 2. Start MoveIt with UR5e

```bash
# Terminal 1: Start robot driver (if not already running)
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    robot_ip:=192.168.1.102 \
    kinematics_config:="${HOME}/ur_calibration/ur5e_calibration.yaml"

# Terminal 2: Start MoveIt
ros2 launch ur_moveit_config ur_moveit.launch.py \
    ur_type:=ur5e \
    launch_rviz:=true
```

### 3. Using MoveIt Planning

1. **In RViz2**, you should see the UR5e robot model
2. **Add** → **MotionPlanning** plugin if not already visible
3. **Planning Group**: Select "ur_manipulator"
4. **Drag the interactive markers** to set goal pose
5. **Plan** to calculate trajectory
6. **Execute** to move the robot

### 4. MoveIt Python Interface

Create a simple script to test MoveIt:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import MoveItErrorCodes
from pymoveit2 import MoveIt2
from geometry_msgs.msg import Pose

class UR5eMoveItExample(Node):
    def __init__(self):
        super().__init__('ur5e_moveit_example')

        # Initialize MoveIt2
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=[
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint"
            ],
            base_link_name="base_link",
            end_effector_name="tool0"
        )

    def move_to_pose(self, x, y, z, roll, pitch, yaw):
        """Move robot to specified pose"""
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        # Convert RPY to quaternion (simplified)
        from scipy.spatial.transform import Rotation as R
        r = R.from_euler('xyz', [roll, pitch, yaw])
        quat = r.as_quat()

        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]

        # Plan and execute
        self.moveit2.move_to_pose(pose)

def main():
    rclpy.init()
    node = UR5eMoveItExample()

    # Example movement
    node.move_to_pose(0.3, 0.2, 0.4, 0, 0, 0)

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Troubleshooting

### Common Issues

#### 1. "Cannot connect to robot"

**Symptoms:** Driver fails to connect, timeout errors

**Solutions:**

```bash
# Check network connectivity
ping 192.168.1.102

# Check robot is in Remote Control mode
# On teach pendant: go to Settings → System → Remote Control → Enable

# Verify robot program is running
# External Control program must be active on robot
```

#### 2. "Robot emergency stop active"

**Symptoms:** Robot doesn't move, safety violations

**Solutions:**

- Release emergency stop button
- Check safety configuration on teach pendant
- Verify workspace limits

#### 3. "Joint trajectory controller not found"

**Symptoms:** Cannot send motion commands

**Solutions:**

```bash
# Check available controllers
ros2 control list_controllers

# Restart driver if needed
# Kill existing driver and restart with:
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.1.102
```

#### 4. "MoveIt planning fails"

**Symptoms:** No valid motion plans found

**Solutions:**

- Check for collisions in planning scene
- Verify goal pose is reachable
- Adjust planning parameters

## Resources

### Official Documentation

- [Universal Robots ROS2 Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
- [ROS2 Control Framework](https://control.ros.org/humble/index.html)
- [MoveIt2 Documentation](https://moveit.picknik.ai/main/index.html)

### Tutorials and Examples

- [MoveIt2 Tutorials](https://moveit.picknik.ai/main/doc/tutorials/tutorials.html)

### Community Support

- [ROS Discourse](https://discourse.ros.org/)

---
