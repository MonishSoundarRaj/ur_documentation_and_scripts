# UR5e Control and Documentation

## Overview

This repository provides comprehensive documentation and control implementations for Universal Robots UR5e robotic arm. It contains two main approaches for robot control:

1. **ROS2 Integration** (`ur_ros_documentation_link/`) - Using the official Universal Robots ROS2 driver
2. **RTDE Control** (`ur_rtde/`) - Direct robot control using Real-Time Data Exchange

This documentation is designed for students learning robotic manipulation and control systems.

## Repository Structure

```
ur5e_control_and_documentation/
├── README.md                     # This file - main overview
├── ur_ros_documentation_link/    # ROS2 driver documentation and setup
│   └── README.md                 # Detailed ROS2 setup guide
├── ur_rtde/                      # Direct RTDE control implementation
│   ├── README.md                 # RTDE setup and usage guide
│   ├── robot_control_main.py     # Main pick-and-place script
│   ├── robot_control_home.py     # Home position utility
│   ├── robot_functions.py        # Robot control class library
│   └── robotiq_gripper.py        # Robotiq gripper interface
```

## Quick Start Guide

### Prerequisites

- UR5e robot with PolyScope 5.1+ (for e-Series)
- Computer with Ubuntu 20.04+ or Windows 10+
- Network connection to the robot
- Basic knowledge of Python programming

### Which Approach to Choose?

| Approach | Best For                                             | Complexity | Features                                                  |
| -------- | ---------------------------------------------------- | ---------- | --------------------------------------------------------- |
| **ROS2** | Research, complex planning, multi-robot systems      | High       | MoveIt planning, visualization, extensive ecosystem       |
| **RTDE** | Simple automation, direct control, quick prototyping | Low        | Real-time control, simple scripting, direct communication |

## Hardware Setup

### Network Configuration

1. **Robot IP Configuration** (on UR tablet):

   - Go to `Setup Robot` → `Network`
   - Set IP address: `192.168.1.102`
   - Subnet mask: `255.255.255.0`
   - Gateway: `192.168.1.1`

2. **Computer Network Setup**:

   - Create new wired connection profile
   - IP address: `192.168.1.101`
   - Subnet mask: `255.255.255.0`
   - Gateway: `192.168.1.1`

3. **Test Connection**:
   ```bash
   ping 192.168.1.102
   ```

### Robot Preparation (This is already done, skip this)

1. **Install External Control URCap**:

   - Download from [Universal Robots GitHub](https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases)
   - Install via UR tablet: `Program` → `URCaps` → `+`

2. **Safety Configuration**:
   - Set appropriate safety limits
   - Configure tool center point (TCP)
   - Calibrate robot if needed

## Control Methods

### Option 1: RTDE Direct Control (Recommended for Beginners)

**Features:**

- Direct Python scripting
- Real-time control
- Simple pick-and-place operations
- Gripper integration
- Quick prototyping

**Setup Time:** ~30 minutes  
**Learning Curve:** Gentle (basic Python knowledge)

👉 **[Complete RTDE Setup Guide](ur_rtde/README.md)**

### Option 2: ROS2 Driver (Recommended for Research)

**Features:**

- Complete ROS2 ecosystem integration
- MoveIt motion planning
- Gazebo simulation support
- Advanced trajectory planning
- Multi-robot coordination

**Setup Time:** ~2-3 hours  
**Learning Curve:** Steep (requires ROS2 knowledge)

👉 **[Complete ROS2 Setup Guide](ur_ros_documentation_link/README.md)**

## External Resources

### Official Documentation

- [Universal Robots ROS2 Driver](https://docs.ros.org/en/humble/p/ur_robot_driver/doc/installation/robot_setup.html)
- [UR Robot Manual](https://www.universal-robots.com/download/manuals-e-series/user/ur-user-manual-e-series-sw51/)
- [PolyScope Manual](https://www.universal-robots.com/download/manuals-e-series/script/script-manual-e-series-sw51/)

### Community Resources

- [Universal Robots ROS Driver GitHub](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
- [ur_rtde Library](https://gitlab.com/sdurobotics/ur_rtde)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)

### Video Tutorials

- [MoveIt2 with UR Robots](https://youtu.be/your-video-link)
- [UR Robot Programming Basics](https://academy.universal-robots.com/)

## Troubleshooting

### Common Issues

1. **Cannot connect to robot**

   - Check network configuration
   - Verify robot IP address
   - Ensure firewall allows connections

2. **Robot movements are jerky**

   - Reduce acceleration/velocity
   - Check network latency
   - Verify RTDE frequency settings

### Getting Help

- Check the specific README files for detailed troubleshooting
- Review UR community forums
