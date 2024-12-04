# Homework 3 - Robotics Lab 2024/2025

This document provides instructions for setting up simulations, running nodes, and implementing vision-based tasks for a 7-DOF robotic manipulator arm in Gazebo.

---

## Environment Setup

### Add the Gazebo Resource Path
Export the Gazebo resource path to include the required models:
```bash
export IGN_GAZEBO_RESOURCE_PATH=~/ros2_ws/src/Roboticslab24/Homework3/ros2_iiwa/iiwa_description/gazebo/models:$IGN_GAZEBO_RESOURCE_PATH
```

---

## Sphere Detection

### Launch the Robot with Vision Enabled
To start the robot with a camera-equipped end-effector and detect a sphere:

1. **Launch the Robot in the Gazebo Environment:**
   ```bash
   ros2 launch iiwa_bringup iiwa_sphere.launch.py use_sim:="true" command_interface:="effort" robot_controller:="effort_controller" use_vision:="true"
   ```

2. **Run the OpenCV Node for Sphere Detection:**
   ```bash
   ros2 run ros2_opencv ros2_opencv_node
   ```

---

## ArUco Marker Detection

### Launch the Robot for ArUco Detection
The following steps involve launching the robot with vision-enabled configurations for detecting ArUco markers.

1. **Launch in Velocity Command Interface:**
   ```bash
   ros2 launch iiwa_bringup iiwa.launch.py use_sim:="true" command_interface:="velocity" robot_controller:="velocity_controller" use_vision:="true"
   ```

2. **Launch in Effort Command Interface:**
   ```bash
   ros2 launch iiwa_bringup iiwa.launch.py use_sim:="true" command_interface:="effort" robot_controller:="effort_controller" use_vision:="true"
   ```

3. **Launch the ArUco Detection Node:**
   ```bash
   ros2 launch aruco_ros aruco_detection.launch.py
   ```

---

## Vision-Based Tasks

### Execute Vision-Based Tasks
Use the `ros2_kdl_package` to run specific vision-based tasks:

#### Velocity Command Interface:

- **Positioning Task:**
  ```bash
  ros2 run ros2_kdl_package ros2_kdl_vision_control --ros-args -p cmd_interface:="velocity" -p task:="positioning"
  ```

- **Look-at-Point Task:**
  ```bash
  ros2 run ros2_kdl_package ros2_kdl_vision_control --ros-args -p cmd_interface:="velocity" -p task:="look-at-point"
  ```

#### Effort Command Interface:

- **Positioning Task:**
  ```bash
  ros2 run ros2_kdl_package ros2_kdl_vision_control --ros-args -p cmd_interface:="effort" -p task:="positioning"
  ```

- **Look-at-Point Task:**
  ```bash
  ros2 run ros2_kdl_package ros2_kdl_vision_control --ros-args -p cmd_interface:="effort" -p task:="look-at-point"
  ```

---
