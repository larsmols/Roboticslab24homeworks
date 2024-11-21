# Homework 2 - Robotics Lab 2024/2025

This document provides instructions for running simulations and controlling the robot in various command interfaces and trajectory types.

---

## Build and Setup

### Build the Required Packages
Use the following command to build all necessary packages:
```bash
colcon build
```

### Source the Setup File
In every terminal you open, source the install directory:
```bash
. install/setup.bash
```

---

## Starting the Simulation

### Launch the Robot in Gazebo
To start the simulation, launch the robot in Gazebo using one of the following command interfaces and controllers:

#### Effort Controller:
```bash
ros2 launch iiwa_bringup iiwa.launch.py use_sim:="true" command_interface:="effort" robot_controller:="effort_controller"
```

#### Velocity Controller:
```bash
ros2 launch iiwa_bringup iiwa.launch.py use_sim:="true" command_interface:="velocity" robot_controller:="velocity_controller"
```

#### Position Controller:
```bash
ros2 launch iiwa_bringup iiwa.launch.py use_sim:="true" command_interface:="position" robot_controller:="iiwa_arm_controller"
```

**Note:** After launching, ensure Gazebo has loaded, then press the **Play** button in the lower-left corner to activate the controllers.

---

## Running the Node

### Command the Robot Using Different Trajectory Types

Run the `ros2_kdl_node` with specific trajectory configurations:

#### Effort Interface:

- **Circular Trapezoidal Trajectory:**
  ```bash
  ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p trajectory_type:="circular_trapezoidal" -p cmd_interface:="effort"
  ```

- **Circular Cubic Trajectory:**
  ```bash
  ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p trajectory_type:="circular_cubic" -p cmd_interface:="effort"
  ```

- **Linear Cubic Trajectory:**
  ```bash
  ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p trajectory_type:="linear_cubic" -p cmd_interface:="effort"
  ```

- **Linear Trapezoidal Trajectory:**
  ```bash
  ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p trajectory_type:="linear_trapezoidal" -p cmd_interface:="effort"
  ```

#### Velocity Interface:

- **Circular Trapezoidal (Velocity):**
  ```bash
  ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p trajectory_type:="circular_trapezoidal" -p cmd_interface:="velocity"
  ```

- **Circular Cubic (Velocity):**
  ```bash
  ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p trajectory_type:="circular_cubic" -p cmd_interface:="velocity"
  ```

- **Linear Cubic (Velocity):**
  ```bash
  ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p trajectory_type:="linear_cubic" -p cmd_interface:="velocity"
  ```

- **Linear Trapezoidal (Velocity):**
  ```bash
  ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p trajectory_type:="linear_trapezoidal" -p cmd_interface:="velocity"
  ```

#### Position Interface:

- **Circular Trapezoidal (Position):**
  ```bash
  ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p trajectory_type:="circular_trapezoidal" -p cmd_interface:="position"
  ```

- **Circular Cubic (Position):**
  ```bash
  ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p trajectory_type:="circular_cubic" -p cmd_interface:="position"
  ```

- **Linear Cubic (Position):**
  ```bash
  ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p trajectory_type:="linear_cubic" -p cmd_interface:="position"
  ```

- **Linear Trapezoidal (Position):**
  ```bash
  ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p trajectory_type:="linear_trapezoidal" -p cmd_interface:="position"
  ```

---

