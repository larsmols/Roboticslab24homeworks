
---

# Homework 4 instructions

This README provides the instructions needed to build and run our homework project.

---

## Build the Package

1. Build the package using `colcon`:
   ```bash
   colcon build --packages-select rl_fra2mo_description
   ```

2. Source the setup file to make the package accessible:
   ```bash
   source install/setup.bash
   ```

---

## Launch Simulation

Open a new terminal for each command to manage the simulation effectively:

1. Launch the Gazebo simulation:
   ```bash
   ros2 launch rl_fra2mo_description gazebo_fra2mo.launch.py
   ```

2. Launch the navigation system:
   ```bash
   ros2 launch rl_fra2mo_description fra2mo_navigation.launch.py
   ```

---

## Execute Waypoint Navigation

To execute the waypoint-following script, run the following command in a new terminal:
```bash
ros2 run rl_fra2mo_description follow_waypoints.py
```

---

## Record Movement Data

To record the robot's odometry data during its movement, use the following command:
```bash
ros2 bag record /model/fra2mo/odometry -o movement_data
```

---

## Replay and Save Data

### Replay Recorded Data
Replay the recorded bag file with this command:
```bash
ros2 bag play movement_data
```

### Save Odometry Data to a Text File
While replaying the data, save the odometry topic to a text file:
```bash
ros2 topic echo /model/fra2mo/odometry --no-arr > trajectory_data.txt
```

---

## Create a Map

To create a map from the collected data, launch these commands in separate terminals:

1. Start the SLAM system:
   ```bash
   ros2 launch rl_fra2mo_description fra2mo_slam.launch.py
   ```

2. Display the map in RViz:
   ```bash
   ros2 launch rl_fra2mo_description display_fra2mo.launch.py
   ```
---

## Create a Map

To create the plots for the trajectory data you need to run the python scripts in the /python_scripts folder

