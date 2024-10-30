Roboticslab24
=============

This project simulates a 4-DOF robotic manipulator with joint controllers and a camera sensor, visualized in RViz and dynamically controlled in Gazebo. It utilizes a Docker environment specified by the RoboticsLab2024 ROS2 Docker scripts to ensure compatibility and reproducibility.

Setup
-----

1. The instructions for the ROS2 Docker are located at: https://github.com/RoboticsLab2024/ros2_docker_scripts.git

2. Run Docker Container:
   - Start the Docker container using the script provided:
     ./docker_run_container.sh <IMAGE_NAME> <CONTAINER_NAME> <FOLDER_NAME>

3. Clone the Project Repository Inside Docker:
   - Clone the robotics manipulator project repository inside the Docker environment using git clone

4. Build the Workspace:
   - Inside the Docker container, navigate to your ROS2 workspace directory and build the project:
     colcon build

5. Source the Workspace:
   - Configure the environment variables for your session inside Docker:
     . install/setup.bash

Running the Simulation
----------------------

- To launch the manipulator in Gazebo within the Docker environment, run:
     ros2 launch arm_gazebo arm_gazebo.launch.py


File Structure
--------------

- urdf/: Contains URDF files defining the manipulator model with links and joints.
- launch/:
- display.launch.py: Opens RViz for visualization.
- arm_gazebo.launch.py: Launches Gazebo with robot controllers and camera.
- config/: Contains controller configurations in arm_control.yaml.
- src/:
- arm_controller_node.cpp: C++ node for joint position control.
- package.xml and CMakeLists.txt: Define project dependencies and build setup.
