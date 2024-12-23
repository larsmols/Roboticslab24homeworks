#!/usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import yaml
import math
import os

def load_goals(yaml_file):
    # Dynamically get the full path to the YAML file
    package_share_directory = os.path.join(os.getenv("ROS_WORKSPACE", "/home/user/ros2_ws"), "src/Roboticslab24/Homework4/rl_fra2mo_description/config")
    yaml_file_path = os.path.join(package_share_directory, yaml_file)
    with open(yaml_file_path, 'r') as file:
        return yaml.safe_load(file)



def create_pose(goal):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    
    # Extract pose data
    pose.pose.position.x = goal["pose"]["x"]
    pose.pose.position.y = goal["pose"]["y"]
    pose.pose.position.z = 0.0  # Assuming a 2D plane

    # Convert yaw (degrees) to quaternion
    yaw_rad = math.radians(goal["pose"]["yaw"])
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = math.sin(yaw_rad / 2)
    pose.pose.orientation.w = math.cos(yaw_rad / 2)

    return pose



def main():
    rclpy.init()
    navigator = BasicNavigator()

    # Load goals from the YAML file
    goals_data = load_goals("goals.yaml")

    # Create PoseStamped messages from the loaded goals
    goal_order = [2, 3, 1, 0]  # Indices: Goal 3 → Goal 4 → Goal 2 → Goal 1
    print(goals_data)
    goal_poses = [create_pose(goal) for goal in goals_data["goals"]]


    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active(localizer="smoother_server")

    # Start the waypoint navigation
    nav_start = navigator.get_clock().now()
    navigator.followWaypoints(goal_poses)

    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()

        if feedback and i % 5 == 0:
            print('Executing current waypoint: ' +
                  str(feedback.current_waypoint + 1) + '/' + str(len(goal_poses)))
            now = navigator.get_clock().now()

            # Some navigation timeout to demo cancellation
            if now - nav_start > Duration(seconds=600):
                navigator.cancelTask()

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    # navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()
