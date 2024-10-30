from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
#remove things s
def generate_launch_description():
    # Path to the robot description (URDF) file
    urdf_file = os.path.join(get_package_share_directory('arm_description'), 'urdf', 'arm.urdf.xacro')

    # Path to the RViz config file
    rviz_config_file = os.path.join(get_package_share_directory('arm_description'), 'config', 'display_config.rviz')
    print(rviz_config_file)

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]  # loads the saved RViz config file if provided
        )
    ])
