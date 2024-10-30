from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Path to the URDF file
    link_description_path = get_package_share_directory('arm_description')
    urdf_path = os.path.join(link_description_path, "urdf", "arm.urdf.xacro")

    # Load URDF file content as robot_description
    robot_description = {"robot_description": Command(['xacro ', urdf_path])}

    # Declare Gazebo arguments
    gz_args = DeclareLaunchArgument(
        'gz_args', default_value='-r -v 1 empty.sdf',
        description='Arguments for gz_sim'
    )

    # Launch Gazebo
    gazebo_ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={'gz_args': LaunchConfiguration('gz_args')}.items()
    )

    # Gazebo Spawn Entity Node
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'arm',
            '-allow_renaming', 'true',
            "-x", "0.0", "-y", "0.0", "-z", "0.45",
        ],
    )

    # Joint State Publisher GUI
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
    )

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    # Controller Manager Node with separate spawner for each controller
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, os.path.join(get_package_share_directory('arm_control'), 'config', 'arm_control.yaml')],
        output="screen",
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen"
    )

    position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller"],
        output="screen"
    )

    # Delay controller spawners to ensure they wait for controller_manager
    delay_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=controller_manager_node,
            on_exit=[joint_state_broadcaster]
        )
    )

    delay_position_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[position_controller]
        )
    )

    # Camera bridge node
    bridge_camera = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '--ros-args', '-r', '/camera:=/videocamera',
        ],
        output='screen'
    )

    # Launch description
    return LaunchDescription([
        gz_args,
        gazebo_ignition,
        gz_spawn_entity,
        joint_state_publisher_node,
        robot_state_publisher_node,
        controller_manager_node,
        delay_joint_state_broadcaster,
        delay_position_controller,
        bridge_camera,
    ])
