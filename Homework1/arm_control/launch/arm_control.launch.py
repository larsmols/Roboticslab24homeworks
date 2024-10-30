from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare launch arguments
    declared_arguments = []

    arm_description_path = get_package_share_directory('arm_description')
    arm_control_path = get_package_share_directory('arm_control')

    declared_arguments.append(
        DeclareLaunchArgument(
            "gz_args",
            default_value="-r -v 1 empty.sdf",
            description="Arguments for gz_sim"
        )
    )

    urdf_path = os.path.join(arm_description_path, "urdf", "arm.urdf.xacro")
    yaml_path = os.path.join(arm_control_path, "config", "arm_control.yaml")

    # Load the URDF as a robot description parameter
    robot_description = {"robot_description": Command(['xacro ', urdf_path])}

    # Nodes to launch
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": True}]
    )

    # Include Gazebo and spawn entity
    gazebo_ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            )
        ),
        launch_arguments={"gz_args": LaunchConfiguration("gz_args")}.items(),
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "arm",
            "-allow_renaming", "true",
        ],
        output="screen",
    )

    # Start controller_manager node
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, yaml_path],
        output="screen"
    )

    # Spawners for the joint state broadcaster and position controller
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "--controller-manager", "/controller_manager"],
    )
    
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
    )


    # Event handlers to delay controller spawning until after controller_manager is up
    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=controller_manager_node,
            on_exit=[joint_state_broadcaster],
        )
    )

    delay_position_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[position_controller],
        )
    )
    
    #     # Add the arm_controller_node
    # arm_controller_node = Node(
    #     package="arm_control",
    #     executable="arm_controller_node",
    #     name="arm_controller_node",
    #     output="screen"
    # )

    # Launch description
    return LaunchDescription(declared_arguments + [
        gazebo_ignition,
        gz_spawn_entity,
        robot_state_publisher_node,
        controller_manager_node,
        delay_joint_state_broadcaster,
        delay_position_controller,
        joint_state_publisher_gui
        # arm_controller_node
    ])
