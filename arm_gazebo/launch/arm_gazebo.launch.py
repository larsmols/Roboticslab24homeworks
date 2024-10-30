from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Define paths
    arm_description_path = get_package_share_directory('arm_description')
    arm_control_path = get_package_share_directory('arm_control')

    # URDF and YAML file paths
    urdf_path = os.path.join(arm_description_path, 'urdf', 'arm.urdf.xacro')
    yaml_path = os.path.join(arm_control_path, 'config', 'arm_control.yaml')

    # Robot description parameter
    robot_description = {"robot_description": Command(['xacro ', urdf_path])}

    # Launch Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": "-r -v 1 empty.sdf"}.items(),
    )

    # Node to spawn the robot entity in Gazebo
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "arm", "-allow_renaming", "true"],
        output="screen",
    )

    # Nodes for state and joint publishers, robot description, and controller manager
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}]
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
        remappings=[("joint_states", "/gui_joint_states")]
    )


    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, yaml_path],
        output="screen",
    )

    # Controller spawners with delay to wait for controller_manager
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
    
    rqt_image_view_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view',
        output='screen',
        arguments=['/videocamera']  # Set the topic for rqt_image_view to listen to
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

    # Arm controller node (verify package name, update if needed)
    arm_controller_node = Node(
        package="arm_control",  # or "arm_controller" if that is the correct package
        executable="arm_controller_node",
        name="arm_controller_node",
        output="screen"
    )

    # Launch description with nodes and arguments
    return LaunchDescription([
        DeclareLaunchArgument("gz_args", default_value="-r -v 1 empty.sdf", description="Arguments for gz_sim"),
        gazebo_launch,
        gz_spawn_entity,
        robot_state_publisher,
        joint_state_publisher_gui,
        controller_manager_node,
        delay_joint_state_broadcaster,
        delay_position_controller,
        bridge_camera,
        arm_controller_node,
        rqt_image_view_node
    ])
