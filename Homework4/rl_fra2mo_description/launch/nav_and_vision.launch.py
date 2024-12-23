from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


 

def generate_launch_description():
    

    #for aruco_ros
    launch_explore_path = PathJoinSubstitution(
        [FindPackageShare("rl_fra2mo_description"), "launch", "task2.launch.py"]
    )    
    launch_explore= IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_explore_path)
    )

    # For camera ----
    rqt_image_view_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view',
        output='screen',
        arguments=['/videocamera'],
    )

    bridge_camera = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '--ros-args', '-r', '/camera:=/videocamera',
        ],
        output='screen',
        )
    # -------

    #for nav and exploration
    launch_aruco_path= PathJoinSubstitution(
        [FindPackageShare("aruco_ros"), "launch", "aruco_detection.launch.py"]
    )
    launch_aruco = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_aruco_path)
    )

    
    return LaunchDescription([
        launch_explore,
        launch_aruco,
        rqt_image_view_node,
        bridge_camera
    ])