from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'marker_id', default_value='201',
            description='Marker ID.'
        ),
        DeclareLaunchArgument(
            'marker_size', default_value='0.1',
            description='Marker size in meters.'
        ),
        DeclareLaunchArgument(
            'camera_topic', default_value='/videocamera',
            description='Camera topic.'
        ),
        DeclareLaunchArgument(
            'camera_info_topic', default_value='/camera_info',
            description='Camera info topic.'
        ),
        DeclareLaunchArgument(
            'marker_frame', default_value='aruco_marker_frame',
            description='Frame for the marker pose.'
        ),
        DeclareLaunchArgument(
            'reference_frame', default_value='',
            description='Reference frame for pose computation.'
        ),
        DeclareLaunchArgument(
            'corner_refinement', default_value='LINES',
            description='Corner refinement method.'
        ),
    ]

    # Node configuration
    aruco_single_node = Node(
        package='aruco_ros',
        executable='single',
        parameters=[{
            'image_is_rectified': True,
            'marker_size': LaunchConfiguration('marker_size'),
            'marker_id': LaunchConfiguration('marker_id'),
            'reference_frame': LaunchConfiguration('reference_frame'),
            'camera_frame': 'camera_optical_frame',
            'marker_frame': LaunchConfiguration('marker_frame'),
            'corner_refinement': LaunchConfiguration('corner_refinement'),
        }],
        remappings=[
            ('/camera_info', LaunchConfiguration('camera_info_topic')),
            ('/image', LaunchConfiguration('camera_topic')),
        ]
    )

    # Combine arguments and node in LaunchDescription
    return LaunchDescription(declared_arguments + [aruco_single_node])
