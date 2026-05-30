from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare('camera_vision'),
        'config',
        'dep_cam.yaml',
    ])

    return LaunchDescription([
        Node(
            package='camera_vision',
            executable='dep_cam_node',
            name='dep_cam',
            output='screen',
            parameters=[params_file],
        )
    ])
