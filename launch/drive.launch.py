from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='diff_drive_simple',
            executable='diff_drive',
            name='diff_drive',
            output='screen',
            parameters=[
                {"min_speed": 0.0},
                {"max_speed": 1.0}
            ]
            # remappings=[
                # ('/input/pose', '/turtlesim1/turtle1/pose'),
                # ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            # ]
        )
    ])