from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pos_controller',
            executable='pre_programmed_path',
            name='pre_programmed_path'
        ),
        Node(
            package='experiment_nodes',
            executable='imu_recorder',
            name='imu_recorder'
        ),
        Node(
            package='experiment_nodes',
            executable='camera_recorder',
            name='camera_recorder'
        )
    ])
