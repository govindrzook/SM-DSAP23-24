from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='digital_architecture',
            executable='central_controller',
            name='central_controller'
        ),
        Node(
            package='digital_architecture',
            executable='logger',
            name='logger'
        ),
        Node(
            package='digital_architecture',
            executable='imu',
            name='imu'
        )
    ])