from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='digital_architecture',
            executable='back_right_solo',
            name='back_right_solo'
        ),
        Node(
            package='digital_architecture',
            executable='back_left_solo',
            name='back_left_solo'
        ),
        Node(
            package='digital_architecture',
            executable='back_right_servo_controller',
            name='back_right_servo_controller'
        ),
        Node(
            package='digital_architecture',
            executable='back_left_servo_controller',
            name='back_left_servo_controller'
        )
    ])