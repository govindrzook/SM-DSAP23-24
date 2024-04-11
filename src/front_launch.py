from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='digital_architecture',
            executable='front_right_solo',
            name='front_right_solo'
        ),
        Node(
            package='digital_architecture',
            executable='front_left_solo',
            name='front_left_solo'
        ),
        Node(
            package='digital_architecture',
            executable='front_right_servo_controller',
            name='front_right_servo_controller'
        ),
        Node(
            package='digital_architecture',
            executable='front_left_servo_controller',
            name='front_left_servo_controller'
        )
    ])