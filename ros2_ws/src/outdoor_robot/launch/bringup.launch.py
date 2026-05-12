from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='outdoor_robot',
            executable='motor_controller',
            name='motor_controller',
            output='screen'
        ),
        Node(
            package='outdoor_robot',
            executable='odometry_publisher',
            name='odometry_publisher',
            output='screen'
        ),
    ])
