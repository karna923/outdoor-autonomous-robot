import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('outdoor_robot')
    ekf_config = os.path.join(pkg, 'config', 'ekf.yaml')
    navsat_config = os.path.join(pkg, 'config', 'navsat_transform.yaml')
    urdf_file = os.path.join(pkg, 'urdf', 'robot.urdf.xacro')

    import xacro
    robot_description = xacro.process_file(urdf_file).toxml()

    return LaunchDescription([

        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=['udp4', '--port', '8888'],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config]
        ),

        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[navsat_config],
            remappings=[
                ('imu/data', '/imu'),
                ('gps/fix', '/gps'),
                ('odometry/filtered', '/odometry/filtered')
            ]
        ),
    ])
