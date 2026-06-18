import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg = get_package_share_directory('outdoor_robot')
    ekf_config = os.path.join(pkg, 'config', 'ekf.yaml')
    navsat_config = os.path.join(pkg, 'config', 'navsat_transform.yaml')
    urdf_file = os.path.join(pkg, 'urdf', 'robot.urdf.xacro')
    
    slam_share_dir = get_package_share_directory('slam_toolbox')
    slam_config = os.path.join(pkg, 'config', 'slam_toolbox.yaml')
    
    nav2_share_dir = get_package_share_directory('nav2_bringup')
    nav2_config = os.path.join(pkg, 'config', 'nav2', 'nav2_params.yaml')
    
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share_dir, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'slam_params_file': slam_config,
                'use_sim_time': 'false'
            }.items()
    )
    
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': nav2_config,
            'use_sim_time': 'false'
        }.items()
    )
    
    

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
        
        slam_launch, 
        
        nav2_launch, 
    ])
