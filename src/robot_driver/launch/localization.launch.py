import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_map = os.path.join(os.path.expanduser('~'), 'maps', 'home_map.yaml')
    default_params = PathJoinSubstitution(
        [FindPackageShare('robot_driver'), 'config', 'amcl_params.yaml']
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('amcl_params_file')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('amcl_params_file', default_value=default_params),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time,
                 'yaml_filename': map_yaml_file},
            ],
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'bond_timeout': 60.0,
                'attempt_respawn_reconnection': True,
                'bond_respawn_max_duration': 30.0,
                'node_names': ['map_server', 'amcl'],
            }],
        ),
    ])
