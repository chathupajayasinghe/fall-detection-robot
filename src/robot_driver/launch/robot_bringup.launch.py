import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    slam_params_default = os.path.join(
        get_package_share_directory('robot_driver'), 'config', 'slam_params.yaml'
    )

    declare_slam_params_file = DeclareLaunchArgument(
        'slam_params_file',
        default_value=slam_params_default,
        description='Path to the slam_toolbox parameters file'
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation / Gazebo clock'
    )
    declare_start_slam_toolbox = DeclareLaunchArgument(
        'start_slam_toolbox',
        default_value='false',
        description='Start slam_toolbox when bringing up the robot'
    )

    slam_params_file = LaunchConfiguration('slam_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_slam_toolbox = LaunchConfiguration('start_slam_toolbox')

    # 2. LIDAR driver as a plain Node (not an Include) so respawn can be set directly.
    lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/rplidar',
            'serial_baudrate': 460800,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
        }],
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )

    # 3. Static Transform Publisher (Tells ROS2 the LIDAR is centered, 10cm above base_link)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser',
        arguments=['0.0', '0.0', '0.1', '3.14159', '0.0', '0.0', 'base_link', 'laser']
    )

    # 4. Your fresh Motor Controller Node
    motor_node = Node(
        package='robot_driver',
        executable='motor_controller',
        name='motor_controller',
        output='screen'
    )

    # 5. slam_toolbox online async node with autostart and lifecycle activation
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )
        ),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': use_sim_time,
            'autostart': 'true',
            'use_lifecycle_manager': 'false'
        }.items(),
        condition=IfCondition(start_slam_toolbox)
    )

    return LaunchDescription([
        declare_slam_params_file,
        declare_use_sim_time,
        declare_start_slam_toolbox,
        lidar_node,
        static_tf_node,
        motor_node,
        slam_toolbox_launch
    ])
