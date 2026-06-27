import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    slam_params_default = os.path.expanduser('~/slam_config.yaml')

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
        default_value='true',
        description='Start slam_toolbox when bringing up the robot'
    )

    slam_params_file = LaunchConfiguration('slam_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_slam_toolbox = LaunchConfiguration('start_slam_toolbox')

    # 1. Path to the standard RPLidar launch file
    rplidar_launch_dir = os.path.join(
        get_package_share_directory('rplidar_ros'), 'launch'
    )
    
    # 2. Include the LIDAR driver with your verified configurations
    lidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(rplidar_launch_dir, 'rplidar_a1_launch.py')),
        launch_arguments={
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': '115200',
            'scan_mode': 'Standard',
            'frame_id': 'laser'
        }.items()
    )

    # 3. Static Transform Publisher (Tells ROS2 the LIDAR is centered, 10cm above base_link)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser',
        arguments=['0.0', '0.0', '0.1', '0.0', '0.0', '0.0', 'base_link', 'laser']
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
