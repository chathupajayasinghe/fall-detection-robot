import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
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

    return LaunchDescription([
        lidar_node,
        static_tf_node,
        motor_node
    ])
