import glob
import os
from setuptools import find_packages, setup

package_name = 'robot_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/robot_bringup.launch.py',
            'launch/localization.launch.py',
            'launch/nav2_navigation.launch.py',
        ]),
        ('share/' + package_name + '/config', glob.glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='et2020045',
    maintainer_email='et2020045@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'motor_controller = robot_driver.motor_controller:main',
            'odom_publisher = robot_driver.odom_publisher:main',
            'firestore_dispatcher = robot_driver.firestore_dispatcher:main',
        ],
    },
)
