# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

"""Livox MID360 + IMU Unitree H1: драйвер, конвертеры и TF.

Цепочка топиков (все внутри namespace `sensors`):

    livox_ros_driver2   -> /sensors/lidar/livox/custom_msg
    livox_to_pointcloud2 -> /sensors/lidar/livox/point_cloud2
    pointcloud_to_laserscan -> /sensors/lidar/livox/scan
    imu_converter       -> /sensors/imu/unitree_h1  (из /lowstate)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _include(package, *path, launch_arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), *path])),
        launch_arguments=(launch_arguments or {}).items())


def generate_launch_description():

    # Смещение и поворот livox_frame относительно mid360_link.
    # Драйвер уже применяет extrinsic (roll=180, pitch=16) из MID360_config.json,
    # поэтому по умолчанию тут единичное преобразование. Подберите значения
    # по RViz (Fixed Frame = mid360_link), если облако смещено или повёрнуто.
    tf_args = [
        DeclareLaunchArgument(name, default_value='0.0', description=desc)
        for name, desc in (
            ('lidar_tf_x', 'x livox_frame в mid360_link, м'),
            ('lidar_tf_y', 'y livox_frame в mid360_link, м'),
            ('lidar_tf_z', 'z livox_frame в mid360_link, м'),
            ('lidar_tf_yaw', 'yaw, рад'),
            ('lidar_tf_pitch', 'pitch, рад'),
            ('lidar_tf_roll', 'roll, рад'),
        )
    ]

    # Step 1. LiDAR driver (publishes livox_ros_driver2/CustomMsg).
    livox_ros_driver2_launch = _include(
        'livox_ros_driver2', 'launch_ROS2', 'msg_MID360_launch.py')

    # Step 2. CustomMsg -> PointCloud2.
    livox_to_pointcloud2_node = Node(
        package='livox_to_pointcloud2',
        executable='livox_to_pointcloud2_node',
        namespace='sensors',
        remappings=[
            ('livox/lidar', 'lidar/livox/custom_msg'),
            ('livox/point_cloud2', 'lidar/livox/point_cloud2'),
        ],
    )

    # Step 3. PointCloud2 -> LaserScan.
    pointcloud_to_laserscan_launch = _include(
        'pointcloud_to_laserscan', 'launch',
        'sample_pointcloud_to_laserscan_launch.py',
        launch_arguments={'transform_tolerance': LaunchConfiguration('transform_tolerance')})

    # Step 4. IMU from /lowstate -> sensor_msgs/Imu.
    imu_converter_node = Node(
        package='imu_converter',
        executable='imu_converter_node',
        namespace='sensors',
    )

    # Step 5. Connect livox_frame to mid360_link (part of the robot description).
    # Формат аргументов: x y z yaw pitch roll parent child.
    transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='livox_static_transform_publisher',
        arguments=[
            LaunchConfiguration('lidar_tf_x'),
            LaunchConfiguration('lidar_tf_y'),
            LaunchConfiguration('lidar_tf_z'),
            LaunchConfiguration('lidar_tf_yaw'),
            LaunchConfiguration('lidar_tf_pitch'),
            LaunchConfiguration('lidar_tf_roll'),
            'mid360_link',
            'livox_frame',
        ],
    )

    # Допуск по времени для TF в pointcloud_to_laserscan (раньше было зашито 5.01 с).
    transform_tolerance_arg = DeclareLaunchArgument(
        'transform_tolerance', default_value='0.1',
        description='Допуск по времени при поиске TF в pointcloud_to_laserscan, с')

    return LaunchDescription([
        *tf_args,
        transform_tolerance_arg,
        livox_ros_driver2_launch,
        livox_to_pointcloud2_node,
        pointcloud_to_laserscan_launch,
        imu_converter_node,
        transform_node,
    ])
