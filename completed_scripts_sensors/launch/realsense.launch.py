# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

"""Передняя камера Intel RealSense D435i (пресеты из realsense_d435i_configs).

Пример:
    ros2 launch completed_scripts_sensors realsense.launch.py
    ros2 launch completed_scripts_sensors realsense.launch.py mode:=rgb_depth
    ros2 launch completed_scripts_sensors realsense.launch.py enable_front_camera:=False
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    enable_front_camera_arg = DeclareLaunchArgument(
        'enable_front_camera',
        default_value='True',
        description='Enable/Disable front camera (Intel Realsense d435i)',
        choices=['True', 'False'],
    )
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='rgb_depth_pointcloud',
        description='Пресет камеры: rgb_usb3 | rgb_usb2 | rgb_depth | rgb_depth_pointcloud',
    )
    serial_no_arg = DeclareLaunchArgument(
        'serial_no', default_value='',
        description='Серийный номер камеры, если подключено несколько')
    parent_frame_arg = DeclareLaunchArgument(
        'parent_frame',
        default_value='realsense_link',
        description='Фрейм робота, к которому привязывается camera_link '
                    '(статический TF с нулевым смещением); пусто = не публиковать')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true')

    front_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('realsense_d435i_configs'), 'launch', 'd435i.launch.py'])),
        launch_arguments={
            'mode': LaunchConfiguration('mode'),
            'serial_no': LaunchConfiguration('serial_no'),
            'parent_frame': LaunchConfiguration('parent_frame'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_front_camera')),
    )

    return LaunchDescription([
        enable_front_camera_arg,
        mode_arg,
        serial_no_arg,
        parent_frame_arg,
        use_sim_time_arg,
        front_camera_launch,
    ])
