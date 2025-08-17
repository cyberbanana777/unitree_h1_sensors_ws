# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():

   # Получаем пути к launch-файлам других пакетов
    pkg1_launch_dir = os.path.join(get_package_share_directory('livox_ros_driver2'), 'launch')
    # pkg2_launch_dir = os.path.join(get_package_share_directory('completed_scripts_teleoperation'), 'launch')

    # Включаем первый launch-файл
    launch_file1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg1_launch_dir, 'msg_MID360_launch.py'))
    )

    # Включаем второй launch-файл
    # launch_file2 = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(pkg2_launch_dir, 'teleoperation_launch.py'))
    # )

    node =  Node(
        package="low_level_control",
        executable="low_level_control_without_hands_node",
    )

    node_1 = Node(
        package="cmd_to_high_level_control_package",
        executable="cmd_to_high_level_control_node",
    ),
    
    return LaunchDescription(
        [
            launch_file1,
            # launch_file2,
            node,
            node_1,
        ]
    )
