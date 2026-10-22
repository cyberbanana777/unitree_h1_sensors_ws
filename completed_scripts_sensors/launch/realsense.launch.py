# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
    SetEnvironmentVariable
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition



def launch_setup(context, *args, **kwargs):

    # ========================================================================
    # Conditions
    

    enable_front_camera_condition = PythonExpression([
        '"', LaunchConfiguration('enable_front_camera'), '" == "True"'
    ])

    # ========================================================================
    # Get paths to directories with launch-files and configs


    config_dir = get_package_share_directory('completed_scripts_sensors')
    config_file = os.path.join(config_dir, 'config', 'realsense.yaml')
    
    realsense_pkg_share = get_package_share_directory('realsense2_camera')
    realsense_launch_file = os.path.join(realsense_pkg_share, 'launch', 'rs_launch.py')
    

    # ========================================================================
    # Change env - configuration for realsense
    change_env = SetEnvironmentVariable(
        name='REALSENSE_CONFIG_FILE',
        value=config_file,
        condition=IfCondition(enable_front_camera_condition),
    )

    # ========================================================================
    # Launches
    front_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_file),
        condition=IfCondition(enable_front_camera_condition),
    )


    # ========================================================================
    # System nodes 
    transform_node =Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'realsense_link', 'camera_link'],
        condition=IfCondition(enable_front_camera_condition),
    )

    return [
        change_env,
        front_camera_launch,
        transform_node,
    ]


def generate_launch_description():
    # ========================================================================
    # Declare launch arguments


    camera_mode_arg = DeclareLaunchArgument(
        'enable_front_camera',
        default_value='True',
        description='Enable/Disable front camera (Intel Realsense d435i)',
        choices=['True', 'False']
    )

    

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    return LaunchDescription([
        camera_mode_arg,
        use_sim_time_arg,
        
        OpaqueFunction(function=launch_setup),
    ])