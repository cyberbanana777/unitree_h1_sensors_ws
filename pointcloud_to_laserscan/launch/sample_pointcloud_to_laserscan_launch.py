# Original code:
# Copyright (c) 2010-2012, Willow Garage, Inc.
# Copyright (c) 2019, Eurotec, Netherlands
#
# Modified by Alexander Grachev and Alice Zenina RTU MIREA (Russia), 2025.
# Changes:
# - Modified point cloud and scanning parameters
# - Updated remappings for Livox compatibility
# - Altered transformation and scanning parameters


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='scanner', default_value='scanner',
            description='Namespace for sample topics'
        ),
        # Node(
        #     package='pointcloud_to_laserscan', 
        #     executable='dummy_pointcloud_publisher',
        #     namespace='sensors',
        #     remappings=[('cloud', 'lidar/livox/point_cloud2')],
        #     parameters=[{'cloud_frame_id': 'livox_frame', 'cloud_extent': 2.0, 'cloud_size': 500}],
        #     name='cloud_publisher_node'
        # ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_transform_publisher',
        #     arguments=['0', '0', '0', '0', '0', '0', '1', 'mid360_link', 'livox_frame']
        # ),
        Node(
            package='pointcloud_to_laserscan', 
            executable='pointcloud_to_laserscan_node',
            namespace='sensors',
            remappings=[
                ('cloud_in', 'lidar/livox/point_cloud2'),
                ('scan', 'lidar/livox/scan')
            ],
            parameters=[{
                'target_frame': 'mid360_link',
                'transform_tolerance': 5.01,
                'min_height': -1.5,
                'max_height': 0.0,
                'angle_min': -3.1416,  # -M_PI
                'angle_max': 3.1416,  # M_PI
                'angle_increment': 0.007,  # M_PI/360.0
                'scan_time': 0.2,
                'range_min': 0.3,
                'range_max': 70.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan_node'
        )
    ])
