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
        Node(
            package='pointcloud_to_laserscan', executable='dummy_pointcloud_publisher',
            remappings=[('cloud', 'livox/point_cloud2')],
            parameters=[{'cloud_frame_id': 'cloud', 'cloud_extent': 2.0, 'cloud_size': 500}],
            name='cloud_publisher'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0', '0', '2', '0', '0', '0', '1', 'map', 'cloud']
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', 'livox/point_cloud2'),
                        ('scan', 'livox/scan')],
            parameters=[{
                'target_frame': 'livox_frame',
                'transform_tolerance': 0.01,
                'min_height': 0.0,
                'max_height': 1.5,
                'angle_min': -3.1416,  # -M_PI
                'angle_max': 3.1416,  # M_PI
                'angle_increment': 0.007,  # M_PI/360.0
                'scan_time': 0.2,
                'range_min': 0.3,
                'range_max': 70.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        )
    ])
