from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Step 0. Init LiDAR.
    package_name = "livox_ros_driver2"
    launch_file = "msg_MID360_launch.py"
    path_to_launch = PathJoinSubstitution([
        FindPackageShare(package_name),
        'launch',
        launch_file
    ])
    livox_ros_driver2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([path_to_launch]),
    )

    # Step 1. First converter. 
    # Livox to pointcloud2.
    livox_to_pointcloud2_node = Node(
            package="livox_to_pointcloud2",
            executable="livox_to_pointcloud2_node"
    )
    
    # Step 2. Second converter. 
    # Pointcloud2 to laserscan.
    package_name = "pointcloud_to_laserscan"
    launch_file = "sample_pointcloud_to_laserscan_launch.py"
    path_to_launch = PathJoinSubstitution([
        FindPackageShare(package_name),
        'launch',
        launch_file
    ])
    pointcloud_to_laserscan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([path_to_launch]),
    )

    return LaunchDescription([
        livox_ros_driver2_launch,
        livox_to_pointcloud2_node,
        pointcloud_to_laserscan_launch,
    ])

