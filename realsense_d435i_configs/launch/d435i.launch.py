# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

"""Запуск realsense2_camera с одним из пресетов для Intel RealSense D435i.

Пример:
    ros2 launch realsense_d435i_configs d435i.launch.py mode:=rgb_depth
    ros2 launch realsense_d435i_configs d435i.launch.py mode:=rgb_usb2 serial_no:=123456789
    ros2 launch realsense_d435i_configs d435i.launch.py parent_frame:=realsense_link
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MODES = {
    'rgb_usb3': 'rgb_usb3.yaml',
    'rgb_usb2': 'rgb_usb2.yaml',
    'rgb_depth': 'rgb_depth.yaml',
    'rgb_depth_pointcloud': 'rgb_depth_pointcloud.yaml',
}

TF_ARGUMENTS = ('tf_x', 'tf_y', 'tf_z', 'tf_yaw', 'tf_pitch', 'tf_roll')


# ==============================================================================
# Pure helpers (no launch context needed, covered by test/test_launch_params.py)
# ==============================================================================

def parse_bool(text, name):
    """Parse 'true'/'false' (and 1/0, yes/no) launch argument values."""
    value = str(text).strip().lower()
    if value in ('true', '1', 'yes', 'on'):
        return True
    if value in ('false', '0', 'no', 'off'):
        return False
    raise ValueError(f"Argument '{name}' must be true or false, got '{text}'")


def build_parameters(config_dir, mode, camera_name, serial_no='', config_file='',
                     initial_reset=True, clip_distance=None, use_sim_time=False):
    """Return the ``parameters`` list for the realsense2_camera node.

    The first element is the preset yaml (or the user's own ``config_file``),
    the second is a dict of overrides applied on top of it.
    """
    if mode not in MODES:
        raise RuntimeError(f"Unknown mode '{mode}', choose one of {list(MODES)}")

    preset = config_file or os.path.join(config_dir, MODES[mode])
    if not os.path.isfile(preset):
        raise RuntimeError(f'Config file not found: {preset}')

    overrides = {
        # Из camera_name строятся имена фреймов (camera_link, camera_color_optical_frame, ...).
        'camera_name': camera_name,
        'initial_reset': bool(initial_reset),
        'use_sim_time': bool(use_sim_time),
    }
    if serial_no:
        # Серийный номер - строка с ведущим '_': иначе число в параметре превращается в int.
        overrides['serial_no'] = serial_no if serial_no.startswith('_') else '_' + serial_no
    if clip_distance is not None:
        overrides['clip_distance'] = float(clip_distance)
    return [preset, overrides]


def build_tf_arguments(parent_frame, camera_name, x, y, z, yaw, pitch, roll):
    """Arguments of tf2_ros static_transform_publisher: x y z yaw pitch roll parent child."""
    return [str(x), str(y), str(z), str(yaw), str(pitch), str(roll),
            parent_frame, f'{camera_name}_link']


# ==============================================================================
# Launch
# ==============================================================================

def _launch_setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    camera_name = arg('camera_name')
    clip_distance = arg('clip_distance')
    parameters = build_parameters(
        config_dir=os.path.join(
            get_package_share_directory('realsense_d435i_configs'), 'config'),
        mode=arg('mode'),
        camera_name=camera_name,
        serial_no=arg('serial_no'),
        config_file=arg('config_file'),
        initial_reset=parse_bool(arg('initial_reset'), 'initial_reset'),
        clip_distance=float(clip_distance) if clip_distance else None,
        use_sim_time=parse_bool(arg('use_sim_time'), 'use_sim_time'),
    )

    actions = [Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name=camera_name,
        namespace=arg('camera_namespace'),
        parameters=parameters,
        arguments=['--ros-args', '--log-level', arg('log_level')],
        output='screen',
        emulate_tty=True,
    )]

    # Привязка камеры к дереву TF робота: parent_frame -> <camera_name>_link.
    parent_frame = arg('parent_frame')
    if parent_frame:
        actions.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'{camera_name}_static_tf',
            arguments=build_tf_arguments(
                parent_frame, camera_name, *[arg(name) for name in TF_ARGUMENTS]),
        ))
    return actions


def generate_launch_description():
    declared = [
        DeclareLaunchArgument(
            'mode', default_value='rgb_depth_pointcloud',
            description='rgb_usb3 | rgb_usb2 | rgb_depth | rgb_depth_pointcloud'),
        DeclareLaunchArgument('camera_name', default_value='camera'),
        DeclareLaunchArgument('camera_namespace', default_value='camera'),
        DeclareLaunchArgument(
            'serial_no', default_value='',
            description='Серийный номер, если камер несколько'),
        DeclareLaunchArgument(
            'config_file', default_value='',
            description='Свой yaml вместо пресета (необязательно)'),
        DeclareLaunchArgument(
            'initial_reset', default_value='true',
            description='Перезагружать камеру при старте (true/false)'),
        DeclareLaunchArgument(
            'clip_distance', default_value='',
            description='Отсечка глубины в метрах; пусто = значение из пресета'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument(
            'parent_frame', default_value='',
            description='Если задан, публикуется статический TF '
                        'parent_frame -> <camera_name>_link'),
    ]
    declared += [
        DeclareLaunchArgument(
            name, default_value='0.0',
            description='Смещение/поворот <camera_name>_link в parent_frame (м, рад)')
        for name in TF_ARGUMENTS
    ]
    return LaunchDescription(declared + [OpaqueFunction(function=_launch_setup)])
