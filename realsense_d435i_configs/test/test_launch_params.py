# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT

import importlib.util
import os

import pytest

PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG_DIR = os.path.join(PACKAGE_DIR, 'config')


def _load_launch_module():
    spec = importlib.util.spec_from_file_location(
        'd435i_launch', os.path.join(PACKAGE_DIR, 'launch', 'd435i.launch.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


launch_module = _load_launch_module()


def test_every_mode_points_to_an_existing_preset():
    for mode, filename in launch_module.MODES.items():
        assert os.path.isfile(os.path.join(CONFIG_DIR, filename)), mode


def test_parameters_start_with_preset_then_overrides():
    params = launch_module.build_parameters(CONFIG_DIR, 'rgb_depth', 'camera')
    assert params[0].endswith('rgb_depth.yaml')
    assert params[1]['camera_name'] == 'camera'
    assert params[1]['initial_reset'] is True
    assert params[1]['use_sim_time'] is False
    assert 'serial_no' not in params[1]
    assert 'clip_distance' not in params[1]


def test_camera_name_is_passed_as_parameter():
    params = launch_module.build_parameters(CONFIG_DIR, 'rgb_usb3', 'front_cam')
    assert params[1]['camera_name'] == 'front_cam'


def test_serial_number_gets_underscore_prefix_once():
    p1 = launch_module.build_parameters(CONFIG_DIR, 'rgb_usb2', 'camera', serial_no='123')
    p2 = launch_module.build_parameters(CONFIG_DIR, 'rgb_usb2', 'camera', serial_no='_123')
    assert p1[1]['serial_no'] == '_123'
    assert p2[1]['serial_no'] == '_123'


def test_clip_distance_and_initial_reset_overrides():
    params = launch_module.build_parameters(
        CONFIG_DIR, 'rgb_depth_pointcloud', 'camera', initial_reset=False, clip_distance='5')
    assert params[1]['clip_distance'] == 5.0
    assert params[1]['initial_reset'] is False


def test_unknown_mode_is_rejected():
    with pytest.raises(RuntimeError, match='Unknown mode'):
        launch_module.build_parameters(CONFIG_DIR, 'rgb_depht', 'camera')


def test_missing_config_file_is_rejected():
    with pytest.raises(RuntimeError, match='not found'):
        launch_module.build_parameters(
            CONFIG_DIR, 'rgb_depth', 'camera', config_file='/no/such/file.yaml')


def test_parse_bool():
    assert launch_module.parse_bool('True', 'x') is True
    assert launch_module.parse_bool('0', 'x') is False
    with pytest.raises(ValueError):
        launch_module.parse_bool('maybe', 'x')


def test_tf_arguments_order_and_child_frame():
    args = launch_module.build_tf_arguments('realsense_link', 'camera', 0.1, 0, 0.2, 0, 0, 0)
    assert args == ['0.1', '0', '0.2', '0', '0', '0', 'realsense_link', 'camera_link']
