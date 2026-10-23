# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT

import glob
import os
import re

import pytest
import yaml

PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PRESETS = sorted(glob.glob(os.path.join(PACKAGE_DIR, 'config', '*.yaml')))
PROFILE = re.compile(r'^(\d+)x(\d+)x(\d+)$')


def _params(path):
    with open(path, encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    return data['/**']['ros__parameters']


def _id(path):
    return os.path.basename(path)


@pytest.mark.parametrize('path', PRESETS, ids=_id)
def test_preset_structure(path):
    params = _params(path)
    assert isinstance(params, dict) and params


@pytest.mark.parametrize('path', PRESETS, ids=_id)
def test_both_profile_parameter_names_are_set_and_equal(path):
    # Имя параметра зависит от версии realsense2_camera; нода молча игнорирует незнакомое.
    params = _params(path)
    if params.get('enable_color'):
        assert params['rgb_camera.profile'] == params['rgb_camera.color_profile']
        assert PROFILE.match(params['rgb_camera.color_profile'])
    if params.get('enable_depth'):
        assert params['depth_module.profile'] == params['depth_module.depth_profile']
        assert PROFILE.match(params['depth_module.depth_profile'])


@pytest.mark.parametrize('path', PRESETS, ids=_id)
def test_pointcloud_needs_depth_and_color(path):
    params = _params(path)
    if params.get('pointcloud.enable'):
        assert params.get('enable_depth') and params.get('enable_color')


@pytest.mark.parametrize('path', PRESETS, ids=_id)
def test_imu_is_disabled(path):
    params = _params(path)
    assert not params.get('enable_gyro') and not params.get('enable_accel')


def test_readme_table_matches_presets():
    """Колонки RGB и Depth таблицы в README должны совпадать с разрешениями в yaml."""
    rows = {}
    with open(os.path.join(PACKAGE_DIR, 'README.md'), encoding='utf-8') as stream:
        for line in stream:
            cells = [cell.strip() for cell in line.strip().strip('|').split('|')]
            if len(cells) >= 3 and re.fullmatch(r'`\w+`', cells[0]):
                rows[cells[0].strip('`')] = cells
    assert rows, 'в README нет таблицы режимов'
    for path in PRESETS:
        mode = os.path.splitext(os.path.basename(path))[0]
        assert mode in rows, f'режим {mode} не описан в README'
        params = _params(path)
        width, height, fps = PROFILE.match(params['rgb_camera.color_profile']).groups()
        assert rows[mode][1].startswith(f'{width}x{height} @ {fps}'), f'{mode}: колонка RGB'
        if params.get('enable_depth'):
            width, height, fps = PROFILE.match(params['depth_module.depth_profile']).groups()
            assert rows[mode][2].startswith(f'{width}x{height} @ {fps}'), f'{mode}: колонка Depth'
        else:
            assert rows[mode][2] == '-', f'{mode}: глубина выключена, в README должен быть "-"'
