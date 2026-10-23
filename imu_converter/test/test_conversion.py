# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT

from types import SimpleNamespace

from imu_converter.imu_converter_node import (
    fill_imu_message,
    quaternion_is_valid,
)
from sensor_msgs.msg import Imu


def _imu_state():
    return SimpleNamespace(
        quaternion=[0.5, 0.1, 0.2, 0.3],   # Unitree order: w, x, y, z
        gyroscope=[1.0, 2.0, 3.0],
        accelerometer=[0.0, 0.0, 9.81],
    )


def test_quaternion_order_is_wxyz_to_xyzw():
    msg = fill_imu_message(Imu(), _imu_state())
    assert (msg.orientation.w, msg.orientation.x,
            msg.orientation.y, msg.orientation.z) == (0.5, 0.1, 0.2, 0.3)


def test_vectors_are_copied():
    msg = fill_imu_message(Imu(), _imu_state())
    assert msg.angular_velocity.z == 3.0
    assert msg.linear_acceleration.z == 9.81


def test_default_covariance_is_unknown_not_minus_one():
    msg = fill_imu_message(Imu(), _imu_state())
    for cov in (msg.orientation_covariance,
                msg.angular_velocity_covariance,
                msg.linear_acceleration_covariance):
        assert all(c == 0.0 for c in cov)


def test_variance_goes_to_diagonal():
    msg = fill_imu_message(Imu(), _imu_state(), orientation_variance=0.01)
    cov = msg.orientation_covariance
    assert cov[0] == cov[4] == cov[8] == 0.01
    assert cov[1] == cov[2] == cov[3] == 0.0


def test_zero_quaternion_is_invalid():
    assert not quaternion_is_valid([0.0, 0.0, 0.0, 0.0])
    assert quaternion_is_valid([1.0, 0.0, 0.0, 0.0])
