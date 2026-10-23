# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT

from types import SimpleNamespace

from nav_msgs.msg import Odometry
from odom_converter.odom_converter_node import fill_odometry_message


def _sport():
    return SimpleNamespace(position=[1.0, 2.0, 0.9], velocity=[0.3, 0.0, 0.0])


def _imu():
    return SimpleNamespace(
        quaternion=[1.0, 0.0, 0.0, 0.0],   # Unitree order: w, x, y, z
        gyroscope=[0.0, 0.0, 0.5],
    )


def test_identity_quaternion_stays_identity():
    # Регрессия: раньше q[0] писалось в x, и w получался равным 0.
    msg = fill_odometry_message(Odometry(), _sport(), _imu())
    q = msg.pose.pose.orientation
    assert (q.x, q.y, q.z, q.w) == (0.0, 0.0, 0.0, 1.0)


def test_quaternion_order_is_wxyz_to_xyzw():
    imu = SimpleNamespace(quaternion=[0.5, 0.1, 0.2, 0.3], gyroscope=[0, 0, 0])
    q = fill_odometry_message(Odometry(), _sport(), imu).pose.pose.orientation
    assert (q.w, q.x, q.y, q.z) == (0.5, 0.1, 0.2, 0.3)


def test_position_velocity_and_gyro():
    msg = fill_odometry_message(Odometry(), _sport(), _imu())
    assert msg.pose.pose.position.y == 2.0
    assert msg.twist.twist.linear.x == 0.3
    assert msg.twist.twist.angular.z == 0.5


def test_covariance_diagonal_only():
    msg = fill_odometry_message(
        Odometry(), _sport(), _imu(), pose_variance=0.1, twist_variance=0.2)
    assert [msg.pose.covariance[i * 7] for i in range(6)] == [0.1] * 6
    assert [msg.twist.covariance[i * 7] for i in range(6)] == [0.2] * 6
    assert msg.pose.covariance[1] == 0.0
