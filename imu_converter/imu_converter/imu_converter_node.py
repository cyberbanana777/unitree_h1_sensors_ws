#!/usr/bin/env python3

# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

'''
АННОТАЦИЯ
Конвертирует данные IMU из сообщений Unitree LowState в стандартный формат
ROS 2 sensor_msgs/Imu. Публикует ориентацию, угловую скорость и линейное
ускорение. Кватернион Unitree (порядок w, x, y, z) переупаковывается в порядок
ROS (x, y, z, w). Ковариации по умолчанию нулевые (по REP-145 это означает
«ковариация неизвестна», а сами данные при этом считаются валидными); при
необходимости дисперсии задаются параметрами узла.

ANNOTATION
Converts IMU data from Unitree LowState messages to the standard ROS 2
sensor_msgs/Imu message. The Unitree quaternion (order w, x, y, z) is repacked
into the ROS order (x, y, z, w). Covariances are all-zero by default (per
REP-145 this means "covariance unknown" while the data itself stays valid);
variances can be set with node parameters.
'''

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from unitree_go.msg import LowState


# ==============================================================================
# Constants
# ==============================================================================

DEFAULT_INPUT_TOPIC = '/lowstate'
DEFAULT_OUTPUT_TOPIC = '/sensors/imu/unitree_h1'
DEFAULT_FRAME_ID = 'imu_link'

# Квадрат нормы кватерниона ниже этого порога считаем «IMU ещё не отдаёт данные».
MIN_QUATERNION_NORM_SQ = 1e-6


# ==============================================================================
# Pure conversion helper (no ROS graph needed, easy to unit-test)
# ==============================================================================

def fill_imu_message(imu_msg, imu_state, orientation_variance=0.0,
                     angular_velocity_variance=0.0,
                     linear_acceleration_variance=0.0):
    """Fill sensor_msgs/Imu fields (except header) from unitree_go/IMUState.

    Args:
        imu_msg: Imu message to fill in place.
        imu_state: object with ``quaternion`` (w, x, y, z), ``gyroscope``
            (rad/s) and ``accelerometer`` (m/s^2) sequences.
        orientation_variance: diagonal value for orientation covariance, 0.0
            keeps the matrix all-zero ("unknown").
        angular_velocity_variance: same for angular velocity.
        linear_acceleration_variance: same for linear acceleration.

    Returns:
        The same ``imu_msg``.
    """
    # Unitree: (w, x, y, z)  ->  ROS: (x, y, z, w)
    imu_msg.orientation.w = float(imu_state.quaternion[0])
    imu_msg.orientation.x = float(imu_state.quaternion[1])
    imu_msg.orientation.y = float(imu_state.quaternion[2])
    imu_msg.orientation.z = float(imu_state.quaternion[3])

    imu_msg.angular_velocity.x = float(imu_state.gyroscope[0])
    imu_msg.angular_velocity.y = float(imu_state.gyroscope[1])
    imu_msg.angular_velocity.z = float(imu_state.gyroscope[2])

    imu_msg.linear_acceleration.x = float(imu_state.accelerometer[0])
    imu_msg.linear_acceleration.y = float(imu_state.accelerometer[1])
    imu_msg.linear_acceleration.z = float(imu_state.accelerometer[2])

    # 3x3 covariances, row-major. Only the diagonal is ever set.
    for covariance, variance in (
        (imu_msg.orientation_covariance, orientation_variance),
        (imu_msg.angular_velocity_covariance, angular_velocity_variance),
        (imu_msg.linear_acceleration_covariance, linear_acceleration_variance),
    ):
        for i in range(3):
            covariance[i * 4] = float(variance)

    return imu_msg


def quaternion_is_valid(quaternion):
    """Return True if the quaternion is not the all-zero 'no data' value."""
    norm_sq = sum(float(c) * float(c) for c in quaternion)
    return math.isfinite(norm_sq) and norm_sq > MIN_QUATERNION_NORM_SQ


# ==============================================================================
# Node
# ==============================================================================

class IMUConverterNode(Node):
    """Convert unitree_go/LowState IMU data to sensor_msgs/Imu."""

    def __init__(self):
        super().__init__('imu_converter_node')

        # Parameters
        self.declare_parameter('input_topic', DEFAULT_INPUT_TOPIC)
        self.declare_parameter('output_topic', DEFAULT_OUTPUT_TOPIC)
        self.declare_parameter('frame_id', DEFAULT_FRAME_ID)
        self.declare_parameter('orientation_variance', 0.0)
        self.declare_parameter('angular_velocity_variance', 0.0)
        self.declare_parameter('linear_acceleration_variance', 0.0)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.orientation_variance = float(
            self.get_parameter('orientation_variance').value)
        self.angular_velocity_variance = float(
            self.get_parameter('angular_velocity_variance').value)
        self.linear_acceleration_variance = float(
            self.get_parameter('linear_acceleration_variance').value)

        # Subscription and publisher
        self.subscription = self.create_subscription(
            LowState, input_topic, self.low_state_callback, 10)
        self.imu_publisher = self.create_publisher(Imu, output_topic, 10)

        self.get_logger().info('IMU converter node started')
        self.get_logger().info(f'Subscribed to: {input_topic}')
        self.get_logger().info(f'Publishing to: {output_topic} '
                               f'(frame_id: {self.frame_id})')

    def low_state_callback(self, msg):
        """Convert the IMU part of a LowState message and publish it."""
        imu_state = msg.imu_state

        # До готовности IMU робот присылает нулевой кватернион.
        if not quaternion_is_valid(imu_state.quaternion):
            self.get_logger().warn(
                'IMU quaternion is all zeros, skipping message',
                throttle_duration_sec=5.0)
            return

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.frame_id
        fill_imu_message(
            imu_msg,
            imu_state,
            self.orientation_variance,
            self.angular_velocity_variance,
            self.linear_acceleration_variance,
        )
        self.imu_publisher.publish(imu_msg)


# ==============================================================================
# Main
# ==============================================================================

def main(args=None):
    """Initialize and run the IMU converter node."""
    rclpy.init(args=args)
    node = IMUConverterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
