#!/usr/bin/env python3

# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

'''
АННОТАЦИЯ
Конвертирует одометрию Unitree (SportModeState из топика odommodestate и
LowState) в стандартный nav_msgs/Odometry. Положение и линейная скорость берутся
из SportModeState, ориентация и угловая скорость — из IMU в LowState. Кватернион
Unitree имеет порядок (w, x, y, z), в сообщение ROS он записывается как
(x, y, z, w). При старте узел включает одометрию робота, при остановке — выключает.
Включение/выключение (Unitree SDK, OdomClient) выполняет отдельный вспомогательный
процесс odom_enabler: SDK и rmw_cyclonedds не могут делить DDS-домен 0 в одном
процессе. Опционально публикует TF odom -> pelvis.

ANNOTATION
Converts Unitree odometry (SportModeState from the odommodestate topic and
LowState) into a standard nav_msgs/Odometry message. Position and linear
velocity come from SportModeState, orientation and angular velocity from the
IMU inside LowState. The Unitree quaternion uses the (w, x, y, z) order and is
written to the ROS message as (x, y, z, w). On start the node enables the
robot odometry and disables it on shutdown. Enabling/disabling (Unitree SDK,
OdomClient) is done by a separate helper process, odom_enabler: the SDK and
rmw_cyclonedds cannot share DDS domain 0 in one process. Can optionally
broadcast the odom -> pelvis TF.
'''

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from odom_converter.odom_enabler import (DEFAULT_CLIENT_TIMEOUT_SEC,
                                         OdomEnablerProcess)
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from unitree_go.msg import LowState, SportModeState


# ==============================================================================
# Constants
# ==============================================================================

DEFAULT_SPORT_TOPIC = 'odommodestate'
DEFAULT_LOWSTATE_TOPIC = 'lowstate'
DEFAULT_ODOM_TOPIC = 'odom'
DEFAULT_ODOM_FRAME = 'odom'
DEFAULT_CHILD_FRAME = 'pelvis'
ODOM_CLIENT_TIMEOUT_SEC = DEFAULT_CLIENT_TIMEOUT_SEC


# ==============================================================================
# Pure conversion helper (no ROS graph needed, easy to unit-test)
# ==============================================================================

def fill_odometry_message(odom_msg, sport_msg, imu_state,
                          pose_variance=0.0, twist_variance=0.0):
    """Fill pose and twist of nav_msgs/Odometry (header is set by the caller).

    Args:
        odom_msg: Odometry message to fill in place.
        sport_msg: object with ``position`` (x, y, z) and ``velocity``
            (vx, vy, vz) sequences.
        imu_state: object with ``quaternion`` (w, x, y, z) and ``gyroscope``
            sequences.
        pose_variance: value for the 6 diagonal elements of the pose
            covariance, 0.0 keeps the matrix all-zero ("unknown").
        twist_variance: same for the twist covariance.

    Returns:
        The same ``odom_msg``.
    """
    odom_msg.pose.pose.position.x = float(sport_msg.position[0])
    odom_msg.pose.pose.position.y = float(sport_msg.position[1])
    odom_msg.pose.pose.position.z = float(sport_msg.position[2])

    # Unitree: (w, x, y, z)  ->  ROS: (x, y, z, w)
    odom_msg.pose.pose.orientation.w = float(imu_state.quaternion[0])
    odom_msg.pose.pose.orientation.x = float(imu_state.quaternion[1])
    odom_msg.pose.pose.orientation.y = float(imu_state.quaternion[2])
    odom_msg.pose.pose.orientation.z = float(imu_state.quaternion[3])

    odom_msg.twist.twist.linear.x = float(sport_msg.velocity[0])
    odom_msg.twist.twist.linear.y = float(sport_msg.velocity[1])
    odom_msg.twist.twist.linear.z = float(sport_msg.velocity[2])

    odom_msg.twist.twist.angular.x = float(imu_state.gyroscope[0])
    odom_msg.twist.twist.angular.y = float(imu_state.gyroscope[1])
    odom_msg.twist.twist.angular.z = float(imu_state.gyroscope[2])

    # 6x6 covariances, row-major. Only the diagonal is ever set.
    for i in range(6):
        odom_msg.pose.covariance[i * 7] = float(pose_variance)
        odom_msg.twist.covariance[i * 7] = float(twist_variance)

    return odom_msg


# ==============================================================================
# Node
# ==============================================================================

class OdometryConverter(Node):
    """Convert Unitree SportModeState + LowState into nav_msgs/Odometry."""

    def __init__(self):
        super().__init__('odometry_converter')

        # Parameters
        self.declare_parameter('sport_topic', DEFAULT_SPORT_TOPIC)
        self.declare_parameter('lowstate_topic', DEFAULT_LOWSTATE_TOPIC)
        self.declare_parameter('odom_topic', DEFAULT_ODOM_TOPIC)
        self.declare_parameter('odom_frame', DEFAULT_ODOM_FRAME)
        self.declare_parameter('child_frame', DEFAULT_CHILD_FRAME)
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('pose_variance', 0.0)
        self.declare_parameter('twist_variance', 0.0)
        # Сетевой интерфейс для DDS Unitree SDK, например 'eth0'. Пусто = по умолчанию.
        self.declare_parameter('network_interface', 'wlp0s20f3')
        # enp3s0

        sport_topic = self.get_parameter('sport_topic').value
        lowstate_topic = self.get_parameter('lowstate_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.pose_variance = float(self.get_parameter('pose_variance').value)
        self.twist_variance = float(self.get_parameter('twist_variance').value)
        network_interface = self.get_parameter('network_interface').value

        # Latest LowState (orientation and angular velocity come from it)
        self.last_lowstate_msg = None

        # Publisher, subscriptions, optional TF broadcaster
        self.publisher_ = self.create_publisher(Odometry, odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.lowstate_subscription = self.create_subscription(
            LowState, lowstate_topic, self.lowstate_callback, 10)
        self.sport_subscription = self.create_subscription(
            SportModeState, sport_topic, self.sport_callback, 10)

        # Robot odometry on/off lives in a separate process (own DDS domain
        # owner, no rclpy). Started last so that a failure here does not leave
        # half-initialised subscriptions behind.
        self.odom_enabler = OdomEnablerProcess(
            network_interface=network_interface,
            client_timeout=ODOM_CLIENT_TIMEOUT_SEC)
        self.odom_enabler.start()

        self.get_logger().info(
            f'Odometry converter started: {sport_topic} + {lowstate_topic} '
            f'-> {odom_topic} ({self.odom_frame} -> {self.child_frame}, '
            f'publish_tf={self.publish_tf})')

    def lowstate_callback(self, msg):
        """Store the latest LowState message."""
        self.last_lowstate_msg = msg

    def sport_callback(self, sport_msg):
        """Build and publish Odometry for every SportModeState message."""
        # Cannot create complete Odometry without LowState data
        if self.last_lowstate_msg is None:
            self.get_logger().warn(
                'Waiting for LowState data...', throttle_duration_sec=5.0)
            return

        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.child_frame
        fill_odometry_message(
            odom_msg,
            sport_msg,
            self.last_lowstate_msg.imu_state,
            self.pose_variance,
            self.twist_variance,
        )
        self.publisher_.publish(odom_msg)

        if self.tf_broadcaster is not None:
            self.tf_broadcaster.sendTransform(self._make_transform(odom_msg))

    def _make_transform(self, odom_msg):
        """Build the odom -> child TF from an Odometry message."""
        transform = TransformStamped()
        transform.header = odom_msg.header
        transform.child_frame_id = odom_msg.child_frame_id
        position = odom_msg.pose.pose.position
        transform.transform.translation.x = position.x
        transform.transform.translation.y = position.y
        transform.transform.translation.z = position.z
        transform.transform.rotation = odom_msg.pose.pose.orientation
        return transform

    def disable_odometry(self):
        """Switch the robot odometry off (safe to call more than once)."""
        enabler = getattr(self, 'odom_enabler', None)
        if enabler is None:
            return
        self.odom_enabler = None
        if not enabler.stop():
            self.get_logger().error(
                'Failed to disable odometry (helper did not exit cleanly)')


# ==============================================================================
# Main
# ==============================================================================

def main(args=None):
    """Initialize and run the odometry converter node."""
    rclpy.init(args=args)
    node = None
    try:
        node = OdometryConverter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.disable_odometry()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
