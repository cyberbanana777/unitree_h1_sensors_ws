#!/usr/bin/env python3

# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

'''
АННОТАЦИЯ
Конвертирует сообщения одометрии Unitree из форматов OdomModeState и LowState
в стандартный nav_msgs/Odometry для ROS 2. Объединяет данные о позиции и
линейной скорости из OdomModeState с ориентацией и угловой скоростью из
LowState. Требует одновременной доступности обоих источников данных для
корректной работы.

ANNOTATION
Converts Unitree odometry messages from OdomModeState and LowState formats to
standard nav_msgs/Odometry for ROS 2. Combines position and linear velocity
data from OdomModeState with orientation and angular velocity from LowState.
Requires simultaneous availability of both data sources for proper operation.
'''

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import (
    Point, Quaternion, Pose, PoseWithCovariance, Twist, TwistWithCovariance
)
from std_msgs.msg import Header
from unitree_go.msg import SportModeState, LowState
from high_level_control import OdomClient


class OdometryConverter(Node):
    """
    Node that converts Unitree OdomModeState and LowState to Odometry messages.
    """
    def __init__(self):
        super().__init__('odometry_converter')
        
        # Initialize channel
        ChannelFactoryInitialize(0)
        
        # Initialize and start OdomClient
        self.odom_client = OdomClient()
        self.odom_client.Init()
        self.odom_client.SetTimeout(1.0)
        self.odom_client.EnableOdom()
        
        # Subscriptions to both message types
        self.sport_subscription = self.create_subscription(
            SportModeState,
            'odommodestate',
            self.sport_callback,
            10
        )
        
        self.lowstate_subscription = self.create_subscription(
            LowState,
            'lowstate',
            self.lowstate_callback,
            10
        )
        
        self.publisher_ = self.create_publisher(Odometry, 'odom', 10)
        
        # Store the latest received messages
        self.last_sport_msg = None
        self.last_lowstate_msg = None
        
        # Covariance matrices
        self.pose_covariance = [0.0] * 36
        self.twist_covariance = [0.0] * 36

        self.get_logger().info('Odometry converter initialized and started')

    def lowstate_callback(self, msg):
        """Store the latest LowState message."""
        self.last_lowstate_msg = msg

    def sport_callback(self, msg):
        """Process SportModeState message and publish Odometry."""
        self.last_sport_msg = msg
        
        # Cannot create complete Odometry without LowState data
        if self.last_lowstate_msg is None:
            self.get_logger().warn('Waiting for LowState data...')
            return
        
        odom_msg = self.create_odometry_message(msg, self.last_lowstate_msg)
        self.publisher_.publish(odom_msg)

    def create_odometry_message(self, sport_msg, lowstate_msg):
        """Create Odometry message from both data sources."""
        odom_msg = Odometry()
        
        # Fill header
        odom_msg.header = Header()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'  # Parent coordinate frame
        odom_msg.child_frame_id = 'pelvis'  # Child coordinate frame
        
        # Position (from SportModeState)
        position = Point()
        position.x = float(sport_msg.position[0])
        position.y = float(sport_msg.position[1])
        position.z = float(sport_msg.position[2])
        
        # Orientation (from IMUState in LowState)
        orientation = Quaternion()
        
        # Always use LowState data for orientation
        if hasattr(lowstate_msg.imu_state, 'quaternion'):
            orientation.x = float(lowstate_msg.imu_state.quaternion[0])
            orientation.y = float(lowstate_msg.imu_state.quaternion[1])
            orientation.z = float(lowstate_msg.imu_state.quaternion[2])
            orientation.w = float(lowstate_msg.imu_state.quaternion[3])
        else:
            # Fallback if IMUState structure changed
            orientation.w = 1.0
        
        odom_msg.pose = PoseWithCovariance()
        odom_msg.pose.pose = Pose(position=position, orientation=orientation)
        odom_msg.pose.covariance = self.pose_covariance
        
        # Linear velocity (from SportModeState)
        twist_linear = sport_msg.velocity
        twist = Twist()
        twist.linear.x = float(twist_linear[0])
        twist.linear.y = float(twist_linear[1])
        twist.linear.z = float(twist_linear[2])
        
        # Angular velocity (from IMUState in LowState)
        # Always use LowState data for angular velocity
        if hasattr(lowstate_msg.imu_state, 'gyroscope'):
            twist.angular.x = float(lowstate_msg.imu_state.gyroscope[0])
            twist.angular.y = float(lowstate_msg.imu_state.gyroscope[1])
            twist.angular.z = float(lowstate_msg.imu_state.gyroscope[2])
        else:
            # Fallback: use yaw_speed from SportModeState
            twist.angular.z = float(sport_msg.yaw_speed)
        
        odom_msg.twist = TwistWithCovariance()
        odom_msg.twist.twist = twist
        odom_msg.twist.covariance = self.twist_covariance
        
        return odom_msg


def main(args=None):
    rclpy.init(args=args)
    node = OdometryConverter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user')
    except Exception as e:
        node.get_logger().error(f'Node stopped due to error: {str(e)}')
    finally:
        node.odom_client.DisableOdom()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()