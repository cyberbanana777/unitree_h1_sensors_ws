#!/usr/bin/env python3

# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.


'''
АННОТАЦИЯ
ROS 2-узел для расчета одометрии на основе команд движения (cmd_vel). Выполняет:
- Подписку на топик /cmd_vel для получения скоростей движения
- Интегрирование скоростей для вычисления позиции и ориентации
- Публикацию одометрии в топик /odom с высокой частотой (до 150 Гц)
- Преобразование угла в кватернион и нормализацию углов

ANNOTATION
ROS 2 node for odometry calculation based on velocity commands (cmd_vel). Features:
- Subscription to /cmd_vel topic for velocity inputs
- Velocity integration for position and orientation calculation
- Odometry publication to /odom topic at high rate (up to 150 Hz)
- Angle to quaternion conversion and angle normalization
'''


import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion
from tf_transformations import quaternion_from_euler
import math


# ==============================================================================
# Constants and Configuration
# ==============================================================================

DEFAULT_PUBLISH_RATE = 150  # Hz


# ==============================================================================
# Odometry Calculator Node
# ==============================================================================

class OdometryCalculator(Node):
    """ROS 2 node that calculates and publishes odometry based on velocity commands."""
    
    def __init__(self):
        super().__init__('odometry_calculator')
        
        # Node parameters
        self.declare_parameter('publish_rate', DEFAULT_PUBLISH_RATE)
        self.publish_rate = self.get_parameter('publish_rate').value
        self.get_logger().info(f'Publish rate set to {self.publish_rate} Hz')
        
        # Initialize pose and velocity state
        self.x = 0.0    # Current x position (meters)
        self.y = 0.0    # Current y position (meters)
        self.th = 0.0   # Current orientation (radians)
        
        self.vx = 0.0   # Linear x velocity (m/s)
        self.vy = 0.0   # Linear y velocity (m/s)
        self.vth = 0.0  # Angular velocity (rad/s)
        
        # Timing control
        self.last_time = self.get_clock().now()
        
        # ROS 2 subscriptions and publishers
        self._setup_communications()
        
        self.get_logger().info('Odometry node started')

    def _setup_communications(self):
        """Initialize all ROS 2 subscriptions and publishers."""
        # Subscribe to velocity commands
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Create odometry publisher with configured rate
        self.timer = self.create_timer(
            1.0 / self.publish_rate, 
            self.publish_odometry
        )
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

    def cmd_vel_callback(self, msg):
        """Handle incoming velocity commands and update current velocities."""
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vth = msg.angular.z

    def publish_odometry(self):
        """Calculate and publish odometry message at configured rate."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).seconds
        self.last_time = current_time
        
        # Calculate position delta
        delta_x = (self.vx * math.cos(self.th)) - (self.vy * math.sin(self.th)) * dt
        delta_y = (self.vx * math.sin(self.th)) + (self.vy * math.cos(self.th)) * dt
        delta_th = self.vth * dt
        
        # Update position
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th
        self.th = self._normalize_angle(self.th)
        
        # Create and populate odometry message
        odom_msg = self._create_odometry_message(current_time)
        self.odom_pub.publish(odom_msg)
        
        self.get_logger().debug(
            f'Odometry: x={self.x:.2f}m, y={self.y:.2f}m, '
            f'θ={math.degrees(self.th):.2f}°'
        )

    def _create_odometry_message(self, current_time):
        """Construct Odometry message with current pose and velocity."""
        odom = Odometry()
        
        # Header information
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom_frame'
        odom.child_frame_id = 'pelvis'
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Orientation (convert yaw to quaternion)
        q = quaternion_from_euler(0, 0, self.th)
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        
        # Velocity
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vth
        
        return odom

    def _normalize_angle(self, angle):
        """Normalize angle to [-π, π] range.
        
        Args:
            angle: Input angle in radians
            
        Returns:
            Angle normalized to [-π, π] range
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


# ==============================================================================
# Main Execution
# ==============================================================================

def main(args=None):
    """Initialize and run the odometry node."""
    rclpy.init(args=args)
    node = OdometryCalculator()
    
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f'Node stopped due to: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()