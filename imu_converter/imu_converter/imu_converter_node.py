#!/usr/bin/env python3

# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

'''
АННОТАЦИЯ
Конвертирует сырые данные IMU из сообщений Unitree LowState в стандартный 
формат ROS 2 Imu. Публикует ориентацию, угловую скорость и линейное ускорение
с меткой времени. Ковариационные матрицы помечаются как неизвестные (-1.0).

ANNOTATION
Converts raw IMU data from Unitree LowState messages to standard ROS 2 Imu
format. Publishes orientation, angular velocity, and linear acceleration with
timestamp. Covariance matrices are marked as unknown (-1.0).
'''

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from unitree_go.msg import LowState, IMUState
from geometry_msgs.msg import Quaternion, Vector3
import numpy as np

class IMUConverterNode(Node):
    def __init__(self):
        super().__init__('imu_converter_node')
        
        # Parametrs
        self.declare_parameter('input_topic', '/lowstate')
        self.declare_parameter('output_topic', '/sensors/imu/unitree_h1')
        self.declare_parameter('frame_id', 'imu_link')
        
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # Publisher and subscription
        self.subscription = self.create_subscription(
            LowState,
            input_topic,
            self.low_state_callback,
            10
        )
        
        self.imu_publisher = self.create_publisher(
            Imu,
            output_topic,
            10
        )
        
        self.get_logger().info(f'IMU Converter Node started')
        self.get_logger().info(f'Subscribed to: {input_topic}')
        self.get_logger().info(f'Published to: {output_topic}')
    
    def low_state_callback(self, msg):
        """Callback for processing LowState message"""
        try:
            imu_msg = self.convert_to_imu(msg.imu_state)
            self.imu_publisher.publish(imu_msg)
        except Exception as e:
            self.get_logger().error(f'Convertation error IMU: {str(e)}')
    
    def convert_to_imu(self, imu_state):
        """Convert IMUState to sensor_msgs/Imu"""
        imu_msg = Imu()
        
        # Set header
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.frame_id
        
        # quaternion of orientetion
        imu_msg.orientation.x = float(imu_state.quaternion[1])
        imu_msg.orientation.y = float(imu_state.quaternion[2])
        imu_msg.orientation.z = float(imu_state.quaternion[3])
        imu_msg.orientation.w = float(imu_state.quaternion[0])
        
        # orientation_covariance (unknown)
        imu_msg.orientation_covariance[0] = -1.0
        
        # Angular velocity (gyroscope)
        imu_msg.angular_velocity.x = float(imu_state.gyroscope[0])
        imu_msg.angular_velocity.y = float(imu_state.gyroscope[1])
        imu_msg.angular_velocity.z = float(imu_state.gyroscope[2])
        
        # Angular velocity covariance (unknown)
        imu_msg.angular_velocity_covariance[0] = -1.0
        
        # Linear acceleration (accelerometer)
        imu_msg.linear_acceleration.x = float(imu_state.accelerometer[0])
        imu_msg.linear_acceleration.y = float(imu_state.accelerometer[1])
        imu_msg.linear_acceleration.z = float(imu_state.accelerometer[2])
        
        # Linear acceleration coveriance (unknown)
        imu_msg.linear_acceleration_covariance[0] = -1.0
        
        return imu_msg

def main(args=None):
    rclpy.init(args=args)
    node = IMUConverterNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user')
    except Exception as e:
        node.get_logger().error(f'Stopped due error {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
