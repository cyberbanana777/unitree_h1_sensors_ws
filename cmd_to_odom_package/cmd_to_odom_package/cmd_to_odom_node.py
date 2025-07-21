#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Quaternion
from tf_transformations import quaternion_from_euler
import math

FREQUENCY = 150

class OdometryCalculator(Node):
    def __init__(self):
        super().__init__('odometry_calculator')
        
        # Параметры
        self.declare_parameter('publish_rate', FREQUENCY)  # Частота в Гц (по умолчанию 10)
        self.publish_rate = self.get_parameter('publish_rate').value
        self.get_logger().info(f'Publish rate set to {self.publish_rate} Hz')
        
        # Текущая позиция и ориентация
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        
        # Текущие скорости (из последнего cmd_vel)
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        
        # Время последнего обновления
        self.last_time = self.get_clock().now()
        
        # Подписка на команды движения
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # Публикация одометрии с заданной частотой
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_odometry)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        self.get_logger().info('Odometry node started')

    def cmd_vel_callback(self, msg):
        # Сохраняем текущие скорости
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vth = msg.angular.z

    def publish_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).seconds # В секундах
        self.last_time = current_time
        
        # Вычисление изменения позиции
        delta_x = (self.vx * math.cos(self.th) - self.vy * math.sin(self.th)) * dt
        delta_y = (self.vx * math.sin(self.th) + self.vy * math.cos(self.th)) * dt
        delta_th = self.vth * dt
        
        # Обновление позиции
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th
        
        # Нормализация угла
        self.th = self.normalize_angle(self.th)
        
        # Создание сообщения Odometry
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom_frame'
        odom.child_frame_id = 'pelvis'
        
        # Заполнение позиции
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Преобразование угла в кватернион
        q = quaternion_from_euler(0, 0, self.th)
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        
        # Заполнение скорости
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vth
        
        # Публикация
        self.odom_pub.publish(odom)
        
        self.get_logger().debug(f'Odometry: x={self.x:.2f}, y={self.y:.2f}, th={math.degrees(self.th):.2f}°')

    def normalize_angle(self, angle):
        """Нормализует угол в диапазон [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = OdometryCalculator()
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().info(f'Node stoped by reason: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
