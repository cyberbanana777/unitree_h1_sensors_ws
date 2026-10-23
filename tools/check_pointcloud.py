#!/usr/bin/env python3
# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT

"""Проверка livox_to_pointcloud2 на роботе.

Для каждого сообщения livox_ros_driver2/CustomMsg ищет PointCloud2 с тем же
header.stamp и проверяет:
  1. width == point_num, height == 1;
  2. point_step == 18, row_step == 18 * width, len(data) == row_step;
  3. набор полей x, y, z, intensity, tag, line;
  4. что первая и последняя точки совпадают с исходными (x, y, z, reflectivity, tag, line);
  5. что frame_id совпадает.

Запуск:
    python3 check_pointcloud.py [--pairs 20] [--timeout 30]
        [--custom /sensors/lidar/livox/custom_msg] [--cloud /sensors/lidar/livox/point_cloud2]
"""

import argparse
import struct
import sys

POINT_STEP = 18
FIELD_NAMES = ['x', 'y', 'z', 'intensity', 'tag', 'line']


def check_pair(custom, cloud):
    """Возвращает список строк с описанием найденных проблем (пустой = всё хорошо)."""
    problems = []
    if cloud.height != 1 or cloud.width != custom.point_num:
        problems.append(f'width={cloud.width} height={cloud.height}, point_num={custom.point_num}')
    if cloud.point_step != POINT_STEP:
        problems.append(f'point_step={cloud.point_step}, ожидается {POINT_STEP}')
    if cloud.row_step != POINT_STEP * cloud.width:
        problems.append(f'row_step={cloud.row_step}, ожидается {POINT_STEP * cloud.width}')
    if len(cloud.data) != cloud.row_step:
        problems.append(f'len(data)={len(cloud.data)}, row_step={cloud.row_step}')
    if [f.name for f in cloud.fields] != FIELD_NAMES:
        problems.append(f'поля {[f.name for f in cloud.fields]}')
    if cloud.header.frame_id != custom.header.frame_id:
        problems.append(f'frame_id {cloud.header.frame_id!r} != {custom.header.frame_id!r}')
    if problems or not custom.points:
        return problems

    for index in (0, len(custom.points) - 1):
        src = custom.points[index]
        got = struct.unpack_from('<ffffBB', bytes(cloud.data), index * POINT_STEP)
        want = (src.x, src.y, src.z, float(src.reflectivity), src.tag, src.line)
        if any(abs(a - b) > 1e-6 for a, b in zip(got, want)):
            problems.append(f'точка {index}: получено {got}, ожидалось {want}')
    return problems


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--pairs', type=int, default=20)
    parser.add_argument('--timeout', type=float, default=30.0)
    parser.add_argument('--custom', default='/sensors/lidar/livox/custom_msg')
    parser.add_argument('--cloud', default='/sensors/lidar/livox/point_cloud2')
    args = parser.parse_args()

    import rclpy
    from livox_ros_driver2.msg import CustomMsg
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2

    rclpy.init()
    node = Node('check_pointcloud')
    customs, clouds = {}, {}
    checked, bad = [], []

    def key(msg):
        return (msg.header.stamp.sec, msg.header.stamp.nanosec)

    def try_match(k):
        if k in customs and k in clouds:
            problems = check_pair(customs.pop(k), clouds.pop(k))
            checked.append(k)
            if problems:
                bad.append((k, problems))

    def on_custom(msg):
        customs[key(msg)] = msg
        try_match(key(msg))

    def on_cloud(msg):
        clouds[key(msg)] = msg
        try_match(key(msg))

    node.create_subscription(CustomMsg, args.custom, on_custom, 10)
    node.create_subscription(PointCloud2, args.cloud, on_cloud, 10)
    end = node.get_clock().now().nanoseconds + int(args.timeout * 1e9)
    while rclpy.ok() and len(checked) < args.pairs and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

    print(f'Сопоставлено пар CustomMsg/PointCloud2: {len(checked)} (нужно {args.pairs})')
    ok = len(checked) >= args.pairs and not bad
    for k, problems in bad[:5]:
        print(f'FAIL stamp={k}: ' + '; '.join(problems))
    if len(checked) < args.pairs:
        print('FAIL слишком мало пар: нет данных на одном из топиков или не совпадают штампы')
    print('ИТОГ:', 'PASS' if ok else 'FAIL')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
