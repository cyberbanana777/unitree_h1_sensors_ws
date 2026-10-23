#!/usr/bin/env python3
# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT

"""Проверка odom_converter на роботе.

Сравнивает nav_msgs/Odometry с sensor_msgs/Imu (оба берут ориентацию из одного
источника - LowState) и проверяет:
  1. frame_id / child_frame_id;
  2. норму кватерниона в /odom;
  3. угол между ориентацией в /odom и в IMU (должен быть мал: ловит ошибку порядка
     компонент кватерниона, при которой получался поворот около 180 градусов);
  4. что положение и скорость конечны (не NaN).
Печатает смещение за время наблюдения - сравнивайте с рулеткой при движении.

Запуск:
    python3 check_odom.py [--duration 10] [--odom /odom] [--imu /sensors/imu/unitree_h1]
                          [--odom-frame odom] [--child-frame pelvis]
"""

import argparse
import math
import sys


def quaternion_norm(q):
    return math.sqrt(sum(c * c for c in q))


def yaw_deg(q):
    """Угол рыскания (град) из кватерниона (x, y, z, w)."""
    x, y, z, w = q
    return math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def quaternion_angle_deg(a, b):
    """Угол (град) между двумя поворотами, заданными кватернионами (x, y, z, w)."""
    na, nb = quaternion_norm(a), quaternion_norm(b)
    dot = abs(sum(x * y for x, y in zip(a, b)) / (na * nb))
    dot = min(1.0, dot)
    return math.degrees(2.0 * math.acos(dot))


def evaluate(pairs, odom_frame, child_frame, angle_tol_deg=2.0, norm_tol=0.01):
    """pairs: список dict (frame, child, q_odom, q_imu, pos, vel)."""
    results = []
    errors = 0

    def report(ok, text):
        nonlocal errors
        results.append(('PASS ' if ok else 'FAIL ') + text)
        if not ok:
            errors += 1

    if not pairs:
        return ['FAIL нет ни одной пары /odom + IMU'], 1

    frames = {(p['frame'], p['child']) for p in pairs}
    report(frames == {(odom_frame, child_frame)},
           f'frames: {sorted(frames)} (ожидается {(odom_frame, child_frame)})')
    norms = [quaternion_norm(p['q_odom']) for p in pairs]
    report(all(abs(n - 1.0) <= norm_tol for n in norms),
           f'норма кватерниона в /odom: {min(norms):.4f} .. {max(norms):.4f}')
    worst = max(quaternion_angle_deg(p['q_odom'], p['q_imu']) for p in pairs)
    report(worst <= angle_tol_deg,
           f'угол между ориентацией /odom и IMU: макс. {worst:.3f} град (допуск {angle_tol_deg})')
    finite = all(math.isfinite(v) for p in pairs for v in (*p['pos'], *p['vel']))
    report(finite, 'положение и скорость конечны')
    return results, errors


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--duration', type=float, default=10.0)
    parser.add_argument('--odom', default='/odom')
    parser.add_argument('--imu', default='/sensors/imu/unitree_h1')
    parser.add_argument('--odom-frame', default='odom')
    parser.add_argument('--child-frame', default='pelvis')
    args = parser.parse_args()

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import Imu

    rclpy.init()
    node = Node('check_odom')
    last = {'imu': None}
    pairs = []

    def on_imu(msg):
        o = msg.orientation
        last['imu'] = (o.x, o.y, o.z, o.w)

    def on_odom(msg):
        if last['imu'] is None:
            return
        o = msg.pose.pose.orientation
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        pairs.append({
            'frame': msg.header.frame_id, 'child': msg.child_frame_id,
            'q_odom': (o.x, o.y, o.z, o.w), 'q_imu': last['imu'],
            'pos': (p.x, p.y, p.z), 'vel': (v.x, v.y, v.z),
        })

    node.create_subscription(Imu, args.imu, on_imu, 10)
    node.create_subscription(Odometry, args.odom, on_odom, 10)
    end = node.get_clock().now().nanoseconds + int(args.duration * 1e9)
    print(f'Собираю данные {args.duration} с...')
    while rclpy.ok() and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

    results, errors = evaluate(pairs, args.odom_frame, args.child_frame)
    print(f'Сообщений /odom: {len(pairs)}')
    print('\n'.join(results))
    if pairs:
        a, b = pairs[0]['pos'], pairs[-1]['pos']
        d = [bi - ai for ai, bi in zip(a, b)]
        print(f'Смещение за время наблюдения: dx={d[0]:+.3f} dy={d[1]:+.3f} dz={d[2]:+.3f} м '
              f'(по плоскости {math.hypot(d[0], d[1]):.3f} м)')
        yaw0, yaw1 = yaw_deg(pairs[0]['q_odom']), yaw_deg(pairs[-1]['q_odom'])
        print(f'Рыскание: {yaw0:+.1f} -> {yaw1:+.1f} град')
        n = len(pairs)
        mean_v = [sum(p['vel'][i] for p in pairs) / n for i in range(3)]
        print(f'Средняя линейная скорость в twist: vx={mean_v[0]:+.3f} vy={mean_v[1]:+.3f} '
              f'vz={mean_v[2]:+.3f} м/с')
    print('\nИТОГ:', 'PASS' if errors == 0 else f'FAIL ({errors})')
    return 1 if errors else 0


if __name__ == '__main__':
    sys.exit(main())
