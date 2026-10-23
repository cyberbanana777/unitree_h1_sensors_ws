#!/usr/bin/env python3
# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT

"""Проверка imu_converter на роботе. Робот должен стоять неподвижно.

Сравнивает sensor_msgs/Imu с исходным unitree_go/LowState и проверяет:
  1. порядок кватерниона: Unitree (w, x, y, z) -> ROS (x, y, z, w);
  2. норму кватерниона (около 1);
  3. модуль ускорения (около 9.81 м/с^2 в покое);
  4. среднюю угловую скорость (около 0 в покое);
  5. ковариации (по умолчанию все нули).
Заодно печатает среднее и дисперсию по осям: их можно взять как
orientation_variance / angular_velocity_variance / linear_acceleration_variance.

Запуск:
    python3 check_imu.py [--duration 10] [--lowstate /lowstate] [--imu /sensors/imu/unitree_h1]
Код возврата 0 = все проверки пройдены, 1 = есть ошибки.
"""

import argparse
import math
import sys

GRAVITY = 9.81


def quaternion_norm(w, x, y, z):
    return math.sqrt(w * w + x * x + y * y + z * z)


def mean_and_variance(values):
    n = len(values)
    mean = sum(values) / n
    variance = sum((v - mean) ** 2 for v in values) / (n - 1) if n > 1 else 0.0
    return mean, variance


def evaluate(samples, quat_tol=0.02, norm_tol=0.01, gravity_tol=0.5, gyro_tol=0.05):
    """samples: список dict с ключами q_unitree, q_ros, gyro, acc, cov_all_zero.

    Возвращает (список строк-результатов, число ошибок, статистика).
    """
    results = []
    errors = 0

    def report(ok, text):
        nonlocal errors
        results.append(('PASS ' if ok else 'FAIL ') + text)
        if not ok:
            errors += 1

    if not samples:
        return ['FAIL нет ни одной пары сообщений (проверьте топики и что узлы запущены)'], 1, {}

    worst = 0.0
    for s in samples:
        w, x, y, z = s['q_unitree']
        rx, ry, rz, rw = s['q_ros']
        worst = max(worst, abs(w - rw), abs(x - rx), abs(y - ry), abs(z - rz))
    report(worst <= quat_tol,
           f'порядок кватерниона (w,x,y,z)->(x,y,z,w): макс. расхождение {worst:.4f} (допуск {quat_tol})')

    norms = [quaternion_norm(*s['q_unitree']) for s in samples]
    report(all(abs(n - 1.0) <= norm_tol for n in norms),
           f'норма кватерниона: {min(norms):.4f} .. {max(norms):.4f} (допуск 1 ± {norm_tol})')

    acc_norms = [math.sqrt(sum(a * a for a in s['acc'])) for s in samples]
    acc_mean = sum(acc_norms) / len(acc_norms)
    report(abs(acc_mean - GRAVITY) <= gravity_tol,
           f'|ускорение| в среднем {acc_mean:.3f} м/с^2 '
           f'(ожидается {GRAVITY} ± {gravity_tol}, робот неподвижен)')

    stats = {}
    for i, axis in enumerate('xyz'):
        stats['gyro_' + axis] = mean_and_variance([s['gyro'][i] for s in samples])
        stats['acc_' + axis] = mean_and_variance([s['acc'][i] for s in samples])
    gyro_mean_max = max(abs(stats['gyro_' + a][0]) for a in 'xyz')
    report(gyro_mean_max <= gyro_tol,
           f'средняя угловая скорость по осям не больше {gyro_mean_max:.4f} рад/с (допуск {gyro_tol})')

    report(all(s['cov_all_zero'] for s in samples),
           'ковариации нулевые (если вы задавали *_variance, эта проверка и не должна проходить)')
    return results, errors, stats


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--duration', type=float, default=10.0)
    parser.add_argument('--lowstate', default='/lowstate')
    parser.add_argument('--imu', default='/sensors/imu/unitree_h1')
    args = parser.parse_args()

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Imu
    from unitree_go.msg import LowState

    rclpy.init()
    node = Node('check_imu')
    last = {'low': None}
    samples = []

    def on_low(msg):
        last['low'] = msg

    def on_imu(msg):
        low = last['low']
        if low is None:
            return
        samples.append({
            'q_unitree': tuple(float(c) for c in low.imu_state.quaternion),
            'q_ros': (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
            'gyro': (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z),
            'acc': (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z),
            'cov_all_zero': not any(msg.orientation_covariance) and
            not any(msg.angular_velocity_covariance) and not any(msg.linear_acceleration_covariance),
        })

    node.create_subscription(LowState, args.lowstate, on_low, 10)
    node.create_subscription(Imu, args.imu, on_imu, 10)
    end = node.get_clock().now().nanoseconds + int(args.duration * 1e9)
    print(f'Собираю данные {args.duration} с. Робот должен стоять неподвижно...')
    while rclpy.ok() and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

    results, errors, stats = evaluate(samples)
    print(f'Сообщений: {len(samples)}')
    print('\n'.join(results))
    if stats:
        print('\nСтатистика в покое (среднее, дисперсия) - для параметров *_variance:')
        for key, (mean, var) in stats.items():
            print(f'  {key}: mean={mean:+.5f}  var={var:.3e}')
    print('\nИТОГ:', 'PASS' if errors == 0 else f'FAIL ({errors})')
    return 1 if errors else 0


if __name__ == '__main__':
    sys.exit(main())
